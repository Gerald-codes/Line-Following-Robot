/**
 * line_following.c - Single IR Sensor Line Following with Side Swapping
 * Uses ONE IR sensor and ONE PID controller
 * GPIO 6 triggers side swap, turns slowly until center of line (0.9+ black) detected
 */

#include "line_following.h"
#include "ir_sensor.h"
#include "motor.h"
#include "config.h"
#include "pin_definitions.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

// ============================================================================
// SIDE SWAP CONTROL
// ============================================================================
#define SWAP_SENSOR_PIN 6
static bool side_inverted = false;
static bool prev_swap_state = false;
static bool turning_to_center = false;
static uint32_t turn_start_time = 0;

// Track intersection count and turn direction
static uint8_t intersection_count = 0;
static bool turn_left = true;

#define MAX_TURN_TIME_MS 3000

// ============================================================================
// PID STATE
// ============================================================================
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output;
    bool initialized;
} PIDController;

static PIDController pid = {
    .kp = LINE_PID_KP,
    .ki = LINE_PID_KI,
    .kd = LINE_PID_KD,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .output = 0.0f,
    .initialized = false
};

// ============================================================================
// LINE STATE
// ============================================================================
static LineFollowState current_state = LINE_FOLLOW_CENTERED;
static float normalized_error = 0.0f;
static float filtered_error = 0.0f;

#define ERROR_FILTER_ALPHA 0.7f

// ============================================================================
// LINE FOLLOWING CONTROL STATE
// ============================================================================
static float L_power = 0.0f;
static float R_power = 0.0f;

static uint32_t line_lost_start = 0;
#define LINE_LOST_TIMEOUT_MS 2000

// Base speeds and limits
#define BASE_POWER 40.0f
#define MIN_POWER 25.0f
#define MAX_POWER 60.0f

// Slow turn speed to find center
#define SLOW_TURN_SPEED 35.0f

// Adaptive PID gains
typedef struct {
    float kp;
    float ki;
    float kd;
} PIDGainSet;

static const PIDGainSet SMOOTH_GAINS = {
    .kp = 1.5f,
    .ki = 0.0f,
    .kd = 0.8f
};

static const PIDGainSet AGGRESSIVE_GAINS = {
    .kp = 3.0f,
    .ki = 0.0f,
    .kd = 2.5f
};

#define SMOOTH_ERROR_THRESHOLD 0.2f
#define AGGRESSIVE_ERROR_THRESHOLD 0.6f

static uint32_t last_debug_time = 0;

// ============================================================================
// MOTOR CONTROL HELPERS
// ============================================================================
static int apply_deadband(float power) {
    int int_power = (int)power;
    
    if (int_power > 0 && int_power < MIN_POWER) {
        return MIN_POWER;
    } else if (int_power < 0 && int_power > -MIN_POWER) {
        return -MIN_POWER;
    }
    
    if (int_power > MAX_POWER) return MAX_POWER;
    if (int_power < -MAX_POWER) return -MAX_POWER;
    
    return int_power;
}

// ============================================================================
// ADAPTIVE GAINS
// ============================================================================
static void apply_adaptive_gains(float error_magnitude) {
    PIDGainSet gains;
    
    if (error_magnitude < SMOOTH_ERROR_THRESHOLD) {
        gains = SMOOTH_GAINS;
    } else if (error_magnitude > AGGRESSIVE_ERROR_THRESHOLD) {
        gains = AGGRESSIVE_GAINS;
    } else {
        float blend = (error_magnitude - SMOOTH_ERROR_THRESHOLD) / 
                     (AGGRESSIVE_ERROR_THRESHOLD - SMOOTH_ERROR_THRESHOLD);
        
        gains.kp = SMOOTH_GAINS.kp + blend * (AGGRESSIVE_GAINS.kp - SMOOTH_GAINS.kp);
        gains.ki = SMOOTH_GAINS.ki + blend * (AGGRESSIVE_GAINS.ki - SMOOTH_GAINS.ki);
        gains.kd = SMOOTH_GAINS.kd + blend * (AGGRESSIVE_GAINS.kd - SMOOTH_GAINS.kd);
    }
    
    pid.kp = gains.kp;
    pid.ki = gains.ki;
    pid.kd = gains.kd;
}

// ============================================================================
// CHECK IF SENSOR SEES CENTER (0.9+ BLACK)
// ============================================================================
static bool sensor_sees_center(float raw_error_magnitude) {
    // Center = error magnitude >= 0.9 (very dark/middle of line)
    return (raw_error_magnitude >= 0.9f);
}

// ============================================================================
// INITIALIZATION
// ============================================================================
void line_following_init(void) {
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
    pid.output = 0.0f;
    pid.initialized = false;
    current_state = LINE_FOLLOW_CENTERED;
    normalized_error = 0.0f;
    filtered_error = 0.0f;
    L_power = 0.0f;
    R_power = 0.0f;
    line_lost_start = 0;
    last_debug_time = 0;
    side_inverted = false;
    prev_swap_state = false;
    turning_to_center = false;
    turn_start_time = 0;
    intersection_count = 0;
    turn_left = true;
    
    gpio_init(SWAP_SENSOR_PIN);
    gpio_set_dir(SWAP_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(SWAP_SENSOR_PIN);
    
    printf("Single IR Line Following Initialized\n");
    printf("  PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pid.kp, pid.ki, pid.kd);
    printf("  Base Power: %.1f, Slow Turn: %.1f\n", BASE_POWER, SLOW_TURN_SPEED);
    printf("  Center detection: error >= 0.9\n");
    printf("  Side swap sensor: GP%d\n", SWAP_SENSOR_PIN);
}

// ============================================================================
// CHECK SIDE SWAP SENSOR
// ============================================================================
static void check_side_swap(uint32_t current_time) {
    bool swap_sensor = !gpio_get(SWAP_SENSOR_PIN);
    
    if (swap_sensor && !prev_swap_state) {
        side_inverted = !side_inverted;
        turning_to_center = true;
        turn_start_time = current_time;
        
        intersection_count++;
        turn_left = (intersection_count % 2 == 0);
        
        printf("\n>>> SIDE SWAP #%d! Turning %s slowly until CENTER (0.9+) <<<\n\n", 
               intersection_count, turn_left ? "LEFT" : "RIGHT");
        
        pid.integral = 0.0f;
        pid.prev_error = 0.0f;
    }
    
    prev_swap_state = swap_sensor;
}

// ============================================================================
// NORMALIZE IR READING TO ERROR SIGNAL
// ============================================================================
static float normalize_ir_to_error(uint16_t ir_reading) {
    uint16_t white = ir_get_white_value();
    uint16_t black = ir_get_black_value();
    uint16_t threshold = ir_get_threshold();
    
    float error;
    
    if (ir_reading < threshold) {
        if (threshold > white) {
            error = -1.0f * (float)(threshold - ir_reading) / (float)(threshold - white);
        } else {
            error = -1.0f;
        }
    } else {
        if (black > threshold) {
            error = +1.0f * (float)(ir_reading - threshold) / (float)(black - threshold);
        } else {
            error = +1.0f;
        }
    }
    
    if (error < -1.0f) error = -1.0f;
    if (error > +1.0f) error = +1.0f;
    
    if (side_inverted) {
        error = -error;
    }
    
    return error;
}

// ============================================================================
// UPDATE LINE STATE
// ============================================================================
static void update_line_state(float error) {
    const float CENTERED_THRESHOLD = 0.15f;
    const float FAR_THRESHOLD = 0.6f;
    
    if (fabsf(error) < CENTERED_THRESHOLD) {
        current_state = LINE_FOLLOW_CENTERED;
    } else if (error < -FAR_THRESHOLD) {
        current_state = LINE_FOLLOW_FAR_LEFT;
    } else if (error > FAR_THRESHOLD) {
        current_state = LINE_FOLLOW_FAR_RIGHT;
    } else if (error < 0) {
        current_state = LINE_FOLLOW_LEFT;
    } else {
        current_state = LINE_FOLLOW_RIGHT;
    }
}

// ============================================================================
// PID UPDATE
// ============================================================================
float line_following_update(float dt) {
    uint16_t ir_reading = ir_read_line_sensor();
    float error = normalize_ir_to_error(ir_reading);
    
    filtered_error = ERROR_FILTER_ALPHA * filtered_error + 
                     (1.0f - ERROR_FILTER_ALPHA) * error;
    
    normalized_error = filtered_error;
    update_line_state(normalized_error);
    
    if (!pid.initialized) {
        pid.prev_error = normalized_error;
        pid.initialized = true;
        return 0.0f;
    }
    
    float p_term = pid.kp * normalized_error;
    
    pid.integral += normalized_error * dt;
    const float INTEGRAL_MAX = 10.0f;
    if (pid.integral > INTEGRAL_MAX) pid.integral = INTEGRAL_MAX;
    if (pid.integral < -INTEGRAL_MAX) pid.integral = -INTEGRAL_MAX;
    float i_term = pid.ki * pid.integral;
    
    float derivative = (normalized_error - pid.prev_error) / dt;
    float d_term = pid.kd * derivative;
    
    pid.output = p_term + i_term + d_term;
    
    float output_max;
    float error_magnitude = fabsf(normalized_error);

    if (error_magnitude > 0.98f) {
        output_max = 20.0f;
    } else if (error_magnitude > 0.7f) {
        output_max = 3.0f;
    } else {
        output_max = 2.5f;
    }

    if (pid.output > output_max) pid.output = output_max;
    if (pid.output < -output_max) pid.output = -output_max;
    
    pid.prev_error = normalized_error;
    
    return pid.output;
}

// ============================================================================
// COMPLETE LINE FOLLOWING UPDATE WITH MOTOR CONTROL
// ============================================================================
bool line_following_control_update(uint32_t current_time, float dt) {
    check_side_swap(current_time);
    
    uint16_t ir_reading = ir_read_line_sensor();
    uint16_t threshold = ir_get_threshold();
    
    bool on_white_surface = (ir_reading < threshold);
    bool on_black_surface = (ir_reading >= threshold);
    
    float raw_error = normalize_ir_to_error(ir_reading);
    float raw_error_magnitude = fabsf(raw_error);
    
    float steering = line_following_update(dt);
    float error = normalized_error;
    
    apply_adaptive_gains(fabsf(error));
    
    // ========================================================================
    // SLOW TURN TO CENTER MODE
    // ========================================================================
    if (turning_to_center) {
        uint32_t elapsed = current_time - turn_start_time;
        
        // Check if center detected (0.9+ black)
        if (sensor_sees_center(raw_error_magnitude)) {
            turning_to_center = false;
            printf("\n>>> CENTER DETECTED (error: %.2f) - RESUMING TRACKING <<<\n\n", raw_error);
            
        } else if (elapsed > MAX_TURN_TIME_MS) {
            turning_to_center = false;
            printf("\n>>> TURN TIMEOUT - RESUMING TRACKING <<<\n\n");
            
        } else {
            // Continue slow turn
            if (turn_left) {
                L_power = 0.0f;
                R_power = SLOW_TURN_SPEED + 15.0f;
                printf("turn to center:right");
            } else {
                L_power = SLOW_TURN_SPEED + 15.0f;
                R_power = 0.0f;
                printf("turn to center:left");
            }
            
            int left_motor = apply_deadband(L_power);
            int right_motor = apply_deadband(R_power);
            
            motor_drive(M1A, M1B, -left_motor);
            motor_drive(M2A, M2B, -right_motor);
            
            if (current_time - last_debug_time >= 200) {
                printf("[TURNING %s] IR:%d | Error: %.2f | L:%d R:%d | Looking for center...\n",
                       turn_left ? "LEFT" : "RIGHT", ir_reading, raw_error, left_motor, right_motor);
                last_debug_time = current_time;
            }
            
            return true;
        }
    }
    
    // ========================================================================
    // NORMAL LINE FOLLOWING MODE
    // ========================================================================
    if (raw_error_magnitude >= 0.9f) {
        if (on_white_surface) {
            if (side_inverted) {
                L_power = 0;
                R_power = MAX_POWER - 10.0f;
            } else {
                L_power = MAX_POWER - 10.0f;
                R_power = 0;
            }
        } else {
            if (side_inverted) {
                L_power = BASE_POWER - steering - 10.0f;
                R_power = BASE_POWER + steering;
            } else {
                L_power = BASE_POWER - steering;
                R_power = BASE_POWER + steering + 10.0f;
            }
        }
    } else {
        L_power = BASE_POWER - steering;
        R_power = BASE_POWER + steering;
    }
    
    int left_motor = apply_deadband(L_power);
    int right_motor = apply_deadband(R_power);
    
    motor_drive(M1A, M1B, -left_motor);
    motor_drive(M2A, M2B, -right_motor);
    
    if (current_time - last_debug_time >= 500) {
        const char* side_str = side_inverted ? "[INV]" : "[NOR]";
        const char* surface = on_white_surface ? "WHITE" : "BLACK";
        
        if (raw_error_magnitude > 0.88f) {
            printf("[PIVOT]%s %s | Raw: %.2f | Filtered: %.2f | L:%d R:%d | %s\n",
                   side_str, surface, raw_error, error, left_motor, right_motor,
                   line_state_to_string(current_state));
        } else {
            printf("[LINE]%s Raw: %.2f | Filtered: %.2f | Steering: %.2f | L:%d R:%d | %s\n",
                   side_str, raw_error, error, steering, left_motor, right_motor,
                   line_state_to_string(current_state));
        }
        last_debug_time = current_time;
    }
    
    if (current_state == LINE_FOLLOW_LOST) {
        if (line_lost_start == 0) {
            line_lost_start = current_time;
        } else if (current_time - line_lost_start > LINE_LOST_TIMEOUT_MS) {
            return false;
        }
    } else {
        line_lost_start = 0;
    }
    
    return true;
}

// ============================================================================
// RESET INTEGRAL
// ============================================================================
void line_following_reset_integral(void) {
    pid.integral = 0.0f;
}

// ============================================================================
// GAIN SETTERS
// ============================================================================
void line_following_set_kp(float kp) {
    pid.kp = kp;
}

void line_following_set_ki(float ki) {
    pid.ki = ki;
}

void line_following_set_kd(float kd) {
    pid.kd = kd;
}

// ============================================================================
// GETTERS
// ============================================================================
LineFollowState line_following_get_state(void) {
    return current_state;
}

float line_following_get_filtered_pos(void) {
    return normalized_error;
}

float line_following_get_error(void) {
    return normalized_error;
}

float line_following_get_output(void) {
    return pid.output;
}

float line_following_get_left_power(void) {
    return L_power;
}

float line_following_get_right_power(void) {
    return R_power;
}

bool line_following_is_side_inverted(void) {
    return side_inverted;
}

const char* line_state_to_string(LineFollowState state) {
    switch (state) {
        case LINE_FOLLOW_CENTERED:    return "CENTERED";
        case LINE_FOLLOW_LEFT:        return "LEFT";
        case LINE_FOLLOW_RIGHT:       return "RIGHT";
        case LINE_FOLLOW_FAR_LEFT:    return "FAR_LEFT";
        case LINE_FOLLOW_FAR_RIGHT:   return "FAR_RIGHT";
        case LINE_FOLLOW_LOST:        return "LOST";
        default:                      return "UNKNOWN";
    }
}
