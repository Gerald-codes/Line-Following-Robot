/**
 * line_following.c - Single IR Sensor Line Following
 * Uses ONE IR sensor and ONE PID controller
 * IR reading is used directly as error signal
 * 
 * Updated with complete line following control logic
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


// Filter for smoothing
#define ERROR_FILTER_ALPHA 0.7f


// ============================================================================
// LINE FOLLOWING CONTROL STATE
// ============================================================================
// Motor power tracking
static float L_power = 0.0f;
static float R_power = 0.0f;


// Line lost detection
static uint32_t line_lost_start = 0;
#define LINE_LOST_TIMEOUT_MS 2000


// Base speeds and limits
#define BASE_POWER 40.0f
#define MIN_POWER 25.0f
#define MAX_POWER 60.0f


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


// Debug timing
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
        // Linear interpolation
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
    
    printf("Single IR Line Following Initialized\n");
    printf("  PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pid.kp, pid.ki, pid.kd);
    printf("  Base Power: %.1f, Min: %.1f, Max: %.1f\n", BASE_POWER, MIN_POWER, MAX_POWER);
}


// ============================================================================
// NORMALIZE IR READING TO ERROR SIGNAL
// ============================================================================
/**
 * Convert raw IR reading to normalized error
 * 
 * IR Reading → Error Signal:
 * - WHITE (< threshold): -1.0 (sensor over white, line is to the right)
 * - BLACK (> threshold): +1.0 (sensor over black/line, need to go left)
 * 
 * The normalization maps:
 *   white_value → -1.0
 *   threshold   →  0.0
 *   black_value → +1.0
 */
static float normalize_ir_to_error(uint16_t ir_reading) {
    uint16_t white = ir_get_white_value();
    uint16_t black = ir_get_black_value();
    uint16_t threshold = ir_get_threshold();
    
    float error;
    
    if (ir_reading < threshold) {
        // On white side - normalize from white to threshold
        if (threshold > white) {
            error = -1.0f * (float)(threshold - ir_reading) / (float)(threshold - white);
        } else {
            error = -1.0f;
        }
    } else {
        // On black side - normalize from threshold to black
        if (black > threshold) {
            error = +1.0f * (float)(ir_reading - threshold) / (float)(black - threshold);
        } else {
            error = +1.0f;
        }
    }
    
    // Clamp to [-1, +1]
    if (error < -1.0f) error = -1.0f;
    if (error > +1.0f) error = +1.0f;
    
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
    // Read IR sensor
    uint16_t ir_reading = ir_read_line_sensor();
    
    // Convert to normalized error
    float error = normalize_ir_to_error(ir_reading);
    
    // Apply exponential filter for smoothing
    filtered_error = ERROR_FILTER_ALPHA * filtered_error + 
                     (1.0f - ERROR_FILTER_ALPHA) * error;
    
    normalized_error = filtered_error;
    
    // Update state
    update_line_state(normalized_error);
    
    // Initialize previous error on first run
    if (!pid.initialized) {
        pid.prev_error = normalized_error;
        pid.initialized = true;
        return 0.0f;
    }
    
    // PID Calculation
    // P term
    float p_term = pid.kp * normalized_error;
    
    // I term with anti-windup
    pid.integral += normalized_error * dt;
    const float INTEGRAL_MAX = 10.0f;
    if (pid.integral > INTEGRAL_MAX) pid.integral = INTEGRAL_MAX;
    if (pid.integral < -INTEGRAL_MAX) pid.integral = -INTEGRAL_MAX;
    float i_term = pid.ki * pid.integral;
    
    // D term
    float derivative = (normalized_error - pid.prev_error) / dt;
    float d_term = pid.kd * derivative;
    
    // Total output
    pid.output = p_term + i_term + d_term;
    
    // Clamp output
    // Adaptive output limit based on error magnitude
    float output_max;
    float error_magnitude = fabsf(normalized_error);


    if (error_magnitude > 0.98f) {
        // Far from line (sharp curves) - allow maximum steering
        output_max = 20.0f;
    } else if (error_magnitude > 0.7f) {
        // Moderate error (gentle curves) - medium steering
        output_max = 3.0f;
    } else {
        // Small error (straight line) - limited steering for stability
        output_max = 2.5f;
    }


    if (pid.output > output_max) pid.output = output_max;
    if (pid.output < -output_max) pid.output = -output_max;
    
    // Update previous error
    pid.prev_error = normalized_error;
    
    return pid.output;
}


// ============================================================================
// COMPLETE LINE FOLLOWING UPDATE WITH MOTOR CONTROL
// ============================================================================
/**
 * Main line following update function that handles:
 * - PID calculation
 * - Adaptive gain adjustment
 * - Motor power calculation and application
 * - Line lost detection
 * - Debug output
 * 
 * Returns: true if line is being followed, false if line is lost
 */
bool line_following_control_update(uint32_t current_time, float dt) {
    // Get RAW error before filtering for extreme detection
    uint16_t ir_reading = ir_read_line_sensor();
    float raw_error = normalize_ir_to_error(ir_reading);
    float raw_error_magnitude = fabsf(raw_error);
    
    // Update PID with filtered error
    float steering = line_following_update(dt);
    float error = normalized_error;  // This is filtered
    
    apply_adaptive_gains(fabsf(error));
    
    // Use RAW error for extreme detection - PIVOT TURN for complete white/black
    if (raw_error_magnitude > 0.99f) {
        // EXTREME: Use pivot turn based on RAW error direction
        if (raw_error < 0) {
            // On white surface, need to turn RIGHT
            L_power = MAX_POWER - 10.0f;  // Stop left wheel completely
            R_power = 0;
        }
        else
        {
            L_power = BASE_POWER - steering;
            R_power = BASE_POWER + steering + 10.0f;
        }
    } else {
        // NORMAL: Differential steering
        L_power = BASE_POWER - steering;
        R_power = BASE_POWER + steering;
    }
    
    // Apply deadband and limits
    int left_motor = apply_deadband(L_power);
    int right_motor = apply_deadband(R_power);
    
    // Drive motors
    motor_drive(M1A, M1B, -left_motor);
    motor_drive(M2A, M2B, -right_motor);
    
    // Debug output every 500ms
    if (current_time - last_debug_time >= 500) {
        if (raw_error_magnitude > 0.98f) {
            printf("[PIVOT] Raw: %.2f | Filtered: %.2f | L:%d R:%d | State: %s\n",
                   raw_error, error, left_motor, right_motor,
                   line_state_to_string(current_state));
        } else {
            printf("[LINE] Raw: %.2f | Filtered: %.2f | Steering: %.2f | L:%d R:%d | State: %s\n",
                   raw_error, error, steering, left_motor, right_motor,
                   line_state_to_string(current_state));
        }
        last_debug_time = current_time;
    }
    
    // Check if line is lost
    if (current_state == LINE_FOLLOW_LOST) {
        if (line_lost_start == 0) {
            line_lost_start = current_time;
        } else if (current_time - line_lost_start > LINE_LOST_TIMEOUT_MS) {
            return false;  // Line lost timeout
        }
    } else {
        line_lost_start = 0;  // Reset if line is found
    }
    
    return true;  // Line following active
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
