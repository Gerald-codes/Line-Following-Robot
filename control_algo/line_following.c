/**
 * line_following_single_ir.c - Single IR Sensor Line Following
 * Uses ONE IR sensor and ONE PID controller
 * IR reading is used directly as error signal
 */

#include "line_following.h"
#include "ir_sensor.h"
#include "config.h"
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
    
    printf("Single IR Line Following Initialized\n");
    printf("  PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pid.kp, pid.ki, pid.kd);
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
    const float OUTPUT_MAX = 3.0f; // was 2
    if (pid.output > OUTPUT_MAX) pid.output = OUTPUT_MAX;
    if (pid.output < -OUTPUT_MAX) pid.output = -OUTPUT_MAX;
    
    // Update previous error
    pid.prev_error = normalized_error;
    
    return pid.output;
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