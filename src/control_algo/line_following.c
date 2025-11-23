/**
 * @file    line_following.c
 * @brief   Single IR sensor line following controller
 * @details Implements robust line tracking with adaptive PID, automatic line
 *          recovery using biased oscillation search, side swap support, and
 *          re-centering logic for embedded robots
 */

/* Includes */
#include "line_following.h"
#include "ir_sensor.h"
#include "motor.h"
#include "config.h"
#include "pin_definitions.h"
#include "barcode_control.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

/* Side swap control */
#define SWAP_SENSOR_PIN         6

static bool side_inverted = false;
static bool prev_swap_state = false;

/* Line recovery state */
typedef enum
{
    RECOVERY_NONE,
    RECOVERY_SEARCHING,
    RECOVERY_RECENTERING
} RecoveryState;

static RecoveryState recovery_state = RECOVERY_NONE;
static uint32_t recovery_start_time = 0;
static uint32_t last_cycle_change = 0;
static uint32_t recenter_start_time = 0;
static int search_cycle = 0;
static bool turning_right = true;
static float last_steering_before_loss = 0.0f;

#define RECOVERY_MAX_TIME_MS    10000
#define MAX_SEARCH_CYCLES       10
#define RECENTER_DURATION_MS    3000

/* PID controller structure */
typedef struct
{
    float kp;               /* Proportional gain */
    float ki;               /* Integral gain */
    float kd;               /* Derivative gain */
    float integral;         /* Accumulated error */
    float prev_error;       /* Last error for derivative */
    float output;           /* Control output */
    bool initialized;       /* Initialization flag */
} PIDController;

static PIDController pid =
{
    .kp = LINE_PID_KP,
    .ki = LINE_PID_KI,
    .kd = LINE_PID_KD,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .output = 0.0f,
    .initialized = false
};

/* Line following state */
static LineFollowState current_state = LINE_FOLLOW_CENTERED;
static float normalized_error = 0.0f;
static float filtered_error = 0.0f;

#define ERROR_FILTER_ALPHA      0.7f

/* Motor control state */
static float L_power = 0.0f;
static float R_power = 0.0f;
static uint32_t line_lost_start = 0;

#define LINE_LOST_TIMEOUT_MS    800
#define BASE_POWER              40.0f
#define MIN_POWER               25.0f
#define MAX_POWER               60.0f

/* Adaptive PID gain sets */
typedef struct
{
    float kp;
    float ki;
    float kd;
} PIDGainSet;

static const PIDGainSet SMOOTH_GAINS =
{
    .kp = 1.5f,
    .ki = 0.0f,
    .kd = 0.8f
};

static const PIDGainSet AGGRESSIVE_GAINS =
{
    .kp = 3.0f,
    .ki = 0.0f,
    .kd = 2.5f
};

#define SMOOTH_ERROR_THRESHOLD      0.2f
#define AGGRESSIVE_ERROR_THRESHOLD  0.6f

static uint32_t last_debug_time = 0;

/**
 * @brief Apply deadband and saturate motor power
 * @param power Float motor power value
 * @return Integer value for motor driver
 */
static int apply_deadband(float power)
{
    int int_power = (int)power;
    
    if (int_power > 0 && int_power < MIN_POWER)
    {
        return MIN_POWER;
    }
    else if (int_power < 0 && int_power > -MIN_POWER)
    {
        return -MIN_POWER;
    }
    
    if (int_power > MAX_POWER)
    {
        return MAX_POWER;
    }
    if (int_power < -MAX_POWER)
    {
        return -MAX_POWER;
    }
    
    return int_power;
}

/**
 * @brief Adjust PID gains based on error magnitude using blending
 * @param error_magnitude Absolute error value
 */
static void apply_adaptive_gains(float error_magnitude)
{
    PIDGainSet gains;
    
    if (error_magnitude < SMOOTH_ERROR_THRESHOLD)
    {
        gains = SMOOTH_GAINS;
    }
    else if (error_magnitude > AGGRESSIVE_ERROR_THRESHOLD)
    {
        gains = AGGRESSIVE_GAINS;
    }
    else
    {
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

/**
 * @brief Initialize line following control and reset all state variables
 */
void line_following_init(void)
{
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
    
    side_inverted = true;
    prev_swap_state = false;
    
    recovery_state = RECOVERY_NONE;
    recovery_start_time = 0;
    last_cycle_change = 0;
    recenter_start_time = 0;
    search_cycle = 0;
    turning_right = true;
    last_steering_before_loss = 0.0f;
    
    gpio_init(SWAP_SENSOR_PIN);
    gpio_set_dir(SWAP_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(SWAP_SENSOR_PIN);
    
    printf("Single IR Line Following Initialized\n");
    printf(" PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pid.kp, pid.ki, pid.kd);
    printf(" Base Power: %.1f, Min: %.1f, Max: %.1f\n", BASE_POWER, MIN_POWER, MAX_POWER);
    printf(" Side swap sensor: GP%d\n", SWAP_SENSOR_PIN);
    printf(" Recovery: BIASED oscillation + RECENTERING enabled\n");
}

/**
 * @brief Check swap sensor and toggle line following side
 */
static void check_side_swap(void)
{
    bool swap_sensor = !gpio_get(SWAP_SENSOR_PIN);
    
    if (swap_sensor && !prev_swap_state)
    {
        side_inverted = !side_inverted;
        printf("\n>>> SIDE SWAP! Now tracing %s side <<<\n\n",
               side_inverted ? "INVERTED" : "NORMAL");
        pid.integral = 0.0f;
        pid.prev_error = 0.0f;
    }
    
    prev_swap_state = swap_sensor;
}

/**
 * @brief Convert sensor value to normalized error for PID input
 * @param ir_reading Raw IR sensor value
 * @return Error in range [-1, 1], sign corrected based on side
 */
static float normalize_ir_to_error(uint16_t ir_reading)
{
    uint16_t white = ir_get_white_value();
    uint16_t black = ir_get_black_value();
    uint16_t threshold = ir_get_threshold();
    float error;
    
    if (ir_reading < threshold)
    {
        if (threshold > white)
        {
            error = -1.0f * (float)(threshold - ir_reading) / (float)(threshold - white);
        }
        else
        {
            error = -1.0f;
        }
    }
    else
    {
        if (black > threshold)
        {
            error = +1.0f * (float)(ir_reading - threshold) / (float)(black - threshold);
        }
        else
        {
            error = +1.0f;
        }
    }
    
    if (error < -1.0f)
    {
        error = -1.0f;
    }
    if (error > +1.0f)
    {
        error = +1.0f;
    }
    
    if (side_inverted)
    {
        error = -error;
    }
    
    return error;
}

/**
 * @brief Update logical state of robot's position relative to line
 * @param error Normalized error value
 */
static void update_line_state(float error)
{
    const float CENTERED_THRESHOLD = 0.15f;
    const float FAR_THRESHOLD = 0.6f;
    
    if (fabsf(error) < CENTERED_THRESHOLD)
    {
        current_state = LINE_FOLLOW_CENTERED;
    }
    else if (error < -FAR_THRESHOLD)
    {
        current_state = LINE_FOLLOW_FAR_LEFT;
    }
    else if (error > FAR_THRESHOLD)
    {
        current_state = LINE_FOLLOW_FAR_RIGHT;
    }
    else if (error < 0)
    {
        current_state = LINE_FOLLOW_LEFT;
    }
    else
    {
        current_state = LINE_FOLLOW_RIGHT;
    }
}

/**
 * @brief PID update for steering correction
 * @param dt Time delta in seconds
 * @return PID output as steering value
 */
float line_following_update(float dt)
{
    uint16_t ir_reading = ir_read_line_sensor();
    float error = normalize_ir_to_error(ir_reading);
    
    filtered_error = ERROR_FILTER_ALPHA * filtered_error +
                     (1.0f - ERROR_FILTER_ALPHA) * error;
    normalized_error = filtered_error;
    
    update_line_state(normalized_error);
    
    if (!pid.initialized)
    {
        pid.prev_error = normalized_error;
        pid.initialized = true;
        return 0.0f;
    }
    
    float p_term = pid.kp * normalized_error;
    
    pid.integral += normalized_error * dt;
    const float INTEGRAL_MAX = 10.0f;
    if (pid.integral > INTEGRAL_MAX)
    {
        pid.integral = INTEGRAL_MAX;
    }
    if (pid.integral < -INTEGRAL_MAX)
    {
        pid.integral = -INTEGRAL_MAX;
    }
    
    float i_term = pid.ki * pid.integral;
    float derivative = (normalized_error - pid.prev_error) / dt;
    float d_term = pid.kd * derivative;
    
    pid.output = p_term + i_term + d_term;
    
    float output_max;
    float error_magnitude = fabsf(normalized_error);
    
    if (error_magnitude > 0.98f)
    {
        output_max = 20.0f;
    }
    else if (error_magnitude > 0.7f)
    {
        output_max = 3.0f;
    }
    else
    {
        output_max = 2.5f;
    }
    
    if (pid.output > output_max)
    {
        pid.output = output_max;
    }
    if (pid.output < -output_max)
    {
        pid.output = -output_max;
    }
    
    pid.prev_error = normalized_error;
    
    return pid.output;
}

/**
 * @brief Attempt to recover line by increasing oscillation if lost
 * @param current_time System time in milliseconds
 * @return true if still searching or recovered, false if failed
 */
static bool attempt_line_recovery(uint32_t current_time)
{
    uint16_t ir_reading = ir_read_line_sensor();
    float current_error = fabsf(normalize_ir_to_error(ir_reading));
    
    if (current_error < 0.8f)
    {
        printf("[RECOVERY] ✓ Line found after %d cycles!\n", search_cycle);
        recovery_state = RECOVERY_RECENTERING;
        recenter_start_time = current_time;
        search_cycle = 0;
        last_cycle_change = 0;
        return true;
    }
    
    if (current_time - recovery_start_time > RECOVERY_MAX_TIME_MS ||
        search_cycle >= MAX_SEARCH_CYCLES)
    {
        printf("[RECOVERY] ✗ Recovery failed\n");
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        return false;
    }
    
    uint32_t cycle_duration = 800 + (search_cycle * 200);
    if (cycle_duration > 2000)
    {
        cycle_duration = 2000;
    }
    
    if (last_cycle_change == 0)
    {
        last_cycle_change = current_time;
        search_cycle = 0;
    }
    
    if (current_time - last_cycle_change >= cycle_duration)
    {
        if (search_cycle > 0)
        {
            turning_right = !turning_right;
        }
        
        search_cycle++;
        last_cycle_change = current_time;
        printf("[RECOVERY] Cycle %d - PIVOT %s (duration: %dms)\n",
               search_cycle, turning_right ? "RIGHT" : "LEFT", cycle_duration);
    }
    
    int base_speed = 25;
    int pivot_speed = base_speed + (search_cycle * 4);
    if (pivot_speed > 45)
    {
        pivot_speed = 45;
    }
    
    if (search_cycle == 0)
    {
        pivot_speed += 10;
        if (pivot_speed > 45)
        {
            pivot_speed = 45;
        }
    }
    
    if (turning_right)
    {
        motor_drive(M1A, M1B, -pivot_speed);
        motor_drive(M2A, M2B, +pivot_speed);
    }
    else
    {
        motor_drive(M1A, M1B, +pivot_speed);
        motor_drive(M2A, M2B, -pivot_speed);
    }
    
    return true;
}

/**
 * @brief Re-center robot after line is found with edge detection
 * @param current_time Current system time in milliseconds
 * @param dt Time difference in seconds
 * @return true if centered or timeout, false otherwise
 */
static bool attempt_recentering(uint32_t current_time, float dt)
{
    uint16_t ir_reading = ir_read_line_sensor();
    uint16_t threshold = ir_get_threshold();
    bool on_black = (ir_reading >= threshold);
    
    static bool edge_found = false;
    static bool started_on_black = false;
    static bool centering_phase = false;
    static uint32_t centering_start = 0;
    
    /* Initialization on first call */
    if (current_time - recenter_start_time < 50)
    {
        started_on_black = on_black;
        edge_found = false;
        centering_phase = false;
        printf("[RECENTER] Starting - sensor on %s\n", on_black ? "BLACK" : "WHITE");
    }
    
    if (current_time - recenter_start_time > RECENTER_DURATION_MS)
    {
        printf("[RECENTER] ✓ Timeout - resuming normal tracking\n");
        recovery_state = RECOVERY_NONE;
        current_state = LINE_FOLLOW_CENTERED;
        edge_found = false;
        centering_phase = false;
        return true;
    }
    
    /* Phase 1: Find edge (black→white) */
    if (!edge_found && started_on_black && !on_black)
    {
        printf("[RECENTER] ✓ Edge found (black→white)! Starting centering...\n");
        edge_found = true;
        centering_phase = true;
        centering_start = current_time;
        return true;
    }
    
    /* Phase 2: Move to center (white→black, error
