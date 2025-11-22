/**
 * line_following.h - Single IR Sensor Line Following Header
 */

#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// LINE FOLLOWING STATES
// ============================================================================
typedef enum {
    LINE_FOLLOW_CENTERED,
    LINE_FOLLOW_LEFT,
    LINE_FOLLOW_RIGHT,
    LINE_FOLLOW_FAR_LEFT,
    LINE_FOLLOW_FAR_RIGHT,
    LINE_FOLLOW_LOST
} LineFollowState;

// ============================================================================
// INITIALIZATION
// ============================================================================
void line_following_init(void);

// ============================================================================
// UPDATE FUNCTIONS
// ============================================================================
/**
 * Basic PID update - returns steering correction
 * @param dt: Time delta in seconds
 * @return: Steering correction value
 */
float line_following_update(float dt);

/**
 * Complete line following control update with motor control
 * Handles PID, adaptive gains, motor power, and line lost detection
 * @param current_time: Current time in milliseconds
 * @param dt: Time delta in seconds
 * @return: true if line is being followed, false if line is lost
 */
bool line_following_control_update(uint32_t current_time, float dt);

// ============================================================================
// CONTROL FUNCTIONS
// ============================================================================
void line_following_reset_integral(void);

// ============================================================================
// GAIN SETTERS
// ============================================================================
void line_following_set_kp(float kp);
void line_following_set_ki(float ki);
void line_following_set_kd(float kd);

// ============================================================================
// GETTERS
// ============================================================================
LineFollowState line_following_get_state(void);
float line_following_get_filtered_pos(void);
float line_following_get_error(void);
float line_following_get_output(void);
float line_following_get_left_power(void);
float line_following_get_right_power(void);

// ============================================================================
// UTILITIES
// ============================================================================
const char* line_state_to_string(LineFollowState state);

#endif // LINE_FOLLOWING_H