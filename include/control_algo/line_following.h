/**
 * @file    line_following.h
 * @brief   Line following module for single IR sensor control
 * @details Interface for initialization, update, and state report functions
 *          for embedded line-following robots with adaptive PID and
 *          recovery logic
 */

#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#include <stdbool.h>
#include <stdint.h>

/* LINE FOLLOWING STATES */

typedef enum
{
    LINE_FOLLOW_CENTERED,
    LINE_FOLLOW_LEFT,
    LINE_FOLLOW_RIGHT,
    LINE_FOLLOW_FAR_LEFT,
    LINE_FOLLOW_FAR_RIGHT,
    LINE_FOLLOW_LOST
} LineFollowState;

/* INITIALIZATION */

/**
 * @brief Initialize the line following module state
 */
void line_following_init(void);

/* UPDATE FUNCTIONS */

/**
 * @brief Basic PID update and returns steering correction
 * @param dt Time delta in seconds
 * @return Steering correction value
 */
float line_following_update(float dt);

/**
 * @brief Complete line following control update with motor control
 * @param current_time Current time in milliseconds
 * @param dt Time delta in seconds
 * @return true if line is being followed, false if line is lost
 */
bool line_following_control_update(uint32_t current_time, float dt);

/* CONTROL FUNCTIONS */

/**
 * @brief Reset the integral term of the PID controller
 */
void line_following_reset_integral(void);

/* GAIN SETTERS */

/**
 * @brief Set PID proportional gain
 * @param kp New Kp value
 */
void line_following_set_kp(float kp);

/**
 * @brief Set PID integral gain
 * @param ki New Ki value
 */
void line_following_set_ki(float ki);

/**
 * @brief Set PID derivative gain
 * @param kd New Kd value
 */
void line_following_set_kd(float kd);

/**
 * @brief Get the current logical state for line following
 * @return Current LineFollowState
 */
LineFollowState line_following_get_state(void);

/**
 * @brief Get the filtered sensor position
 * @return Filtered position value
 */
float line_following_get_filtered_pos(void);

/**
 * @brief Get the current tracked error
 * @return Normalized error
 */
float line_following_get_error(void);

/**
 * @brief Get the latest PID output
 * @return PID output
 */
float line_following_get_output(void);

/**
 * @brief Get last left motor power value
 * @return Left power
 */
float line_following_get_left_power(void);

/**
 * @brief Get last right motor power value
 * @return Right power
 */
float line_following_get_right_power(void);

/**
 * @brief Convert state enum to a human-readable string
 * @param state LineFollowState value
 * @return String representation of state
 */
const char* line_state_to_string(LineFollowState state);

/**
 * @brief Check if line side following is inverted
 * @return true if inverted
 */
bool line_following_is_side_inverted(void);

#endif /* LINE_FOLLOWING_H */
