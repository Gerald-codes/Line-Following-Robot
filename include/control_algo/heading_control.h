/**
 * @file    heading_control.h
 * @brief   Heading/orientation control using IMU and PID
 * @details Provides heading control with PID controller and deadband feature
 *          for stable orientation maintenance during line following and
 *          obstacle avoidance maneuvers
 */

#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include <stdbool.h>
#include "pico/stdlib.h"
#include "pid.h"

typedef struct
{
    PIDController pid;
    float target_heading;
    float current_heading;
    float correction;
    bool at_target;
    float deadband;             /* Deadband in degrees (no correction if error < deadband) */
    uint32_t last_update_time;
} HeadingController;

/**
 * @brief Initialize heading controller
 * @param ctrl Pointer to HeadingController structure
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param max_correction Maximum correction output
 */
void heading_control_init(HeadingController *ctrl, float Kp, float Ki, float Kd, float max_correction);

/**
 * @brief Set target heading
 * @param ctrl Pointer to HeadingController
 * @param heading Target heading in degrees
 */
void heading_control_set_target(HeadingController *ctrl, float heading);

/**
 * @brief Set heading deadband
 * @param ctrl Pointer to HeadingController
 * @param deadband_degrees Deadband threshold in degrees
 */
void heading_control_set_deadband(HeadingController *ctrl, float deadband_degrees);

/**
 * @brief Update heading control
 * @param ctrl Pointer to HeadingController
 * @param current_heading Current heading from IMU in degrees
 * @return Correction value to apply to motors
 */
float heading_control_update(HeadingController *ctrl, float current_heading);

/**
 * @brief Check if heading is at target
 * @param ctrl Pointer to HeadingController
 * @param tolerance Tolerance in degrees
 * @return true if within tolerance
 */
bool heading_is_at_target(HeadingController *ctrl, float tolerance);

/**
 * @brief Reset heading controller state
 * @param ctrl Pointer to HeadingController
 */
void heading_control_reset(HeadingController *ctrl);

/**
 * @brief Get current heading error
 * @param ctrl Pointer to HeadingController
 * @return Heading error in degrees
 */
float heading_get_error(HeadingController *ctrl);

/**
 * @brief Normalize angle to [-180, 180] range
 * @param angle Angle in degrees
 * @return Normalized angle
 */
float normalize_angle(float angle);

/**
 * @brief Print heading control status for debugging
 * @param ctrl Pointer to HeadingController
 */
void heading_control_print_status(HeadingController *ctrl);

#endif /* HEADING_CONTROL_H */
