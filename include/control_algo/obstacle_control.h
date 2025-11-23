/**
 * @file    obstacle_control.h
 * @brief   High-level obstacle detection and handling interface
 * @details Provides functions for checking obstacles and handling various
 *          obstacle-related states in the robot control system
 */

#ifndef OBSTACLE_CONTROL_H
#define OBSTACLE_CONTROL_H

#include <stdbool.h>
#include "state_machine.h"

/**
 * @brief Check for obstacles in front of robot
 * @return true if obstacle detected
 */
bool check_for_obstacles(void);

/**
 * @brief Handle obstacle detected state
 */
void handle_obstacle_detected(void);

/**
 * @brief Handle obstacle scanning state
 */
void handle_obstacle_scanning(void);

/**
 * @brief Handle obstacle avoidance state
 */
void handle_obstacle_avoidance(void);

#endif /* OBSTACLE_CONTROL_H */