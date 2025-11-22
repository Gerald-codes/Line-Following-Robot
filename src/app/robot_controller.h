/**
 * robot_controller.h
 * 
 * High-level robot behavior coordination
 * Implements state handlers and system orchestration
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "state_machine.h"

// Initialize robot controller
void robot_controller_init(void);

// Main control loop - call this in main loop
void robot_controller_update(float dt);

// State-specific handlers
void robot_handle_line_following(float dt);
void robot_handle_obstacle_detection(void);
void robot_handle_obstacle_scanning(void);
void robot_handle_obstacle_avoiding(void);
void robot_handle_returning_to_line(void);
void robot_handle_line_lost(void);
void robot_handle_stopped(void);

// Emergency stop
void robot_emergency_stop(void);

#endif // ROBOT_CONTROLLER_H
