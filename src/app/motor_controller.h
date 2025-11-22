/**
 * motor_controller.h
 * 
 * Motor control abstraction
 * Handles power calculations, deadband, and PID coordination
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

// Initialize motor controller
void motor_controller_init(void);

// Update based on line following
void motor_controller_update_line_following(float dt);

// Direct motor control (for search, avoidance, etc.)
void motor_controller_drive_direct(int left_power, int right_power);

// Stop all motors
void motor_controller_stop_all(void);

// Get current motor powers
void motor_controller_get_powers(float *left, float *right);

#endif // MOTOR_CONTROLLER_H
