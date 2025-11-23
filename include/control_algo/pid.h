#ifndef PID_H
#define PID_H

#include "pico/stdlib.h"

typedef struct {
    // PID gains
    float Kp;
    float Ki;
    float Kd;
    
    // PID state
    float setpoint;           // Target value
    float integral;           // Accumulated error
    float previous_error;     // Last error for derivative
    
    // Output limits
    float output_min;
    float output_max;
    
    // Anti-windup
    float integral_max;
    
} PIDController;

// Initialize PID controller
void pid_init(PIDController *pid, float Kp, float Ki, float Kd, 
              float out_min, float out_max);

// Update PID (call this regularly)
float pid_compute(PIDController *pid, float measured_value, float dt);

// Set new target
void pid_set_target(PIDController *pid, float setpoint);

// Reset PID state
void pid_reset(PIDController *pid);

#endif