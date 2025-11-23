/**
 * heading_control.c
 * Heading/orientation control using IMU and PID
 * Maintains straight paths and executes accurate turns
 * 
 * NEW: Added heading deadband feature
 */

#include "heading_control.h"
#include "config.h"
#include <math.h>
#include <stdio.h>

// Normalize angle to -180 to +180 range
float normalize_angle(float angle) {
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

void heading_control_init(HeadingController *ctrl, 
                         float Kp, float Ki, float Kd, 
                         float max_correction) {
    // Initialize PID controller
    pid_init(&ctrl->pid, Kp, Ki, Kd, -max_correction, max_correction);
    
    // Initialize state
    ctrl->target_heading = 0.0f;
    ctrl->current_heading = 0.0f;
    ctrl->correction = 0.0f;
    ctrl->at_target = false;
    ctrl->deadband = 0.0f;  // Default: no deadband
    ctrl->last_update_time = to_ms_since_boot(get_absolute_time());
    
    printf("Heading control initialized: Kp=%.2f, Ki=%.3f, Kd=%.3f\n", 
           Kp, Ki, Kd);
}

void heading_control_set_target(HeadingController *ctrl, float heading) {
    ctrl->target_heading = normalize_angle(heading);
    
    // Reset PID integral when changing target
    pid_reset(&ctrl->pid);
    ctrl->at_target = false;
    
    printf("Heading target set: %.1f°\n", ctrl->target_heading);
}

void heading_control_set_deadband(HeadingController *ctrl, float deadband_degrees) {
    ctrl->deadband = fabsf(deadband_degrees);
    printf("Heading deadband set: ±%.1f°\n", ctrl->deadband);
}

float heading_control_update(HeadingController *ctrl, float current_heading) {
    // Update current heading
    ctrl->current_heading = normalize_angle(current_heading);
    
    // Calculate time delta
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    float dt = (current_time - ctrl->last_update_time) / 1000.0f;
    ctrl->last_update_time = current_time;
    
    // Clamp dt to reasonable range
    if (dt > 0.1f) dt = 0.02f;  // Default to 20ms if too large
    if (dt < 0.001f) dt = 0.001f;  // Minimum 1ms
    
    // Calculate heading error with angle wrapping
    float error = ctrl->target_heading - ctrl->current_heading;
    error = normalize_angle(error);
    
    // Apply deadband - no correction if error is small
    float abs_error = fabsf(error);
    if (abs_error < ctrl->deadband) {
        // Within deadband - no correction needed
        ctrl->correction = 0.0f;
        ctrl->at_target = true;
        
        // Also reset PID integral to prevent windup
        pid_reset(&ctrl->pid);
        
        return 0.0f;
    }
    
    // Outside deadband - apply PID correction
    ctrl->at_target = false;
    
    // Update PID setpoint to target heading
    pid_set_target(&ctrl->pid, ctrl->target_heading);
    
    // Compute PID output
    // Note: We pass current_heading as the "measured value"
    // The PID will calculate error internally as (target - measured)
    ctrl->correction = pid_compute(&ctrl->pid, ctrl->current_heading, dt);
    
    return ctrl->correction;
}

bool heading_is_at_target(HeadingController *ctrl, float tolerance) {
    float error = ctrl->target_heading - ctrl->current_heading;
    error = normalize_angle(error);
    return (fabsf(error) < tolerance);
}

void heading_control_reset(HeadingController *ctrl) {
    pid_reset(&ctrl->pid);
    ctrl->correction = 0.0f;
    ctrl->at_target = false;
    ctrl->last_update_time = to_ms_since_boot(get_absolute_time());
}

float heading_get_error(HeadingController *ctrl) {
    float error = ctrl->target_heading - ctrl->current_heading;
    return normalize_angle(error);
}

void heading_control_print_status(HeadingController *ctrl) {
    float error = heading_get_error(ctrl);
    const char* status;
    
    if (fabsf(error) < ctrl->deadband) {
        status = "[IN DEADBAND]";
    } else if (ctrl->at_target) {
        status = "[AT TARGET]";
    } else {
        status = "[CORRECTING]";
    }
    
    printf("Heading: Target=%.1f° Current=%.1f° Error=%.1f° Correction=%.1f %s\n",
           ctrl->target_heading,
           ctrl->current_heading,
           error,
           ctrl->correction,
           status);
}