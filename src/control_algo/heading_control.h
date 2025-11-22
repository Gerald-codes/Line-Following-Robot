/**
 * heading_control.h
 * Heading/orientation control using IMU and PID
 * 
 * NEW: Added heading deadband feature
 */

#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include "pico/stdlib.h"
#include "pid.h"
#include <stdbool.h>

typedef struct {
    PIDController pid;
    float target_heading;
    float current_heading;
    float correction;
    bool at_target;
    float deadband;            // NEW: Deadband in degrees (no correction if error < deadband)
    uint32_t last_update_time;
} HeadingController;

void heading_control_init(HeadingController *ctrl, float Kp, float Ki, float Kd, float max_correction);
void heading_control_set_target(HeadingController *ctrl, float heading);
void heading_control_set_deadband(HeadingController *ctrl, float deadband_degrees);  // NEW
float heading_control_update(HeadingController *ctrl, float current_heading);
bool heading_is_at_target(HeadingController *ctrl, float tolerance);
void heading_control_reset(HeadingController *ctrl);
float heading_get_error(HeadingController *ctrl);
float normalize_angle(float angle);
void heading_control_print_status(HeadingController *ctrl);

#endif // HEADING_CONTROL_H