/**
 * telemetry.h
 * Telemetry and data logging interface
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "pico/stdlib.h"
#include <stdbool.h>

void telemetry_init(void);
bool telemetry_should_publish(void);
void telemetry_publish_speed(float left_speed, float right_speed);
void telemetry_publish_heading(float heading, float raw_heading);
void telemetry_publish_distance(float left_distance, float right_distance);
void telemetry_publish_line_position(int32_t position);
void telemetry_publish_barcode(const char *command);
void telemetry_publish_obstacle(float distance, float width);
void telemetry_publish_state(const char *state_name);
void telemetry_publish_error(const char *error_message);
void telemetry_publish_motor_output(float left_output, float right_output);
void telemetry_publish_imu_data(float heading, float gyro_z, float accel_x, float accel_y);
void telemetry_enable(bool enable);
bool telemetry_is_enabled(void);
void telemetry_print_separator(void);
void telemetry_print_header(const char *title);

#endif // TELEMETRY_H