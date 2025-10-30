/**
 * telemetry.c
 * Simplified telemetry - Serial output only (no MQTT hardware needed)
 */

#include "telemetry.h"
#include "config.h"
#include <stdio.h>

static bool telemetry_enabled = true;
static uint32_t last_publish_time = 0;

void telemetry_init(void) {
    telemetry_enabled = true;
    last_publish_time = to_ms_since_boot(get_absolute_time());
    
    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║              TELEMETRY SYSTEM INITIALIZED                     ║\n");
    printf("║              (Serial Output Only - No MQTT)                   ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
}

bool telemetry_should_publish(void) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    if (current_time - last_publish_time >= TELEMETRY_INTERVAL_MS) {
        last_publish_time = current_time;
        return true;
    }
    
    return false;
}

void telemetry_publish_speed(float left_speed, float right_speed) {
    if (!telemetry_enabled) return;
    printf("[SPEED] L: %6.1f mm/s | R: %6.1f mm/s\n", left_speed, right_speed);
}

void telemetry_publish_heading(float heading, float raw_heading) {
    if (!telemetry_enabled) return;
    printf("[HEADING] Filtered: %6.1f° | Raw: %6.1f°\n", heading, raw_heading);
}

void telemetry_publish_distance(float left_distance, float right_distance) {
    if (!telemetry_enabled) return;
    float avg_distance = (left_distance + right_distance) / 2.0f;
    printf("[DISTANCE] L: %7.1f mm | R: %7.1f mm | Avg: %7.1f mm\n", 
           left_distance, right_distance, avg_distance);
}

void telemetry_publish_line_position(int32_t position) {
    if (!telemetry_enabled) return;
    printf("[LINE] Position: %ld\n", position);
}

void telemetry_publish_barcode(const char *command) {
    if (!telemetry_enabled) return;
    printf("[BARCODE] Command: %s\n", command);
}

void telemetry_publish_obstacle(float distance, float width) {
    if (!telemetry_enabled) return;
    printf("[OBSTACLE] Distance: %.1f mm | Width: %.1f mm\n", distance, width);
}

void telemetry_publish_state(const char *state_name) {
    if (!telemetry_enabled) return;
    printf("[STATE] %s\n", state_name);
}

void telemetry_publish_error(const char *error_message) {
    if (!telemetry_enabled) return;
    printf("[ERROR] %s\n", error_message);
}

void telemetry_publish_motor_output(float left_output, float right_output) {
    if (!telemetry_enabled) return;
    printf("[MOTOR PWM] L: %+6.1f%% | R: %+6.1f%%\n", left_output, right_output);
}

void telemetry_publish_imu_data(float heading, float gyro_z, 
                                float accel_x, float accel_y) {
    if (!telemetry_enabled) return;
    printf("[IMU] Heading: %6.1f° | Gyro Z: %+6.1f°/s | Accel X: %+5.2fg Y: %+5.2fg\n",
           heading, gyro_z, accel_x, accel_y);
}

void telemetry_enable(bool enable) {
    telemetry_enabled = enable;
    if (enable) {
        printf("✓ Telemetry enabled\n");
    } else {
        printf("✗ Telemetry disabled\n");
    }
}

bool telemetry_is_enabled(void) {
    return telemetry_enabled;
}

void telemetry_print_separator(void) {
    printf("───────────────────────────────────────────────────────────────\n");
}

void telemetry_print_header(const char *title) {
    printf("\n╔═══════════════════════════════════════════════════════════╗\n");
    printf("║ %-57s ║\n", title);
    printf("╚═══════════════════════════════════════════════════════════╝\n\n");
}