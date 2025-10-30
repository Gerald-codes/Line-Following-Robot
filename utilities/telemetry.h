/**
 * telemetry.h
 * MQTT-Based Telemetry over WiFi for Robotic Car Project
 * 
 * Demo 1 Requirements:
 * - Publish speed, distance, heading (raw & filtered) via MQTT
 * - Real-time data transmission to PC/mobile dashboard
 * - Connection status monitoring
 * - Error handling and reconnection logic
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "pico/stdlib.h"
#include <stdbool.h>

// ============================================================================
// WIFI CONFIGURATION
// ============================================================================
#define WIFI_SSID "YOUR_WIFI_SSID"           // Change to your WiFi network
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"   // Change to your WiFi password
#define WIFI_CONNECT_TIMEOUT_MS 10000
#define WIFI_RECONNECT_INTERVAL_MS 5000

// ============================================================================
// MQTT BROKER CONFIGURATION
// ============================================================================
#define MQTT_BROKER_HOST "localhost"  // Public broker (or use your own)
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "robotic_car_demo1"
#define MQTT_USERNAME ""                      // Leave empty if no auth
#define MQTT_PASSWORD ""
#define MQTT_KEEPALIVE_SEC 60
#define MQTT_RECONNECT_DELAY_MS 3000

// ============================================================================
// MQTT TOPICS - Structured Hierarchy
// ============================================================================
// Speed telemetry
#define TOPIC_SPEED_LEFT "robot/demo1/speed/left"
#define TOPIC_SPEED_RIGHT "robot/demo1/speed/right"
#define TOPIC_SPEED_AVG "robot/demo1/speed/average"

// Distance telemetry
#define TOPIC_DISTANCE_LEFT "robot/demo1/distance/left"
#define TOPIC_DISTANCE_RIGHT "robot/demo1/distance/right"
#define TOPIC_DISTANCE_TOTAL "robot/demo1/distance/total"

// Heading telemetry (IMU)
#define TOPIC_HEADING_FILTERED "robot/demo1/heading/filtered"
#define TOPIC_HEADING_RAW "robot/demo1/heading/raw"
#define TOPIC_HEADING_ERROR "robot/demo1/heading/error"

// IMU data (raw and filtered)
#define TOPIC_IMU_GYRO_Z "robot/demo1/imu/gyro_z"
#define TOPIC_IMU_ACCEL_X "robot/demo1/imu/accel_x"
#define TOPIC_IMU_ACCEL_Y "robot/demo1/imu/accel_y"

// Motor control
#define TOPIC_MOTOR_LEFT_PWM "robot/demo1/motor/left/pwm"
#define TOPIC_MOTOR_RIGHT_PWM "robot/demo1/motor/right/pwm"

// System status
#define TOPIC_STATUS "robot/demo1/status"
#define TOPIC_ERROR "robot/demo1/error"
#define TOPIC_CONNECTION "robot/demo1/connection"

// Command topic (subscribe) - for receiving commands
#define TOPIC_COMMAND "robot/demo1/command"

// ============================================================================
// MQTT QoS LEVELS
// ============================================================================
typedef enum {
    QOS_AT_MOST_ONCE = 0,   // Fire and forget (fastest)
    QOS_AT_LEAST_ONCE = 1,  // Guaranteed delivery
    QOS_EXACTLY_ONCE = 2    // Exactly once (slowest)
} MQTTQoS;

// Default QoS for different message types
#define QOS_TELEMETRY QOS_AT_MOST_ONCE      // Speed, distance - can afford to lose some
#define QOS_STATUS QOS_AT_LEAST_ONCE         // Status messages - important
#define QOS_ERROR QOS_AT_LEAST_ONCE          // Errors - important
#define QOS_COMMAND QOS_AT_LEAST_ONCE        // Commands - must be received

// ============================================================================
// TELEMETRY CONFIGURATION
// ============================================================================
#define TELEMETRY_PUBLISH_INTERVAL_MS 500    // 2Hz - matches config.h
#define TELEMETRY_BUFFER_SIZE 256
#define TELEMETRY_MAX_RETRIES 3
#define TELEMETRY_ENABLE_SERIAL_FALLBACK true  // Print to serial if MQTT fails

// ============================================================================
// CONNECTION STATUS
// ============================================================================
typedef enum {
    CONN_STATUS_DISCONNECTED = 0,
    CONN_STATUS_WIFI_CONNECTING,
    CONN_STATUS_WIFI_CONNECTED,
    CONN_STATUS_MQTT_CONNECTING,
    CONN_STATUS_MQTT_CONNECTED,
    CONN_STATUS_ERROR
} ConnectionStatus;

// ============================================================================
// TELEMETRY DATA STRUCTURES
// ============================================================================

// Telemetry packet for speed data
typedef struct {
    float left_speed_mm_s;
    float right_speed_mm_s;
    float average_speed_mm_s;
    uint32_t timestamp_ms;
} SpeedTelemetry;

// Telemetry packet for distance data
typedef struct {
    float left_distance_mm;
    float right_distance_mm;
    float total_distance_mm;
    uint32_t timestamp_ms;
} DistanceTelemetry;

// Telemetry packet for heading data
typedef struct {
    float filtered_heading_deg;
    float raw_heading_deg;
    float heading_error_deg;
    float target_heading_deg;
    uint32_t timestamp_ms;
} HeadingTelemetry;

// Telemetry packet for IMU data
typedef struct {
    float gyro_z_deg_s;
    float accel_x_g;
    float accel_y_g;
    bool filter_active;
    uint32_t timestamp_ms;
} IMUTelemetry;

// System status packet
typedef struct {
    ConnectionStatus connection_status;
    uint32_t uptime_sec;
    uint32_t packets_sent;
    uint32_t packets_failed;
    float mqtt_latency_ms;
    bool wifi_connected;
    bool mqtt_connected;
} SystemStatus;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Initialization and connection
void telemetry_init(void);
bool telemetry_connect_wifi(const char *ssid, const char *password);
bool telemetry_connect_mqtt(void);
void telemetry_disconnect(void);
bool telemetry_reconnect(void);

// Publishing functions - Demo 1 specific
void telemetry_publish_speed(float left_speed, float right_speed);
void telemetry_publish_heading(float filtered_heading, float raw_heading);
void telemetry_publish_distance(float left_distance, float right_distance);
void telemetry_publish_imu_data(float heading, float gyro_z, float accel_x, float accel_y);
void telemetry_publish_motor_output(float left_output, float right_output);
void telemetry_publish_state(const char *state_name);
void telemetry_publish_error(const char *error_message);

// Structured data publishing
bool telemetry_publish_speed_packet(const SpeedTelemetry *packet);
bool telemetry_publish_distance_packet(const DistanceTelemetry *packet);
bool telemetry_publish_heading_packet(const HeadingTelemetry *packet);
bool telemetry_publish_imu_packet(const IMUTelemetry *packet);
bool telemetry_publish_system_status(const SystemStatus *status);

// Generic MQTT publish with QoS
bool telemetry_publish(const char *topic, const char *payload, MQTTQoS qos);
bool telemetry_publish_float(const char *topic, float value, MQTTQoS qos);
bool telemetry_publish_json(const char *topic, const char *json, MQTTQoS qos);

// Command receiving (for future use)
bool telemetry_subscribe(const char *topic, MQTTQoS qos);
void telemetry_set_command_callback(void (*callback)(const char *topic, const char *payload));

// Status and control
ConnectionStatus telemetry_get_status(void);
bool telemetry_is_connected(void);
bool telemetry_wifi_is_connected(void);
bool telemetry_mqtt_is_connected(void);
uint32_t telemetry_get_packets_sent(void);
uint32_t telemetry_get_packets_failed(void);
float telemetry_get_latency_ms(void);

// Timing and control
bool telemetry_should_publish(void);
void telemetry_enable(bool enable);
bool telemetry_is_enabled(void);
void telemetry_update(void);  // Call this regularly in main loop

// Utility functions
void telemetry_print_status(void);
void telemetry_print_separator(void);
void telemetry_print_header(const char *title);
const char* telemetry_connection_status_string(ConnectionStatus status);

// Legacy compatibility (for existing code)
void telemetry_publish_line_position(int32_t position);
void telemetry_publish_barcode(const char *command);
void telemetry_publish_obstacle(float distance, float width);

#endif // TELEMETRY_H