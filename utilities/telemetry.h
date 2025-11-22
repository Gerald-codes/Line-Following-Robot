#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

// MQTT Configuration
#define MQTT_BROKER_ADDRESS "tcp://10.193.42.160:1883"  // Change to your broker address
#define MQTT_CLIENT_ID "pico_robot_car"
#define MQTT_QOS 1
#define MQTT_TIMEOUT 10000L

// MQTT Topics
#define TOPIC_OBSTACLE_WIDTH "robot/obstacle/width"
#define TOPIC_OBSTACLE_COUNT "robot/obstacle/count"
#define TOPIC_OBSTACLE_DISTANCE "robot/obstacle/distance"
#define TOPIC_OBSTACLE_ANGLES "robot/obstacle/angles"
#define TOPIC_OBSTACLE_ALL "robot/obstacle/all"
#define TOPIC_STATUS "robot/status"
#define TOPIC_SCAN_COMPLETE "robot/scan/complete"

// Telemetry Status Codes
typedef enum {
    TELEMETRY_SUCCESS = 0,
    TELEMETRY_ERROR_CONNECTION = -1,
    TELEMETRY_ERROR_PUBLISH = -2,
    TELEMETRY_ERROR_NOT_CONNECTED = -3,
    TELEMETRY_ERROR_INVALID_PARAM = -4
} TelemetryStatus;

// Obstacle Data Structure for Telemetry
typedef struct {
    int obstacle_id;
    int angle_start;
    int angle_end;
    int angle_span;
    uint64_t min_distance;
    float width;
    float smoothed_width;
} ObstacleTelemetry;

// Complete Scan Data Structure
typedef struct {
    int obstacle_count;
    ObstacleTelemetry obstacles[20];
    uint64_t scan_timestamp;
} ScanTelemetry;

/**
 * @brief Initialize the telemetry system and connect to MQTT broker
 * 
 * @param broker_address MQTT broker address 
 * @param client_id Unique client identifier
 * @return TelemetryStatus Success or error code
 */
TelemetryStatus telemetry_init(const char* broker_address, const char* client_id);

/**
 * @brief Publish obstacle width data
 * 
 * @param obstacle_id Obstacle identifier
 * @param width Calculated obstacle width in cm
 * @param smoothed_width Smoothed obstacle width in cm
 * @return TelemetryStatus Success or error code
 */
TelemetryStatus telemetry_publish_obstacle_width(int obstacle_id, float width, float smoothed_width);

/**
 * @brief Publish obstacle count
 * 
 * @param count Number of obstacles detected
 * @return TelemetryStatus Success or error code
 */
TelemetryStatus telemetry_publish_obstacle_count(int count);

/**
 * @brief Publish obstacle distance
 * 
 * @param obstacle_id Obstacle identifier
 * @param distance Distance to obstacle in cm
 * @return TelemetryStatus Success or error code
 */
TelemetryStatus telemetry_publish_obstacle_distance(int obstacle_id, uint64_t distance);

/**
 * @brief Publish obstacle angle information
 * 
 * @param obstacle_id Obstacle identifier
 * @param angle_start Starting angle of obstacle
 * @param angle_end Ending angle of obstacle
 * @param angle_span Total angular span
 * @return TelemetryStatus Success or error code
 */
TelemetryStatus telemetry_publish_obstacle_angles(int obstacle_id, int angle_start, 
                                                   int angle_end, int angle_span);

/**
 * @brief Publish complete obstacle data
 * 
 * @param obstacle Complete obstacle telemetry data
 * @return TelemetryStatus Success or error code
 */
TelemetryStatus telemetry_publish_obstacle(const ObstacleTelemetry* obstacle);

/**
 * @brief Publish complete scan results
 * 
 * @param scan_data Complete scan telemetry data
 * @return TelemetryStatus Success or error code
 */
TelemetryStatus telemetry_publish_scan_results(const ScanTelemetry* scan_data);

/**
 * @brief Publish a status message
 * 
 * @param message Status message string
 * @return TelemetryStatus Success or error code
 */
TelemetryStatus telemetry_publish_status(const char* message);

/**
 * @brief Check if telemetry system is connected
 * 
 * @return bool True if connected, false otherwise
 */
bool telemetry_is_connected(void);

/**
 * @brief Disconnect from MQTT broker and cleanup
 */
void telemetry_cleanup(void);

/**
 * @brief Process any pending MQTT messages (call periodically in main loop)
 * 
 * @return TelemetryStatus Success or error code
 */
TelemetryStatus telemetry_process(void);

#endif // TELEMETRY_H