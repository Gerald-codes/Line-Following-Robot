/**
 * telemetry.h
 * Comprehensive telemetry interface for robotic car
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include "line_following.h"  // This defines LineFollowState
#include "imu.h"
#include "config.h"           // This already defines RobotState and ObstacleState!
#include "obstacle_scanner.h"
#include "barcode.h"
#include "avoidance_maneuver.h"

// REMOVED the duplicate enum definitions - they're already in config.h!

/**
 * Initialize telemetry system
 * Must be called after WiFi is connected
 * 
 * @param broker_ip MQTT broker IP address
 * @param broker_port MQTT broker port
 * @param client_id MQTT client ID
 * @return true if initialized successfully
 */
bool telemetry_init(const char *broker_ip, uint16_t broker_port, const char *client_id);

/**
 * Check if telemetry system is ready
 * @return true if connected to MQTT broker
 */
bool telemetry_is_ready(void);

/**
 * Publish all telemetry data in one call
 * This is the main function to call from your main loop
 * 
 * @param L_power Left motor power (0-100)
 * @param R_power Right motor power (0-100)
 * @param base_power Base power setting
 * @param steering_power Steering correction value
 * @param ir_reading Raw IR sensor reading
 * @param line_position Current line position
 * @param line_pos_filtered Filtered line position
 * @param line_state Current line following state (LineFollowState)
 * @param imu Pointer to IMU structure
 * @param robot_state Current robot state (from RobotState enum)
 * @param obstacle_state Current obstacle state
 * @param elapsed Time elapsed since start (ms)
 * @return true if all data published successfully
 */
bool telemetry_publish_all(
    int ir_reading,
    float line_position, float line_pos_filtered,
    LineFollowState line_state,
    IMU *imu,
    RobotState robot_state,
    ObstacleState obstacle_state,
    uint32_t elapsed
);

/**
 * NEW: Publish encoder and speed data
 * @param speed_mm_s Current speed in mm/s
 * @param distance_mm Total distance travelled in mm
 * @param left_count Left encoder count
 * @param right_count Right encoder count
 */
bool telemetry_publish_encoder(float speed_mm_s, float distance_mm, 
                                int32_t left_count, int32_t right_count);

/**
 * NEW: Publish barcode detection data
 * @param result Pointer to BarcodeResult structure
 */
bool telemetry_publish_barcode(const BarcodeResult *result);

/**
 * NEW: Publish obstacle scan results
 * @param scan_result Pointer to ScanResult structure
 */
bool telemetry_publish_obstacle_scan(const ScanResult *scan_result);

/**
 * NEW: Publish avoidance maneuver status
 * @param direction Avoidance direction (LEFT/RIGHT/NONE)
 * @param state Current avoidance state
 * @param obstacle_cleared Whether obstacle has been cleared
 */
bool telemetry_publish_avoidance(AvoidanceDirection direction, 
                                  AvoidanceState state,
                                  bool obstacle_cleared);

/**
 * NEW: Publish individual obstacle data (for real-time monitoring)
 * @param distance_mm Ultrasonic distance reading
 * @param width_cm Calculated obstacle width
 * @param clearance_left_cm Left clearance
 * @param clearance_right_cm Right clearance
 */
bool telemetry_publish_obstacle_data(uint64_t distance_mm, 
                                      float width_cm,
                                      float clearance_left_cm,
                                      float clearance_right_cm);

/**
 * NEW: Publish state change events
 * @param prev_state Previous robot state
 * @param new_state New robot state
 * @param duration_ms Time spent in previous state
 */
bool telemetry_publish_state_change(RobotState prev_state, 
                                     RobotState new_state,
                                     uint32_t duration_ms);

/**
 * Publish calibration data
 * @param white White surface reading
 * @param black Black surface reading
 * @param threshold Calculated threshold
 * @param range Dynamic range
 */
bool telemetry_publish_calibration(int white, int black, int threshold, int range);

/**
 * Publish status/diagnostic messages
 * @param message Status message string
 */
bool telemetry_publish_status(const char *message);

/**
 * Publish error messages
 * @param error_code Error code
 * @param error_msg Error message string
 */
bool telemetry_publish_error(int error_code, const char *error_msg);

/**
 * Helper: Convert RobotState enum to string
 */
const char* telemetry_robot_state_to_string(RobotState state);

/**
 * Helper: Convert ObstacleState enum to string
 */
const char* telemetry_obstacle_state_to_string(ObstacleState state);

/**
 * Helper: Convert AvoidanceDirection to string
 */
const char* telemetry_avoidance_dir_to_string(AvoidanceDirection dir);

/**
 * Helper: Convert LineFollowState to string
 */
const char* telemetry_line_state_to_string(LineFollowState state);

#endif // TELEMETRY_H
