/**
 * @file    telemetry.h
 * @brief   Robot telemetry interface
 * @details Comprehensive telemetry system for robotic car including
 *          line following, IMU data, obstacle detection, barcode scanning,
 *          and state monitoring with MQTT publishing
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>
#include "line_following.h"
#include "imu.h"
#include "config.h"
#include "obstacle_scanner.h"
#include "barcode_scanner.h"
#include "avoidance_maneuver.h"

/**
 * @brief Initialize telemetry system
 * @details Must be called after WiFi is connected
 * @param broker_ip MQTT broker IP address
 * @param broker_port MQTT broker port
 * @param client_id MQTT client ID
 * @return true if initialized successfully
 */
bool telemetry_init(const char *broker_ip, uint16_t broker_port, const char *client_id);

/**
 * @brief Check if telemetry system is ready
 * @return true if connected to MQTT broker
 */
bool telemetry_is_ready(void);

/**
 * @brief Publish all telemetry data
 * @details Main function to call from control loop
 * @param ir_reading Raw IR sensor reading
 * @param line_position Current line position
 * @param line_pos_filtered Filtered line position
 * @param line_state Current line following state
 * @param imu Pointer to IMU structure
 * @param robot_state Current robot state
 * @param obstacle_state Current obstacle state
 * @param elapsed Time elapsed since start in milliseconds
 * @return true if all data published successfully
 */
bool telemetry_publish_all(int ir_reading,
                           float line_position,
                           float line_pos_filtered,
                           LineFollowState line_state,
                           IMU *imu,
                           RobotState robot_state,
                           ObstacleState obstacle_state,
                           uint32_t elapsed);

/**
 * @brief Publish encoder and speed data
 * @param speed_mm_s Current speed in mm/s
 * @param distance_mm Total distance travelled in mm
 * @param left_count Left encoder count
 * @param right_count Right encoder count
 * @return true if published successfully
 */
bool telemetry_publish_encoder(float speed_mm_s,
                                float distance_mm,
                                int32_t left_count,
                                int32_t right_count);

/**
 * @brief Publish barcode detection data
 * @param command The decoded barcode command
 * @param character The actual character decoded
 * @return true if published successfully
 */
bool telemetry_publish_barcode(BarcodeCommand command, char character);

/**
 * @brief Publish obstacle scan results
 * @param scan_result Pointer to ScanResult structure
 * @return true if published successfully
 */
bool telemetry_publish_obstacle_scan(const ScanResult *scan_result);

/**
 * @brief Publish avoidance maneuver status
 * @param direction Avoidance direction (LEFT/RIGHT/NONE)
 * @param state Current avoidance state
 * @param obstacle_cleared Whether obstacle has been cleared
 * @return true if published successfully
 */
bool telemetry_publish_avoidance(AvoidanceDirection direction,
                                  AvoidanceState state,
                                  bool obstacle_cleared);

/**
 * @brief Publish obstacle data for real-time monitoring
 * @param distance_mm Ultrasonic distance reading
 * @param width_cm Calculated obstacle width
 * @param clearance_left_cm Left clearance
 * @param clearance_right_cm Right clearance
 * @return true if published successfully
 */
bool telemetry_publish_obstacle_data(uint64_t distance_mm,
                                      float width_cm,
                                      float clearance_left_cm,
                                      float clearance_right_cm);

/**
 * @brief Publish state change events
 * @param prev_state Previous robot state
 * @param new_state New robot state
 * @param duration_ms Time spent in previous state
 * @return true if published successfully
 */
bool telemetry_publish_state_change(RobotState prev_state,
                                     RobotState new_state,
                                     uint32_t duration_ms);

/**
 * @brief Publish calibration data
 * @param white White surface reading
 * @param black Black surface reading
 * @param threshold Calculated threshold
 * @param range Dynamic range
 * @return true if published successfully
 */
bool telemetry_publish_calibration(int white, int black, int threshold, int range);

/**
 * @brief Publish status or diagnostic messages
 * @param message Status message string
 * @return true if published successfully
 */
bool telemetry_publish_status(const char *message);

/**
 * @brief Publish error messages
 * @param error_code Error code
 * @param error_msg Error message string
 * @return true if published successfully
 */
bool telemetry_publish_error(int error_code, const char *error_msg);

/**
 * @brief Convert RobotState enum to string
 * @param state RobotState to convert
 * @return String representation
 */
const char *telemetry_robot_state_to_string(RobotState state);

/**
 * @brief Convert ObstacleState enum to string
 * @param state ObstacleState to convert
 * @return String representation
 */
const char *telemetry_obstacle_state_to_string(ObstacleState state);

/**
 * @brief Convert AvoidanceDirection to string
 * @param dir AvoidanceDirection to convert
 * @return String representation
 */
const char *telemetry_avoidance_dir_to_string(AvoidanceDirection dir);

/**
 * @brief Convert LineFollowState to string
 * @param state LineFollowState to convert
 * @return String representation
 */
const char *telemetry_line_state_to_string(LineFollowState state);

#endif /* TELEMETRY_H */
