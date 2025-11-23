/**
 * telemetry.c
 * Comprehensive telemetry implementation with JSON formatting
 */

#include "telemetry.h"
#include "mqtt_client.h"
#include "config.h"
#include "encoder.h"
#include "encoder_utils.h"
#include "barcode_scanner.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define JSON_BUFFER_SIZE 512
#define QOS_LEVEL 0

static bool telemetry_initialized = false;
static uint8_t publish_cycle = 0;

bool telemetry_init(const char *broker_ip, uint16_t broker_port, const char *client_id) {
    mqtt_init(broker_ip, broker_port, client_id);
    
    if (!mqtt_connect()) {
        printf("Telemetry: Failed to connect to MQTT broker\n");
        return false;
    }
    
    telemetry_initialized = true;
    printf("Telemetry initialized\n");
    return true;
}

bool telemetry_is_ready(void) {
    return telemetry_initialized && (mqtt_get_status() == MQTT_CONNECTED);
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static bool publish_json(const char *topic, const char *json_buffer) {
    size_t len = strlen(json_buffer);
    printf("JSON size: %zu bytes (buffer: %d)\n", len, JSON_BUFFER_SIZE);
    
    if (len >= JSON_BUFFER_SIZE - 1) {
        printf("ERROR: JSON buffer overflow! %zu >= %d\n", len, JSON_BUFFER_SIZE);
        return false;
    }
    
    return mqtt_publish(topic, json_buffer, QOS_LEVEL);
}


const char* telemetry_line_state_to_string(LineFollowState state) {
    switch (state) {
        case LINE_FOLLOW_CENTERED: return "CENTERED";
        case LINE_FOLLOW_LEFT: return "LEFT";
        case LINE_FOLLOW_RIGHT: return "RIGHT";
        case LINE_FOLLOW_FAR_LEFT: return "FAR_LEFT";
        case LINE_FOLLOW_FAR_RIGHT: return "FAR_RIGHT";
        case LINE_FOLLOW_LOST: return "LOST";
        default: return "UNKNOWN";
    }
}

const char* telemetry_robot_state_to_string(RobotState state) {
    switch (state) {
        case ROBOT_STATE_IDLE: return "IDLE";
        case ROBOT_STATE_FOLLOWING: return "FOLLOWING";
        case ROBOT_STATE_TURNING: return "TURNING";
        case ROBOT_STATE_STOPPED: return "STOPPED";
        case ROBOT_STATE_SCANNING: return "SCANNING";
        case ROBOT_STATE_AVOIDING: return "AVOIDING";
        case ROBOT_STATE_RETURNING: return "RETURNING";
        case ROBOT_STATE_LOST: return "LOST";
        case ROBOT_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

const char* telemetry_obstacle_state_to_string(ObstacleState state) {
    switch (state) {
        case OBSTACLE_STATE_NONE: return "NONE";
        case OBSTACLE_STATE_DETECTED: return "DETECTED";
        case OBSTACLE_STATE_SCANNING: return "SCANNING";
        case OBSTACLE_STATE_AVOIDING: return "AVOIDING";
        case OBSTACLE_STATE_RETURNING: return "RETURNING";
        default: return "UNKNOWN";
    }
}

const char* telemetry_avoidance_dir_to_string(AvoidanceDirection dir) {
    switch (dir) {
        case AVOID_LEFT: return "LEFT";
        case AVOID_RIGHT: return "RIGHT";
        case AVOID_NONE: return "NONE";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// INDIVIDUAL PUBLISH FUNCTIONS
// ============================================================================

static bool publish_motor_data(float L_power, float R_power, float base_power, float steering_power) {
    char json[JSON_BUFFER_SIZE];
    snprintf(json, sizeof(json),
             "{\"L\":%.1f,\"R\":%.1f,\"base\":%.1f,\"steer\":%.1f}",
             L_power, R_power, base_power, steering_power);
    return publish_json(MQTT_TOPIC_MOTOR, json);
}

static bool publish_line_data(int ir_reading, float line_pos, float line_pos_filtered, LineFollowState state) {
    char json[JSON_BUFFER_SIZE];
    const char *state_str = telemetry_line_state_to_string(state);
    
    snprintf(json, sizeof(json),
             "{\"ir\":%d,\"pos\":%.1f,\"pos_filt\":%.1f,\"state\":\"%s\"}",
             ir_reading, line_pos, line_pos_filtered, state_str);
    return publish_json(MQTT_TOPIC_LINE, json);
}

static bool publish_imu_data(IMU *imu) {
    char json[JSON_BUFFER_SIZE];
    snprintf(json, sizeof(json),
             "{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"heading\":%.1f}",
             imu->ax, imu->ay, imu->az, imu->yaw);  // FIXED: Use correct field names
    return publish_json(MQTT_TOPIC_IMU, json);
}


static bool publish_state_data(RobotState robot_state, ObstacleState obstacle_state, uint32_t elapsed) {
    char json[JSON_BUFFER_SIZE];
    snprintf(json, sizeof(json),
             "{\"robot\":\"%s\",\"obstacle\":\"%s\",\"time\":%lu}",
             telemetry_robot_state_to_string(robot_state),
             telemetry_obstacle_state_to_string(obstacle_state),
             elapsed);
    return publish_json(MQTT_TOPIC_STATE, json);
}

// ============================================================================
// NEW: ENCODER AND SPEED TELEMETRY
// ============================================================================

bool telemetry_publish_encoder(float speed_mm_s, float distance_mm, 
                                int32_t left_count, int32_t right_count) {
    char json[JSON_BUFFER_SIZE];
    snprintf(json, sizeof(json),
             "{\"speed\":%.1f,\"distance\":%.1f,\"L_enc\":%ld,\"R_enc\":%ld}",
             speed_mm_s, distance_mm, (long)left_count, (long)right_count);
    return publish_json(MQTT_TOPIC_ENCODER, json);
}

// ============================================================================
// NEW: BARCODE TELEMETRY
// ============================================================================

bool telemetry_publish_barcode(BarcodeCommand command, char character) {
    char json[JSON_BUFFER_SIZE];
    
    const char *cmd_str = barcode_command_to_string(command);
    
    snprintf(json, sizeof(json),
             "{\"command\":\"%s\",\"char\":\"%c\"}",
             cmd_str, character);
    
    return publish_json(MQTT_TOPIC_BARCODE, json);
}

// ============================================================================
// NEW: OBSTACLE DETECTION TELEMETRY
// ============================================================================

bool telemetry_publish_obstacle_data(uint64_t distance_mm, 
                                      float width_cm,
                                      float clearance_left_cm,
                                      float clearance_right_cm) {
    char json[JSON_BUFFER_SIZE];
    snprintf(json, sizeof(json),
             "{\"dist\":%llu,\"width\":%.1f,\"clear_L\":%.1f,\"clear_R\":%.1f}",
             (unsigned long long)distance_mm, width_cm, clearance_left_cm, clearance_right_cm);
    return publish_json(MQTT_TOPIC_OBSTACLE, json);
}

bool telemetry_publish_obstacle_scan(const ScanResult *scan_result) {
    if (!scan_result || !scan_result->is_scanning) {
        return false;
    }
    
    char json[JSON_BUFFER_SIZE];
    
    if (scan_result->obstacle_count > 0) {
        const Obstacle *obs = &scan_result->obstacles[0];
        snprintf(json, sizeof(json),
                 "{\"count\":%d,\"angle_start\":%d,\"angle_end\":%d,\"span\":%d,\"width\":%.1f,\"min_dist\":%llu}",
                 scan_result->obstacle_count,
                 obs->angle_start,
                 obs->angle_end,
                 obs->angle_span,
                 obs->smoothed_width,
                 (unsigned long long)obs->min_distance);
    } else {
        snprintf(json, sizeof(json), "{\"count\":0,\"clear\":true}");
    }
    
    return publish_json(MQTT_TOPIC_SCAN, json);
}

// ============================================================================
// NEW: AVOIDANCE TELEMETRY
// ============================================================================

bool telemetry_publish_avoidance(AvoidanceDirection direction, 
                                  AvoidanceState state,
                                  bool obstacle_cleared) {
    char json[JSON_BUFFER_SIZE];
    
    // Get state string - match your actual AvoidanceState enum
    const char *state_str = "UNKNOWN";
    switch(state) {
        case AVOIDANCE_IDLE: state_str = "IDLE"; break;
        case AVOIDANCE_TURNING_OFF_LINE: state_str = "TURN_OFF"; break;
        case AVOIDANCE_REALIGN_FORWARD: state_str = "REALIGN_FWD"; break;  // FIXED
        case AVOIDANCE_MOVING_PARALLEL: state_str = "PARALLEL"; break;      // FIXED
        case AVOIDANCE_TURNING_BACK: state_str = "TURN_BACK"; break;
        case AVOIDANCE_SEARCHING_LINE: state_str = "SEARCHING"; break;
        case AVOIDANCE_COMPLETE: state_str = "COMPLETE"; break;
        case AVOIDANCE_FAILED: state_str = "FAILED"; break;
        default: break;
    }
    
    snprintf(json, sizeof(json),
             "{\"dir\":\"%s\",\"state\":\"%s\",\"cleared\":%s}",
             telemetry_avoidance_dir_to_string(direction),
             state_str,
             obstacle_cleared ? "true" : "false");
    return publish_json(MQTT_TOPIC_AVOIDANCE, json);
}


// ============================================================================
// NEW: STATE CHANGE TELEMETRY
// ============================================================================

bool telemetry_publish_state_change(RobotState prev_state, 
                                     RobotState new_state,
                                     uint32_t duration_ms) {
    char json[JSON_BUFFER_SIZE];
    snprintf(json, sizeof(json),
             "{\"from\":\"%s\",\"to\":\"%s\",\"duration\":%lu}",
             telemetry_robot_state_to_string(prev_state),
             telemetry_robot_state_to_string(new_state),
             duration_ms);
    return publish_json(MQTT_TOPIC_STATE, json);
}

// ============================================================================
// EXISTING FUNCTIONS
// ============================================================================

bool telemetry_publish_calibration(int white, int black, int threshold, int range) {
    char json[JSON_BUFFER_SIZE];
    snprintf(json, sizeof(json),
             "{\"white\":%d,\"black\":%d,\"thresh\":%d,\"range\":%d}",
             white, black, threshold, range);
    return publish_json(MQTT_TOPIC_CALIBRATION, json);
}

bool telemetry_publish_status(const char *message) {
    char json[JSON_BUFFER_SIZE];
    snprintf(json, sizeof(json), "{\"status\":\"%s\"}", message);
    return publish_json(MQTT_TOPIC_STATUS, json);
}

bool telemetry_publish_error(int error_code, const char *error_msg) {
    char json[JSON_BUFFER_SIZE];
    snprintf(json, sizeof(json),
             "{\"code\":%d,\"msg\":\"%s\"}",
             error_code, error_msg);
    return publish_json(MQTT_TOPIC_ERROR, json);
}

// ============================================================================
// MAIN PUBLISH ALL FUNCTION
// ============================================================================

bool telemetry_publish_all(
    int ir_reading,
    float line_position, float line_pos_filtered,
    LineFollowState line_state,
    IMU *imu,
    RobotState robot_state,
    ObstacleState obstacle_state,
    uint32_t elapsed
) {
    if (!telemetry_is_ready()) {
        printf("Telemetry: Not ready! initialized=%d mqtt_status=%d\n", 
               telemetry_initialized, mqtt_get_status());
        return false;
    }

    printf("\n=== Publishing telemetry (cycle=%d) ===\n", publish_cycle);
    
    bool success = true;
    
    // PUBLISH ALL DATA EVERY TIME (no more rotation!)
    
    // 1. Line data
    printf("Publishing line data...\n");
    success &= publish_line_data(ir_reading, line_position, line_pos_filtered, line_state);
    sleep_ms(20);
    
    // 2. IMU data
    printf("Publishing IMU data...\n");
    success &= publish_imu_data(imu);
    sleep_ms(20);
    
    // 3. State data
    printf("Publishing state data...\n");
    success &= publish_state_data(robot_state, obstacle_state, elapsed);
    sleep_ms(20);
    
    // 4. Encoder data (with speed calculation)
    printf("Publishing encoder data...\n");
    int32_t left_enc = get_left_encoder();
    int32_t right_enc = get_right_encoder();
    float distance_mm = get_average_distance_mm();
    
    static uint32_t last_encoder_time = 0;
    static int32_t last_left_count = 0;
    static int32_t last_right_count = 0;
    
    uint32_t current_time = elapsed;
    float dt = (current_time - last_encoder_time) / 1000.0f;
    float speed_mm_s = 0.0f;
    
    if (dt > 0.001f) {
        int32_t delta_left = left_enc - last_left_count;
        int32_t delta_right = right_enc - last_right_count;
        float delta_dist = (left_pulses_to_mm(delta_left) + right_pulses_to_mm(delta_right)) / 2.0f;
        speed_mm_s = delta_dist / dt;
    }
    
    success &= telemetry_publish_encoder(speed_mm_s, distance_mm, left_enc, right_enc);
    sleep_ms(20);
    
    last_encoder_time = current_time;
    last_left_count = left_enc;
    last_right_count = right_enc;
    
    printf("=== Telemetry publish complete (success=%d) ===\n\n", success);
    
    publish_cycle++;  // Keep for tracking, but not used for rotation anymore
    return success;
}