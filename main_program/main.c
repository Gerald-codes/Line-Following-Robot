/**
 * integrated_main.c
 * 
 * INTEGRATED LINE FOLLOWING + OBSTACLE AVOIDANCE + TELEMETRY
 * 
 * Features:
 * - Line following using single IR sensor
 * - Obstacle detection with ultrasonic sensor
 * - Obstacle avoidance maneuvers
 * - State machine coordination
 * - WiFi connectivity
 * - MQTT telemetry publishing
 * - Ready for barcode integration
 */

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdbool.h>

// State
#include "state_machine.h"

// Hardware drivers
#include "motor.h"
#include "encoder.h"
#include "encoder_utils.h"
#include "imu.h"
#include "ir_sensor.h"
#include "ultrasonic.h"
#include "servo.h"
#include "obstacle_control.h"

// Control algorithms
#include "line_following.h"
#include "obstacle_scanner.h"
#include "avoidance_maneuver.h"

// Utilities
#include "timer_manager.h"
#include "calibration.h"
#include "config.h"
#include "pin_definitions.h"

// NEW: WiFi and Telemetry
#include "wifi.h"
#include "telemetry.h"
#include "barcode.h"
#include "mqtt_client.h"

// ============================================================================
// SYSTEM STATE MACHINE
// ============================================================================
static SystemState current_state = STATE_IDLE;
static SystemState previous_state = STATE_IDLE;

// ============================================================================
// CONFIGURATION
// ============================================================================
// Obstacle detection
#define OBSTACLE_CHECK_DISTANCE_CM 20
#define OBSTACLE_CHECK_INTERVAL_MS 500
#define CRITICAL_DISTANCE_CM 15

// ============================================================================
// GLOBAL STATE
// ============================================================================
static IMU imu;
static uint32_t last_obstacle_check = 0;
static uint32_t state_entry_time = 0;

// NEW: Telemetry tracking
static uint32_t last_telemetry_time = 0;
static RobotState prev_robot_state = ROBOT_STATE_IDLE;
static uint32_t robot_state_enter_time = 0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
static void init_hardware(void);
static bool init_wifi_and_telemetry(void);
static void publish_telemetry_data(uint32_t current_time);
static void handle_returning_to_line(void);
static void handle_line_lost(void);
static const char* state_to_string(SystemState state);
static RobotState system_state_to_robot_state(SystemState state);
static ObstacleState get_obstacle_state(void);

SystemState get_current_state(void) {
    return current_state;
}

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================
static void init_hardware(void) {
    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║ INTEGRATED LINE FOLLOWING + OBSTACLE AVOIDANCE + TELEMETRY ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    printf("Initializing hardware...\n");

    // Motors
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    printf(" ✓ Motors\n");

    // Encoders
    encoder_init();
    printf(" ✓ Encoders\n");

    // IMU (struct-based for line following)
    imu_init(&imu);
    imu_calibrate(&imu);
    printf(" ✓ IMU (struct)\n");

    // IMU Helper (for avoidance maneuvers)
    imu_helper_init();
    printf(" ✓ IMU Helper\n");

    // IR Sensors
    ir_sensor_init();
    printf(" ✓ IR Sensors\n");

    // Ultrasonic
    ultrasonic_init(TRIG_PIN, ECHO_PIN);
    printf(" ✓ Ultrasonic\n");

    // Servo
    servo_init(SERVO_PIN);
    servo_set_angle(ANGLE_CENTER);
    printf(" ✓ Servo\n");

    // Calibration button
    calibration_init();
    printf(" ✓ Calibration Button (GP20)\n");

    // Line following controller
    line_following_init();
    printf(" ✓ Line Following Controller\n");

    // Obstacle systems
    scanner_init();
    avoidance_init();
    timer_manager_init();
    printf(" ✓ Obstacle Detection Systems\n");

    printf("\n✓ All hardware initialized\n\n");
}

// ============================================================================
// NEW: WIFI & TELEMETRY INITIALIZATION
// ============================================================================
static bool init_wifi_and_telemetry(void) {
    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║ WiFi & Telemetry Initialization                              ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");

    // Initialize WiFi
    wifi_init();
    printf("Connecting to WiFi: %s\n", WIFI_SSID);
    
    if (!wifi_connect(WIFI_SSID, WIFI_PASSWORD)) {
        printf("✗ Failed to connect to WiFi\n");
        return false;
    }
    
    printf("✓ WiFi connected\n");
    
    // Get IP address
    char ip_buf[16];
    if (wifi_get_ip(ip_buf, sizeof(ip_buf))) {
        printf("  IP Address: %s\n", ip_buf);
    }

    // Initialize telemetry
    printf("Connecting to MQTT broker: %s:%d\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
    
    if (!telemetry_init(MQTT_BROKER_IP, MQTT_BROKER_PORT, MQTT_CLIENT_ID)) {
        printf("✗ Failed to connect to MQTT broker\n");
        return false;
    }
    
    printf("✓ MQTT connected\n");
    printf("\n✓ Telemetry system ready\n\n");
    
    // Publish initial status
    telemetry_publish_status("Robot initialized and ready");
    
    return true;
}

// ============================================================================
// STATE MANAGEMENT
// ============================================================================
void change_state(SystemState new_state) {
    if (new_state != current_state) {
        previous_state = current_state;
        current_state = new_state;
        state_entry_time = to_ms_since_boot(get_absolute_time());
        
        printf("\n>>> STATE CHANGE: %s -> %s\n",
               state_to_string(previous_state),
               state_to_string(new_state));
        
        // Publish state change to telemetry
        RobotState new_robot_state = system_state_to_robot_state(new_state);
        if (telemetry_is_ready()) {
            uint32_t duration = to_ms_since_boot(get_absolute_time()) - robot_state_enter_time;
            telemetry_publish_state_change(prev_robot_state, new_robot_state, duration);
            prev_robot_state = new_robot_state;
            robot_state_enter_time = to_ms_since_boot(get_absolute_time());
        }
    }
}

static const char* state_to_string(SystemState state) {
    switch (state) {
        case STATE_IDLE: return "IDLE";
        case STATE_LINE_FOLLOWING: return "LINE_FOLLOWING";
        case STATE_OBSTACLE_DETECTED: return "OBSTACLE_DETECTED";
        case STATE_OBSTACLE_SCANNING: return "OBSTACLE_SCANNING";
        case STATE_OBSTACLE_AVOIDING: return "OBSTACLE_AVOIDING";
        case STATE_RETURNING_TO_LINE: return "RETURNING_TO_LINE";
        case STATE_LINE_LOST: return "LINE_LOST";
        case STATE_STOPPED: return "STOPPED";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// NEW: STATE CONVERSION FOR TELEMETRY
// ============================================================================
static RobotState system_state_to_robot_state(SystemState state) {
    switch (state) {
        case STATE_IDLE: return ROBOT_STATE_IDLE;
        case STATE_LINE_FOLLOWING: return ROBOT_STATE_FOLLOWING;
        case STATE_OBSTACLE_DETECTED: return ROBOT_STATE_STOPPED;
        case STATE_OBSTACLE_SCANNING: return ROBOT_STATE_SCANNING;
        case STATE_OBSTACLE_AVOIDING: return ROBOT_STATE_AVOIDING;
        case STATE_RETURNING_TO_LINE: return ROBOT_STATE_RETURNING;
        case STATE_LINE_LOST: return ROBOT_STATE_LOST;
        case STATE_STOPPED: return ROBOT_STATE_STOPPED;
        default: return ROBOT_STATE_ERROR;
    }
}

static ObstacleState get_obstacle_state(void) {
    switch (current_state) {
        case STATE_OBSTACLE_DETECTED: return OBSTACLE_STATE_DETECTED;
        case STATE_OBSTACLE_SCANNING: return OBSTACLE_STATE_SCANNING;
        case STATE_OBSTACLE_AVOIDING: return OBSTACLE_STATE_AVOIDING;
        case STATE_RETURNING_TO_LINE: return OBSTACLE_STATE_RETURNING;
        default: return OBSTACLE_STATE_NONE;
    }
}

// ============================================================================
// NEW: TELEMETRY PUBLISHING
// ============================================================================
static void publish_telemetry_data(uint32_t current_time) {
    if (!telemetry_is_ready()) {
        return;
    }
    
    // Publish at regular interval
    if (current_time - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
        // Get IR sensor data
        uint16_t ir_reading = ir_read_line_sensor();
        int32_t line_pos = ir_get_line_position();
        float line_pos_filtered = (float)line_pos;
        
        // Get line state
        LineFollowState line_state;
        if (ir_line_detected()) {
            if (line_pos > 500) line_state = LINE_FOLLOW_LEFT;
            else if (line_pos < -500) line_state = LINE_FOLLOW_RIGHT;
            else line_state = LINE_FOLLOW_CENTERED;
        } else {
            line_state = LINE_FOLLOW_LOST;
        }
        
        // Get current states
        RobotState robot_state = system_state_to_robot_state(current_state);
        ObstacleState obstacle_state = get_obstacle_state();
        
        // Publish all telemetry (NO MOTOR VALUES!)
        telemetry_publish_all(
            (int)ir_reading,
            (float)line_pos,
            line_pos_filtered,
            line_state,
            &imu,
            robot_state,
            obstacle_state,
            current_time
        );
        
        last_telemetry_time = current_time;
    }
}


static void handle_returning_to_line(void) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // Check if we found the line
    if (ir_line_detected()) {
        printf("\n[LINE] Line detected! Resuming normal following\n");
        change_state(STATE_LINE_FOLLOWING);
        return;
    }
    
    // Continue moving forward slowly to find line
    motor_drive(M1A, M1B, -30);
    motor_drive(M2A, M2B, -30);
    
    // Timeout after 3 seconds
    if (current_time - state_entry_time > 3000) {
        printf("\n[LINE] Could not find line - entering search mode\n");
        change_state(STATE_LINE_LOST);
    }
}

// ============================================================================
// LINE LOST HANDLING
// ============================================================================
static void handle_line_lost(void) {
    printf("[LINE] Line lost - searching...\n");
    
    // Simple search: move forward slowly
    motor_drive(M1A, M1B, -25);
    motor_drive(M2A, M2B, -25);
    
    // Check if line is found
    if (ir_line_detected()) {
        printf("[LINE] ✓ Line found!\n");
        change_state(STATE_LINE_FOLLOWING);
        return;
    }
    
    // Timeout after 5 seconds
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - state_entry_time > 5000) {
        printf("[LINE] Search timeout - stopping\n");
        change_state(STATE_STOPPED);
    }
}

// ============================================================================
// MAIN PROGRAM
// ============================================================================
int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(2000);

    // Initialize hardware
    init_hardware();
    
    // NEW: Initialize WiFi and Telemetry
    if (!init_wifi_and_telemetry()) {
        printf("\n⚠ WARNING: Telemetry not available - continuing without it\n\n");
        // Continue without telemetry
    }

    // Run calibration
    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf("               IR SENSOR CALIBRATION\n");
    printf("            Press GP20 to start calibration\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");

    while (!calibration_button_pressed()) {
        sleep_ms(10);
    }

    calibration_run_sequence();

    // Wait for start button
    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf("                Press GP20 to start robot\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");

    while (!calibration_button_pressed()) {
        sleep_ms(10);
    }

    sleep_ms(200); // Debounce

    // Start in line following mode
    change_state(STATE_LINE_FOLLOWING);
    printf("\n>>> ROBOT STARTED <<<\n\n");

    // Main loop
    uint32_t last_update = to_ms_since_boot(get_absolute_time());
    bool running = true;

    while (running) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        float dt = (current_time - last_update) / 1000.0f;

        // Clamp dt to reasonable range
        if (dt > 0.1f) dt = 0.02f;
        if (dt < 0.001f) dt = 0.001f;
        last_update = current_time;

        // Update IMU (both interfaces)
        imu_update(&imu);
        imu_helper_update();

        // NEW: Publish telemetry data
        publish_telemetry_data(current_time);

        mqtt_process();

        // State machine
        switch (current_state) {
            case STATE_IDLE:
                break;

            case STATE_LINE_FOLLOWING:
                if (!line_following_control_update(current_time, dt)) {
                    change_state(STATE_LINE_LOST);
                }

                // Periodically check for obstacles
                if (current_time - last_obstacle_check >= OBSTACLE_CHECK_INTERVAL_MS) {
                    if (check_for_obstacles()) {
                        handle_obstacle_detected();
                    }
                    last_obstacle_check = current_time;
                }
                break;

            case STATE_OBSTACLE_DETECTED:
                handle_obstacle_detected();
                break;

            case STATE_OBSTACLE_SCANNING:
                handle_obstacle_scanning();
                break;

            case STATE_OBSTACLE_AVOIDING:
                handle_obstacle_avoidance();
                break;

            case STATE_RETURNING_TO_LINE:
                handle_returning_to_line();
                break;

            case STATE_LINE_LOST:
                handle_line_lost();
                break;

            case STATE_STOPPED:
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
                if (calibration_button_pressed()) {
                    change_state(STATE_LINE_FOLLOWING);
                }
                break;
        }

        // Emergency stop button
        if (calibration_button_pressed() && current_state != STATE_STOPPED) {
            printf("\n>>> EMERGENCY STOP <<<\n");
            telemetry_publish_status("Emergency stop activated");
            change_state(STATE_STOPPED);
        }

        sleep_ms(10); // 100Hz loop
    }

    // Cleanup
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    telemetry_publish_status("Robot stopped");
    printf("\nProgram ended.\n");

    return 0;
}
