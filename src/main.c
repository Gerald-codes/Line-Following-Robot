/**
 * @file    main.c
 * @brief   Integrated robot control system
 * @details Combines line following, obstacle avoidance, barcode scanning,
 *          and real-time telemetry with state machine coordination.
 *          Features: single IR line tracking, ultrasonic obstacle detection,
 *          barcode detection/turning, WiFi/MQTT telemetry publishing
 */

/* Includes - Standard library */
#include "pico/stdlib.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

/* Includes - State management */
#include "state_machine.h"

/* Includes - Hardware drivers */
#include "motor.h"
#include "encoder.h"
#include "encoder_utils.h"
#include "imu.h"
#include "ir_sensor.h"
#include "ultrasonic.h"
#include "servo.h"

/* Includes - Control systems */
#include "obstacle_control.h"
#include "barcode_control.h"

/* Includes - Control algorithms */
#include "line_following.h"
#include "obstacle_scanner.h"
#include "avoidance_maneuver.h"

/* Includes - Utilities */
#include "timer_manager.h"
#include "calibration.h"
#include "config.h"
#include "pin_definitions.h"

/* Includes - WiFi and Telemetry */
#include "wifi.h"
#include "telemetry.h"
#include "barcode_scanner.h"
#include "mqtt_client.h"

/* System state machine */
static SystemState current_state = STATE_IDLE;
static SystemState previous_state = STATE_IDLE;

/* Configuration - Obstacle detection */
#define OBSTACLE_CHECK_DISTANCE_CM  20
#define OBSTACLE_CHECK_INTERVAL_MS  500
#define CRITICAL_DISTANCE_CM        15

/* Global state */
static IMU imu;
static uint32_t last_obstacle_check = 0;
static uint32_t state_entry_time = 0;

/* Telemetry tracking */
static uint32_t last_telemetry_time = 0;
static RobotState prev_robot_state = ROBOT_STATE_IDLE;
static uint32_t robot_state_enter_time = 0;

/* Barcode telemetry tracking */
static BarcodeCommand last_published_barcode = BARCODE_CMD_NONE;

/* Function prototypes */
static void init_hardware(void);
static bool init_wifi_and_telemetry(void);
static void publish_telemetry_data(uint32_t current_time);
static void handle_returning_to_line(void);
static void handle_line_lost(void);
static void handle_realigning_heading(void);
static const char* state_to_string(SystemState state);
static RobotState system_state_to_robot_state(SystemState state);
static ObstacleState get_obstacle_state(void);

/**
 * @brief Get current system state
 * @return Current SystemState
 */
SystemState get_current_state(void)
{
    return current_state;
}

/**
 * @brief Initialize all hardware components
 */
static void init_hardware(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║   INTEGRATED LINE + OBSTACLE + BARCODE SYSTEM                ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    printf("Initializing hardware...\n");

    /* Motors */
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    printf(" ✓ Motors\n");

    /* Encoders */
    encoder_init();
    printf(" ✓ Encoders\n");

    /* IMU (struct-based for line following) */
    imu_init(&imu);
    imu_calibrate(&imu);
    printf(" ✓ IMU (struct)\n");

    /* IMU Helper (for avoidance maneuvers) */
    imu_helper_init();
    printf(" ✓ IMU Helper\n");

    /* IR Sensors */
    ir_sensor_init();
    printf(" ✓ IR Sensors\n");

    /* Ultrasonic */
    ultrasonic_init(TRIG_PIN, ECHO_PIN);
    printf(" ✓ Ultrasonic\n");

    /* Servo */
    servo_init(SERVO_PIN);
    servo_set_angle(ANGLE_CENTER);
    printf(" ✓ Servo\n");

    /* Calibration button */
    calibration_init();
    printf(" ✓ Calibration Button (GP20)\n");

    /* Line following controller */
    line_following_init();
    printf("  ✓ Line Following Controller\n");
    
    /* Barcode control system */
    barcode_control_init();
    
    /* Obstacle systems */
    scanner_init();
    avoidance_init();
    timer_manager_init();
    printf(" ✓ Obstacle Detection Systems\n");

    printf("\n✓ All hardware initialized\n\n");
}

/**
 * @brief Initialize WiFi and telemetry system
 * @return true if successful, false otherwise
 */
static bool init_wifi_and_telemetry(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║ WiFi & Telemetry Initialization                              ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");

    /* Initialize WiFi */
    wifi_init();
    printf("Connecting to WiFi: %s\n", WIFI_SSID);
    
    if (!wifi_connect(WIFI_SSID, WIFI_PASSWORD))
    {
        printf("✗ Failed to connect to WiFi\n");
        return false;
    }
    
    printf("✓ WiFi connected\n");
    
    /* Get IP address */
    char ip_buf[16];
    if (wifi_get_ip(ip_buf, sizeof(ip_buf)))
    {
        printf("  IP Address: %s\n", ip_buf);
    }

    /* Initialize telemetry */
    printf("Connecting to MQTT broker: %s:%d\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
    
    if (!telemetry_init(MQTT_BROKER_IP, MQTT_BROKER_PORT, MQTT_CLIENT_ID))
    {
        printf("✗ Failed to connect to MQTT broker\n");
        return false;
    }
    
    printf("✓ MQTT connected\n");
    printf("\n✓ Telemetry system ready\n\n");
    
    /* Publish initial status */
    telemetry_publish_status("Robot initialized and ready");
    
    return true;
}

/**
 * @brief Change system state and log transition
 * @param new_state State to transition to
 */
void change_state(SystemState new_state)
{
    if (new_state != current_state)
    {
        previous_state = current_state;
        current_state = new_state;
        state_entry_time = to_ms_since_boot(get_absolute_time());
        
        printf("\n>>> STATE CHANGE: %s -> %s\n",
               state_to_string(previous_state),
               state_to_string(new_state));
        
        /* Publish state change to telemetry */
        RobotState new_robot_state = system_state_to_robot_state(new_state);
        if (telemetry_is_ready())
        {
            uint32_t duration = to_ms_since_boot(get_absolute_time()) - robot_state_enter_time;
            telemetry_publish_state_change(prev_robot_state, new_robot_state, duration);
            prev_robot_state = new_robot_state;
            robot_state_enter_time = to_ms_since_boot(get_absolute_time());
        }
    }
}

/**
 * @brief Convert system state to string representation
 * @param state SystemState value
 * @return String representation
 */
static const char* state_to_string(SystemState state)
{
    switch (state)
    {
        case STATE_IDLE:
            return "IDLE";
        case STATE_LINE_FOLLOWING:
            return "LINE_FOLLOWING";
        case STATE_BARCODE_DETECTED:
            return "BARCODE_DETECTED";
        case STATE_BARCODE_TURN:
            return "BARCODE_TURN";
        case STATE_OBSTACLE_DETECTED:
            return "OBSTACLE_DETECTED";
        case STATE_OBSTACLE_SCANNING:
            return "OBSTACLE_SCANNING";
        case STATE_OBSTACLE_AVOIDING:
            return "OBSTACLE_AVOIDING";
        case STATE_RETURNING_TO_LINE:
            return "RETURNING_TO_LINE";
        case STATE_REALIGNING_HEADING:
            return "REALIGNING_HEADING";
        case STATE_LINE_LOST:
            return "LINE_LOST";
        case STATE_STOPPED:
            return "STOPPED";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief Convert system state to robot state for telemetry
 * @param state SystemState value
 * @return Corresponding RobotState
 */
static RobotState system_state_to_robot_state(SystemState state)
{
    switch (state)
    {
        case STATE_IDLE:
            return ROBOT_STATE_IDLE;
        case STATE_LINE_FOLLOWING:
            return ROBOT_STATE_FOLLOWING;
        case STATE_OBSTACLE_DETECTED:
            return ROBOT_STATE_STOPPED;
        case STATE_OBSTACLE_SCANNING:
            return ROBOT_STATE_SCANNING;
        case STATE_OBSTACLE_AVOIDING:
            return ROBOT_STATE_AVOIDING;
        case STATE_RETURNING_TO_LINE:
            return ROBOT_STATE_RETURNING;
        case STATE_LINE_LOST:
            return ROBOT_STATE_LOST;
        case STATE_STOPPED:
            return ROBOT_STATE_STOPPED;
        default:
            return ROBOT_STATE_ERROR;
    }
}

/**
 * @brief Get current obstacle state
 * @return ObstacleState corresponding to current system state
 */
static ObstacleState get_obstacle_state(void)
{
    switch (current_state)
    {
        case STATE_OBSTACLE_DETECTED:
            return OBSTACLE_STATE_DETECTED;
        case STATE_OBSTACLE_SCANNING:
            return OBSTACLE_STATE_SCANNING;
        case STATE_OBSTACLE_AVOIDING:
            return OBSTACLE_STATE_AVOIDING;
        case STATE_RETURNING_TO_LINE:
            return OBSTACLE_STATE_RETURNING;
        default:
            return OBSTACLE_STATE_NONE;
    }
}

/**
 * @brief Publish telemetry data at regular intervals
 * @param current_time Current system time in milliseconds
 */
static void publish_telemetry_data(uint32_t current_time)
{
    if (!telemetry_is_ready())
    {
        return;
    }
    
    /* Publish at regular interval */
    if (current_time - last_telemetry_time >= TELEMETRY_INTERVAL_MS)
    {
        /* Get IR sensor data */
        uint16_t ir_reading = ir_read_line_sensor();
        int32_t line_pos = ir_get_line_position();
        float line_pos_filtered = (float)line_pos;
        
        /* Get line state */
        LineFollowState line_state;
        if (ir_line_detected())
        {
            if (line_pos > 500)
            {
                line_state = LINE_FOLLOW_LEFT;
            }
            else if (line_pos < -500)
            {
                line_state = LINE_FOLLOW_RIGHT;
            }
            else
            {
                line_state = LINE_FOLLOW_CENTERED;
            }
        }
        else
        {
            line_state = LINE_FOLLOW_LOST;
        }
        
        /* Get current states */
        RobotState robot_state = system_state_to_robot_state(current_state);
        ObstacleState obstacle_state = get_obstacle_state();
        
        /* Publish all telemetry */
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
        
        /* Check and publish barcode detections */
        BarcodeCommand barcode_cmd = barcode_scanner_update();
        if (barcode_cmd != BARCODE_CMD_NONE && barcode_cmd != last_published_barcode)
        {
            char decoded_char = barcode_scanner_get_last_character();
            
            /* Publish to MQTT */
            telemetry_publish_barcode(barcode_cmd, decoded_char);
            
            /* Log to console */
            printf("\n");
            printf("========================================================\n");
            printf("  BARCODE TELEMETRY: '%c' -> %-23s\n", 
                   decoded_char,
                   barcode_command_to_string(barcode_cmd));
            printf("========================================================\n\n");
            
            last_published_barcode = barcode_cmd;
        }
        
        /* Reset published barcode when scanner is ready for next scan */
        if (barcode_scanner_is_ready() && last_published_barcode != BARCODE_CMD_NONE)
        {
            last_published_barcode = BARCODE_CMD_NONE;
        }
        
        last_telemetry_time = current_time;
    }
}

/**
 * @brief Handle returning to line state
 */
static void handle_returning_to_line(void)
{
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    /* Check if we found the line */
    if (ir_line_detected())
    {
        printf("\n[LINE] Line detected! Resuming normal following\n");
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        change_state(STATE_REALIGNING_HEADING);
        return;
    }
    
    /* Continue moving forward slowly to find line */
    motor_drive(M1A, M1B, -30);
    motor_drive(M2A, M2B, -30);
    
    /* Timeout after 3 seconds */
    if (current_time - state_entry_time > 3000)
    {
        printf("\n[LINE] Could not find line - entering search mode\n");
        change_state(STATE_LINE_LOST);
    }
}

/**
 * @brief Handle line lost state with search pattern
 */
static void handle_line_lost(void)
{
    printf("[LINE] Line lost - searching...\n");
    
    /* Simple search: move forward slowly */
    motor_drive(M1A, M1B, -25);
    motor_drive(M2A, M2B, -25);
    
    /* Check if line is found */
    if (ir_line_detected())
    {
        printf("[LINE] ✓ Line found!\n");
        change_state(STATE_LINE_FOLLOWING);
        return;
    }
    
    /* Timeout after 5 seconds */
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - state_entry_time > 5000)
    {
        printf("[LINE] Search timeout - stopping\n");
        change_state(STATE_STOPPED);
    }
}

/**
 * @brief Handle heading realignment after avoidance
 */
static void handle_realigning_heading(void)
{
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    /* Get saved original heading */
    float target_heading = avoidance_get_original_heading();
    
    /* Update IMU */
    imu_helper_update();
    float current_heading = imu_helper_get_relative_heading();
    
    /* Calculate heading error */
    float heading_error = current_heading - target_heading;
    
    /* Normalize to -180 to +180 */
    while (heading_error > 180.0f)
    {
        heading_error -= 360.0f;
    }
    while (heading_error < -180.0f)
    {
        heading_error += 360.0f;
    }
    
    printf("[REALIGN] Current: %.1f° | Target: %.1f° | Error: %.1f°\n", 
           current_heading, target_heading, heading_error);
    
    /* Check if aligned (within 5° tolerance) */
    if (fabsf(heading_error) <= 5.0f)
    {
        printf("[REALIGN] ✓ Heading aligned! Resuming normal line following\n");
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        change_state(STATE_LINE_FOLLOWING);
        return;
    }
    
    /* Turn to correct heading (slower, more precise) */
    int turn_speed_outer = 35;
    int turn_speed_inner = 15;
    
    if (heading_error > 0)
    {
        /* Need to turn right */
        motor_drive(M1A, M1B, -turn_speed_inner);
        motor_drive(M2A, M2B, -turn_speed_outer);
    }
    else
    {
        /* Need to turn left */
        motor_drive(M1A, M1B, -turn_speed_outer);
        motor_drive(M2A, M2B, -turn_speed_inner);
    }
    
    /* Timeout after 3 seconds */
    if (current_time - state_entry_time > 3000)
    {
        printf("[REALIGN] ⚠️ Realignment timeout - proceeding anyway\n");
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        change_state(STATE_LINE_FOLLOWING);
    }
}

/**
 * @brief Main program entry point
 * @return Exit status
 */
int main()
{
    /* Initialize stdio */
    stdio_init_all();
    sleep_ms(2000);

    /* Initialize hardware */
    init_hardware();
    
    /* Initialize WiFi and Telemetry */
    if (!init_wifi_and_telemetry())
    {
        printf("\n⚠ WARNING: Telemetry not available - continuing without it\n\n");
    }

    /* Run calibration */
    printf("\n");
    printf("===================================================================\n");
    printf("               IR SENSOR CALIBRATION\n");
    printf("            Press GP20 to start calibration\n");
    printf("===================================================================\n\n");

    while (!calibration_button_pressed())
    {
        sleep_ms(10);
    }

    calibration_run_sequence();

    /* Wait for start button */
    printf("\n");
    printf("===================================================================\n");
    printf("                Press GP20 to start robot\n");
    printf("===================================================================\n\n");

    while (!calibration_button_pressed())
    {
        sleep_ms(10);
    }

    sleep_ms(200); /* Debounce */

    /* Start in line following mode */
    change_state(STATE_LINE_FOLLOWING);
    printf("\n>>> ROBOT STARTED <<<\n\n");

    /* Main loop */
    uint32_t last_update = to_ms_since_boot(get_absolute_time());
    bool running = true;

    while (running)
    {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        float dt = (current_time - last_update) / 1000.0f;

        /* Clamp dt to reasonable range */
        if (dt > 0.1f)
        {
            dt = 0.02f;
        }
        if (dt < 0.001f)
        {
            dt = 0.001f;
        }
        last_update = current_time;

        /* Update IMU (both interfaces) */
        imu_update(&imu);
        imu_helper_update();

        /* Publish telemetry data */
        publish_telemetry_data(current_time);

        mqtt_process();

        /* State machine */
        switch (current_state)
        {
            case STATE_IDLE:
                break;

            case STATE_LINE_FOLLOWING:
                /* Check for barcode detection first (highest priority) */
                {
                    BarcodeCommand barcode_cmd = barcode_check_for_detection();
                    if (barcode_cmd != BARCODE_CMD_NONE && barcode_cmd != BARCODE_CMD_UNKNOWN)
                    {
                        BarcodeAction action = handle_barcode_detected(barcode_cmd);
                        
                        /* Handle based on action type */
                        if (action == BARCODE_ACTION_TURN_LEFT || action == BARCODE_ACTION_TURN_RIGHT)
                        {
                            /* Turn immediately */
                            change_state(STATE_BARCODE_TURN);
                            break;
                        }
                        /* Speed changes don't require state change - just continue */
                        else if (action == BARCODE_ACTION_SPEED_SLOW || action == BARCODE_ACTION_SPEED_FAST)
                        {
                            printf("[BARCODE] Speed changed, continuing line following\n");
                        }
                    }
                }
                
                /* Use the integrated line following update function */
                if (!line_following_control_update(current_time, dt))
                {
                    change_state(STATE_LINE_LOST);
                }

                /* Periodically check for obstacles */
                if (current_time - last_obstacle_check >= OBSTACLE_CHECK_INTERVAL_MS)
                {
                    if (check_for_obstacles())
                    {
                        handle_obstacle_detected();
                    }
                    last_obstacle_check = current_time;
                }
                break;
                
            case STATE_BARCODE_DETECTED:
                /* Handled inline above */
                break;
                
            case STATE_BARCODE_TURN:
                if (handle_barcode_turn())
                {
                    /* Turn complete, return to line */
                    change_state(STATE_RETURNING_TO_LINE);
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
                if (calibration_button_pressed())
                {
                    barcode_control_reset();
                    change_state(STATE_LINE_FOLLOWING);
                }
                break;

            case STATE_REALIGNING_HEADING:
                handle_realigning_heading();
                break;
        }

        /* Emergency stop button */
        if (calibration_button_pressed() && current_state != STATE_STOPPED)
        {
            printf("\n>>> EMERGENCY STOP <<<\n");
            telemetry_publish_status("Emergency stop activated");
            change_state(STATE_STOPPED);
        }

        sleep_ms(10); /* 100Hz loop */
    }

    /* Cleanup */
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    telemetry_publish_status("Robot stopped");
    printf("\nProgram ended.\n");

    return 0;
}
