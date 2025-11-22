/**
 * robot_controller.c
 */

#include "robot_controller.h"
#include "state_machine.h"
#include "motor_controller.h"
#include "line_following.h"
#include "obstacle_scanner.h"
#include "avoidance_maneuver.h"
#include "ir_sensor.h"
#include "ultrasonic.h"
#include "servo.h"
#include "calibration.h"
#include "config.h"
#include "pin_definitions.h"
#include <stdio.h>

// Private state variables
static uint32_t last_obstacle_check = 0;
static uint32_t line_lost_start = 0;

void robot_controller_init(void) {
    last_obstacle_check = 0;
    line_lost_start = 0;
    motor_controller_init();
}

void robot_controller_update(float dt) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    SystemState state = state_machine_get_current();
    
    switch (state) {
        case STATE_IDLE:
            // Do nothing
            break;
            
        case STATE_LINE_FOLLOWING:
            robot_handle_line_following(dt);
            
            // Periodic obstacle check
            if (current_time - last_obstacle_check >= OBSTACLE_CHECK_INTERVAL_MS) {
                robot_handle_obstacle_detection();
                last_obstacle_check = current_time;
            }
            break;
            
        case STATE_OBSTACLE_DETECTED:
            state_machine_transition(STATE_OBSTACLE_SCANNING);
            break;
            
        case STATE_OBSTACLE_SCANNING:
            robot_handle_obstacle_scanning();
            break;
            
        case STATE_OBSTACLE_AVOIDING:
            robot_handle_obstacle_avoiding();
            break;
            
        case STATE_RETURNING_TO_LINE:
            robot_handle_returning_to_line();
            break;
            
        case STATE_LINE_LOST:
            robot_handle_line_lost();
            break;
            
        case STATE_STOPPED:
            robot_handle_stopped();
            break;
    }
    
    // Emergency stop check
    if (calibration_button_pressed() && state != STATE_STOPPED) {
        robot_emergency_stop();
    }
}

void robot_handle_line_following(float dt) {
    // Update line following
    motor_controller_update_line_following(dt);
    
    // Check for line lost
    LineFollowState line_state = line_following_get_state();
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    if (line_state == LINE_FOLLOW_LOST) {
        if (line_lost_start == 0) {
            line_lost_start = current_time;
        } else if (current_time - line_lost_start > LINE_LOST_TIMEOUT_MS) {
            state_machine_transition(STATE_LINE_LOST);
            line_lost_start = 0;
        }
    } else {
        line_lost_start = 0;
    }
}

void robot_handle_obstacle_detection(void) {
    servo_set_angle(ANGLE_CENTER);
    sleep_ms(100);
    
    uint64_t distance;
    int status = ultrasonic_get_distance(TRIG_PIN, ECHO_PIN, &distance);
    
    if (status == SUCCESS && distance <= OBSTACLE_CHECK_DISTANCE_CM) {
        printf("[OBSTACLE] Detected at %llu cm - stopping\n", distance);
        motor_controller_stop_all();
        state_machine_transition(STATE_OBSTACLE_SCANNING);
    }
}

void robot_handle_obstacle_scanning(void) {
    printf("\n[OBSTACLE] Performing scan...\n");
    
    ScanResult result = scanner_perform_scan();
    scanner_print_results(result);
    
    AvoidanceDirection direction = scanner_get_best_avoidance_direction(result);
    
    if (direction == AVOID_NONE) {
        printf("[OBSTACLE] No clear path\n");
        state_machine_transition(STATE_STOPPED);
        return;
    }
    
    if (avoidance_start(direction)) {
        state_machine_transition(STATE_OBSTACLE_AVOIDING);
    } else {
        printf("[OBSTACLE] Failed to start avoidance\n");
        state_machine_transition(STATE_STOPPED);
    }
}

void robot_handle_obstacle_avoiding(void) {
    avoidance_update();
    
    if (avoidance_is_complete()) {
        if (avoidance_was_successful()) {
            printf("\n[OBSTACLE] Avoidance complete\n");
            avoidance_reset();
            state_machine_transition(STATE_RETURNING_TO_LINE);
        } else {
            printf("\n[OBSTACLE] Avoidance failed\n");
            state_machine_transition(STATE_STOPPED);
        }
    }
}

void robot_handle_returning_to_line(void) {
    if (ir_line_detected()) {
        printf("\n[LINE] Line found - resuming\n");
        state_machine_transition(STATE_LINE_FOLLOWING);
        return;
    }
    
    // Move forward slowly
    motor_controller_drive_direct(30, 30);
    
    // Timeout check
    if (state_machine_time_in_state() > 3000) {
        printf("\n[LINE] Timeout - entering search\n");
        state_machine_transition(STATE_LINE_LOST);
    }
}

void robot_handle_line_lost(void) {
    printf("[LINE] Searching...\n");
    
    if (ir_line_detected()) {
        printf("[LINE] Found!\n");
        state_machine_transition(STATE_LINE_FOLLOWING);
        return;
    }
    
    motor_controller_drive_direct(25, 25);
    
    if (state_machine_time_in_state() > 5000) {
        printf("[LINE] Search timeout\n");
        state_machine_transition(STATE_STOPPED);
    }
}

void robot_handle_stopped(void) {
    motor_controller_stop_all();
    
    if (calibration_button_pressed()) {
        state_machine_transition(STATE_LINE_FOLLOWING);
    }
}

void robot_emergency_stop(void) {
    printf("\n>>> EMERGENCY STOP <<<\n");
    motor_controller_stop_all();
    state_machine_transition(STATE_STOPPED);
}
