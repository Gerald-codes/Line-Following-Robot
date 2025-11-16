#include "avoidance_maneuver.h"
#include "motor.h"
#include "servo.h"
#include "ultrasonic.h"
#include "timer_manager.h"
#include "telemetry.h"
#include "imu_helper.h"
#include <stdio.h>
#include <math.h>

// Global context
static AvoidanceContext context;

// Motor speed parameters
#define SPEED_FORWARD 40
#define SPEED_TURN_OUTER 50
#define SPEED_TURN_INNER 20

void avoidance_init(void) {
    context.direction = AVOID_NONE;
    context.state = AVOIDANCE_IDLE;
    context.state_start_time = 0;
    context.obstacle_cleared = false;
    
    printf("[AVOIDANCE] System initialized\n");
}

const char* avoidance_get_state_string(AvoidanceState state) {
    switch (state) {
        case AVOIDANCE_IDLE: return "IDLE";
        case AVOIDANCE_TURNING_OFF_LINE: return "TURNING_OFF_LINE";
        case AVOIDANCE_MOVING_PARALLEL: return "MOVING_PARALLEL";
        case AVOIDANCE_CHECKING_CLEARANCE: return "CHECKING_CLEARANCE";
        case AVOIDANCE_TURNING_BACK: return "TURNING_BACK";
        case AVOIDANCE_SEARCHING_LINE: return "SEARCHING_LINE";
        case AVOIDANCE_COMPLETE: return "COMPLETE";
        case AVOIDANCE_FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}

bool avoidance_start(AvoidanceDirection direction) {
    if (direction == AVOID_NONE) {
        printf("[AVOIDANCE] No avoidance needed\n");
        return false;
    }
    
    context.direction = direction;
    context.state = AVOIDANCE_TURNING_OFF_LINE;
    context.state_start_time = to_ms_since_boot(get_absolute_time());
    context.obstacle_cleared = false;
    
    // Reset IMU heading for this maneuver
    imu_start_avoidance();
    
    const char* dir_str = (direction == AVOID_LEFT) ? "LEFT" : "RIGHT";
    printf("[AVOIDANCE] Starting maneuver - Direction: %s\n", dir_str);
    
    if (telemetry_is_connected()) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Starting avoidance: %s", dir_str);
        telemetry_publish_status(msg);
    }
    
    return true;
}

bool avoidance_check_obstacle_cleared(void) {
    // Point servo forward
    servo_set_angle(ANGLE_CENTER);
    timer_wait_ms(100);
    
    // Check distance ahead
    uint64_t distance;
    int status = ultrasonic_get_distance(TRIG_PIN, ECHO_PIN, &distance);
    
    if (status == SUCCESS) {
        printf("[AVOIDANCE] Distance check: %llu cm ", distance);
        if (distance > OBSTACLE_CLEAR_DISTANCE) {
            printf("✓ CLEAR\n");
            return true;
        } else {
            printf("⚠️ Still blocked\n");
            return false;
        }
    }
    
    printf("(sensor error)\n");
    return false;  // Assume not clear on error
}

AvoidanceState avoidance_update(void) {
    if (context.state == AVOIDANCE_IDLE || 
        context.state == AVOIDANCE_COMPLETE || 
        context.state == AVOIDANCE_FAILED) {
        return context.state;
    }
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    uint32_t elapsed = current_time - context.state_start_time;
    
    switch (context.state) {
        case AVOIDANCE_TURNING_OFF_LINE:
            printf("[AVOIDANCE] State: Turning off line (%s)...\n", 
                   (context.direction == AVOID_LEFT) ? "LEFT" : "RIGHT");
            
            // Update IMU reading
            imu_update();
            float current_turn = imu_get_relative_heading();
            float target = (context.direction == AVOID_LEFT) ? 45.0f : -45.0f;
            
            printf("[AVOIDANCE] Current turn: %.1f°, Target: %.1f°\n", current_turn, target);
            
            // Turn away from line (45 degrees)
            if (context.direction == AVOID_LEFT) {
                // Turn left: slow left motor, fast right motor
                motor_drive(M1A, M1B, -SPEED_TURN_INNER);
                motor_drive(M2A, M2B, -SPEED_TURN_OUTER);
            } else {
                // Turn right: fast left motor, slow right motor
                motor_drive(M1A, M1B, -SPEED_TURN_OUTER);
                motor_drive(M2A, M2B, -SPEED_TURN_INNER);
            }
            
            // Check if turn is complete using IMU
            if (imu_has_turned(target, 5.0f)) {  // 5° tolerance
                printf("[AVOIDANCE] Turn complete (IMU-verified), moving parallel...\n");
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
                context.state = AVOIDANCE_MOVING_PARALLEL;
                context.state_start_time = current_time;
                
                if (telemetry_is_connected()) {
                    telemetry_publish_status("Moving parallel to obstacle");
                }
            }
            
            // Fallback: timeout after max duration
            if (elapsed >= TURN_DURATION_MS * 2) {
                printf("[AVOIDANCE] Turn timeout (fallback to time-based)\n");
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
                context.state = AVOIDANCE_MOVING_PARALLEL;
                context.state_start_time = current_time;
            }
            break;
            
        case AVOIDANCE_MOVING_PARALLEL:
            printf("[AVOIDANCE] State: Moving parallel (elapsed: %lu ms)...\n", elapsed);
            
            // Move forward parallel to line
            motor_drive(M1A, M1B, -SPEED_FORWARD);
            motor_drive(M2A, M2B, -SPEED_FORWARD);
            
            // After moving forward for a bit, check if obstacle is cleared
            if (elapsed >= FORWARD_AVOID_DURATION_MS) {
                printf("[AVOIDANCE] Checking if obstacle cleared...\n");
                context.state = AVOIDANCE_CHECKING_CLEARANCE;
                context.state_start_time = current_time;
                
                // Stop to check
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
            }
            break;
            
        case AVOIDANCE_CHECKING_CLEARANCE:
            printf("[AVOIDANCE] State: Checking clearance...\n");
            
            // Check if obstacle is cleared
            if (avoidance_check_obstacle_cleared()) {
                context.obstacle_cleared = true;
                printf("[AVOIDANCE] Obstacle cleared! Turning back to line...\n");
                context.state = AVOIDANCE_TURNING_BACK;
                context.state_start_time = current_time;
                
                if (telemetry_is_connected()) {
                    telemetry_publish_status("Obstacle cleared, returning to line");
                }
            } else {
                // Not cleared yet, continue moving forward
                printf("[AVOIDANCE] Not cleared, continuing...\n");
                context.state = AVOIDANCE_MOVING_PARALLEL;
                context.state_start_time = current_time;
            }
            break;
            
        case AVOIDANCE_TURNING_BACK:
            printf("[AVOIDANCE] State: Turning back toward line...\n");
            
            // Update IMU reading
            imu_update();
            float current_turn_back = imu_get_relative_heading();
            printf("[AVOIDANCE] Current heading: %.1f° (target: ~0°)\n", current_turn_back);
            
            // Turn back toward line (opposite direction)
            if (context.direction == AVOID_LEFT) {
                // Turn right to come back
                motor_drive(M1A, M1B, -SPEED_TURN_OUTER);
                motor_drive(M2A, M2B, -SPEED_TURN_INNER);
            } else {
                // Turn left to come back
                motor_drive(M1A, M1B, -SPEED_TURN_INNER);
                motor_drive(M2A, M2B, -SPEED_TURN_OUTER);
            }
            
            // Check if we're back to original heading (within 10° tolerance)
            if (fabsf(current_turn_back) <= 10.0f) {
                printf("[AVOIDANCE] Turn complete (IMU-verified), searching for line...\n");
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
                context.state = AVOIDANCE_SEARCHING_LINE;
                context.state_start_time = current_time;
                
                if (telemetry_is_connected()) {
                    telemetry_publish_status("Searching for line");
                }
            }
            
            // Fallback: timeout
            if (elapsed >= RETURN_TURN_DURATION_MS * 2) {
                printf("[AVOIDANCE] Return turn timeout (fallback)\n");
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
                context.state = AVOIDANCE_SEARCHING_LINE;
                context.state_start_time = current_time;
            }
            break;
            
        case AVOIDANCE_SEARCHING_LINE:
            printf("[AVOIDANCE] State: Searching for line (elapsed: %lu ms)...\n", elapsed);
            
            // Move forward slowly to find line
            motor_drive(M1A, M1B, -SPEED_FORWARD * 0.7);
            motor_drive(M2A, M2B, -SPEED_FORWARD * 0.7);
            
            // TODO: Your teammate's line follower will detect line here
            // For now, just move forward for a set duration
            if (elapsed >= SEARCH_LINE_DURATION_MS) {
                printf("[AVOIDANCE] Search complete - should be near line now\n");
                context.state = AVOIDANCE_COMPLETE;
                
                // Stop motors
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
                
                printf("[AVOIDANCE] ✓ Maneuver complete!\n");
                
                if (telemetry_is_connected()) {
                    telemetry_publish_status("Avoidance complete - ready for line following");
                }
            }
            break;
            
        default:
            break;
    }
    
    return context.state;
}

bool avoidance_is_complete(void) {
    return (context.state == AVOIDANCE_COMPLETE || 
            context.state == AVOIDANCE_FAILED);
}

bool avoidance_was_successful(void) {
    return (context.state == AVOIDANCE_COMPLETE && context.obstacle_cleared);
}

void avoidance_reset(void) {
    context.direction = AVOID_NONE;
    context.state = AVOIDANCE_IDLE;
    context.state_start_time = 0;
    context.obstacle_cleared = false;
    
    // Make sure motors are stopped
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    printf("[AVOIDANCE] System reset\n");
}