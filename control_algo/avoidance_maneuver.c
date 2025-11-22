#include "avoidance_maneuver.h"
#include "motor.h"
#include "servo.h"
#include "ultrasonic.h"
#include "timer_manager.h"
#include "telemetry.h"
#include "imu.h"
#include <stdio.h>
#include <math.h>
#include "ir_sensor.h"


// Global context
static AvoidanceContext context;

// Motor speed parameters
#define SPEED_FORWARD 40
#define SPEED_TURN_OUTER 45
#define SPEED_TURN_INNER 15

void avoidance_init(void) {
    context.direction = AVOID_NONE;
    context.state = AVOIDANCE_IDLE;
    context.state_start_time = 0;
    context.obstacle_cleared = false;
    context.forward_duration_ms = FORWARD_AVOID_DURATION_MS;
    printf("[AVOIDANCE] System initialized\n");
}

void avoidance_set_forward_duration(uint32_t duration_ms) {
    context.forward_duration_ms = duration_ms;
}

const char* avoidance_get_state_string(AvoidanceState state) {
    switch (state) {
        case AVOIDANCE_IDLE: return "IDLE";
        case AVOIDANCE_TURNING_OFF_LINE: return "TURNING_OFF_LINE";
        case AVOIDANCE_MOVING_PARALLEL: return "MOVING_PARALLEL";
        case AVOIDANCE_CHECKING_CLEARANCE: return "CHECKING_CLEARANCE";
        case AVOIDANCE_TURNING_BACK: return "TURNING_BACK";
        case AVOIDANCE_REALIGN_FORWARD: return "REALIGN_FORWARD";
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
    imu_helper_start_avoidance();

    // SAVE THE ORIGINAL HEADING
    context.original_heading = imu_helper_get_relative_heading();
    printf("[AVOIDANCE] Original heading saved: %.1f°\n", context.original_heading);
    
    const char* dir_str = (direction == AVOID_LEFT) ? "LEFT" : "RIGHT";
    printf("[AVOIDANCE] Starting maneuver - Direction: %s\n", dir_str);
    
    // NEW: Publish avoidance start
    if (telemetry_is_ready()) {
        telemetry_publish_avoidance(direction, context.state, false);
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
    return false; // Assume not clear on error
}

AvoidanceState avoidance_update(void) {
    if (context.state == AVOIDANCE_IDLE ||
        context.state == AVOIDANCE_COMPLETE ||
        context.state == AVOIDANCE_FAILED) {
        return context.state;
    }

    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    uint32_t elapsed = current_time - context.state_start_time;
    
    // NEW: Track state changes for telemetry
    static AvoidanceState last_published_state = AVOIDANCE_IDLE;
    
    switch (context.state) {
        case AVOIDANCE_TURNING_OFF_LINE:
            printf("[AVOIDANCE] State: Turning off line (%s)...\n",
                   (context.direction == AVOID_LEFT) ? "LEFT" : "RIGHT");
            
            // Update IMU reading
            imu_helper_update();
            float current_turn = imu_helper_get_relative_heading();
            float target = (context.direction == AVOID_LEFT) ? TURN_ANGLE_LEFT_DEG : -TURN_ANGLE_RIGHT_DEG;
            printf("[AVOIDANCE] Current turn: %.1f°, Target: %.1f°\n", current_turn, target);
            
            // Turn away from line (45 degrees)
            if (context.direction == AVOID_LEFT) {
                printf("DEBUG: TURN_ANGLE_LEFT_DEG macro value: %d, target: %.1f\n", TURN_ANGLE_LEFT_DEG, target);
                motor_drive(M1A, M1B, -SPEED_TURN_OUTER);
                motor_drive(M2A, M2B, -SPEED_TURN_INNER);
            } else {
                motor_drive(M1A, M1B, -SPEED_TURN_INNER);
                motor_drive(M2A, M2B, -SPEED_TURN_OUTER);
            }
            
            // Check if turn is complete using IMU
            if (imu_helper_has_turned(target, 5.0f)) {
                printf("[AVOIDANCE] Turn complete (IMU-verified), moving parallel...\n");
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
                context.state = AVOIDANCE_MOVING_PARALLEL;
                context.state_start_time = current_time;
                
                // NEW: Publish state change
                if (telemetry_is_ready() && last_published_state != context.state) {
                    telemetry_publish_avoidance(context.direction, context.state, context.obstacle_cleared);
                    last_published_state = context.state;
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
            if (elapsed >= context.forward_duration_ms) {
                printf("[AVOIDANCE] Checking if obstacle cleared...\n");
                context.state = AVOIDANCE_CHECKING_CLEARANCE;
                context.state_start_time = current_time;
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
            }
            break;
            
        case AVOIDANCE_CHECKING_CLEARANCE:
            printf("[AVOIDANCE] State: Checking clearance...\n");
            
            if (avoidance_check_obstacle_cleared()) {
                context.obstacle_cleared = true;
                printf("[AVOIDANCE] Obstacle cleared! Turning back to line...\n");
                context.state = AVOIDANCE_TURNING_BACK;
                context.state_start_time = current_time;
                
                // NEW: Publish obstacle cleared status
                if (telemetry_is_ready()) {
                    telemetry_publish_avoidance(context.direction, context.state, context.obstacle_cleared);
                    last_published_state = context.state;
                }
            } else {
                printf("[AVOIDANCE] Not cleared, continuing...\n");
                context.state = AVOIDANCE_MOVING_PARALLEL;
                context.state_start_time = current_time;
            }
            break;
            
        case AVOIDANCE_TURNING_BACK:
            printf("[AVOIDANCE] State: Turning back toward line...\n");
            
            // Update IMU reading
            imu_helper_update();
            float current_turn_back = imu_helper_get_relative_heading();
            
            float target_turn_back;
            if (context.direction == AVOID_LEFT) {
                target_turn_back = -TURN_BACK_LEFT_DEG;
            } else {
                target_turn_back = TURN_BACK_RIGHT_DEG;
            }
            
            printf("[AVOIDANCE] Current heading: %.1f° (target: %.1f°)\n", current_turn_back, target_turn_back);
            
            // Turn back toward line (opposite direction)
            if (context.direction == AVOID_LEFT) {
                motor_drive(M1A, M1B, -SPEED_TURN_INNER);
                motor_drive(M2A, M2B, -SPEED_TURN_OUTER);
            } else {
                motor_drive(M1A, M1B, -SPEED_TURN_OUTER);
                motor_drive(M2A, M2B, -SPEED_TURN_INNER);
            }
            
            // Check if we're back to original heading
            if (fabsf(current_turn_back - target_turn_back) <= 10.0f) {
                printf("[AVOIDANCE] Turn complete (IMU-verified), searching for line...\n");
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
                context.state = AVOIDANCE_REALIGN_FORWARD;
                context.state_start_time = current_time;
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
            
        case AVOIDANCE_REALIGN_FORWARD:
            printf("[AVOIDANCE] State: Final realign forward (elapsed: %lu ms)...\n", elapsed);
            motor_drive(M1A, M1B, -SPEED_FORWARD);
            motor_drive(M2A, M2B, -SPEED_FORWARD);
            
            if (elapsed >= FORWARD_AVOID_DURATION_MS) {
                printf("[AVOIDANCE] Foward complete. Searching for line...\n");
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
                context.state = AVOIDANCE_SEARCHING_LINE;
                context.state_start_time = current_time;
            }
            break;
            
        case AVOIDANCE_SEARCHING_LINE:
            printf("[AVOIDANCE] State: Searching for line (elapsed: %lu ms)...\n", elapsed);
            
            // Actively check for line using IR or line following module!
            if (ir_line_detected()) {   // <-- Replace with your detection function
                printf("[AVOIDANCE] ✓ Line detected! Stopping for realignment.\n");
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);

                // Transition to heading realignment state. If you control this in main.c, set a flag or context.
                context.state = AVOIDANCE_CENTER_ON_LINE; // (or set a global flag if state machine in main.c)
                context.state_start_time = current_time;

                if (telemetry_is_connected()) {
                    telemetry_publish_status("Line found - realigning to original heading");
                }
                break; // Stop further processing of elapsed timeout
            }

            // Timeout fallback for searching (if line not found in time)
            if (elapsed >= SEARCH_LINE_DURATION_MS) {
                printf("[AVOIDANCE] Search complete - should be near line now\n");
                context.state = AVOIDANCE_COMPLETE;

                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);

                printf("[AVOIDANCE] ✓ Maneuver complete!\n");

                              
                // NEW: Publish completion
                if (telemetry_is_ready()) {
                    telemetry_publish_avoidance(context.direction, context.state, context.obstacle_cleared);
                    last_published_state = context.state;
                }
            }
            break;
        case AVOIDANCE_CENTER_ON_LINE:
            printf("[AVOIDANCE] Centering on line (elapsed: %lu ms)...\n", elapsed);

            // Move forward slowly for a short, fixed time (e.g., 400-800 ms, tune to your robot)
            motor_drive(M1A, M1B, -SPEED_FORWARD);
            motor_drive(M2A, M2B, -SPEED_FORWARD);
            if (elapsed >= 2500) { // 500 ms center, tune as needed
                printf("[AVOIDANCE] Centering complete. Ready to realign heading.\n");
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);

                context.state = AVOIDANCE_REALIGN_TO_HEADING;
                context.state_start_time = current_time;
            }
            break;
        case AVOIDANCE_REALIGN_TO_HEADING:
            printf("[AVOIDANCE] Realigning to original heading...\n");
            imu_helper_update();
            float current_heading = imu_helper_get_relative_heading();

            float target_heading = context.original_heading;
            float error = current_heading - target_heading;
            while (error > 180.0f) error -= 360.0f;
            while (error < -180.0f) error += 360.0f;

            // Turn until aligned
            if (fabsf(error) <= 5.0f) {
                printf("[AVOIDANCE] Heading aligned!\n");
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
                context.state = AVOIDANCE_COMPLETE; // Or hand-off to line following
            } else if (error > 0) {
                // Turn right
                motor_drive(M1A, M1B, -SPEED_TURN_INNER);
                motor_drive(M2A, M2B, -SPEED_TURN_OUTER);
            } else {
                // Turn left
                motor_drive(M1A, M1B, -SPEED_TURN_OUTER);
                motor_drive(M2A, M2B, -SPEED_TURN_INNER);
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


float avoidance_get_original_heading(void) {
    return context.original_heading;
}
