/**
 * line_following.c
 * Line following control using single IR sensor edge detection
 */

#include "line_following.h"
#include "ir_sensor.h"
#include "pid.h"
#include "config.h"
#include <stdio.h>
#include <math.h>

static PIDController line_pid;
static LineFollowState current_state = LINE_STATE_SEARCHING;
static int32_t last_known_position = 0;
static uint32_t line_lost_time = 0;

void line_following_init(void) {
    // Initialize line following PID
    pid_init(&line_pid,
             LINE_PID_KP,
             LINE_PID_KI,
             LINE_PID_KD,
             -LINE_STEERING_MAX,
             LINE_STEERING_MAX);
    
    pid_set_target(&line_pid, 0);  // Target is edge (position = 0)
    
    current_state = LINE_STATE_SEARCHING;
    last_known_position = 0;
    
    printf("✓ Line following initialized\n");
    printf("  PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
           LINE_PID_KP, LINE_PID_KI, LINE_PID_KD);
}

float line_following_update(float dt) {
    int32_t line_position = ir_get_line_position();
    bool line_detected = ir_line_detected();
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    float steering_correction = 0.0f;
    
    switch (current_state) {
        case LINE_STATE_FOLLOWING:
            if (line_detected) {
                // Normal line following - PID on edge
                steering_correction = pid_compute(&line_pid, line_position, dt);
                last_known_position = line_position;
                
            } else {
                // Line lost!
                current_state = LINE_STATE_LOST;
                line_lost_time = current_time;
            }
            break;
            
        case LINE_STATE_LOST:
            // Try to recover - turn in direction of last known position
            if (line_detected) {
                // Found line again!
                current_state = LINE_STATE_FOLLOWING;
                pid_reset(&line_pid);
                
            } else {
                // Keep searching - turn toward last known position
                if (last_known_position > 0) {
                    steering_correction = LINE_STEERING_MAX * 0.5f;  // Turn left
                } else {
                    steering_correction = -LINE_STEERING_MAX * 0.5f; // Turn right
                }
                
                // Give up after timeout
                if (current_time - line_lost_time > SAFETY_LINE_LOST_TIMEOUT_MS) {
                    current_state = LINE_STATE_STOPPED;
                }
            }
            break;
            
        case LINE_STATE_SEARCHING:
            // Initial search - slow oscillation
            if (line_detected) {
                current_state = LINE_STATE_FOLLOWING;
                pid_reset(&line_pid);
            } else {
                // Oscillate slowly to find line
                float search_amplitude = LINE_STEERING_MAX * 0.3f;
                float search_frequency = 0.5f;  // Hz
                steering_correction = search_amplitude * 
                    sinf(2.0f * 3.14159f * search_frequency * current_time / 1000.0f);
            }
            break;
            
        case LINE_STATE_STOPPED:
            // Stopped due to line lost timeout
            steering_correction = 0.0f;
            break;
            
        case LINE_STATE_TURNING:
            // Controlled turn (used for barcode commands)
            // Steering handled externally
            break;
    }
    
    return steering_correction;
}

void line_following_set_state(LineFollowState state) {
    current_state = state;
    
    if (state == LINE_STATE_FOLLOWING || state == LINE_STATE_SEARCHING) {
        pid_reset(&line_pid);
    }
}

LineFollowState line_following_get_state(void) {
    return current_state;
}

void line_following_reset(void) {
    pid_reset(&line_pid);
    current_state = LINE_STATE_SEARCHING;
    last_known_position = 0;
}

const char* line_state_to_string(LineFollowState state) {
    switch (state) {
        case LINE_STATE_FOLLOWING: return "FOLLOWING";
        case LINE_STATE_LOST:      return "LOST";
        case LINE_STATE_SEARCHING: return "SEARCHING";
        case LINE_STATE_STOPPED:   return "STOPPED";
        case LINE_STATE_TURNING:   return "TURNING";
        default:                   return "UNKNOWN";
    }
}
