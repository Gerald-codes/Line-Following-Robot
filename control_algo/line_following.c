/**
 * line_following.c
 * SIMPLIFIED: Single PID controller for line following
 * No mode switching, no complexity - just smooth PID control
 */

#include "line_following.h"
#include "ir_sensor.h"
#include "pid.h"
#include "config.h"
#include <stdio.h>
#include <stdlib.h>

static PIDController line_pid;
static LineFollowState current_state = LINE_FOLLOW_IDLE;
static float filtered_position = 0.0f;

void line_following_init(void) {
    pid_init(&line_pid,
             LINE_PID_KP,
             LINE_PID_KI,
             LINE_PID_KD,
             -LINE_STEERING_MAX,
             LINE_STEERING_MAX);
    
    current_state = LINE_FOLLOW_IDLE;
    filtered_position = 0.0f;
    
    printf("✓ Line following initialized\n");
    printf("  Kp: %.3f, Ki: %.3f, Kd: %.3f\n",
           LINE_PID_KP, LINE_PID_KI, LINE_PID_KD);
    printf("  Max steering: %d mm/s\n", LINE_STEERING_MAX);
}

void line_following_reset(void) {
    pid_reset(&line_pid);
    current_state = LINE_FOLLOW_IDLE;
    filtered_position = 0.0f;
}

LineFollowState line_following_get_state(void) {
    return current_state;
}

const char* line_state_to_string(LineFollowState state) {
    switch (state) {
        case LINE_FOLLOW_IDLE:
            return "IDLE";
        case LINE_FOLLOW_ON_EDGE:
            return "On Edge";
        case LINE_FOLLOW_SLIGHT_LEFT:
            return "Left";
        case LINE_FOLLOW_SLIGHT_RIGHT:
            return "Right";
        case LINE_FOLLOW_TURN_LEFT:
            return "Sharp Left";
        case LINE_FOLLOW_TURN_RIGHT:
            return "Sharp Right";
        case LINE_FOLLOW_LOST:
            return "Lost";
        default:
            return "UNKNOWN";
    }
}

float line_following_update(float dt) {
    // Read raw position from sensor
    int32_t raw_position = ir_get_line_position();
    
    // Apply low-pass filter to smooth out noise
    // 0.7 = keep 70% of old value, 0.3 = add 30% of new value
    filtered_position = 0.7f * filtered_position + 0.3f * (float)raw_position;
    
    // Update display state based on filtered position
    int32_t abs_pos = abs((int32_t)filtered_position);
    if (abs_pos < 200) {
        current_state = LINE_FOLLOW_ON_EDGE;
    } else if (abs_pos < 800) {
        current_state = (filtered_position > 0) ? LINE_FOLLOW_SLIGHT_LEFT : LINE_FOLLOW_SLIGHT_RIGHT;
    } else if (abs_pos < 1500) {
        current_state = (filtered_position > 0) ? LINE_FOLLOW_TURN_LEFT : LINE_FOLLOW_TURN_RIGHT;
    } else {
        current_state = LINE_FOLLOW_LOST;
    }
    
    // Compute PID - target is 0 (on the edge)
    pid_set_target(&line_pid, 0);
    float steering = pid_compute(&line_pid, -filtered_position, dt);
    
    // Clamp steering to max limit
    if (steering > LINE_STEERING_MAX) steering = LINE_STEERING_MAX;
    if (steering < -LINE_STEERING_MAX) steering = -LINE_STEERING_MAX;
    
    // Debug output every 500ms
    static uint32_t last_debug = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_debug > 500) {
        printf("PID: raw=%+5ld, filt=%+6.0f, steer=%+5.1f\n", 
               raw_position, filtered_position, steering);
        last_debug = current_time;
    }
    
    return steering;
}