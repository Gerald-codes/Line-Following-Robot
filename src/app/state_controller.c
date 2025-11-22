/**
 * state_machine.c
 */

#include "state_machine.h"
#include "telemetry.h"
#include "pico/stdlib.h"
#include <stdio.h>

static SystemState current_state = STATE_IDLE;
static SystemState previous_state = STATE_IDLE;
static uint32_t state_entry_time = 0;

void state_machine_init(void) {
    current_state = STATE_IDLE;
    previous_state = STATE_IDLE;
    state_entry_time = to_ms_since_boot(get_absolute_time());
}

SystemState state_machine_get_current(void) {
    return current_state;
}

SystemState state_machine_get_previous(void) {
    return previous_state;
}

void state_machine_transition(SystemState new_state) {
    if (new_state != current_state) {
        previous_state = current_state;
        current_state = new_state;
        state_entry_time = to_ms_since_boot(get_absolute_time());
        
        printf("\n>>> STATE CHANGE: %s -> %s\n", 
               state_machine_get_name(previous_state),
               state_machine_get_name(new_state));
        
        if (telemetry_is_connected()) {
            char msg[64];
            snprintf(msg, sizeof(msg), "State: %s", 
                     state_machine_get_name(new_state));
            telemetry_publish_status(msg);
        }
    }
}

uint32_t state_machine_time_in_state(void) {
    return to_ms_since_boot(get_absolute_time()) - state_entry_time;
}

const char* state_machine_get_name(SystemState state) {
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
