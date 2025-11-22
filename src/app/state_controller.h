/**
 * state_machine.h
 * 
 * Core state machine for robot navigation
 * Coordinates transitions between operational modes
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    STATE_IDLE,
    STATE_LINE_FOLLOWING,
    STATE_OBSTACLE_DETECTED,
    STATE_OBSTACLE_SCANNING,
    STATE_OBSTACLE_AVOIDING,
    STATE_RETURNING_TO_LINE,
    STATE_LINE_LOST,
    STATE_STOPPED
} SystemState;

// Initialize state machine
void state_machine_init(void);

// Get current state
SystemState state_machine_get_current(void);
SystemState state_machine_get_previous(void);

// State transitions
void state_machine_transition(SystemState new_state);

// Get time in current state
uint32_t state_machine_time_in_state(void);

// State name utilities
const char* state_machine_get_name(SystemState state);

#endif // STATE_MACHINE_H
