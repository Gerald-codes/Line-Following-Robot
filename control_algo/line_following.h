/**
 * line_following.h
 * Line following control interface
 */

#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#include "pico/stdlib.h"

// Line following states
typedef enum {
    LINE_STATE_FOLLOWING,    // Following line edge
    LINE_STATE_LOST,         // Line lost, searching
    LINE_STATE_SEARCHING,    // Initial search for line
    LINE_STATE_STOPPED,      // Stopped (timeout or command)
    LINE_STATE_TURNING       // Executing turn command
} LineFollowState;

// Initialize line following controller
void line_following_init(void);

// Update line following control
// dt: Time since last update (seconds)
// Returns: Steering correction (-LINE_STEERING_MAX to +LINE_STEERING_MAX)
float line_following_update(float dt);

// Set line following state
void line_following_set_state(LineFollowState state);

// Get current state
LineFollowState line_following_get_state(void);

// Reset line following controller
void line_following_reset(void);

// Convert state to string
const char* line_state_to_string(LineFollowState state);

#endif // LINE_FOLLOWING_H
