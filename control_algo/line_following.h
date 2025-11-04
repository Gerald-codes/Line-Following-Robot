/**
 * line_following.h
 * Line following controller interface
 */

#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#include "pico/stdlib.h"
#include <stdbool.h>

// Line following states
typedef enum {
    LINE_FOLLOW_IDLE = 0,
    LINE_FOLLOW_ON_EDGE,
    LINE_FOLLOW_SLIGHT_LEFT,
    LINE_FOLLOW_SLIGHT_RIGHT,
    LINE_FOLLOW_TURN_LEFT,
    LINE_FOLLOW_TURN_RIGHT,
    LINE_FOLLOW_LOST
} LineFollowState;

// Initialize line following controller
void line_following_init(void);

// Update line following (call every control cycle)
// Returns: steering correction value (negative = turn right, positive = turn left)
float line_following_update(float dt);

// Reset line following state
void line_following_reset(void);

// Get current state
LineFollowState line_following_get_state(void);

// Convert state to string for debugging
const char* line_state_to_string(LineFollowState state);

#endif // LINE_FOLLOWING_H