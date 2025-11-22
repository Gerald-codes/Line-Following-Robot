/**
 * line_following.h - Single IR Line Following Interface
 */

#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// LINE FOLLOWING STATE
// ============================================================================
typedef enum {
    LINE_FOLLOW_CENTERED,      // On the edge (target position)
    LINE_FOLLOW_LEFT,          // Slightly left of edge
    LINE_FOLLOW_RIGHT,         // Slightly right of edge
    LINE_FOLLOW_FAR_LEFT,      // Far left - need strong correction
    LINE_FOLLOW_FAR_RIGHT,     // Far right - need strong correction
    LINE_FOLLOW_LOST           // Line completely lost
} LineFollowState;

// ============================================================================
// INITIALIZATION
// ============================================================================
void line_following_init(void);

// ============================================================================
// CONTROL UPDATE
// ============================================================================
/**
 * Update PID controller based on current IR reading
 * 
 * @param dt Time step in seconds
 * @return Steering correction value (normalized, typically -2 to +2)
 *         Positive = steer right, Negative = steer left
 */
float line_following_update(float dt);

// ============================================================================
// RESET
// ============================================================================
void line_following_reset_integral(void);

// ============================================================================
// GAIN ADJUSTMENT (for adaptive control)
// ============================================================================
void line_following_set_kp(float kp);
void line_following_set_ki(float ki);
void line_following_set_kd(float kd);

// ============================================================================
// GETTERS
// ============================================================================
LineFollowState line_following_get_state(void);
float line_following_get_filtered_pos(void);  // Returns normalized error [-1, +1]
float line_following_get_error(void);         // Returns current error
float line_following_get_output(void);        // Returns PID output

// ============================================================================
// UTILITY
// ============================================================================
const char* line_state_to_string(LineFollowState state);

void update_line_following(uint32_t current_time, float dt);
#endif // LINE_FOLLOWING_H