#ifndef AVOIDANCE_MANEUVER_H
#define AVOIDANCE_MANEUVER_H

#include <stdbool.h>
#include "obstacle_scanner.h"

// Maneuver parameters (tunable)
#define TURN_ANGLE_DEG 45           // Degrees to turn when leaving line
#define TURN_DURATION_MS 800        // Time to turn 45 degrees
#define FORWARD_AVOID_DURATION_MS 1500  // Time to move forward past obstacle
#define RETURN_TURN_DURATION_MS 800     // Time to turn back toward line
#define SEARCH_LINE_DURATION_MS 2000    // Time to search for line
#define OBSTACLE_CLEAR_DISTANCE 35      // cm - distance at which obstacle is "cleared"

// Avoidance states
typedef enum {
    AVOIDANCE_IDLE,
    AVOIDANCE_TURNING_OFF_LINE,     // Initial turn away from line
    AVOIDANCE_MOVING_PARALLEL,      // Moving parallel to line, past obstacle
    AVOIDANCE_CHECKING_CLEARANCE,   // Checking if obstacle is cleared
    AVOIDANCE_TURNING_BACK,         // Turning back toward line
    AVOIDANCE_SEARCHING_LINE,       // Moving forward to find line
    AVOIDANCE_COMPLETE,
    AVOIDANCE_FAILED
} AvoidanceState;

// Maneuver context
typedef struct {
    AvoidanceDirection direction;    // Which way to avoid (LEFT or RIGHT)
    AvoidanceState state;            // Current state in maneuver
    uint32_t state_start_time;       // When current state started
    bool obstacle_cleared;           // Flag for obstacle clearance
} AvoidanceContext;

/**
 * Initialize avoidance maneuver system
 */
void avoidance_init(void);

/**
 * Start obstacle avoidance maneuver
 * @param direction Which direction to avoid (from scanner_get_best_avoidance_direction)
 * @return true if maneuver started successfully
 */
bool avoidance_start(AvoidanceDirection direction);

/**
 * Update avoidance maneuver (call repeatedly in main loop)
 * @return Current state of maneuver
 */
AvoidanceState avoidance_update(void);

/**
 * Check if avoidance maneuver is complete
 * @return true if complete (success or failure)
 */
bool avoidance_is_complete(void);

/**
 * Check if avoidance maneuver succeeded
 * @return true if successfully returned to line position
 */
bool avoidance_was_successful(void);

/**
 * Reset avoidance system to idle state
 */
void avoidance_reset(void);

/**
 * Get current avoidance state as string (for debugging)
 */
const char* avoidance_get_state_string(AvoidanceState state);

/**
 * Check if obstacle is cleared (by scanning forward)
 * @return true if path ahead is clear
 */
bool avoidance_check_obstacle_cleared(void);

#endif // AVOIDANCE_MANEUVER_H