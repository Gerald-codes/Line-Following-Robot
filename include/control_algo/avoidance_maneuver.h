/**
 * @file    avoidance_maneuver.h
 * @brief   Obstacle avoidance maneuver controller
 * @details Implements state machine for obstacle avoidance including turning
 *          off line, moving parallel, checking clearance, and returning to
 *          line with heading realignment
 */

#ifndef AVOIDANCE_MANEUVER_H
#define AVOIDANCE_MANEUVER_H

#include <stdbool.h>
#include "obstacle_scanner.h"

/* ========================================================================== */
/* MANEUVER PARAMETERS                                                        */
/* ========================================================================== */

#define TURN_ANGLE_DEG              45      /* Degrees to turn when leaving line */
#define TURN_ANGLE_RIGHT_DEG        50      /* For right avoidance */
#define TURN_ANGLE_LEFT_DEG         45      /* For left avoidance */
#define TURN_BACK_LEFT_DEG          60      /* Turn-back angle for left avoidance */
#define TURN_BACK_RIGHT_DEG         50      /* Turn-back angle for right avoidance */
#define TURN_DURATION_MS            3000    /* Time to turn 45 degrees */
#define FORWARD_AVOID_DURATION_MS   100     /* Time to move forward past obstacle */
#define RETURN_TURN_DURATION_MS     3000    /* Time to turn back toward line */
#define SEARCH_LINE_DURATION_MS     4000    /* Time to search for line */
#define OBSTACLE_CLEAR_DISTANCE     35      /* cm - distance at which obstacle is cleared */

/* ========================================================================== */
/* AVOIDANCE STATES                                                           */
/* ========================================================================== */

typedef enum
{
    AVOIDANCE_IDLE,
    AVOIDANCE_TURNING_OFF_LINE,         /* Initial turn away from line */
    AVOIDANCE_MOVING_PARALLEL,          /* Moving parallel to line, past obstacle */
    AVOIDANCE_CHECKING_CLEARANCE,       /* Checking if obstacle is cleared */
    AVOIDANCE_TURNING_BACK,             /* Turning back toward line */
    AVOIDANCE_REALIGN_FORWARD,
    AVOIDANCE_SEARCHING_LINE,           /* Moving forward to find line */
    AVOIDANCE_CENTER_ON_LINE,
    AVOIDANCE_COMPLETE,
    AVOIDANCE_REALIGN_TO_HEADING,
    AVOIDANCE_FAILED
} AvoidanceState;

/* ========================================================================== */
/* MANEUVER CONTEXT                                                           */
/* ========================================================================== */

typedef struct
{
    AvoidanceDirection direction;       /* Which way to avoid (LEFT or RIGHT) */
    AvoidanceState state;               /* Current state in maneuver */
    uint32_t state_start_time;          /* When current state started */
    bool obstacle_cleared;              /* Flag for obstacle clearance */
    uint32_t forward_duration_ms;
    float original_heading;             /* Heading when avoidance started */
} AvoidanceContext;

/* ========================================================================== */
/* INITIALIZATION AND CONTROL                                                 */
/* ========================================================================== */

/**
 * @brief Initialize avoidance maneuver system
 */
void avoidance_init(void);

/**
 * @brief Start obstacle avoidance maneuver
 * @param direction Which direction to avoid (from scanner_get_best_avoidance_direction)
 * @return true if maneuver started successfully
 */
bool avoidance_start(AvoidanceDirection direction);

/**
 * @brief Update avoidance maneuver
 * @details Call repeatedly in main loop
 * @return Current state of maneuver
 */
AvoidanceState avoidance_update(void);

/**
 * @brief Reset avoidance system to idle state
 */
void avoidance_reset(void);

/* ========================================================================== */
/* STATUS QUERIES                                                             */
/* ========================================================================== */

/**
 * @brief Check if avoidance maneuver is complete
 * @return true if complete (success or failure)
 */
bool avoidance_is_complete(void);

/**
 * @brief Check if avoidance maneuver succeeded
 * @return true if successfully returned to line position
 */
bool avoidance_was_successful(void);

/**
 * @brief Check if obstacle is cleared (by scanning forward)
 * @return true if path ahead is clear
 */
bool avoidance_check_obstacle_cleared(void);

/**
 * @brief Get the original heading saved when avoidance started
 * @return Original heading in degrees
 */
float avoidance_get_original_heading(void);

/* ========================================================================== */
/* CONFIGURATION                                                              */
/* ========================================================================== */

/**
 * @brief Set forward movement duration for parallel movement phase
 * @param duration_ms Duration in milliseconds
 */
void avoidance_set_forward_duration(uint32_t duration_ms);

/* ========================================================================== */
/* UTILITIES                                                                  */
/* ========================================================================== */

/**
 * @brief Get current avoidance state as string
 * @details For debugging purposes
 * @param state AvoidanceState value
 * @return String representation of state
 */
const char* avoidance_get_state_string(AvoidanceState state);

#endif /* AVOIDANCE_MANEUVER_H */
