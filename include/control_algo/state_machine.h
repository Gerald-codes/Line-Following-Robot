/**
 * @file    state_machine.h
 * @brief   System state machine definitions
 * @details Defines system states and state transition functions for
 *          robot control system including line following, barcode detection,
 *          and obstacle avoidance
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

typedef enum
{
    STATE_IDLE,
    STATE_LINE_FOLLOWING,
    STATE_BARCODE_DETECTED,
    STATE_BARCODE_TURN,
    STATE_OBSTACLE_DETECTED,
    STATE_OBSTACLE_SCANNING,
    STATE_OBSTACLE_AVOIDING,
    STATE_RETURNING_TO_LINE,
    STATE_REALIGNING_HEADING,
    STATE_LINE_LOST,
    STATE_STOPPED
} SystemState;

/**
 * @brief Change system state
 * @param new_state The state to transition to
 */
void change_state(SystemState new_state);

/**
 * @brief Get current system state
 * @return Current SystemState
 */
SystemState get_current_state(void);

#endif /* STATE_MACHINE_H */
