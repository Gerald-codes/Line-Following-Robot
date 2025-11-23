/**
 * @file    barcode_control.h
 * @brief   High-level barcode control interface
 * @details Immediate turn execution - no junction waiting. Letter mapping:
 *          A-M (odd) turn RIGHT, N-Z (even) turn LEFT, 1-5 speed SLOW,
 *          6-9 speed FAST
 */

#ifndef BARCODE_CONTROL_H
#define BARCODE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "barcode_scanner.h"



typedef enum
{
    BARCODE_ACTION_NONE,
    BARCODE_ACTION_TURN_LEFT,
    BARCODE_ACTION_TURN_RIGHT,
    BARCODE_ACTION_SPEED_SLOW,
    BARCODE_ACTION_SPEED_FAST
} BarcodeAction;

/* CONFIGURATION */

#define BARCODE_TURN_DURATION_MS    1000        /* How long to turn for barcode commands */
#define BARCODE_TURN_SPEED          35          /* Turn speed for barcode turns */
#define LEFT_MOTOR_COMPENSATION     2           /* Add this to left motor (weaker motor) */
#define BARCODE_DECODE_PAUSE_MS     1000        /* Pause after decoding before executing turn */

/* Speed levels (adjust these to your robot's needs) */
#define BARCODE_SPEED_SLOW          25          /* Slow speed for line following */
#define BARCODE_SPEED_FAST          45          /* Fast speed for line following */
#define BARCODE_SCAN_SPEED_FACTOR   0.70f       /* Scan at 70% of current speed */

/* PUBLIC FUNCTIONS */

/**
 * @brief Initialize barcode control system
 * @details Sets up barcode scanner and initializes state
 */
void barcode_control_init(void);

/**
 * @brief Check for barcode detection
 * @details Call this periodically in main loop during line following
 * @return BarcodeCommand detected, or BARCODE_CMD_NONE
 */
BarcodeCommand barcode_check_for_detection(void);

/**
 * @brief Get the action for a barcode command
 * @details Converts barcode command to action (turn or speed change)
 * @param cmd The barcode command
 * @return The action to take
 */
BarcodeAction barcode_get_action(BarcodeCommand cmd);

/**
 * @brief Handle barcode detected state
 * @details Stops motors and prepares for turn or processes speed change
 * @param cmd The barcode command that was detected
 * @return The action that will be taken
 */
BarcodeAction handle_barcode_detected(BarcodeCommand cmd);

/**
 * @brief Handle barcode turn state
 * @details Executes the turn maneuver
 * @return true if turn is complete, false if still turning
 */
bool handle_barcode_turn(void);

/**
 * @brief Check if currently scanning a barcode
 * @details Returns true while barcode scan is in progress.
 *          Robot automatically slows to 70% speed when scanning
 * @return true if scanning, false otherwise
 */
bool barcode_is_scanning(void);

/**
 * @brief Check if decode pause is complete
 * @details Used internally by handle_barcode_turn
 * @return true if pause is done, false if still pausing
 */
bool barcode_decode_pause_complete(void);

/**
 * @brief Get current speed setting from barcode system
 * @details Automatically returns 70% of current speed if currently scanning!
 *          Otherwise returns the set speed (SLOW or FAST)
 * @return Current speed (automatically adjusts for scanning)
 */
int barcode_get_current_speed(void);

/**
 * @brief Reset barcode control system
 * @details Call this when resuming from stopped state
 */
void barcode_control_reset(void);

#endif /* BARCODE_CONTROL_H */
