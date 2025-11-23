/**
 * barcode_control.h
 * High-level barcode control interface
 * 
 * Immediate turn execution - no junction waiting
 * 
 * Letter Mapping:
 * - A-M (odd): Turn RIGHT
 * - N-Z (even): Turn LEFT  
 * - 1-5: Speed SLOW
 * - 6-9: Speed FAST
 */

#ifndef BARCODE_CONTROL_H
#define BARCODE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "barcode_scanner.h"

// ============================================================================
// TYPES
// ============================================================================

typedef enum {
    BARCODE_ACTION_NONE,
    BARCODE_ACTION_TURN_LEFT,
    BARCODE_ACTION_TURN_RIGHT,
    BARCODE_ACTION_SPEED_SLOW,
    BARCODE_ACTION_SPEED_FAST
} BarcodeAction;

// ============================================================================
// CONFIGURATION
// ============================================================================

#define BARCODE_TURN_DURATION_MS 1000  // How long to turn for barcode commands
#define BARCODE_TURN_SPEED 35          // Turn speed for barcode turns
#define LEFT_MOTOR_COMPENSATION 2      // Add this to left motor (weaker motor)
#define BARCODE_DECODE_PAUSE_MS 1000   // Pause after decoding before executing turn

// Speed levels (adjust these to your robot's needs)
#define BARCODE_SPEED_SLOW 25          // Slow speed for line following
#define BARCODE_SPEED_FAST 45          // Fast speed for line following
#define BARCODE_SCAN_SPEED_FACTOR 0.70f  // Scan at 70% of current speed

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

/**
 * Initialize barcode control system
 * Sets up barcode scanner and initializes state
 */
void barcode_control_init(void);

/**
 * Check for barcode detection
 * Call this periodically in main loop during line following
 * 
 * @return BarcodeCommand detected, or BARCODE_CMD_NONE
 */
BarcodeCommand barcode_check_for_detection(void);

/**
 * Get the action for a barcode command
 * Converts barcode command to action (turn or speed change)
 * 
 * @param cmd The barcode command
 * @return The action to take
 */
BarcodeAction barcode_get_action(BarcodeCommand cmd);

/**
 * Handle barcode detected state
 * Stops motors and prepares for turn or processes speed change
 * 
 * @param cmd The barcode command that was detected
 * @return The action that will be taken
 */
BarcodeAction handle_barcode_detected(BarcodeCommand cmd);

/**
 * Handle barcode turn state
 * Executes the turn maneuver
 * 
 * @return true if turn is complete, false if still turning
 */
bool handle_barcode_turn(void);

/**
 * Check if currently scanning a barcode
 * Returns true while barcode scan is in progress
 * Robot automatically slows to 70% speed when scanning
 * 
 * @return true if scanning, false otherwise
 */
bool barcode_is_scanning(void);

/**
 * Check if decode pause is complete
 * Used internally by handle_barcode_turn
 * 
 * @return true if pause is done, false if still pausing
 */
bool barcode_decode_pause_complete(void);

/**
 * Get current speed setting from barcode system
 * Automatically returns 70% of current speed if currently scanning!
 * Otherwise returns the set speed (SLOW or FAST)
 * 
 * @return Current speed (automatically adjusts for scanning)
 */
int barcode_get_current_speed(void);

/**
 * Reset barcode control system
 * Call this when resuming from stopped state
 */
void barcode_control_reset(void);

#endif // BARCODE_CONTROL_H