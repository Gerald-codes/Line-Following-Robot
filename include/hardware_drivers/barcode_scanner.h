/**
 * barcode_scanner.h
 * Interrupt-based Code 39 barcode scanner module
 * 
 * Provides a clean interface for barcode scanning with automatic
 * direction detection from Code 39 barcodes.
 */

#ifndef BARCODE_SCANNER_H
#define BARCODE_SCANNER_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// PUBLIC TYPES
// ============================================================================

/**
 * Barcode command type - represents the decoded direction
 */
typedef enum {
    BARCODE_CMD_NONE,      // No command/still scanning
    BARCODE_CMD_LEFT,      // Turn left (letters: B, D, F, H, J, L, N, P, R, T, V, X, Z)
    BARCODE_CMD_RIGHT,     // Turn right (letters: A, C, E, G, I, K, M, O, Q, S, U, W, Y)
    BARCODE_CMD_UNKNOWN    // Barcode scanned but couldn't decode
} BarcodeCommand;

/**
 * Scanner state - for external monitoring if needed
 */
typedef enum {
    BARCODE_SCAN_IDLE,         // Ready to scan
    BARCODE_SCAN_IN_PROGRESS,  // Currently scanning
    BARCODE_SCAN_COMPLETE,     // Scan finished, decoding
    BARCODE_SCAN_COOLDOWN      // Cooldown period after scan
} BarcodeScanState;

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

/**
 * Initialize the barcode scanner
 * 
 * Sets up GPIO interrupt on the barcode sensor pin.
 * Must be called before using any other barcode functions.
 * 
 * @param barcode_pin GPIO pin number for the IR barcode sensor
 */
void barcode_scanner_init(uint8_t barcode_pin);

/**
 * Update the barcode scanner (call in main loop)
 * 
 * Checks for scan completion/timeout and decodes barcodes.
 * Returns BARCODE_CMD_NONE during scanning or cooldown.
 * 
 * @return BarcodeCommand - the decoded direction or NONE
 */
BarcodeCommand barcode_scanner_update(void);

/**
 * Reset the scanner state
 * 
 * Clears any in-progress scans and resets to IDLE.
 * Useful for recovering from errors or reinitializing.
 */
void barcode_scanner_reset(void);

/**
 * Get current scanner state
 * 
 * @return Current BarcodeScanState
 */
BarcodeScanState barcode_scanner_get_state(void);

/**
 * Get the last decoded character
 * This is useful for handling numbers and special characters
 * that don't map to LEFT/RIGHT commands
 * 
 * @return The last decoded character, or '?' if none
 */
char barcode_scanner_get_last_character(void);

/**
 * Check if scanner is ready for a new scan
 * 
 * @return true if in IDLE state, false otherwise
 */
bool barcode_scanner_is_ready(void);

/**
 * Convert BarcodeCommand to string (for debugging)
 * 
 * @param cmd The command to convert
 * @return String representation of the command
 */
const char* barcode_command_to_string(BarcodeCommand cmd);

#endif // BARCODE_SCANNER_H