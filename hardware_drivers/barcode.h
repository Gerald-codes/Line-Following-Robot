/**
 * barcode.h
 * Code 39 barcode detection and command decoding
 */

#ifndef BARCODE_H
#define BARCODE_H

#include "pico/stdlib.h"
#include <stdbool.h>

// Barcode commands
typedef enum {
    BARCODE_NONE = 0,
    BARCODE_LEFT,
    BARCODE_RIGHT,
    BARCODE_STOP,
    BARCODE_UNKNOWN
} BarcodeCommand;

// Barcode detection state
typedef enum {
    BARCODE_STATE_IDLE,
    BARCODE_STATE_DETECTING,
    BARCODE_STATE_COMPLETE,
    BARCODE_STATE_COOLDOWN
} BarcodeState;

// Initialize barcode decoder
void barcode_init(void);

// Reset barcode decoder
void barcode_reset(void);

// Update barcode detection (call regularly)
// Returns: Latest detected command (or BARCODE_NONE if no new command)
BarcodeCommand barcode_update(void);

// Get last detected command
BarcodeCommand barcode_get_last_command(void);

// Clear last command
void barcode_clear_command(void);

// Get current detection state
BarcodeState barcode_get_state(void);

// Convert command to string
const char* barcode_command_to_string(BarcodeCommand cmd);

// Get decoded barcode string (for debugging)
const char* barcode_get_decoded_string(void);

#endif // BARCODE_H