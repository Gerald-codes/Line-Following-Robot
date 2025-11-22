/**
 * barcode_scanner.h
 * Interrupt-based barcode scanner for Code39
 * Uses GPIO interrupt to capture black/white bar timing
 */

#ifndef BARCODE_SCANNER_H
#define BARCODE_SCANNER_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_BAR_COUNT 100           // Maximum bars to capture
#define MIN_BAR_DURATION_US 1000    // Minimum bar duration (1ms)

// Bar structure to store timing data
typedef struct {
    uint32_t duration_us;    // Duration in microseconds
    bool is_black;           // true = black, false = white
} BarData;

// Barcode scan result
typedef struct {
    bool valid;              // Was decode successful?
    char message[64];        // Decoded message
    uint8_t bar_count;       // Number of bars captured
    uint32_t total_time_ms;  // Total scan time
} BarcodeResult;

/**
 * Initialize barcode scanner
 * @param gpio_pin GPIO pin for barcode sensor
 */
void barcode_init(uint8_t gpio_pin);

/**
 * Start barcode scanning
 * Enables interrupt-based capture
 */
void barcode_start_scan(void);

/**
 * Stop barcode scanning
 * Disables interrupts
 */
void barcode_stop_scan(void);

/**
 * Check if scan is complete
 * @return true if enough bars captured or timeout
 */
bool barcode_is_scan_complete(void);

/**
 * Get current bar count
 * @return number of bars captured so far
 */
uint8_t barcode_get_bar_count(void);

/**
 * Decode captured barcode
 * @param result Pointer to result structure
 * @return true if decode successful
 */
bool barcode_decode(BarcodeResult* result);

/**
 * Reset scanner for next scan
 */
void barcode_reset(void);

/**
 * Print captured bar data for debugging
 */
void barcode_print_bars(void);

#endif // BARCODE_SCANNER_H