/**
 * calibration.h
 * SIMPLIFIED: Just button handling and basic calibration
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "pico/stdlib.h"
#include <stdbool.h>

// Button configuration
#define CALIBRATION_BUTTON_PIN 20
#define BUTTON_DEBOUNCE_MS 200

/**
 * Initialize calibration button
 */
void calibration_init(void);

/**
 * Check if calibration button was pressed
 * Returns true on button press (with debouncing)
 */
bool calibration_button_pressed(void);

/**
 * Run the calibration sequence
 * - Prompts user to place sensor on white surface
 * - Prompts user to place sensor on black surface
 * - Calculates threshold and max deviation
 * - Updates IR sensor with calibrated values
 */
void calibration_run_sequence(void);


#endif // CALIBRATION_H