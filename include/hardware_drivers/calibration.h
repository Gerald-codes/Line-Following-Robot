/**
 * @file    calibration.h
 * @brief   Simplified: Just button handling and basic calibration
 * @details
 *   Interface for calibration button and basic sensor calibration.
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "pico/stdlib.h"
#include <stdbool.h>

/**
 * @brief   Button configuration
 */
#define CALIBRATION_BUTTON_PIN 20
#define BUTTON_DEBOUNCE_MS 200

/**
 * @brief   Initialize calibration button
 */
void calibration_init(void);

/**
 * @brief   Check if calibration button was pressed
 * @details Returns true on button press (with debouncing)
 * @return  true when the button is pressed, false otherwise
 */
bool calibration_button_pressed(void);

/**
 * @brief   Run the calibration sequence
 * @details
 *   Prompts user to place sensor on white surface.
 *   Prompts user to place sensor on black surface.
 *   Calculates threshold and max deviation.
 *   Updates IR sensor with calibrated values.
 */
void calibration_run_sequence(void);

#endif /* CALIBRATION_H */
