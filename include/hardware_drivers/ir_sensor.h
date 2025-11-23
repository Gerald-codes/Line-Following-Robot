/**
 * @file    ir_sensor.h
 * @brief   Single IR sensor interface for edge-detection line following
 * @details
 *   Original version. No inversion functions. Provides IR sensor routines
 *   for line following, barcode detection, calibration, and debugging.
 */

#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "pico/stdlib.h"
#include <stdbool.h>

/**
 * @brief   Line position range (same as config.h for consistency)
 */
#define LINE_POSITION_MIN   -2000
#define LINE_POSITION_MAX    2000

/**
 * @brief   Typical ADC thresholds (12-bit ADC: 0-4095)
 */
#define IR_THRESHOLD_WHITE  2600    /**< White surface */
#define IR_THRESHOLD_BLACK   150    /**< Black line */
#define IR_EDGE_THRESHOLD   1375    /**< Edge (gray) - midpoint */

/**
 * @brief   Initialize IR sensors (call before reading)
 */
void ir_sensor_init(void);

/**
 * @brief   Read raw ADC value from line tracking sensor
 * @return  ADC value (0-4095)
 */
uint16_t ir_read_line_sensor(void);

/**
 * @brief   Read raw ADC value from barcode detection sensor
 * @return  ADC value (0-4095)
 */
uint16_t ir_read_barcode_sensor(void);

/**
 * @brief   Get line position error for PID
 * @details Returns -2000 to +2000. Negative = line to right; positive = line to left.
 * @return  Signed position error for PID control
 */
int32_t ir_get_line_position(void);

/**
 * @brief   Set maximum allowable deviation
 * @param   deviation Maximum allowable deviation in ADC units
 */
void ir_set_max_deviation(uint16_t deviation);

/**
 * @brief   Check if line is detected
 * @return  true if line detected, false otherwise
 */
bool ir_line_detected(void);

/**
 * @brief   Check if barcode stripe detected
 * @return  true if barcode detected, false otherwise
 */
bool ir_barcode_detected(void);

/**
 * @brief   Calibrate sensors interactively
 */
void ir_calibrate(void);

/**
 * @brief   Print current readings for debugging
 */
void ir_print_readings(void);

/**
 * @brief   Get edge threshold value
 * @return  ADC threshold value
 */
uint16_t ir_get_threshold(void);

/**
 * @brief   Set edge threshold value
 * @param   threshold New ADC threshold value
 */
void ir_set_threshold(uint16_t threshold);

/**
 * @brief   Get calibrated white value
 * @return  ADC value for white surface
 */
uint16_t ir_get_white_value(void);

/**
 * @brief   Get calibrated black value
 * @return  ADC value for black line
 */
uint16_t ir_get_black_value(void);

/**
 * @brief   Set white and black calibration values
 * @param   white ADC value for white surface
 * @param   black ADC value for black line
 */
void ir_set_white_black_values(uint16_t white, uint16_t black);

/**
 * @brief   Check if barcode detected (digital output)
 * @return  true if barcode detected, false otherwise
 */
bool ir_barcode_digital_detected(void);

/**
 * @brief   Test barcode sensor (diagnostic)
 */
void test_barcode_sensor(void);

#endif /* IR_SENSOR_H */
