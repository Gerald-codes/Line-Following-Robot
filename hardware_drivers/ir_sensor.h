/**
 * ir_sensor.h
 * Single IR sensor interface for edge-detection line following
 * ORIGINAL VERSION - No inversion functions
 */

#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "pico/stdlib.h"
#include <stdbool.h>

// Line position range (same as config.h for consistency)
#define LINE_POSITION_MIN -2000
#define LINE_POSITION_MAX 2000

// Typical ADC thresholds (12-bit ADC: 0-4095)
#define IR_THRESHOLD_WHITE 2600      // White surface
#define IR_THRESHOLD_BLACK 150       // Black line
#define IR_EDGE_THRESHOLD  1375     // Edge (gray) - midpoint

// Initialize IR sensors
void ir_sensor_init(void);

// Read raw ADC values
uint16_t ir_read_line_sensor(void);      // Line tracking sensor
uint16_t ir_read_barcode_sensor(void);   // Barcode detection sensor

// Get line position error for PID
// Returns: -2000 to +2000
//   Negative = line is to the right (turn right)
//   Positive = line is to the left (turn left)
//   Zero = on edge (perfect tracking)
int32_t ir_get_line_position(void);

void ir_set_max_deviation(uint16_t deviation);
// Check if line is detected
bool ir_line_detected(void);

// Check if barcode stripe detected
bool ir_barcode_detected(void);

// Calibrate sensors (interactive)
void ir_calibrate(void);

// Print current readings for debugging
void ir_print_readings(void);

// Get/set edge threshold
uint16_t ir_get_threshold(void);
void ir_set_threshold(uint16_t threshold);
// Add these function declarations to ir_sensor.h

uint16_t ir_get_threshold(void);
void ir_set_threshold(uint16_t threshold);
uint16_t ir_get_white_value(void);
uint16_t ir_get_black_value(void);
void ir_set_white_black_values(uint16_t white, uint16_t black);
bool ir_barcode_digital_detected(void);
void test_barcode_sensor(void);
#endif // IR_SENSOR_H