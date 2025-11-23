/**
 * @file    IR_T3.c
 * @brief   IR_T3: Test analog line IR on WHITE, BLACK, and THRESHOLD.
 * @details
 *   Uses a mock ADC value for the line sensor.
 *   Verifies ir_line_detected() behaviour for:
 *     - pure WHITE (calibrated white_value)
 *     - pure BLACK (calibrated black_value)
 *     - edge/THRESHOLD (line_threshold)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "ir_sensor.h"

/* Mock ADC reading for line sensor */
static uint16_t mock_line_adc = 0;

/* ADC stub implementations (unused in actual logic) */
void adc_init(void) {}
void adc_gpio_init(uint gpio) { (void)gpio; }
void adc_select_input(uint input) { (void)input; }
uint16_t adc_read(void) { return 0; }

/**
 * @brief   Override line sensor read to use the mock value
 * @return  ADC value for test case
 */
uint16_t ir_read_line_sensor(void) { return mock_line_adc; }

/* Minimal GPIO/time stubs to satisfy ir_sensor.c dependencies */
void gpio_init(uint gpio) { (void)gpio; }
void gpio_set_dir(uint gpio, bool out) { (void)gpio; (void)out; }
bool gpio_get(uint gpio) { (void)gpio; return false; }
uint32_t to_ms_since_boot(absolute_time_t t) { (void)t; return 0; }
absolute_time_t get_absolute_time(void) { absolute_time_t t; return t; }
void sleep_ms(uint32_t ms) { (void)ms; }

/* Assert macros */
#define ASSERT_TRUE(cond, msg) \
    do { if (!(cond)) { printf("  FAIL: %s\n", msg); return 1; } \
         else { printf("  PASS: %s\n", msg); } } while(0)

#define ASSERT_FALSE(cond, msg) \
    do { if (cond) { printf("  FAIL: %s\n", msg); return 1; } \
         else { printf("  PASS: %s\n", msg); } } while(0)

/**
 * @brief   Main test entry point for IR_T3
 * @return  0 if successful, 1 if any test fails
 */
int main(void) {
    printf("=== IR_T3: Line IR analog (WHITE vs BLACK vs THRESHOLD) ===\n");

    uint16_t white = ir_get_white_value();
    uint16_t black = ir_get_black_value();
    uint16_t thr   = ir_get_threshold();

    printf("  Calibration: white=%u, black=%u, threshold=%u\n", white, black, thr);

    /* Case 1: PURE WHITE surface */
    mock_line_adc = white;
    ASSERT_FALSE(ir_line_detected(), "Line sensor on WHITE: ir_line_detected() == false");

    /* Case 2: PURE BLACK line */
    mock_line_adc = black;
    ASSERT_FALSE(ir_line_detected(), "Line sensor on BLACK: ir_line_detected() == false (not on edge)");

    /* Case 3: NEAR THRESHOLD (edge between white and black) */
    mock_line_adc = thr;
    ASSERT_TRUE(ir_line_detected(), "Line sensor at THRESHOLD: ir_line_detected() == true (on edge)");

    printf("\nIR_T3 COMPLETED\n");
    return 0;
}
