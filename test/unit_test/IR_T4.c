/**
 * @file    IR_T4.c
 * @brief   IR_T4: Test IR calibration value storage and threshold logic.
 * @details Tests storing and fetching of white, black, and threshold.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "ir_sensor.h"
#include "calibration.h"

void adc_init(void) {}
void adc_gpio_init(uint gpio) { (void)gpio; }
void adc_select_input(uint input) { (void)input; }
uint16_t adc_read(void) { return 0; }
void gpio_init(uint gpio) { (void)gpio; }
void gpio_set_dir(uint gpio, bool out) { (void)gpio; (void)out; }
bool gpio_get(uint gpio) { (void)gpio; return false; }
uint32_t to_ms_since_boot(absolute_time_t t) { (void)t; return 0; }
absolute_time_t get_absolute_time(void) { absolute_time_t t; return t; }
void sleep_ms(uint32_t ms) { (void)ms; }
void calibration_init(void) {}
bool calibration_button_pressed(void) { return false; }
void calibration_run_sequence(void) {}

#define ASSERT_EQUAL(exp, act, msg) \
    do { if ((exp) != (act)) { \
            printf("  FAIL: %s (expected=%d, got=%d)\n", msg, (int)(exp), (int)(act)); \
            return 1; \
         } else { \
            printf("  PASS: %s (value=%d)\n", msg, (int)(act)); \
         } } while(0)

int main(void) {
    printf("=== IR_T4: Calibration value logic (ir_* API) ===\n");

    // Set calibration values
    ir_set_white_black_values(3000, 200);

    // Check stored white/black values
    ASSERT_EQUAL(3000, ir_get_white_value(), "White value stored correctly");
    ASSERT_EQUAL(200,  ir_get_black_value(), "Black value stored correctly");

    // Check threshold = midpoint
    uint16_t expected_thr = (3000 + 200) / 2;
    ASSERT_EQUAL(expected_thr, ir_get_threshold(), "Threshold set to midpoint after calibration");

    // Check manual override of threshold
    ir_set_threshold(1800);
    ASSERT_EQUAL(1800, ir_get_threshold(), "Manual threshold override works");

    printf("IR_T4 COMPLETED\n");
    return 0;
}
