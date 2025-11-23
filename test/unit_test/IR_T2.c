/**
 * @file    IR_T2.c
 * @brief   IR_T2: Test IR line position normalization and range logic.
 * @details Mocks IR ADC input and checks normalized position output.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "ir_sensor.h"

static uint16_t mock_line_adc = 0;

void adc_init(void) {}
void adc_gpio_init(uint gpio) {}
void adc_select_input(uint input) { (void)input; }
uint16_t adc_read(void) { return 0; }
void gpio_init(uint gpio) { (void)gpio; }
void gpio_set_dir(uint gpio, bool out) { (void)gpio; (void)out; }
bool gpio_get(uint gpio) { (void)gpio; return false; }
uint32_t to_ms_since_boot(absolute_time_t t) { (void)t; return 0; }
absolute_time_t get_absolute_time(void) { absolute_time_t t; return t; }
void sleep_ms(uint32_t ms) { (void)ms; }
uint16_t ir_read_line_sensor(void) { return mock_line_adc; }

#define ASSERT_IN_RANGE(val, min, max, msg) \
    do { if ((val) < (min) || (val) > (max)) { \
            printf("  FAIL: %s (value=%ld, range=[%d,%d])\n", msg, (long)(val), (min), (max)); \
            return 1; \
         } else { \
            printf("  PASS: %s (value=%ld)\n", msg, (long)(val)); \
         } } while(0)

#define ASSERT(cond, msg) \
    do { if (!(cond)) { printf("  FAIL: %s\n", msg); return 1; } \
         else { printf("  PASS: %s\n", msg); } } while(0)

int main(void) {
    printf("=== IR_T2: ir_get_line_position() normalization ===\n");
    ir_set_white_black_values(2600, 150);
    uint16_t thr = ir_get_threshold();
    uint16_t white = ir_get_white_value();
    uint16_t black = ir_get_black_value();

    printf("  Using white=%u, black=%u, threshold=%u\n", white, black, thr);

    mock_line_adc = thr;
    int32_t pos_thr = ir_get_line_position();
    ASSERT_IN_RANGE(pos_thr, -200, 200, "Position at threshold ~ 0");

    mock_line_adc = (white + thr) / 2;
    int32_t pos_white_side = ir_get_line_position();
    ASSERT(pos_white_side > 0, "Reading between white and threshold -> positive position");

    mock_line_adc = (black + thr) / 2;
    int32_t pos_black_side = ir_get_line_position();
    ASSERT(pos_black_side < 0, "Reading between threshold and black -> negative position");

    mock_line_adc = white;
    int32_t pos_ext_white = ir_get_line_position();
    ASSERT_IN_RANGE(pos_ext_white, 0, LINE_POSITION_MAX, "Reading at white -> positive side range");

    mock_line_adc = black;
    int32_t pos_ext_black = ir_get_line_position();
    ASSERT_IN_RANGE(pos_ext_black, LINE_POSITION_MIN, 0, "Reading at black -> negative side range");

    printf("IR_T2 COMPLETED\n");
    return 0;
}
