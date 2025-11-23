// test_ir_line_position.c
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "ir_sensor.h"

// ===== MOCKS / STUBS =====

static uint16_t mock_line_adc = 0;

// ADC/GPIO/time stubs as before (minimal)
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

// Override the line sensor read to return our mock ADC
uint16_t ir_read_line_sensor(void) {
    return mock_line_adc;
}

// ===== ASSERTS =====
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
    printf("=== UT-IR-002: ir_get_line_position() normalization ===\n");

    // Use calibration API to match your ir_sensor.c behavior
    // Set explicit white/black so we know ranges
    ir_set_white_black_values(2600, 150);
    uint16_t thr = ir_get_threshold();
    uint16_t white = ir_get_white_value();
    uint16_t black = ir_get_black_value();

    printf("  Using white=%u, black=%u, threshold=%u\n", white, black, thr);

    // 1) Reading exactly at threshold -> position around 0
    mock_line_adc = thr;
    int32_t pos_thr = ir_get_line_position();
    ASSERT_IN_RANGE(pos_thr, -200, 200, "Position at threshold ~ 0");

    // 2) Reading closer to white side (reading < threshold)
    // In your code: reading < threshold → positive position (turn LEFT)
    mock_line_adc = (white + thr) / 2;  // midpoint between white and threshold
    int32_t pos_white_side = ir_get_line_position();
    ASSERT(pos_white_side > 0, "Reading between white and threshold -> positive position");

    // 3) Reading closer to black side (reading >= threshold)
    // In your code: reading >= threshold → negative position (turn RIGHT)
    mock_line_adc = (black + thr) / 2;  // midpoint between threshold and black
    int32_t pos_black_side = ir_get_line_position();
    ASSERT(pos_black_side < 0, "Reading between threshold and black -> negative position");

    // 4) Extreme white reading -> near +LINE_POSITION_MAX
    mock_line_adc = white;
    int32_t pos_ext_white = ir_get_line_position();
    ASSERT_IN_RANGE(pos_ext_white, 0, LINE_POSITION_MAX, "Reading at white -> positive side range");

    // 5) Extreme black reading -> near -LINE_POSITION_MAX
    mock_line_adc = black;
    int32_t pos_ext_black = ir_get_line_position();
    ASSERT_IN_RANGE(pos_ext_black, LINE_POSITION_MIN, 0, "Reading at black -> negative side range");

    printf("UT-IR-002 COMPLETED\n");
    return 0;
}
