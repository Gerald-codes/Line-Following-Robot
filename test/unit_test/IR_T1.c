/**
 * @file    IR_T1.c
 * @brief   IR_T1: Unit test for digital barcode IR (GP6).
 *
 * @details
 *  - Verifies BLACK vs WHITE detection based on pin level.
 *  - Verifies detection stays stable over a simulated time interval
 *    when the surface stays BLACK or WHITE.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "ir_sensor.h"

/* --------------------------------------------------------------------------
 * Mocks and Stubs for Hardware (digital only)
 * -------------------------------------------------------------------------- */

// Mock state for barcode IR digital pin (GP6)
static bool mock_gpio_pin6_state = false;

// --- GPIO stubs ---
void gpio_init(uint gpio)          { (void)gpio; }
void gpio_set_dir(uint gpio, bool out) { (void)gpio; (void)out; }
bool gpio_get(uint gpio)          { (void)gpio; return mock_gpio_pin6_state; }

// Time/sleep stubs for simulated duration
static uint32_t fake_time_ms = 0;
uint32_t to_ms_since_boot(absolute_time_t t) { (void)t; return fake_time_ms; }
absolute_time_t get_absolute_time(void)      { absolute_time_t t; return t; }
void sleep_ms(uint32_t ms)                   { fake_time_ms += ms; }

// Dummy ADC stubs (not used here, just to satisfy linker if needed)
void adc_init(void) {}
void adc_gpio_init(uint gpio) { (void)gpio; }
void adc_select_input(uint input) { (void)input; }
uint16_t adc_read(void) { return 0; }

// We do NOT override ir_read_line_sensor() here because this test
// is ONLY for the digital barcode path (ir_barcode_digital_detected).

/* --------------------------------------------------------------------------
 * Assert Macros
 * -------------------------------------------------------------------------- */
#define ASSERT_TRUE(cond, msg) \
    do { if (!(cond)) { printf("  FAIL: %s\n", msg); return 1; } \
         else { printf("  PASS: %s\n", msg); } } while(0)

#define ASSERT_EQ_UINT(exp, act, msg) \
    do { if ((uint32_t)(exp) != (uint32_t)(act)) { \
            printf("  FAIL: %s (expected=%u, got=%u)\n", msg, (unsigned)(exp), (unsigned)(act)); \
            return 1; \
         } else { \
            printf("  PASS: %s (value=%u)\n", msg, (unsigned)(act)); \
         } } while(0)

/* --------------------------------------------------------------------------
 * Main Test Entry Point
 * -------------------------------------------------------------------------- */
int main(void) {
    printf("=== IR_T1 (Barcode Digital IR) ===\n");
    printf("Test: ir_barcode_digital_detected() using GPIO level on GP6\n\n");

    /* -------------------- Case 1: WHITE surface -------------------- */
    // Assumption from your ir_sensor.c:
    //   ir_barcode_digital_detected() returns !gpio_get(BARCODE_SENSOR_PIN)
    // So:
    //   GP6 HIGH  -> !HIGH = false  -> no black (WHITE)
    //   GP6 LOW   -> !LOW  = true   -> black detected

    mock_gpio_pin6_state = true;  // GP6 = HIGH
    ASSERT_TRUE(!ir_barcode_digital_detected(),
                "Case 1: GP6 HIGH -> WHITE (no black detected)");

    /* -------------------- Case 2: BLACK stripe -------------------- */
    mock_gpio_pin6_state = false; // GP6 = LOW
    ASSERT_TRUE(ir_barcode_digital_detected(),
                "Case 2: GP6 LOW -> BLACK detected");

    /* -------------------- Case 3: Duration on BLACK -------------------- */
    printf("\n--- Duration test: BLACK then WHITE (simulated) ---\n");
    fake_time_ms = 0;

    // Stay on BLACK for 150 ms
    mock_gpio_pin6_state = false; // LOW = black
    uint32_t t_black_start = fake_time_ms;

    while (fake_time_ms - t_black_start < 150) {
        ASSERT_TRUE(ir_barcode_digital_detected(),
                    "During BLACK interval: still detecting black");
        sleep_ms(10);  // advance fake time
    }
    uint32_t t_black_end = fake_time_ms;
    ASSERT_EQ_UINT(150u, t_black_end - t_black_start,
                   "Simulated time on BLACK = 150 ms");

    /* -------------------- Case 4: Duration on WHITE -------------------- */
    // Now swap to WHITE and stay for 200 ms
    mock_gpio_pin6_state = true;  // HIGH = white
    uint32_t t_white_start = fake_time_ms;

    while (fake_time_ms - t_white_start < 200) {
        ASSERT_TRUE(!ir_barcode_digital_detected(),
                    "During WHITE interval: not detecting black");
        sleep_ms(10);
    }
    uint32_t t_white_end = fake_time_ms;
    ASSERT_EQ_UINT(200u, t_white_end - t_white_start,
                   "Simulated time on WHITE = 200 ms");

    printf("\nIR_T1 (Barcode Digital IR) COMPLETED\n");
    return 0;
}
