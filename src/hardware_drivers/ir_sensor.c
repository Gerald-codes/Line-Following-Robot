/**
 * @file    ir_sensor.c
 * @brief   Single IR sensor driver for edge-detection line following
 * @details
 *   FOR INVERTED SENSORS (QTR-style): Black=HIGH, White=LOW.
 *   Provides position normalization for advanced PID control, supports
 *   barcode sensor digital/analog detection, and calibration routines.
 */

#include "ir_sensor.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <stdlib.h>

/* ADC channels for sensors */
#define LINE_SENSOR_ADC_CHANNEL     0
#define BARCODE_SENSOR_ADC_CHANNEL  1
#define BARCODE_SENSOR_PIN          6

/* Calibration variables (configured at runtime) */
static uint16_t line_threshold = IR_EDGE_THRESHOLD;
static uint16_t white_value    = IR_THRESHOLD_WHITE;
static uint16_t black_value    = IR_THRESHOLD_BLACK;

/**
 * @brief   Initialize IR sensors for line and barcode detection
 */
void ir_sensor_init(void) {
    adc_init();
    adc_gpio_init(26);
    gpio_init(BARCODE_SENSOR_PIN);
    gpio_set_dir(BARCODE_SENSOR_PIN, GPIO_IN);

    printf("✓ IR sensors initialized\n");
    printf("  Line sensor: GP26 (ADC0)\n");
    printf("  Barcode sensor: GP6 (D0)\n");
    printf("  Edge threshold: %d\n", line_threshold);
    printf("  White value: %d, Black value: %d\n", white_value, black_value);
    printf("  INVERTED SENSOR MODE (Black=HIGH, White=LOW)\n");
    printf("  LEFT SIDE TRACING\n");
}

/**
 * @brief   Read raw ADC value from line sensor
 * @return  ADC value
 */
uint16_t ir_read_line_sensor(void) {
    adc_select_input(LINE_SENSOR_ADC_CHANNEL);
    return adc_read();
}

/**
 * @brief   Read raw ADC value from barcode sensor
 * @return  ADC value
 */
uint16_t ir_read_barcode_sensor(void) {
    adc_select_input(BARCODE_SENSOR_ADC_CHANNEL);
    return adc_read();
}

/**
 * @brief   Return normalized position value for PID control
 * @return  Position: -LINE_POSITION_MAX to +LINE_POSITION_MAX
 */
int32_t ir_get_line_position(void) {
    uint16_t reading = ir_read_line_sensor();
    int32_t position;

    if (reading < line_threshold) {
        int32_t white_range = line_threshold - white_value;
        int32_t offset = line_threshold - reading;
        if (white_range > 0) position = (offset * LINE_POSITION_MAX) / white_range;
        else                 position = LINE_POSITION_MAX;
    } else {
        int32_t black_range = black_value - line_threshold;
        int32_t offset = reading - line_threshold;
        if (black_range > 0) position = -(offset * LINE_POSITION_MAX) / black_range;
        else                 position = -LINE_POSITION_MAX;
    }

    static uint32_t last_debug = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_debug > 500) {
        printf("IR_DEBUG: read=%d, thr=%d, white=%d, black=%d, pos_calc=%ld\n",
            reading, line_threshold, white_value, black_value, position);
        last_debug = now;
    }

    if (position > LINE_POSITION_MAX) position = LINE_POSITION_MAX;
    if (position < LINE_POSITION_MIN) position = LINE_POSITION_MIN;
    return position;
}

/**
 * @brief   Set max deviation (deprecated, kept for compatibility)
 * @param   deviation Max deviation ignored (uses calibrated values instead)
 */
void ir_set_max_deviation(uint16_t deviation) {
    printf("Info: Using calibrated white/black values for position calculation\n");
}

/**
 * @brief   Set calibration values for white/black and update threshold
 * @param   white ADC value for white surface
 * @param   black ADC value for black line
 */
void ir_set_white_black_values(uint16_t white, uint16_t black) {
    white_value = white;
    black_value = black;
    line_threshold = (white + black) / 2;
    printf("Calibration set: white=%d, black=%d, threshold=%d\n", 
        white, black, line_threshold);
}

/**
 * @brief   Check if line is detected (near edge threshold)
 * @return  true if line detected, false otherwise
 */
bool ir_line_detected(void) {
    uint16_t reading = ir_read_line_sensor();
    int32_t diff = abs((int32_t)reading - (int32_t)line_threshold);
    int32_t tolerance = abs(black_value - white_value) * 30 / 100;
    if (tolerance < 200) tolerance = 200;
    return (diff < tolerance);
}

/**
 * @brief   Test barcode sensor pin (user-driven diagnostic)
 */
void test_barcode_sensor(void) {
    sleep_ms(3000);
    printf("\n=== BARCODE SENSOR TEST ===\n");
    printf("Hold sensor over WHITE surface...\n");
    sleep_ms(3000);
    bool white = gpio_get(BARCODE_SENSOR_PIN);
    printf("White reading: %d\n", white);

    printf("Hold sensor over BLACK surface...\n");
    sleep_ms(3000);
    bool black = gpio_get(BARCODE_SENSOR_PIN);
    printf("Black reading: %d\n", black);

    if (black && !white) {
        printf("✓ Use: return gpio_get()\n");
    } else if (!black && white) {
        printf("✓ Use: return !gpio_get()\n");
    } else {
        printf("⚠ Sensor not working or always same value!\n");
    }
    printf("===========================\n\n");
}

/**
 * @brief   Check if barcode is detected on digital pin (LOW = black detected)
 * @return  true if black detected, false otherwise
 */
bool ir_barcode_digital_detected(void) {
    return !gpio_get(BARCODE_SENSOR_PIN);
}

/**
 * @brief   Compatibility: Check if barcode detected, direct digital
 * @return  true if barcode detected
 */
bool ir_barcode_detected_digital(void) {
    bool pin_state = gpio_get(BARCODE_SENSOR_PIN);
    return pin_state;
    // Or: return !pin_state;  // If sensor output is inverted
}

/**
 * @brief   Check if barcode (stripe) is detected using ADC
 * @return  true if barcode detected
 */
bool ir_barcode_detected(void) {
    uint16_t reading = ir_read_barcode_sensor();
    return (reading > IR_THRESHOLD_BLACK);
}

/**
 * @brief   Interactive calibration routine for IR sensors
 */
void ir_calibrate(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║                  IR SENSOR CALIBRATION                        ║\n");
    printf("║              (INVERTED SENSOR MODE)                           ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");

    printf("1. Place sensor over WHITE surface\n");
    printf("   Press Enter when ready...\n");
    getchar();

    uint32_t white_sum = 0;
    for (int i = 0; i < 50; i++) {
        white_sum += ir_read_line_sensor();
        sleep_ms(10);
    }
    white_value = white_sum / 50;
    printf("   White value: %d (LOW is correct for inverted sensor)\n\n", white_value);

    sleep_ms(500);

    printf("2. Place sensor over BLACK line\n");
    printf("   Press Enter when ready...\n");
    getchar();

    uint32_t black_sum = 0;
    for (int i = 0; i < 50; i++) {
        black_sum += ir_read_line_sensor();
        sleep_ms(10);
    }
    black_value = black_sum / 50;
    printf("   Black value: %d (HIGH is correct for inverted sensor)\n\n", black_value);

    line_threshold = (white_value + black_value) / 2;

    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║              CALIBRATION COMPLETE                             ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    printf("White: %d | Black: %d | Edge: %d\n\n", 
        white_value, black_value, line_threshold);
}

/**
 * @brief   Print latest sensor readings and interpreted position
 */
void ir_print_readings(void) {
    uint16_t line = ir_read_line_sensor();
    uint16_t barcode = ir_read_barcode_sensor();
    int32_t position = ir_get_line_position();

    printf("Line: %4d | Barcode: %4d | Position: %+5ld | ", 
        line, barcode, position);

    if (ir_line_detected()) {
        printf("✓ On edge");
    } else if (line > line_threshold + 300) {
        printf("⬛ Black (turn RIGHT)");
    } else if (line < line_threshold - 300) {
        printf("⬜ White (turn LEFT)");
    } else {
        printf("? Searching");
    }

    if (ir_barcode_detected()) {
        printf(" | ▬ BARCODE");
    }

    printf("\n");
}

/**
 * @brief   Get current calibrated edge threshold
 * @return  Threshold value
 */
uint16_t ir_get_threshold(void) {
    return line_threshold;
}

/**
 * @brief   Set the edge threshold value
 * @param   threshold ADC value for edge
 */
void ir_set_threshold(uint16_t threshold) {
    line_threshold = threshold;
    printf("Edge threshold set to: %d\n", threshold);
}

/**
 * @brief   Get current calibrated white value
 * @return  White ADC value
 */
uint16_t ir_get_white_value(void) {
    return white_value;
}

/**
 * @brief   Get current calibrated black value
 * @return  Black ADC value
 */
uint16_t ir_get_black_value(void) {
    return black_value;
}
