/**
 * ir_sensor.c
 * Single IR sensor driver for edge-detection line following
 * Plus separate barcode sensor
 */

#include "ir_sensor.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include <stdio.h>

// ADC channels
#define LINE_SENSOR_ADC_CHANNEL 0    // GP26/ADC0 - Line tracking
#define BARCODE_SENSOR_ADC_CHANNEL 1 // GP27/ADC1 - Barcode detection

// Calibration values (can be tuned)
static uint16_t line_threshold = IR_EDGE_THRESHOLD;
static uint16_t white_value = IR_THRESHOLD_WHITE;
static uint16_t black_value = IR_THRESHOLD_BLACK;

void ir_sensor_init(void) {
    // Initialize ADC
    adc_init();
    
    // Initialize line sensor (GP26 = ADC0)
    adc_gpio_init(26);
    
    // Initialize barcode sensor (GP27 = ADC1)
    adc_gpio_init(27);
    
    printf("✓ IR sensors initialized\n");
    printf("  Line sensor: GP26 (ADC0)\n");
    printf("  Barcode sensor: GP27 (ADC1)\n");
    printf("  Edge threshold: %d\n", line_threshold);
}

uint16_t ir_read_line_sensor(void) {
    adc_select_input(LINE_SENSOR_ADC_CHANNEL);
    return adc_read();
}

uint16_t ir_read_barcode_sensor(void) {
    adc_select_input(BARCODE_SENSOR_ADC_CHANNEL);
    return adc_read();
}

int32_t ir_get_line_position(void) {
    uint16_t reading = ir_read_line_sensor();
    
    // Convert ADC reading to position error
    // Reading at edge (threshold) = 0 error
    // Reading > threshold (white) = positive error (line is left, turn left)
    // Reading < threshold (black) = negative error (line is right, turn right)
    
    int32_t error = (int32_t)reading - (int32_t)line_threshold;
    
    // Scale error to ±LINE_POSITION_MAX range
    // ADC range is 0-4095, threshold is ~2000
    // Max deviation: ±2000
    error = (error * LINE_POSITION_MAX) / 2000;
    
    // Clamp to valid range
    if (error > LINE_POSITION_MAX) error = LINE_POSITION_MAX;
    if (error < LINE_POSITION_MIN) error = LINE_POSITION_MIN;
    
    return error;
}

bool ir_line_detected(void) {
    uint16_t reading = ir_read_line_sensor();
    
    // Line detected if reading is near edge threshold
    // Allow some tolerance (±500)
    int32_t diff = abs((int32_t)reading - (int32_t)line_threshold);
    
    return (diff < 1000);  // Within 1000 ADC units of edge
}

bool ir_barcode_detected(void) {
    uint16_t reading = ir_read_barcode_sensor();
    
    // Barcode stripe detected if sensor sees black
    return (reading < IR_THRESHOLD_BLACK);
}

void ir_calibrate(void) {
    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                  IR SENSOR CALIBRATION                        ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    // Calibrate WHITE (off line)
    printf("1. Place sensor over WHITE surface\n");
    printf("   Press Enter when ready...\n");
    getchar();
    
    uint32_t white_sum = 0;
    for (int i = 0; i < 50; i++) {
        white_sum += ir_read_line_sensor();
        sleep_ms(10);
    }
    white_value = white_sum / 50;
    printf("   White value: %d\n\n", white_value);
    
    sleep_ms(500);
    
    // Calibrate BLACK (on line)
    printf("2. Place sensor over BLACK line\n");
    printf("   Press Enter when ready...\n");
    getchar();
    
    uint32_t black_sum = 0;
    for (int i = 0; i < 50; i++) {
        black_sum += ir_read_line_sensor();
        sleep_ms(10);
    }
    black_value = black_sum / 50;
    printf("   Black value: %d\n\n", black_value);
    
    // Calculate edge threshold (midpoint)
    line_threshold = (white_value + black_value) / 2;
    
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║              CALIBRATION COMPLETE                             ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    printf("White: %d | Black: %d | Edge: %d\n\n", 
           white_value, black_value, line_threshold);
}

void ir_print_readings(void) {
    uint16_t line = ir_read_line_sensor();
    uint16_t barcode = ir_read_barcode_sensor();
    int32_t position = ir_get_line_position();
    
    printf("Line: %4d | Barcode: %4d | Position: %+5ld | ", 
           line, barcode, position);
    
    if (ir_line_detected()) {
        printf("✓ On edge");
    } else if (line > line_threshold + 500) {
        printf("← White (turn LEFT)");
    } else if (line < line_threshold - 500) {
        printf("→ Black (turn RIGHT)");
    } else {
        printf("? Searching");
    }
    
    if (ir_barcode_detected()) {
        printf(" | ■ BARCODE");
    }
    
    printf("\n");
}

uint16_t ir_get_threshold(void) {
    return line_threshold;
}

void ir_set_threshold(uint16_t threshold) {
    line_threshold = threshold;
    printf("Edge threshold set to: %d\n", threshold);
}
