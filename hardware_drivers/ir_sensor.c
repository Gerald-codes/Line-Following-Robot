/**
 * ir_sensor.c
 * Single IR sensor driver for edge-detection line following
 * FOR INVERTED SENSORS (QTR-style): Black=HIGH, White=LOW
 * IMPROVED: Normalized position output for better PID control
 */

#include "ir_sensor.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <stdlib.h>

// ADC channels
#define LINE_SENSOR_ADC_CHANNEL 0    // GP26/ADC0 - Line tracking
#define BARCODE_SENSOR_ADC_CHANNEL 1 // GP27/ADC1 - Barcode detection

// Calibration values (set by calibration)
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
    printf("  ⚠️  INVERTED SENSOR MODE (Black=HIGH, White=LOW)\n");
    printf("  📍 LEFT SIDE TRACING\n");
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
    
    // Simple linear calculation
    int32_t error = (int32_t)line_threshold - (int32_t)reading;
    
    // Scale down by dividing by 2
    // This makes position range ±1000 instead of ±2000
    error = error / 2;
    
    // Clamp to ±1000
    if (error > 1000) error = 1000;
    if (error < -1000) error = -1000;
    
    return error;
}

void ir_set_max_deviation(uint16_t deviation) {
    // This function is kept for compatibility but not used anymore
    // Position is now calculated from white/black calibration values
    printf("Info: Using calibrated white/black values for position calculation\n");
}

void ir_set_white_black_values(uint16_t white, uint16_t black) {
    white_value = white;
    black_value = black;
    line_threshold = (white + black) / 2;
    printf("Calibration set: white=%d, black=%d, threshold=%d\n", 
           white, black, line_threshold);
}

bool ir_line_detected(void) {
    uint16_t reading = ir_read_line_sensor();
    
    // Line detected if reading is near edge threshold
    int32_t diff = abs((int32_t)reading - (int32_t)line_threshold);
    
    // Use 30% of sensor range as tolerance
    int32_t tolerance = abs(black_value - white_value) * 30 / 100;
    if (tolerance < 200) tolerance = 200;  // Minimum tolerance
    
    return (diff < tolerance);
}

bool ir_barcode_detected(void) {
    uint16_t reading = ir_read_barcode_sensor();
    
    // FOR INVERTED SENSOR: Barcode stripe detected if sensor sees HIGH value (black)
    return (reading > IR_THRESHOLD_BLACK);
}

void ir_calibrate(void) {
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║                  IR SENSOR CALIBRATION                        ║\n");
    printf("║              (INVERTED SENSOR MODE)                           ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    
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
    printf("   White value: %d (LOW is correct for inverted sensor)\n\n", white_value);
    
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
    printf("   Black value: %d (HIGH is correct for inverted sensor)\n\n", black_value);
    
    // Calculate edge threshold (midpoint)
    line_threshold = (white_value + black_value) / 2;
    
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║              CALIBRATION COMPLETE                             ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
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

uint16_t ir_get_threshold(void) {
    return line_threshold;
}

void ir_set_threshold(uint16_t threshold) {
    line_threshold = threshold;
    printf("Edge threshold set to: %d\n", threshold);
}

uint16_t ir_get_white_value(void) {
    return white_value;
}

uint16_t ir_get_black_value(void) {
    return black_value;
}