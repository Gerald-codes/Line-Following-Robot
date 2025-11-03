/**
 * calibration.c
 * Button-triggered calibration for IR sensors
 * SIMPLIFIED: Just sets threshold and max deviation, no mode switching
 */

#include "calibration.h"
#include "ir_sensor.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <stdlib.h>

// Button state tracking
static uint32_t last_button_time = 0;
static bool last_button_state = false;

void calibration_init(void) {
    gpio_init(CALIBRATION_BUTTON_PIN);
    gpio_set_dir(CALIBRATION_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(CALIBRATION_BUTTON_PIN);
    
    printf("✓ Calibration button initialized on GP20\n");
    printf("  Press button to start IR sensor calibration\n");
}

bool calibration_button_pressed(void) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    bool button_state = !gpio_get(CALIBRATION_BUTTON_PIN);
    
    if (button_state && !last_button_state) {
        if (current_time - last_button_time > BUTTON_DEBOUNCE_MS) {
            last_button_time = current_time;
            last_button_state = button_state;
            return true;
        }
    }
    
    last_button_state = button_state;
    return false;
}

void calibration_run_sequence(void) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║              IR SENSOR CALIBRATION                           ║\n");
    printf("║              (INVERTED SENSOR MODE)                          ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    // Step 1: Calibrate WHITE surface
    printf("STEP 1: WHITE SURFACE CALIBRATION\n");
    printf("────────────────────────────────────────\n");
    printf("• Place the IR sensor over a WHITE surface\n");
    printf("• Make sure the sensor is stable (5-10mm height)\n");
    printf("• Press the button when ready...\n\n");
    
    while (!calibration_button_pressed()) {
        sleep_ms(10);
    }
    
    printf("⏳ Calibrating white surface...\n");
    uint32_t white_sum = 0;
    for (int i = 0; i < 50; i++) {
        white_sum += ir_read_line_sensor();
        sleep_ms(10);
        if (i % 10 == 0) printf(".");
    }
    uint16_t white_value = white_sum / 50;
    printf("\n✓ White value: %d", white_value);
    
    // Check if this is an inverted sensor
    if (white_value < 1000) {
        printf(" ✓ (LOW - inverted sensor detected)\n\n");
    } else {
        printf(" (Reading seems HIGH - check sensor type)\n\n");
    }
    
    sleep_ms(500);
    
    // Step 2: Calibrate BLACK surface
    printf("STEP 2: BLACK SURFACE CALIBRATION\n");
    printf("────────────────────────────────────────\n");
    printf("• Place the IR sensor over a BLACK line/surface\n");
    printf("• Make sure the sensor is stable (same height)\n");
    printf("• Press the button when ready...\n\n");
    
    while (!calibration_button_pressed()) {
        sleep_ms(10);
    }
    
    printf("⏳ Calibrating black surface...\n");
    uint32_t black_sum = 0;
    for (int i = 0; i < 50; i++) {
        black_sum += ir_read_line_sensor();
        sleep_ms(10);
        if (i % 10 == 0) printf(".");
    }
    uint16_t black_value = black_sum / 50;
    printf("\n✓ Black value: %d", black_value);
    
    // Check if this is an inverted sensor
    if (black_value > 2000) {
        printf(" ✓ (HIGH - inverted sensor confirmed)\n\n");
    } else {
        printf(" (Reading seems LOW - check sensor type)\n\n");
    }
    
    // Calculate edge threshold (midpoint between white and black)
    uint16_t edge_threshold = (white_value + black_value) / 2;
    
    // Update IR sensor with calibrated values
    ir_set_threshold(edge_threshold);
    int32_t sensor_range = abs(white_value - black_value);
    ir_set_max_deviation(sensor_range);
    
    // Display results
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║              CALIBRATION COMPLETE! ✓                         ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("📊 Calibration Results:\n");
    printf("   • White value:    %4d (LOW for inverted sensor)\n", white_value);
    printf("   • Black value:    %4d (HIGH for inverted sensor)\n", black_value);
    printf("   • Edge threshold: %4d (midpoint)\n", edge_threshold);
    printf("   • Sensor range:   %4d ADC units\n", sensor_range);
    printf("   • Position range: ±%4d\n", sensor_range);
    printf("\n");
    
    // Validate sensor type
    if (white_value < black_value) {
        printf("✓ Inverted sensor confirmed (Black > White)\n");
        printf("  This is normal for QTR-style reflective sensors.\n");
    } else {
        printf("⚠️  WARNING: Sensor appears to be NORMAL type (White > Black)\n");
        printf("  You may be using the WRONG version of ir_sensor.c!\n");
        printf("  Use the non-inverted version instead.\n");
    }
    
    printf("\n");
    
    // Validate contrast quality
    if (sensor_range < 500) {
        printf("⚠️  WARNING: Low contrast detected!\n");
        printf("   The difference between white and black is small.\n");
        printf("   Consider:\n");
        printf("   • Adjusting sensor height (try 5-10mm)\n");
        printf("   • Improving lighting conditions\n");
        printf("   • Cleaning sensor and track surface\n");
    } else if (sensor_range > 3000) {
        printf("✓ Excellent contrast! PID should work well.\n");
    } else if (sensor_range > 1500) {
        printf("✓ Good contrast. PID should work well.\n");
    } else {
        printf("⚠️  Moderate contrast. May work but could be better.\n");
    }
    
    printf("\n");
    printf("Press button to start Demo 2...\n");
}