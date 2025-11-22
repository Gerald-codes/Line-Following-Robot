/**
 * calibration.c
 * Button-triggered calibration for IR sensors
 */

#include "calibration.h"
#include "ir_sensor.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <stdlib.h>

#ifndef CALIBRATION_BUTTON_PIN
#define CALIBRATION_BUTTON_PIN 20  // Default button pin
#endif

#ifndef BUTTON_DEBOUNCE_MS
#define BUTTON_DEBOUNCE_MS 200
#endif

// Button state tracking
static uint32_t last_button_time = 0;
static bool last_button_state = false;

void calibration_init(void) {
    gpio_init(CALIBRATION_BUTTON_PIN);
    gpio_set_dir(CALIBRATION_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(CALIBRATION_BUTTON_PIN);
    
    printf("✓ Calibration button initialized on GP%d\n", CALIBRATION_BUTTON_PIN);
}

bool calibration_button_pressed(void) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    bool button_state = !gpio_get(CALIBRATION_BUTTON_PIN);  // Active low
    
    // Detect rising edge with debounce
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
    printf("\n✓ White value: %d\n\n", white_value);
    
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
    printf("\n✓ Black value: %d\n\n", black_value);
    
    // Calculate threshold and update sensor
    uint16_t threshold = (white_value + black_value) / 2;
    int32_t sensor_range = abs((int32_t)white_value - (int32_t)black_value);
    
    // Update IR sensor with calibrated values
    ir_set_white_black_values(white_value, black_value);
    ir_set_threshold(threshold);
    ir_set_max_deviation(sensor_range);
    
    // Display results
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║              CALIBRATION COMPLETE! ✓                         ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("📊 Calibration Results:\n");
    printf("   • White value:    %4d\n", white_value);
    printf("   • Black value:    %4d\n", black_value);
    printf("   • Edge threshold: %4d\n", threshold);
    printf("   • Sensor range:   %4d\n", sensor_range);
    printf("\n");
    
    // Validate sensor type
    if (white_value < black_value) {
        printf("✓ Inverted sensor (Black > White)\n");
    } else {
        printf("✓ Normal sensor (White > Black)\n");
    }
    
    // Validate contrast
    if (sensor_range < 500) {
        printf("⚠️  WARNING: Low contrast!\n");
        printf("   Consider adjusting sensor height\n");
    } else if (sensor_range > 1500) {
        printf("✓ Good contrast!\n");
    }
    
    printf("\n");
}