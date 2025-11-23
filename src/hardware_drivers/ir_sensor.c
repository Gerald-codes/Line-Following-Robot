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
#define BARCODE_SENSOR_PIN 6  
void ir_sensor_init(void) {
    // Initialize ADC
    adc_init();
    
    // Initialize line sensor (GP26 = ADC0)
    adc_gpio_init(26);
    
    gpio_init(BARCODE_SENSOR_PIN);
    gpio_init(BARCODE_SENSOR_PIN);                    // Initialize GPIO pin 6
    gpio_set_dir(BARCODE_SENSOR_PIN, GPIO_IN);
    
    
    printf("✓ IR sensors initialized\n");
    printf("  Line sensor: GP26 (ADC0)\n");
    printf("  Barcode sensor: GP6 (D0)\n");
    printf("  Edge threshold: %d\n", line_threshold);
    printf("  White value: %d, Black value: %d\n", white_value, black_value);
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
    
    // ========================================================================
    // NORMALIZED POSITION CALCULATION (Better for PID)
    // ========================================================================
    // Instead of using max_deviation, we normalize based on calibrated values
    // This gives more consistent behavior across different sensors
    
    int32_t position;
    
    if (reading < line_threshold) {
        // Sensor is on WHITE side (reading < threshold)
        // Map white_value...threshold → 0...LINE_POSITION_MAX
        // Positive position = turn LEFT to return to edge
        
        int32_t white_range = line_threshold - white_value;
        int32_t offset = line_threshold - reading;
        
        if (white_range > 0) {
            position = (offset * LINE_POSITION_MAX) / white_range;
        } else {
            position = LINE_POSITION_MAX;  // Failsafe
        }
        
    } else {
        // Sensor is on BLACK side (reading >= threshold)
        // Map threshold...black_value → 0...-LINE_POSITION_MAX
        // Negative position = turn RIGHT to return to edge
        
        int32_t black_range = black_value - line_threshold;
        int32_t offset = reading - line_threshold;
        
        if (black_range > 0) {
            position = -(offset * LINE_POSITION_MAX) / black_range;
        } else {
            position = -LINE_POSITION_MAX;  // Failsafe
        }
    }
    
    // Debug every 500ms
    static uint32_t last_debug = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_debug > 500) {
        printf("IR_DEBUG: read=%d, thr=%d, white=%d, black=%d, pos_calc=%ld\n",
               reading, line_threshold, white_value, black_value, position);
        last_debug = now;
    }
    
    // Clamp to valid range
    if (position > LINE_POSITION_MAX) position = LINE_POSITION_MAX;
    if (position < LINE_POSITION_MIN) position = LINE_POSITION_MIN;
    
    return position;
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
bool ir_barcode_digital_detected(void) {
    // Read GP6 - returns true if black detected
    // Adjust logic based on your sensor (may need to invert with !)
    return !gpio_get(BARCODE_SENSOR_PIN);  // Assuming LOW = black detected
}

// Keep old function for compatibility, but use digital version
bool ir_barcode_detected_digital(void) {
    bool pin_state = gpio_get(BARCODE_SENSOR_PIN);
    
    // Test both ways and see which works for your sensor:
    // Option 1: If sensor outputs HIGH when seeing black
    return pin_state;
    
    // Option 2: If sensor outputs LOW when seeing black (inverted)
    // return !pin_state;
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