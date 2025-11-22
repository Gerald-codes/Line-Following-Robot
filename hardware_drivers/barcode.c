/**
 * barcode_scanner.c
 * Interrupt-based barcode scanner for Code39
 */

//#include "barcode.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Code39 lookup table
typedef struct {
    char ch;
    const char *pattern; // 9 characters, each 'n' (narrow) or 'w' (wide)
} Code39;

static const Code39 code39_table[] = {
    {'0',"nnnwwnwnw"}, {'1',"wnnwnnnnw"}, {'2',"nnwwnnnnw"}, {'3',"wnwwnnnnn"},
    {'4',"nnnwwnnnw"}, {'5',"wnnwwnnnn"}, {'6',"nnwwwnnnn"}, {'7',"nnnwnnwnw"},
    {'8',"wnnwnnwnn"}, {'9',"nnwwnnwnn"}, {'A',"wnnnnwnnw"}, {'B',"nnwnnwnnw"},
    {'C',"wnwnnwnnn"}, {'D',"nnnnwwnnw"}, {'E',"wnnnwwnnn"}, {'F',"nnwnwwnnn"},
    {'G',"nnnnnwwnw"}, {'H',"wnnnnwwnn"}, {'I',"nnwnnwwnn"}, {'J',"nnnnwwwnn"},
    {'K',"wnnnnnnww"}, {'L',"nnwnnnnww"}, {'M',"wnwnnnnwn"}, {'N',"nnnnwnnww"},
    {'O',"wnnnwnnwn"}, {'P',"nnwnwnnwn"}, {'Q',"nnnnnnwww"}, {'R',"wnnnnnwwn"},
    {'S',"nnwnnnwwn"}, {'T',"nnnnwnwwn"}, {'U',"wwnnnnnnw"}, {'V',"nwwnnnnnw"},
    {'W',"wwwnnnnnn"}, {'X',"nwnnwnnnw"}, {'Y',"wwnnwnnnn"}, {'Z',"nwwnwnnnn"},
    {'-',"nwnnnnwnw"}, {'.',"wwnnnnwnn"}, {' ',"nwwnnnwnn"}, {'$',"nwnwnwnnn"},
    {'/',"nwnwnnnwn"}, {'+',"nwnnnwnwn"}, {'%',"nnnwnwnwn"}, {'*',"nwnnwnwnn"} // Start/stop
};
static const int code39_len = sizeof(code39_table) / sizeof(Code39);

// Global state
static uint8_t barcode_gpio = 0;
static BarData bars[MAX_BAR_COUNT];
static volatile uint8_t bar_count = 0;
static volatile bool scanning = false;
static volatile uint64_t last_edge_time = 0;
static volatile bool last_state = false;
static volatile uint64_t scan_start_time = 0;

// Forward declarations
static void barcode_gpio_callback(uint gpio, uint32_t events);
static char match_code39_pattern(const char *pattern9);

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

void barcode_init(uint8_t gpio_pin) {
    barcode_gpio = gpio_pin;
    
    // Initialize GPIO
    gpio_init(barcode_gpio);
    gpio_set_dir(barcode_gpio, GPIO_IN);
    gpio_disable_pulls(barcode_gpio);
    
    // Set up interrupt for both edges
    gpio_set_irq_enabled_with_callback(barcode_gpio, 
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &barcode_gpio_callback);
    
    // Disable initially
    gpio_set_irq_enabled(barcode_gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    
    printf("Barcode scanner initialized on GPIO %d\n", barcode_gpio);
}

void barcode_start_scan(void) {
    // Reset state
    bar_count = 0;
    scanning = true;
    scan_start_time = time_us_64();
    last_edge_time = scan_start_time;
    last_state = gpio_get(barcode_gpio);
    
    // Enable interrupts
    gpio_set_irq_enabled(barcode_gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    
    printf("Barcode scan started (initial state: %s)\n", last_state ? "WHITE" : "BLACK");
}

void barcode_stop_scan(void) {
    // Disable interrupts
    gpio_set_irq_enabled(barcode_gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    scanning = false;
    
    printf("Barcode scan stopped (%d bars captured)\n", bar_count);
}

bool barcode_is_scan_complete(void) {
    if (!scanning) return true;
    
    // Check if we have enough bars
    if (bar_count >= MAX_BAR_COUNT - 1) {
        barcode_stop_scan();
        return true;
    }
    
    // Check for timeout (2 seconds since last edge = end of barcode)
    uint64_t current_time = time_us_64();
    if (bar_count > 10 && (current_time - last_edge_time) > 2000000) {
        barcode_stop_scan();
        return true;
    }
    
    return false;
}

uint8_t barcode_get_bar_count(void) {
    return bar_count;
}

void barcode_reset(void) {
    barcode_stop_scan();
    bar_count = 0;
    scanning = false;
}

void barcode_print_bars(void) {
    printf("\n=== Captured Bars (%d total) ===\n", bar_count);
    
    if (bar_count == 0) {
        printf("No bars captured!\n");
        return;
    }
    
    // Find minimum bar width
    uint32_t min_width = 0xFFFFFFFF;
    for (int i = 0; i < bar_count; i++) {
        if (bars[i].duration_us < min_width) {
            min_width = bars[i].duration_us;
        }
    }
    
    printf("Minimum bar width: %lu us\n\n", min_width);
    printf("Bar | Type  | Duration (us) | Duration (ms) | Units\n");
    printf("----+-------+---------------+---------------+------\n");
    
    for (int i = 0; i < bar_count; i++) {
        uint8_t units = (bars[i].duration_us + min_width/2) / min_width;
        printf("%3d | %s | %13lu | %13.2f | %d\n",
               i,
               bars[i].is_black ? "BLACK" : "WHITE",
               bars[i].duration_us,
               bars[i].duration_us / 1000.0f,
               units);
    }
}

bool barcode_decode(BarcodeResult* result) {
    memset(result, 0, sizeof(BarcodeResult));
    
    if (bar_count < 18) {
        printf("ERROR: Not enough bars! Need at least 18, got %d\n", bar_count);
        return false;
    }
    
    result->bar_count = bar_count;
    result->total_time_ms = (time_us_64() - scan_start_time) / 1000;
    
    // Find minimum bar width (unit width)
    uint32_t min_width = 0xFFFFFFFF;
    for (int i = 0; i < bar_count; i++) {
        if (bars[i].duration_us < min_width) {
            min_width = bars[i].duration_us;
        }
    }
    
    printf("\n=== Decoding Barcode ===\n");
    printf("Total bars: %d\n", bar_count);
    printf("Unit width: %lu us (%.2f ms)\n", min_width, min_width / 1000.0f);
    
    // Skip initial quiet zone if present
    int start_index = 0;
    if (!bars[0].is_black && bars[0].duration_us > bars[1].duration_us * 3) {
        printf("Skipping initial quiet zone\n");
        start_index = 1;
    }
    
    // Calculate threshold: narrow vs wide
    // Typically wide = 3x narrow, so threshold at 2x
    uint32_t threshold = min_width * 2;
    printf("Threshold: %lu us (narrow < threshold < wide)\n", threshold);
    
    // Build pattern string of 'n' and 'w'
    char pattern[512];
    int pattern_idx = 0;
    
    for (int i = start_index; i < bar_count; i++) {
        pattern[pattern_idx++] = (bars[i].duration_us > threshold) ? 'w' : 'n';
    }
    pattern[pattern_idx] = '\0';
    
    printf("Pattern: %s (length: %d)\n", pattern, pattern_idx);
    
    // Decode into characters (9 bars per character)
    char decoded[128];
    int decoded_idx = 0;
    
    for (int i = 0; i + 9 <= pattern_idx; i += 9) {
        char char_pattern[10];
        strncpy(char_pattern, &pattern[i], 9);
        char_pattern[9] = '\0';
        
        char c = match_code39_pattern(char_pattern);
        decoded[decoded_idx++] = c;
        
        // Skip inter-character gap (narrow bar)
        if (i + 9 < pattern_idx && pattern[i + 9] == 'n') {
            i++;
        }
    }
    decoded[decoded_idx] = '\0';
    
    printf("Decoded characters: %s\n", decoded);
    
    // Extract message between start/stop '*' markers
    char *start = strchr(decoded, '*');
    char *end = strrchr(decoded, '*');
    
    if (start && end && end > start) {
        int len = end - start - 1;
        if (len > 0 && len < 64) {
            strncpy(result->message, start + 1, len);
            result->message[len] = '\0';
            result->valid = true;
            
            printf(">>> DECODED MESSAGE: %s <<<\n", result->message);
            return true;
        }
    }
    
    printf("Decode failed - missing or invalid start/stop markers\n");
    return false;
}

// ============================================================================
// PRIVATE FUNCTIONS
// ============================================================================

static char match_code39_pattern(const char *pattern9) {
    for (int i = 0; i < code39_len; i++) {
        if (strncmp(pattern9, code39_table[i].pattern, 9) == 0) {
            return code39_table[i].ch;
        }
    }
    return '?';
}

// GPIO interrupt callback
static void barcode_gpio_callback(uint gpio, uint32_t events) {
    if (!scanning || gpio != barcode_gpio) return;
    
    uint64_t current_time = time_us_64();
    uint64_t duration = current_time - last_edge_time;
    
    // Ignore very short pulses (noise)
    if (duration < MIN_BAR_DURATION_US) {
        return;
    }
    
    // Store the bar that just ended
    if (bar_count < MAX_BAR_COUNT) {
        bars[bar_count].duration_us = duration;
        bars[bar_count].is_black = last_state;  // The state we WERE in
        bar_count++;
    }
    
    // Update state for next bar
    last_edge_time = current_time;
    last_state = gpio_get(barcode_gpio);
}