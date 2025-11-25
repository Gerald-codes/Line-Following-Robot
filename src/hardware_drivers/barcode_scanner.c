/**
 * @file    barcode_scanner.c (CONSISTENCY ENHANCED)
 * @brief   Interrupt-based Code 39 barcode scanner - TUNED FOR RELIABILITY
 * @details Added threshold validation and better filtering
 */

#include "barcode_scanner.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_BARS                100
#define BARCODE_TIMEOUT_US 1500000
#define MIN_BAR_TIME_US         200
#define MAX_BAR_TIME_US     500000
#define COOLDOWN_US        1000000

/* TUNING: Threshold ratio limits */
#define MIN_ACCEPTABLE_RATIO    1.8f    /* Wide/narrow should be at least 1.8x */
#define MAX_ACCEPTABLE_RATIO    4.0f    /* But not more than 4x (too different = noise) */

typedef struct {
    char character;
    const char* pattern;
} Code39Entry;

static const Code39Entry code39_table[] = {
    {'0', "nnnwwnwnn"}, {'1', "wnnwnnnnw"}, {'2', "nnwwnnnnw"}, {'3', "wnwwnnnnn"},
    {'4', "nnnwwnnnw"}, {'5', "wnnwwnnnn"}, {'6', "nnwwwnnnn"}, {'7', "nnnwnnwnw"},
    {'8', "wnnwnnwnn"}, {'9', "nnwwnnwnn"}, {'A', "wnnnnwnnw"}, {'B', "nnwnnwnnw"},
    {'C', "wnwnnwnnn"}, {'D', "nnnnwwnnw"}, {'E', "wnnnwwnnn"}, {'F', "nnwnwwnnn"},
    {'G', "nnnnnwwnw"}, {'H', "wnnnnwwnn"}, {'I', "nnwnnwwnn"}, {'J', "nnnnwwwnn"},
    {'K', "wnnnnnnww"}, {'L', "nnwnnnnww"}, {'M', "wnwnnnnwn"}, {'N', "nnnnwnnww"},
    {'O', "wnnnwnnwn"}, {'P', "nnwnwnnwn"}, {'Q', "nnnnnnwww"}, {'R', "wnnnnnwwn"},
    {'S', "nnwnnnwwn"}, {'T', "nnnnwnwwn"}, {'U', "wwnnnnnnw"}, {'V', "nwwnnnnnw"},
    {'W', "wwwnnnnnn"}, {'X', "nwnnwnnnw"}, {'Y', "wwnnwnnnn"}, {'Z', "nwwnwnnnn"},
    {'-', "nwnnnnwnw"}, {'.', "wwnnnnwnn"}, {' ', "nwwnnnwnn"}, {'$', "nwnwnwnnn"},
    {'/', "nwnwnnnwn"}, {'+', "nwnnnwnwn"}, {'%', "nnnwnwnwn"}, {'*', "nwnnwnwnn"}
};

#define CODE39_TABLE_SIZE (sizeof(code39_table) / sizeof(Code39Entry))

static BarcodeCommand letter_to_direction(char letter) {
    switch (letter) {
        case 'A': case 'C': case 'E': case 'G': case 'I':
        case 'K': case 'M': case 'O': case 'Q': case 'S':
        case 'U': case 'W': case 'Y':
            return BARCODE_CMD_RIGHT;
        case 'B': case 'D': case 'F': case 'H': case 'J':
        case 'L': case 'N': case 'P': case 'R': case 'T':
        case 'V': case 'X': case 'Z':
            return BARCODE_CMD_LEFT;
        default:
            return BARCODE_CMD_UNKNOWN;
    }
}

static uint8_t barcode_pin = 0;
static volatile BarcodeScanState scan_state = BARCODE_SCAN_IDLE;
static volatile uint32_t bar_times[MAX_BARS];
static volatile uint8_t bar_count = 0;
static volatile uint32_t last_edge_time = 0;
static volatile bool last_state = false;
static volatile uint32_t cooldown_start_time = 0;
static char last_decoded_char = '?';

static void gpio_callback(uint gpio, uint32_t events) {
    if (gpio != barcode_pin) return;
    uint32_t current_time = time_us_32();
    bool current_state = !gpio_get(barcode_pin);
    uint32_t elapsed = current_time - last_edge_time;

    if (scan_state == BARCODE_SCAN_IDLE) {
        if (current_state) {
            scan_state = BARCODE_SCAN_IN_PROGRESS;
            bar_count = 0;
            last_edge_time = current_time;
            last_state = true;
            printf("[BARCODE] Scan started\n");
        }
    } 
    else if (scan_state == BARCODE_SCAN_IN_PROGRESS) {
        if (elapsed >= MIN_BAR_TIME_US && elapsed <= MAX_BAR_TIME_US) {
            if (bar_count < MAX_BARS) bar_times[bar_count++] = elapsed;
        }
        last_edge_time = current_time;
        last_state = current_state;
    }
}

/**
 * ENHANCED: Calculate threshold with validation
 */
static uint32_t calculate_threshold(uint32_t* times, uint8_t count) {
    if (count < 9) return 0;
    
    uint32_t min_time = times[0];
    uint32_t max_time = times[0];
    
    for (int i = 1; i < count; i++) {
        if (times[i] < min_time) min_time = times[i];
        if (times[i] > max_time) max_time = times[i];
    }
    
    float ratio = (float)max_time / min_time;
    uint32_t threshold = (min_time + max_time) / 2;
    
    printf("[BARCODE] Threshold: %lu us (min: %lu, max: %lu, ratio: %.2f)", 
           threshold, min_time, max_time, ratio);
    
    /* ENHANCED: Validate ratio */
    if (ratio < MIN_ACCEPTABLE_RATIO) {
        printf(" ⚠️ WARN: Ratio too low (bars too similar)\n");
        return 0;  /* Reject - likely noise or bad scan */
    }
    if (ratio > MAX_ACCEPTABLE_RATIO) {
        printf(" ⚠️ WARN: Ratio too high (inconsistent timing)\n");
        return 0;  /* Reject - likely speed issue */
    }
    
    printf(" ✓\n");
    return threshold;
}

static char match_code39_pattern(const char *pattern9) {
    for (int i = 0; i < CODE39_TABLE_SIZE; i++)
        if (strncmp(pattern9, code39_table[i].pattern, 9) == 0) return code39_table[i].character;
    return '?';
}

static char decode_character(uint32_t* times, uint32_t threshold) {
    char pattern[10];
    for (int i = 0; i < 9; i++) pattern[i] = (times[i] > threshold) ? 'w' : 'n';
    pattern[9] = '\0';
    printf("  Pattern: %s -> ", pattern);

    char result = match_code39_pattern(pattern);
    if (result != '?') { printf("%c\n", result); return result; }
    
    char reversed[10];
    for (int i = 0; i < 9; i++) reversed[i] = pattern[8 - i];
    reversed[9] = '\0';
    result = match_code39_pattern(reversed);
    if (result != '?') { printf("%c (reversed)\n", result); return result; }
    
    printf("?\n");
    return '?';
}

static BarcodeCommand decode_barcode(void) {
    printf("\n[BARCODE] ========== DECODING ==========\n");
    
    /* Copy volatile data to local buffer */
    uint8_t local_bar_count = bar_count;
    uint32_t local_times[MAX_BARS];
    for (int i = 0; i < local_bar_count; i++) {
        local_times[i] = bar_times[i];
    }
    
    printf("[BARCODE] Transitions captured: %d\n", local_bar_count);
    if (local_bar_count < 9) {
        printf("[BARCODE] Too few bars: %d (need ≥9)\n", local_bar_count);
        return BARCODE_CMD_UNKNOWN;
    }

    /* ENHANCED: Calculate and validate threshold */
    uint32_t threshold = calculate_threshold(local_times, local_bar_count);
    if (threshold == 0) {
        printf("[BARCODE] ✗ Threshold validation failed - rejecting scan\n");
        return BARCODE_CMD_UNKNOWN;
    }

    printf("[BARCODE] Raw timings (first 20): ");
    for (int i = 0; i < local_bar_count && i < 20; i++) printf("%lu ", local_times[i]);
    if (local_bar_count > 20) printf("... (%d more)", local_bar_count - 20);
    printf("\n");
    
    char decoded[20];
    int bar_index = 0, char_index = 0;
    printf("[BARCODE] Decoding characters:\n");
    
    while (bar_index + 9 <= local_bar_count && char_index < 19) {
        char c = decode_character(&local_times[bar_index], threshold);
        decoded[char_index++] = c;
        bar_index += 10;
    }
    decoded[char_index] = '\0';
    printf("[BARCODE] Decoded string: '%s'\n", decoded);

    char letter = '?';
    for (int i = 0; i < strlen(decoded); i++) {
        if ((decoded[i] >= 'A' && decoded[i] <= 'Z') || 
            (decoded[i] >= '0' && decoded[i] <= '9')) {
            letter = decoded[i];
            break;
        }
    }
    last_decoded_char = letter;
    
    if (letter == '?') {
        printf("[BARCODE] ✗ No valid letter found\n");
        return BARCODE_CMD_UNKNOWN;
    }
    
    BarcodeCommand cmd = letter_to_direction(letter);
    printf("[BARCODE] ===== RESULT: '%c' -> %s =====\n\n", letter, barcode_command_to_string(cmd));
    return cmd;
}

void barcode_scanner_init(uint8_t pin) {
    barcode_pin = pin;
    gpio_init(barcode_pin);
    gpio_set_dir(barcode_pin, GPIO_IN);
    gpio_pull_up(barcode_pin);
    gpio_set_irq_enabled_with_callback(
        barcode_pin,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &gpio_callback
    );
    scan_state = BARCODE_SCAN_IDLE;
    bar_count = 0;
    printf("[BARCODE] Scanner initialized on GP%d\n", barcode_pin);
}

BarcodeCommand barcode_scanner_update(void) {
    uint32_t current_time = time_us_32();
    if (scan_state == BARCODE_SCAN_COOLDOWN) {
        if (current_time - cooldown_start_time > COOLDOWN_US) {
            scan_state = BARCODE_SCAN_IDLE;
            printf("[BARCODE] Ready for next scan\n");
        }
        return BARCODE_CMD_NONE;
    }
    if (scan_state == BARCODE_SCAN_IN_PROGRESS) {
        uint32_t elapsed = current_time - last_edge_time;
        if (elapsed > BARCODE_TIMEOUT_US && bar_count > 0) {
            scan_state = BARCODE_SCAN_COMPLETE;
            printf("[BARCODE] Scan complete - %d transitions\n", bar_count);
        }
    }
    if (scan_state == BARCODE_SCAN_COMPLETE) {
        BarcodeCommand cmd = decode_barcode();
        bar_count = 0;
        cooldown_start_time = current_time;
        scan_state = BARCODE_SCAN_COOLDOWN;
        return cmd;
    }
    return BARCODE_CMD_NONE;
}

void barcode_scanner_reset(void) {
    scan_state = BARCODE_SCAN_IDLE;
    bar_count = 0;
    cooldown_start_time = 0;
}

BarcodeScanState barcode_scanner_get_state(void) {
    return scan_state;
}

char barcode_scanner_get_last_character(void) {
    return last_decoded_char;
}

bool barcode_scanner_is_ready(void) {
    return scan_state == BARCODE_SCAN_IDLE;
}

const char* barcode_command_to_string(BarcodeCommand cmd) {
    switch (cmd) {
        case BARCODE_CMD_NONE:    return "NONE";
        case BARCODE_CMD_LEFT:    return "LEFT";
        case BARCODE_CMD_RIGHT:   return "RIGHT";
        case BARCODE_CMD_UNKNOWN: return "UNKNOWN";
        default:                  return "INVALID";
    }
}