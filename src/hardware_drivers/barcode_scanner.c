/**
 * @file    barcode_scanner.c
 * @brief   Interrupt-based Code 39 barcode scanner implementation
 * @details
 *   Features:
 *   - Automatic narrow/wide threshold calculation
 *   - Bidirectional scanning support
 *   - 1.5 second timeout for scan completion
 *   - 1 second cooldown between scans
 */

#include "barcode_scanner.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/**
 * @brief   Timing and scan configuration
 */
#define MAX_BARS                100         /**< Maximum bars/spaces to store */
#define BARCODE_TIMEOUT_US 1500000          /**< 1.5 second timeout */
#define MIN_BAR_TIME_US         200         /**< 0.2ms minimum */
#define MAX_BAR_TIME_US     500000          /**< 500ms maximum */
#define COOLDOWN_US        1000000          /**< 1 second cooldown */

/**
 * @brief   Code 39 lookup table definitions
 */
typedef struct {
    char character;
    const char* pattern;  /* 9 characters: 'n' = narrow, 'w' = wide */
} Code39Entry;

/** Patterns: 9 elements per character (5 bars + 4 spaces). */
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

/**
 * @brief   Convert character to direction command
 * @param   letter Decoded Code 39 letter
 * @return  BarcodeCommand assigned (LEFT/RIGHT/UNKNOWN)
 */
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

/**
 * @brief   Internal state for barcode scanner (volatile for ISR)
 */
static uint8_t barcode_pin = 0;
static volatile BarcodeScanState scan_state = BARCODE_SCAN_IDLE;
static volatile uint32_t bar_times[MAX_BARS];
static volatile uint8_t bar_count = 0;
static volatile uint32_t last_edge_time = 0;
static volatile bool last_state = false;
static volatile uint32_t cooldown_start_time = 0;
static char last_decoded_char = '?';

/**
 * @brief   GPIO interrupt handler for barcode scan pin
 * @param   gpio GPIO pin number
 * @param   events Event type (edge rise/fall)
 */
static void gpio_callback(uint gpio, uint32_t events) {
    if (gpio != barcode_pin) return;
    uint32_t current_time = time_us_32();
    bool current_state = !gpio_get(barcode_pin);  /* INVERTED: LOW = black */
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
 * @brief   Calculate bar threshold for wide/narrow separation
 * @return  Threshold in microseconds
 */
static uint32_t calculate_threshold(void) {
    if (bar_count < 9) return 0;
    uint32_t min_time = bar_times[0];
    uint32_t max_time = bar_times[0];
    for (int i = 1; i < bar_count; i++) {
        if (bar_times[i] < min_time) min_time = bar_times[i];
        if (bar_times[i] > max_time) max_time = bar_times[i];
    }
    uint32_t threshold = (min_time + max_time) / 2;
    printf("[BARCODE] Threshold: %lu us (min: %lu, max: %lu, ratio: %.2f)\n", 
           threshold, min_time, max_time, (float)max_time / min_time);
    return threshold;
}

/**
 * @brief   Match a 9-symbol pattern to Code 39 lookup
 * @param   pattern9 Pattern string (9 'n' or 'w')
 * @return  Matching character or '?'
 */
static char match_code39_pattern(const char *pattern9) {
    for (int i = 0; i < CODE39_TABLE_SIZE; i++)
        if (strncmp(pattern9, code39_table[i].pattern, 9) == 0) return code39_table[i].character;
    return '?';
}

/**
 * @brief   Decode 9 timing bars into a Code 39 character
 * @param   times Array of 9 bar/space timings
 * @param   threshold Wide/narrow threshold
 * @return  Matched character or '?'
 */
static char decode_character(volatile uint32_t* times, uint32_t threshold) {
    char pattern[10];
    for (int i = 0; i < 9; i++) pattern[i] = (times[i] > threshold) ? 'w' : 'n';
    pattern[9] = '\0';
    printf("  Pattern: %s -> ", pattern);

    char result = match_code39_pattern(pattern);  /* Try normal */
    if (result != '?') { printf("%c\n", result); return result; }
    char reversed[10];
    for (int i = 0; i < 9; i++) reversed[i] = pattern[8 - i];
    reversed[9] = '\0';
    result = match_code39_pattern(reversed);  /* Try reversed */
    if (result != '?') { printf("%c (reversed)\n", result); return result; }
    printf("?\n");
    return '?';
}

/**
 * @brief   Decode full barcode sequence into command
 * @return  BarcodeCommand result (LEFT/RIGHT/UNKNOWN)
 */
static BarcodeCommand decode_barcode(void) {
    printf("\n[BARCODE] ========== DECODING ==========\n");
    printf("[BARCODE] Transitions captured: %d\n", bar_count);
    if (bar_count < 9) {
        printf("[BARCODE] Too few bars: %d (need ≥9)\n", bar_count);
        return BARCODE_CMD_UNKNOWN;
    }

    uint32_t threshold = calculate_threshold();
    if (threshold == 0) {
        printf("[BARCODE] Could not calculate threshold\n");
        return BARCODE_CMD_UNKNOWN;
    }

    printf("[BARCODE] Raw timings: ");
    for (int i = 0; i < bar_count && i < 20; i++) printf("%lu ", bar_times[i]);
    if (bar_count > 20) printf("... (%d more)", bar_count - 20);
    printf("\n");
    char decoded[20];
    int bar_index = 0, char_index = 0;
    printf("[BARCODE] Decoding characters:\n");
    while (bar_index + 9 <= bar_count && char_index < 19) {
        char c = decode_character(&bar_times[bar_index], threshold);
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

/**
 * @brief   Initialize barcode scanner (setup GPIO and interrupts)
 * @param   pin GPIO pin to use for IR barcode sensor
 */
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

/**
 * @brief   Update barcode scanner state (call in main loop)
 * @return  BarcodeCommand result, or NONE (if scan not ready)
 */
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

/**
 * @brief   Reset barcode scanner internal state/data
 */
void barcode_scanner_reset(void) {
    scan_state = BARCODE_SCAN_IDLE;
    bar_count = 0;
    cooldown_start_time = 0;
}

/**
 * @brief   Get current barcode scanner state
 * @return  BarcodeScanState (enum value)
 */
BarcodeScanState barcode_scanner_get_state(void) {
    return scan_state;
}

/**
 * @brief   Get most recently decoded character
 * @return  Character code, or '?' if none
 */
char barcode_scanner_get_last_character(void) {
    return last_decoded_char;
}

/**
 * @brief   Check if scanner is ready for next scan
 * @return  true if scanner is IDLE, false otherwise
 */
bool barcode_scanner_is_ready(void) {
    return scan_state == BARCODE_SCAN_IDLE;
}

/**
 * @brief   Convert BarcodeCommand to string for debug/printing
 * @param   cmd Command to convert
 * @return  String representation of command
 */
const char* barcode_command_to_string(BarcodeCommand cmd) {
    switch (cmd) {
        case BARCODE_CMD_NONE:    return "NONE";
        case BARCODE_CMD_LEFT:    return "LEFT";
        case BARCODE_CMD_RIGHT:   return "RIGHT";
        case BARCODE_CMD_UNKNOWN: return "UNKNOWN";
        default:                  return "INVALID";
    }
}
