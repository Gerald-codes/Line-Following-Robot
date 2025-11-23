/**
 * INTEG-T1.c
 *
 * Integration Test ID: INTEG-T1
 *
 * Line Following + Barcode Decoding (Code 39, interrupt-based)
 *
 * Verifies:
 *  - Robot follows a line using 1 IR line sensor
 *  - Barcode sensor detects Code 39 pattern
 *  - Code 39 decoded into a letter
 *  - Letter mapped to LEFT / RIGHT / STOP command
 *  - Robot executes command and then resumes line following
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/timer.h"

// ============================================================================
// PIN DEFINITIONS (ADJUST AS NEEDED)
// ============================================================================

// Line sensor: analog on ADC0 (GP26)
#define IR_LINE_ADC_CH   0
#define IR_LINE_PIN      26

// Barcode sensor: digital input (active LOW = black)
#define IR_BARCODE_PIN   6

// Motors (direction only)
#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11

// ============================================================================
// LINE FOLLOWING CONFIG
// ============================================================================

#define LINE_THRESHOLD   2000   // tune from your ADC values
#define LINE_LOOP_DELAY_MS  30  // main loop delay

// ============================================================================
// BARCODE CONFIG (Code 39 via narrow/wide)
// ============================================================================

#define MAX_BARS               64
#define BARCODE_TIMEOUT_US     1500000  // 1.5s: no edge → end of scan
#define MIN_BAR_TIME_US        200      // ignore small glitches
#define MAX_BAR_TIME_US        500000   // ignore crazy long stuff

typedef enum {
    SCAN_IDLE,
    SCAN_IN_PROGRESS,
    SCAN_COMPLETE
} ScanState;

typedef enum {
    CMD_NONE,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_STOP,
    CMD_UNKNOWN
} BarcodeCommand;

// Volatile values updated inside IRQ
static volatile ScanState scan_state = SCAN_IDLE;
static volatile uint32_t bar_times[MAX_BARS];
static volatile uint8_t  bar_count = 0;
static volatile uint32_t last_edge_time = 0;

// ============================================================================
// CODE39 TABLE + LETTER → DIRECTION
// ============================================================================

typedef struct {
    char character;
    const char *pattern;   // 9 chars: 'n' or 'w'
} Code39Entry;

// Code 39 patterns (bars + spaces, 9 elements each)
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
    {'/', "nwnwnnnwn"}, {'+', "nwnnnwnwn"}, {'%', "nnnwnwnwn"}, {'*', "nwnnwnwnn"}  // start/stop
};

#define CODE39_TABLE_SIZE (sizeof(code39_table)/sizeof(Code39Entry))

// Map a letter to LEFT / RIGHT (same mapping you gave)
static BarcodeCommand letter_to_direction(char letter)
{
    switch (letter)
    {
        // RIGHT letters
        case 'A': case 'C': case 'E': case 'G': case 'I':
        case 'K': case 'M': case 'O': case 'Q': case 'S':
        case 'U': case 'W': case 'Y':
            return CMD_RIGHT;

        // LEFT letters
        case 'B': case 'D': case 'F': case 'H': case 'J':
        case 'L': case 'N': case 'P': case 'R': case 'T':
        case 'V': case 'X': case 'Z':
            return CMD_LEFT;

        default:
            return CMD_UNKNOWN;
    }
}

// ============================================================================
// MOTOR HELPERS
// ============================================================================

static void motors_stop(void)
{
    gpio_put(M1A, 0);
    gpio_put(M1B, 0);
    gpio_put(M2A, 0);
    gpio_put(M2B, 0);
}

static void motors_forward(void)
{
    gpio_put(M1A, 1);
    gpio_put(M1B, 0);
    gpio_put(M2A, 1);
    gpio_put(M2B, 0);
}

static void motors_turn_left(void)
{
    gpio_put(M1A, 0);
    gpio_put(M1B, 1);
    gpio_put(M2A, 1);
    gpio_put(M2B, 0);
}

static void motors_turn_right(void)
{
    gpio_put(M1A, 1);
    gpio_put(M1B, 0);
    gpio_put(M2A, 0);
    gpio_put(M2B, 1);
}

// ============================================================================
// LINE SENSOR (ADC)
// ============================================================================

static uint16_t read_line_sensor(void)
{
    adc_select_input(IR_LINE_ADC_CH);
    return adc_read();
}

// ============================================================================
// BARCODE IRQ HANDLER
// ============================================================================

static void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio != IR_BARCODE_PIN)
    {
        return;
    }

    uint32_t now = time_us_32();
    bool state = gpio_get(IR_BARCODE_PIN);  // assume LOW = black, HIGH = white

    if (scan_state == SCAN_IDLE)
    {
        // Start scanning on first edge
        scan_state = SCAN_IN_PROGRESS;
        bar_count = 0;
        last_edge_time = now;
        printf("[IRQ] Scan started.\n");
    }
    else if (scan_state == SCAN_IN_PROGRESS)
    {
        uint32_t dt = now - last_edge_time;

        if ((dt >= MIN_BAR_TIME_US) && (dt <= MAX_BAR_TIME_US))
        {
            if (bar_count < MAX_BARS)
            {
                bar_times[bar_count++] = dt;
            }
        }
        last_edge_time = now;
    }

    (void)state; // not used right now, but kept for clarity
}

// ============================================================================
// BARCODE DECODING HELPERS
// ============================================================================

static uint32_t calc_threshold(void)
{
    if (bar_count < 9)
    {
        return 0;
    }

    uint32_t min_t = bar_times[0];
    uint32_t max_t = bar_times[0];

    for (uint8_t i = 1; i < bar_count; i++)
    {
        if (bar_times[i] < min_t) min_t = bar_times[i];
        if (bar_times[i] > max_t) max_t = bar_times[i];
    }

    uint32_t thr = (min_t + max_t) / 2;
    printf("[BARCODE] min=%lu  max=%lu  thr=%lu  ratio=%.2f\n",
           min_t, max_t, thr, (float)max_t/(float)min_t);

    return thr;
}

static char match_pattern(const char *pat9)
{
    for (size_t i = 0; i < CODE39_TABLE_SIZE; i++)
    {
        if (strncmp(pat9, code39_table[i].pattern, 9) == 0)
        {
            return code39_table[i].character;
        }
    }
    return '?';
}

// Decode first 9 timings as one Code39 character
static char decode_first_character(uint32_t threshold)
{
    if (bar_count < 9)
    {
        return '?';
    }

    char pattern[10];
    for (int i = 0; i < 9; i++)
    {
        pattern[i] = (bar_times[i] > threshold) ? 'w' : 'n';
    }
    pattern[9] = '\0';

    printf("[BARCODE] pattern = %s -> ", pattern);

    char c = match_pattern(pattern);
    if (c != '?')
    {
        printf("%c\n", c);
        return c;
    }

    // Try reversed order (if scanned backwards)
    char rev[10];
    for (int i = 0; i < 9; i++)
    {
        rev[i] = pattern[8 - i];
    }
    rev[9] = '\0';

    c = match_pattern(rev);
    if (c != '?')
    {
        printf("%c (rev)\n", c);
        return c;
    }

    printf("?\n");
    return '?';
}

// Called from main loop when SCAN_COMPLETE
static BarcodeCommand process_barcode(void)
{
    printf("\n[BARCODE] Decoding %u transitions...\n", bar_count);

    uint32_t thr = calc_threshold();
    if (thr == 0)
    {
        printf("[BARCODE] Threshold calc failed.\n");
        return CMD_UNKNOWN;
    }

    // Only decode first character (simplified for INTEG-T1)
    char ch = decode_first_character(thr);
    if (ch == '?' || ch == '*')
    {
        printf("[BARCODE] Invalid character '%c'\n", ch);
        return CMD_UNKNOWN;
    }

    BarcodeCommand cmd = letter_to_direction(ch);

    printf("[BARCODE] Letter '%c' -> ", ch);
    switch (cmd)
    {
        case CMD_LEFT:  printf("LEFT\n");  break;
        case CMD_RIGHT: printf("RIGHT\n"); break;
        default:        printf("UNKNOWN\n"); break;
    }

    return cmd;
}

// ============================================================================
// INITIALISATION
// ============================================================================

static void init_hardware(void)
{
    stdio_init_all();
    sleep_ms(1500);

    // ADC for line sensor
    adc_init();
    adc_gpio_init(IR_LINE_PIN); // GP26 -> ADC0

    // Motor pins
    gpio_init(M1A); gpio_set_dir(M1A, GPIO_OUT);
    gpio_init(M1B); gpio_set_dir(M1B, GPIO_OUT);
    gpio_init(M2A); gpio_set_dir(M2A, GPIO_OUT);
    gpio_init(M2B); gpio_set_dir(M2B, GPIO_OUT);
    motors_stop();

    // Barcode pin as input with pull-up
    gpio_init(IR_BARCODE_PIN);
    gpio_set_dir(IR_BARCODE_PIN, GPIO_IN);
    gpio_pull_up(IR_BARCODE_PIN);

    // Attach interrupt handler
    gpio_set_irq_enabled_with_callback(
        IR_BARCODE_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &gpio_callback
    );

    printf("INTEG-T1: Line + Barcode Integration Test\n");
}

// ============================================================================
// MAIN
// ============================================================================

int main(void)
{
    init_hardware();

    while (true)
    {
        // ========= BARCODE TIMEOUT CHECK =========
        if (scan_state == SCAN_IN_PROGRESS)
        {
            uint32_t now = time_us_32();
            if ((now - last_edge_time) > BARCODE_TIMEOUT_US && bar_count >= 9)
            {
                scan_state = SCAN_COMPLETE;
                printf("[BARCODE] Scan complete (%u transitions)\n", bar_count);
            }
        }

        // ========= PROCESS COMPLETED BARCODE =========
        if (scan_state == SCAN_COMPLETE)
        {
            BarcodeCommand cmd = process_barcode();

            // Reset scanner state
            scan_state = SCAN_IDLE;
            bar_count = 0;

            if (cmd == CMD_LEFT)
            {
                printf("[CMD] Executing LEFT turn.\n");
                motors_turn_left();
                sleep_ms(800);
            }
            else if (cmd == CMD_RIGHT)
            {
                printf("[CMD] Executing RIGHT turn.\n");
                motors_turn_right();
                sleep_ms(800);
            }
            else if (cmd == CMD_STOP)
            {
                printf("[CMD] Executing STOP.\n");
                motors_stop();
                sleep_ms(1500);
            }

            // Resume line following after command
        }

        // ========= LINE FOLLOWING (simple bang-bang) =========
        uint16_t line_val = read_line_sensor();

        if (line_val > LINE_THRESHOLD)
        {
            // On dark line
            motors_turn_left();
            printf("[LINE] BLACK (%u) -> LEFT\n", line_val);
        }
        else
        {
            // Off line
            motors_turn_right();
            printf("[LINE] WHITE (%u) -> RIGHT\n", line_val);
        }

        sleep_ms(LINE_LOOP_DELAY_MS);
    }

    return 0;
}
