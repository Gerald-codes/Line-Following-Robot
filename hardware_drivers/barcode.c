/**
 * barcode.c
 * Code 39 barcode detection and decoding
 * Detects Code 39 format barcodes and interprets LEFT/RIGHT/STOP commands
 */

#include "barcode.h"
#include "ir_sensor.h"
#include "config.h"
#include <stdio.h>
#include <string.h>

// Code 39 timing (adjust based on speed and barcode size)
#define NARROW_BAR_MIN_MS 20
#define NARROW_BAR_MAX_MS 80
#define WIDE_BAR_MIN_MS 80
#define WIDE_BAR_MAX_MS 200

// Barcode state
static BarcodeState barcode_state = BARCODE_STATE_IDLE;
static uint32_t last_transition_time = 0;
static uint32_t bar_times[50];  // Store bar/space widths
static uint8_t bar_count = 0;
static bool last_sensor_state = false;
static BarcodeCommand last_command = BARCODE_NONE;
static char decoded_string[20];

// Code 39 character encoding (9 bars: narrow=0, wide=1)
// Format: bsbsbsbsb (b=bar, s=space)
typedef struct {
    char character;
    const char* pattern;  // 9 bits: 0=narrow, 1=wide
} Code39Char;

// Subset of Code 39 characters (just what we need + start/stop)
static const Code39Char code39_table[] = {
    {'*', "100101101"},  // Start/Stop character
    {'L', "101010011"},  // LEFT command
    {'R', "101001011"},  // RIGHT command
    {'S', "110010101"},  // STOP command
    {'0', "101001101"},
    {'1', "110100101"},
    {'2', "101100101"},
    {'3', "110110100"},
    {'4', "101001110"},
    {'5', "110100110"},
    {'6', "101100110"},
    {'7', "101001011"},
    {'8', "110100101"},
    {'9', "101100101"},
};

#define CODE39_TABLE_SIZE (sizeof(code39_table) / sizeof(Code39Char))

void barcode_init(void) {
    barcode_state = BARCODE_STATE_IDLE;
    bar_count = 0;
    last_sensor_state = false;
    last_command = BARCODE_NONE;
    decoded_string[0] = '\0';
    printf("✓ Code 39 barcode decoder initialized\n");
}

void barcode_reset(void) {
    barcode_state = BARCODE_STATE_IDLE;
    bar_count = 0;
    last_command = BARCODE_NONE;
    decoded_string[0] = '\0';
}

// Classify bar width as narrow (0) or wide (1)
static char classify_bar(uint32_t width_ms) {
    if (width_ms >= NARROW_BAR_MIN_MS && width_ms <= NARROW_BAR_MAX_MS) {
        return '0';  // Narrow
    } else if (width_ms >= WIDE_BAR_MIN_MS && width_ms <= WIDE_BAR_MAX_MS) {
        return '1';  // Wide
    } else {
        return '?';  // Invalid
    }
}

// Decode 9 bars/spaces into Code 39 character
static char decode_code39_char(uint32_t* bars, uint8_t count) {
    if (count != 9) return '?';
    
    char pattern[10];
    bool valid = true;
    
    // Convert bar widths to pattern string
    for (int i = 0; i < 9; i++) {
        pattern[i] = classify_bar(bars[i]);
        if (pattern[i] == '?') {
            valid = false;
            break;
        }
    }
    pattern[9] = '\0';
    
    if (!valid) return '?';
    
    // Match against Code 39 table
    for (int i = 0; i < CODE39_TABLE_SIZE; i++) {
        if (strcmp(pattern, code39_table[i].pattern) == 0) {
            return code39_table[i].character;
        }
    }
    
    return '?';  // No match
}

// Decode complete barcode string
static BarcodeCommand decode_barcode_command(const char* str) {
    // Code 39 format: *COMMAND*
    // Example: *L* = LEFT, *R* = RIGHT, *S* = STOP
    
    if (strlen(str) < 3) return BARCODE_UNKNOWN;
    
    // Check for start/stop markers
    if (str[0] != '*' || str[strlen(str)-1] != '*') {
        return BARCODE_UNKNOWN;
    }
    
    // Extract command (character between * markers)
    char cmd = str[1];
    
    switch (cmd) {
        case 'L':
            return BARCODE_LEFT;
        case 'R':
            return BARCODE_RIGHT;
        case 'S':
            return BARCODE_STOP;
        default:
            return BARCODE_UNKNOWN;
    }
}

BarcodeCommand barcode_update(void) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    bool sensor_sees_black = ir_barcode_detected();
    
    // Detect transitions (black→white or white→black)
    if (sensor_sees_black != last_sensor_state) {
        uint32_t bar_width = current_time - last_transition_time;
        
        switch (barcode_state) {
            case BARCODE_STATE_IDLE:
                // First transition - start detecting
                barcode_state = BARCODE_STATE_DETECTING;
                bar_count = 0;
                decoded_string[0] = '\0';
                last_transition_time = current_time;
                break;
                
            case BARCODE_STATE_DETECTING:
                // Record bar/space width
                if (bar_width >= NARROW_BAR_MIN_MS) {
                    bar_times[bar_count++] = bar_width;
                    
                    // Try to decode every 9 bars (one Code 39 character)
                    if (bar_count >= 9) {
                        char decoded_char = decode_code39_char(bar_times, 9);
                        
                        if (decoded_char != '?') {
                            // Valid character decoded
                            size_t len = strlen(decoded_string);
                            if (len < 19) {
                                decoded_string[len] = decoded_char;
                                decoded_string[len + 1] = '\0';
                            }
                            
                            // Check if complete barcode (ends with *)
                            if (decoded_char == '*' && len > 0) {
                                barcode_state = BARCODE_STATE_COMPLETE;
                            } else {
                                // Shift bars for next character
                                // Keep last bar as it might be start of next char
                                bar_count = 0;
                            }
                        } else {
                            // Invalid character - reset
                            barcode_state = BARCODE_STATE_IDLE;
                        }
                    }
                    
                    // Safety limit
                    if (bar_count >= 50) {
                        barcode_state = BARCODE_STATE_IDLE;
                    }
                }
                
                last_transition_time = current_time;
                break;
                
            case BARCODE_STATE_COMPLETE:
                // Already decoded - waiting for cooldown
                break;
                
            case BARCODE_STATE_COOLDOWN:
                // In cooldown period
                break;
        }
    }
    
    // Timeout detection
    if (barcode_state == BARCODE_STATE_DETECTING) {
        if (current_time - last_transition_time > 500) {  // 500ms timeout
            // Too long since last transition - probably end of barcode or error
            if (strlen(decoded_string) > 0 && decoded_string[strlen(decoded_string)-1] == '*') {
                barcode_state = BARCODE_STATE_COMPLETE;
            } else {
                barcode_state = BARCODE_STATE_IDLE;
            }
        }
    }
    
    // Process complete barcode
    if (barcode_state == BARCODE_STATE_COMPLETE) {
        last_command = decode_barcode_command(decoded_string);
        
        printf("\n[BARCODE] Decoded: '%s' → ", decoded_string);
        switch (last_command) {
            case BARCODE_LEFT:
                printf("TURN LEFT\n");
                break;
            case BARCODE_RIGHT:
                printf("TURN RIGHT\n");
                break;
            case BARCODE_STOP:
                printf("STOP\n");
                break;
            default:
                printf("UNKNOWN\n");
                break;
        }
        
        barcode_state = BARCODE_STATE_COOLDOWN;
        last_transition_time = current_time;
    }
    
    // Cooldown period
    if (barcode_state == BARCODE_STATE_COOLDOWN) {
        if (current_time - last_transition_time > 1000) {  // 1 second cooldown
            barcode_state = BARCODE_STATE_IDLE;
        }
    }
    
    last_sensor_state = sensor_sees_black;
    
    return last_command;
}

BarcodeCommand barcode_get_last_command(void) {
    return last_command;
}

void barcode_clear_command(void) {
    last_command = BARCODE_NONE;
}

BarcodeState barcode_get_state(void) {
    return barcode_state;
}

const char* barcode_command_to_string(BarcodeCommand cmd) {
    switch (cmd) {
        case BARCODE_LEFT:  return "TURN LEFT";
        case BARCODE_RIGHT: return "TURN RIGHT";
        case BARCODE_STOP:  return "STOP";
        case BARCODE_NONE:  return "NONE";
        default:            return "UNKNOWN";
    }
}

const char* barcode_get_decoded_string(void) {
    return decoded_string;
}
