/**
 * @file    barcode_control.c (FIXED VERSION)
 * @brief   High-level barcode control implementation
 * @details Simplified version with reliable turn execution
 *          
 *   FIXES APPLIED:
 *   - Removed decode pause (executes turns immediately)
 *   - Removed speed reduction during scanning
 *   - Simplified state management
 *   - Better motor compensation
 */

/* Includes */
#include "barcode_control.h"
#include "barcode_scanner.h"
#include "motor.h"
#include "pico/stdlib.h"
#include <stdio.h>

/* Barcode sensor pin */
#ifndef IR_BARCODE_PIN
#define IR_BARCODE_PIN 6
#endif

/* Internal state */
static BarcodeCommand pending_turn = BARCODE_CMD_NONE;
static uint32_t turn_start_time = 0;
static int current_speed = BARCODE_SPEED_FAST;

/**
 * @brief Decode barcode character to determine action
 * @param letter Character from barcode scanner
 * @return BarcodeAction to execute
 */
static BarcodeAction decode_barcode_action(char letter)
{
    printf("[BARCODE] Decoding character: '%c'\n", letter);
    
    /* Numbers 1-5: SLOW speed */
    if (letter >= '1' && letter <= '5')
    {
        current_speed = BARCODE_SPEED_SLOW;
        printf("[BARCODE] Speed set to SLOW: %d\n", current_speed);
        return BARCODE_ACTION_SPEED_SLOW;
    }
    
    /* Numbers 6-9: FAST speed */
    if (letter >= '6' && letter <= '9')
    {
        current_speed = BARCODE_SPEED_FAST;
        printf("[BARCODE] Speed set to FAST: %d\n", current_speed);
        return BARCODE_ACTION_SPEED_FAST;
    }
    
    /* Number 0: Also FAST speed */
    if (letter == '0')
    {
        current_speed = BARCODE_SPEED_FAST;
        printf("[BARCODE] Speed set to FAST: %d\n", current_speed);
        return BARCODE_ACTION_SPEED_FAST;
    }
    
    /* Letters: Check if LEFT or RIGHT turn */
    switch (letter)
    {
        /* RIGHT letters */
        case 'A': case 'C': case 'E': case 'G': case 'I':
        case 'K': case 'M': case 'O': case 'Q': case 'S':
        case 'U': case 'W': case 'Y':
            return BARCODE_ACTION_TURN_RIGHT;
        
        /* LEFT letters */
        case 'B': case 'D': case 'F': case 'H': case 'J':
        case 'L': case 'N': case 'P': case 'R': case 'T':
        case 'V': case 'X': case 'Z':
            return BARCODE_ACTION_TURN_LEFT;
        
        default:
            return BARCODE_ACTION_NONE;
    }
}

/**
 * @brief Initialize barcode control system
 */
void barcode_control_init(void)
{
    barcode_scanner_init(IR_BARCODE_PIN);
    pending_turn = BARCODE_CMD_NONE;
    turn_start_time = 0;
    current_speed = BARCODE_SPEED_FAST;
    
    printf(" ✓ Barcode Control System\n");
    printf("   Initial speed: %d (FAST)\n", current_speed);
}

/**
 * @brief Check for barcode detection
 * @return BarcodeCommand if detected, BARCODE_CMD_NONE otherwise
 */
BarcodeCommand barcode_check_for_detection(void)
{
    return barcode_scanner_update();
}

/**
 * @brief Get action for barcode command
 * @param cmd BarcodeCommand to decode
 * @return BarcodeAction to execute
 */
BarcodeAction barcode_get_action(BarcodeCommand cmd)
{
    /* For turn commands, return directly */
    if (cmd == BARCODE_CMD_LEFT)
    {
        return BARCODE_ACTION_TURN_LEFT;
    }
    else if (cmd == BARCODE_CMD_RIGHT)
    {
        return BARCODE_ACTION_TURN_RIGHT;
    }
    
    /* For UNKNOWN commands (numbers), decode the character */
    if (cmd == BARCODE_CMD_UNKNOWN)
    {
        char decoded_char = barcode_scanner_get_last_character();
        return decode_barcode_action(decoded_char);
    }
    
    return BARCODE_ACTION_NONE;
}

/**
 * @brief Handle barcode detected state
 * @param cmd BarcodeCommand that was detected
 * @return BarcodeAction that will be executed
 */
BarcodeAction handle_barcode_detected(BarcodeCommand cmd)
{
    printf("\n[BARCODE] ========================================\n");
    printf("[BARCODE] Command detected: %s\n", barcode_command_to_string(cmd));
    printf("[BARCODE] Current speed before: %d\n", current_speed);
    
    /* Get the decoded character for all cases */
    char decoded_char = barcode_scanner_get_last_character();
    printf("[BARCODE] Decoded character: '%c'\n", decoded_char);
    
    /* Decode what action to take */
    BarcodeAction action = decode_barcode_action(decoded_char);
    printf("[BARCODE] Action: ");
    
    switch (action)
    {
        case BARCODE_ACTION_TURN_LEFT:
            printf("TURN LEFT (immediate execution)\n");
            
            /* FIX #1: Start turn IMMEDIATELY - no pause! */
            pending_turn = BARCODE_CMD_LEFT;
            turn_start_time = to_ms_since_boot(get_absolute_time());
            
            /* Stop motors briefly before turn */
            motor_stop(M1A, M1B);
            motor_stop(M2A, M2B);
            sleep_ms(50);  /* Very brief pause for momentum to settle */
            break;
        
        case BARCODE_ACTION_TURN_RIGHT:
            printf("TURN RIGHT (immediate execution)\n");
            
            /* FIX #1: Start turn IMMEDIATELY - no pause! */
            pending_turn = BARCODE_CMD_RIGHT;
            turn_start_time = to_ms_since_boot(get_absolute_time());
            
            /* Stop motors briefly before turn */
            motor_stop(M1A, M1B);
            motor_stop(M2A, M2B);
            sleep_ms(50);  /* Very brief pause for momentum to settle */
            break;
        
        case BARCODE_ACTION_SPEED_SLOW:
            printf("SPEED SLOW (%d)\n", current_speed);
            break;
        
        case BARCODE_ACTION_SPEED_FAST:
            printf("SPEED FAST (%d)\n", current_speed);
            break;
        
        default:
            printf("NONE\n");
            break;
    }
    
    printf("[BARCODE] Current speed after: %d\n", current_speed);
    printf("[BARCODE] ========================================\n\n");
    
    return action;
}

/**
 * @brief Handle barcode turn execution
 * @return true if turn complete, false if still turning
 */
bool handle_barcode_turn(void)
{
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    uint32_t elapsed = current_time - turn_start_time;
    
    if (elapsed < BARCODE_TURN_DURATION_MS)
    {
        /* Execute the turn with left motor compensation */
        if (pending_turn == BARCODE_CMD_LEFT)
        {
            /* Turn left: left motor backward, right motor forward */
            motor_drive(M1A, M1B, -(BARCODE_TURN_SPEED + LEFT_MOTOR_COMPENSATION));
            motor_drive(M2A, M2B, BARCODE_TURN_SPEED);
        }
        else if (pending_turn == BARCODE_CMD_RIGHT)
        {
            /* Turn right: left motor forward, right motor backward */
            motor_drive(M1A, M1B, (BARCODE_TURN_SPEED + LEFT_MOTOR_COMPENSATION));
            motor_drive(M2A, M2B, -BARCODE_TURN_SPEED);
        }
        
        return false;
    }
    else
    {
        /* Turn complete */
        printf("\n[BARCODE] Turn complete - returning to line following\n");
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        pending_turn = BARCODE_CMD_NONE;
        return true;
    }
}

/**
 * @brief Check if currently scanning a barcode
 * @return true if scanning in progress
 */
bool barcode_is_scanning(void)
{
    BarcodeScanState state = barcode_scanner_get_state();
    return (state == BARCODE_SCAN_IN_PROGRESS || state == BARCODE_SCAN_COMPLETE);
}

/**
 * @brief Get current speed setting
 * @details FIX #2: No longer reduces speed during scanning!
 * @return Current speed value
 */
int barcode_get_current_speed(void)
{
    /* FIX #2: Always return the set speed - no reduction during scanning */
    /* This keeps timing consistent for reliable barcode detection */
    return current_speed;
}

/**
 * @brief Reset barcode control system
 */
void barcode_control_reset(void)
{
    barcode_scanner_reset();
    pending_turn = BARCODE_CMD_NONE;
    turn_start_time = 0;
    current_speed = BARCODE_SPEED_FAST;
}