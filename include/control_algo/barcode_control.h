/**
 * @file    barcode_control.h (TUNED VERSION)
 * @brief   High-level barcode control interface - TUNED FOR CONSISTENCY
 */

#ifndef BARCODE_CONTROL_H
#define BARCODE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "barcode_scanner.h"

typedef enum
{
    BARCODE_ACTION_NONE,
    BARCODE_ACTION_TURN_LEFT,
    BARCODE_ACTION_TURN_RIGHT,
    BARCODE_ACTION_SPEED_SLOW,
    BARCODE_ACTION_SPEED_FAST
} BarcodeAction;

/* CONFIGURATION - TUNED FOR CONSISTENCY */

#define BARCODE_TURN_DURATION_MS    1000        /* How long to turn for barcode commands */
#define BARCODE_TURN_SPEED          35          /* Turn speed for barcode turns */
#define LEFT_MOTOR_COMPENSATION     2           /* Add this to left motor (weaker motor) */

/* Speed levels - REDUCED FOR BETTER DETECTION */
/* Lower speeds = more consistent timing = better detection! */
#define BARCODE_SPEED_SLOW          20          /* Slow speed (was 25) */
#define BARCODE_SPEED_FAST          30          /* Fast speed (was 45) - Try this first! */

/* If still inconsistent, try even slower: */
// #define BARCODE_SPEED_SLOW          15
// #define BARCODE_SPEED_FAST          25

/* PUBLIC FUNCTIONS */

void barcode_control_init(void);
BarcodeCommand barcode_check_for_detection(void);
BarcodeAction barcode_get_action(BarcodeCommand cmd);
BarcodeAction handle_barcode_detected(BarcodeCommand cmd);
bool handle_barcode_turn(void);
bool barcode_is_scanning(void);
int barcode_get_current_speed(void);
void barcode_control_reset(void);

#endif /* BARCODE_CONTROL_H */