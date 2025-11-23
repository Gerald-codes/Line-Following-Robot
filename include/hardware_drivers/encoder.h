/**
 * @file    encoder.h
 * @brief   Encoder hardware interface and utilities
 * @details
 *   Encoder pin assignments, encoder counters, and key functions to
 *   initialize, reset, and read Grove encoders. Includes diagnostic
 *   utilities for test and wiring verification.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"

/**
 * @brief   Encoder pin assignments (Grove 2 and 3)
 * @details Comments and numbers should match your physical GPIO.
 *          Use diagnostic functions to verify correct pin usage.
 */
#define LEFT_ENCODER    3  /**< GP3 (Grove 2 - Signal pin, verify with test) */
#define RIGHT_ENCODER   5  /**< GP5 (Grove 3 - Signal pin, verify with test) */

/**
 * @brief   Alternative pins if wiring is different
 * @details Grove 2 can be: GP2 or GP3; Grove 3 can be: GP4 or GP5
 */

/**
 * @brief   Encoder counters (extern for main application access)
 */
extern volatile int left_count;
extern volatile int right_count;

/**
 * @brief   Last state for direction detection
 */
extern volatile uint8_t left_last_state;
extern volatile uint8_t right_last_state;

/**
 * @brief   Initialize encoders and hardware
 */
void encoder_init(void);

/**
 * @brief   Reset encoder counters
 */
void encoder_reset(void);

/**
 * @brief   Print encoder values for debugging
 */
void print_encoders(void);

/**
 * @brief   Get left encoder count
 * @return  Count value of left encoder
 */
int get_left_encoder(void);

/**
 * @brief   Get right encoder count
 * @return  Count value of right encoder
 */
int get_right_encoder(void);

/**
 * @brief   Test the raw encoder signals (diagnostic)
 * @details Verify current pin assignments for encoders
 */
void test_raw_encoder_signals(void);

/**
 * @brief   Scan all Grove pins to find encoder connections (diagnostic)
 */
void test_all_grove_pins(void);

#endif /* ENCODER_H */
