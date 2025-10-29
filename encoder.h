#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"

// Encoder pins - Grove 2 and 3
#define LEFT_ENCODER 3      // GP2 (Grove 2)
#define RIGHT_ENCODER 5     // GP4 (Grove 3)

// Encoder counters (extern so main can access them)
extern volatile int left_count;
extern volatile int right_count;

// Last state for direction detection
extern volatile uint8_t left_last_state;
extern volatile uint8_t right_last_state;

// Function declarations
void encoder_init(void);
void encoder_reset(void);
void print_encoders(void);
int get_left_encoder(void);
int get_right_encoder(void);
void test_raw_encoder_signals(void);
void test_all_grove_pins(void);

#endif // ENCODER_H