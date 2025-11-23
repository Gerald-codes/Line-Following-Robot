#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"

// Encoder pins - Grove 2 and 3
// Note: Comments MUST match the actual GPIO numbers used!
// Run test_all_grove_pins() to verify which pins your encoders use
#define LEFT_ENCODER 3      // GP3 (Grove 2 - Signal pin, verify with test)
#define RIGHT_ENCODER 5     // GP5 (Grove 3 - Signal pin, verify with test)

// Alternative pins if your wiring is different:
// Grove 2 can be: GP2 or GP3
// Grove 3 can be: GP4 or GP5

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

// Diagnostic functions - use these to verify your encoder wiring!
void test_raw_encoder_signals(void);  // Test current pin assignments
void test_all_grove_pins(void);       // Scan all Grove 2/3 pins to find encoders

#endif // ENCODER_H
