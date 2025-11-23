#include "encoder.h"
#include "hardware/gpio.h"
#include <stdio.h>

// Encoder counters
volatile int left_count = 0;
volatile int right_count = 0;

// Last state for direction detection (if using single channel)
volatile uint8_t left_last_state = 0;
volatile uint8_t right_last_state = 0;

// Encoder interrupt handler for ACTIVE-LOW encoders
// Active-low means: idle=0, pulse=0→1→0
void encoder_isr(uint gpio, uint32_t events) {
    if (gpio == LEFT_ENCODER) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            left_count++;  // Count rising edge (0→1)
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            left_count++;  // Count falling edge (1→0) for better resolution
        }
    } 
    else if (gpio == RIGHT_ENCODER) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            right_count++;  // Count rising edge (0→1)
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            right_count++;  // Count falling edge (1→0) for better resolution
        }
    }
}

// Initialize encoders for ACTIVE-LOW operation on GP2 and GP4
void encoder_init(void) {
    // Configure encoder pins as inputs with pull-up resistors
    // Pull-up ensures clean transition for active-low encoders
    gpio_init(LEFT_ENCODER);
    gpio_set_dir(LEFT_ENCODER, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER);
    
    gpio_init(RIGHT_ENCODER);
    gpio_set_dir(RIGHT_ENCODER, GPIO_IN);
    gpio_pull_up(RIGHT_ENCODER);
    
    // Enable interrupts on BOTH rising and falling edges for maximum resolution
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER, 
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_isr);
    gpio_set_irq_enabled(RIGHT_ENCODER, 
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    
    printf("Encoders initialized (ACTIVE-LOW)\n");
    printf("Left Encoder: GP%d (Grove 2)\n", LEFT_ENCODER);
    printf("Right Encoder: GP%d (Grove 3)\n", RIGHT_ENCODER);
}

// Reset encoder counts
void encoder_reset(void) {
    left_count = 0;
    right_count = 0;
}

// Print encoder values with pin states
void print_encoders(void) {
    int left_state = gpio_get(LEFT_ENCODER);
    int right_state = gpio_get(RIGHT_ENCODER);
    printf("Left Encoder: %d (GP%d=%d) | Right Encoder: %d (GP%d=%d)\n", 
           left_count, LEFT_ENCODER, left_state, right_count, RIGHT_ENCODER, right_state);
}

// Get individual encoder counts
int get_left_encoder(void) {
    return left_count;
}

int get_right_encoder(void) {
    return right_count;
}

// Test raw encoder signals
void test_raw_encoder_signals(void) {
    printf("\n=== RAW ENCODER SIGNAL TEST (ACTIVE-LOW) ===\n");
    printf("Testing GP%d (Left) and GP%d (Right)\n", LEFT_ENCODER, RIGHT_ENCODER);
    printf("Idle state should be 0, pulses should go 0→1→0\n");
    printf("Manually spin the wheels NOW...\n\n");
    
    int left_changes = 0;
    int right_changes = 0;
    int last_left = gpio_get(LEFT_ENCODER);
    int last_right = gpio_get(RIGHT_ENCODER);
    
    for(int i = 0; i < 100; i++) {
        int curr_left = gpio_get(LEFT_ENCODER);
        int curr_right = gpio_get(RIGHT_ENCODER);
        
        if (curr_left != last_left) {
            left_changes++;
            printf(">>> LEFT CHANGED %d→%d ", last_left, curr_left);
        }
        if (curr_right != last_right) {
            right_changes++;
            printf(">>> RIGHT CHANGED %d→%d ", last_right, curr_right);
        }
        
        printf("GP%d:%d GP%d:%d | Changes L:%d R:%d | Counts L:%d R:%d\n", 
               LEFT_ENCODER, curr_left, RIGHT_ENCODER, curr_right, 
               left_changes, right_changes, left_count, right_count);
        
        last_left = curr_left;
        last_right = curr_right;
        sleep_ms(50);
    }
    
    printf("\n=== TEST COMPLETE ===\n");
    printf("GP%d (Left) pin changes detected: %d\n", LEFT_ENCODER, left_changes);
    printf("GP%d (Right) pin changes detected: %d\n", RIGHT_ENCODER, right_changes);
    printf("Left interrupt count: %d\n", left_count);
    printf("Right interrupt count: %d\n", right_count);
    
    if (left_changes == 0 && right_changes == 0) {
        printf("\n⚠️  NO CHANGES DETECTED!\n");
        printf("Check: Signal wires connected to GP2 and GP4?\n");
        printf("Check: Encoders powered (red LEDs on)?\n");
    }
}

// Test all Grove 2 & 3 pins to verify connections
void test_all_grove_pins(void) {
    printf("\n=== TESTING GROVE 2 & 3 PINS ===\n");
    printf("Grove 2: GP2, GP3\n");
    printf("Grove 3: GP4, GP5\n");
    printf("Spin the wheels now and see which pins change!\n\n");
    
    // Initialize all Grove 2 and 3 pins
    gpio_init(2);
    gpio_set_dir(2, GPIO_IN);
    gpio_pull_up(2);
    
    gpio_init(3);
    gpio_set_dir(3, GPIO_IN);
    gpio_pull_up(3);
    
    gpio_init(4);
    gpio_set_dir(4, GPIO_IN);
    gpio_pull_up(4);
    
    gpio_init(5);
    gpio_set_dir(5, GPIO_IN);
    gpio_pull_up(5);
    
    int prev_2 = gpio_get(2);
    int prev_3 = gpio_get(3);
    int prev_4 = gpio_get(4);
    int prev_5 = gpio_get(5);
    
    for(int i = 0; i < 50; i++) {
        int curr_2 = gpio_get(2);
        int curr_3 = gpio_get(3);
        int curr_4 = gpio_get(4);
        int curr_5 = gpio_get(5);
        
        // Highlight changes
        if (curr_2 != prev_2) printf(">>> GP2 CHANGED! ");
        if (curr_3 != prev_3) printf(">>> GP3 CHANGED! ");
        if (curr_4 != prev_4) printf(">>> GP4 CHANGED! ");
        if (curr_5 != prev_5) printf(">>> GP5 CHANGED! ");
        
        printf("GP2:%d  GP3:%d  GP4:%d  GP5:%d\n",
               curr_2, curr_3, curr_4, curr_5);
        
        prev_2 = curr_2;
        prev_3 = curr_3;
        prev_4 = curr_4;
        prev_5 = curr_5;
        
        sleep_ms(100);
    }
    
    printf("\n=== TEST COMPLETE ===\n");
    printf("The pins that showed 'CHANGED' are your encoder signal pins!\n");
    printf("Update LEFT_ENCODER and RIGHT_ENCODER in encoder.h accordingly.\n");
}