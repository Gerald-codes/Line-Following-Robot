/**
 * motor_test_simple.c
 * Test motors at fixed power - NO PID, NO control
 * Just drives both motors at same power to verify basic operation
 */

#include "pico/stdlib.h"
#include "motor.h"
#include "encoder.h"
#include <stdio.h>

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                  SIMPLE MOTOR TEST                            ║\n");
    printf("║              No PID - Just Fixed Power                        ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    encoder_init();
    
    printf("Motors and encoders initialized\n\n");
    
    // Test 1: Forward at 30% power
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("TEST 1: Both motors at +30 power (should go FORWARD)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");
    
    encoder_reset();
    motor_drive(M1A, M1B, 30);
    motor_drive(M2A, M2B, 30);
    
    printf("Running for 3 seconds...\n");
    for (int i = 0; i < 30; i++) {
        sleep_ms(100);
        printf("L: %5d | R: %5d\n", get_left_encoder(), get_right_encoder());
    }
    
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    int left_total = get_left_encoder();
    int right_total = get_right_encoder();
    
    printf("\n✓ Test 1 Complete\n");
    printf("Left encoder:  %d pulses\n", left_total);
    printf("Right encoder: %d pulses\n", right_total);
    
    if (left_total > 0 && right_total > 0) {
        printf("✓ Both wheels moved FORWARD\n");
    } else if (left_total < 0 && right_total < 0) {
        printf("✗ Both wheels moved BACKWARD (power sign is inverted!)\n");
    } else {
        printf("✗ Wheels moved in different directions (wiring problem!)\n");
    }
    
    sleep_ms(2000);
    
    // Test 2: Backward at -30% power
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("TEST 2: Both motors at -30 power (should go BACKWARD)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");
    
    encoder_reset();
    motor_drive(M1A, M1B, -30);
    motor_drive(M2A, M2B, -30);
    
    printf("Running for 3 seconds...\n");
    for (int i = 0; i < 30; i++) {
        sleep_ms(100);
        printf("L: %5d | R: %5d\n", get_left_encoder(), get_right_encoder());
    }
    
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    left_total = get_left_encoder();
    right_total = get_right_encoder();
    
    printf("\n✓ Test 2 Complete\n");
    printf("Left encoder:  %d pulses\n", left_total);
    printf("Right encoder: %d pulses\n", right_total);
    
    if (left_total < 0 && right_total < 0) {
        printf("✓ Both wheels moved BACKWARD\n");
    } else if (left_total > 0 && right_total > 0) {
        printf("✗ Both wheels moved FORWARD (power sign is inverted!)\n");
    } else {
        printf("✗ Wheels moved in different directions (wiring problem!)\n");
    }
    
    sleep_ms(2000);
    
    // Test 3: Different speeds
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("TEST 3: Left=50, Right=30 (should turn right slightly)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");
    
    encoder_reset();
    motor_drive(M1A, M1B, 50);
    motor_drive(M2A, M2B, 30);
    
    printf("Running for 3 seconds...\n");
    for (int i = 0; i < 30; i++) {
        sleep_ms(100);
        printf("L: %5d | R: %5d\n", get_left_encoder(), get_right_encoder());
    }
    
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    left_total = get_left_encoder();
    right_total = get_right_encoder();
    
    printf("\n✓ Test 3 Complete\n");
    printf("Left encoder:  %d pulses (should be higher)\n", left_total);
    printf("Right encoder: %d pulses (should be lower)\n", right_total);
    
    if (left_total > right_total) {
        printf("✓ Left wheel moved more (correct!)\n");
    } else {
        printf("✗ Right wheel moved more (motors might be swapped!)\n");
    }
    
    printf("\n\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    DIAGNOSIS SUMMARY                          ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    printf("Based on results above:\n\n");
    printf("1. If positive power goes BACKWARD:\n");
    printf("   → Use: motor_drive(M1A, M1B, -(int)output)\n\n");
    
    printf("2. If positive power goes FORWARD:\n");
    printf("   → Use: motor_drive(M1A, M1B, (int)output)\n\n");
    
    printf("3. If wheels go different directions:\n");
    printf("   → Check motor wiring!\n\n");
    
    printf("4. If one wheel doesn't move:\n");
    printf("   → Check that motor connections!\n\n");
    
    printf("Press Ctrl+C to exit\n");
    
    while (true) {
        tight_loop_contents();
    }
    
    return 0;
}