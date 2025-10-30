/**
 * encoder_test_main.c
 * Test program to calibrate encoder pulses per revolution
 */

#include "pico/stdlib.h"
#include "encoder.h"
#include "motor.h"
#include <stdio.h>

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║            ENCODER CALIBRATION TEST                           ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    // Initialize encoders
    encoder_init();
    printf("✓ Encoders initialized\n\n");
    
    // Initialize motors (in case we need to test with power)
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    printf("✓ Motors initialized and stopped\n\n");
    
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  MANUAL ROTATION TEST\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    // Test LEFT wheel
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("TEST 1: LEFT WHEEL\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");
    
    encoder_reset();
    printf("1. Lift the robot so LEFT wheel can spin freely\n");
    printf("2. Rotate LEFT wheel EXACTLY 10 full rotations (count carefully!)\n");
    printf("3. Press Enter when done\n\n");
    printf("Waiting for input...\n");
    
    getchar();
    
    int left_pulses = get_left_encoder();
    int left_ppr = left_pulses / 10;
    
    printf("\n");
    printf("━━━ LEFT WHEEL RESULTS ━━━\n");
    printf("Total pulses counted: %d\n", left_pulses);
    printf("Pulses per revolution: %d\n", left_ppr);
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");
    
    sleep_ms(2000);
    
    // Test RIGHT wheel
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("TEST 2: RIGHT WHEEL\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");
    
    encoder_reset();
    printf("1. Keep robot lifted so RIGHT wheel can spin freely\n");
    printf("2. Rotate RIGHT wheel EXACTLY 10 full rotations (count carefully!)\n");
    printf("3. Press Enter when done\n\n");
    printf("Waiting for input...\n");
    
    getchar();
    
    int right_pulses = get_right_encoder();
    int right_ppr = right_pulses / 10;
    
    printf("\n");
    printf("━━━ RIGHT WHEEL RESULTS ━━━\n");
    printf("Total pulses counted: %d\n", right_pulses);
    printf("Pulses per revolution: %d\n", right_ppr);
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");
    
    // Summary
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    CALIBRATION SUMMARY                        ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    printf("LEFT wheel:  %d pulses per revolution\n", left_ppr);
    printf("RIGHT wheel: %d pulses per revolution\n", right_ppr);
    printf("\n");
    
    if (left_ppr == right_ppr) {
        printf("✓ Both wheels have same encoder resolution\n");
        printf("\n");
        printf("═══════════════════════════════════════════════════════════════\n");
        printf("  UPDATE YOUR CONFIG.H:\n");
        printf("═══════════════════════════════════════════════════════════════\n\n");
        printf("Change this line in config.h:\n");
        printf("  #define PULSES_PER_REVOLUTION %d\n", left_ppr);
        printf("\n");
    } else {
        printf("⚠ WARNING: Left and right wheels have different counts!\n");
        printf("  This might indicate:\n");
        printf("  - You didn't count rotations carefully\n");
        printf("  - One encoder is faulty\n");
        printf("  - Encoders are different types\n");
        printf("\n");
        printf("Recommended: Use average value = %d\n", (left_ppr + right_ppr) / 2);
        printf("\n");
    }
    
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  ADDITIONAL DIAGNOSTIC INFO\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    printf("Current config.h says:\n");
    printf("  PULSES_PER_REVOLUTION = 360\n");
    printf("  WHEEL_DIAMETER_MM = 65.0\n");
    printf("  WHEEL_BASE_MM = 130.0\n\n");
    
    if (left_ppr < 100) {
        printf("ℹ Your encoders give %d PPR (relatively low resolution)\n", left_ppr);
        printf("  This is normal for simple encoder discs\n");
    } else if (left_ppr > 300) {
        printf("ℹ Your encoders give %d PPR (high resolution)\n", left_ppr);
        printf("  This is good for accurate motion control\n");
    } else {
        printf("ℹ Your encoders give %d PPR (moderate resolution)\n", left_ppr);
    }
    
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  OPTIONAL: POWERED TEST\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    printf("Want to test with motor power? (y/n): ");
    char response = getchar();
    
    if (response == 'y' || response == 'Y') {
        printf("\n\n⚠ POWERED TEST STARTING...\n");
        printf("Robot will drive motors at 30%% power for 2 seconds\n");
        printf("Press Enter to start...\n");
        while(getchar() != '\n');  // Clear buffer
        getchar();
        
        encoder_reset();
        printf("\nRunning motors...\n");
        
        motor_drive(M1A, M1B, -30);
        motor_drive(M2A, M2B, -30);
        
        // Monitor for 2 seconds
        for (int i = 0; i < 20; i++) {
            sleep_ms(100);
            printf("L: %5d | R: %5d\n", get_left_encoder(), get_right_encoder());
        }
        
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        
        int final_left = get_left_encoder();
        int final_right = get_right_encoder();
        
        printf("\nPowered test results:\n");
        printf("  Left:  %d pulses in 2 seconds\n", final_left);
        printf("  Right: %d pulses in 2 seconds\n", final_right);
        
        if (final_left > 0 && final_right > 0) {
            printf("  ✓ Both encoders working with motor power\n");
        } else {
            printf("  ✗ Problem detected - check encoder connections!\n");
        }
    }
    
    printf("\n\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    TEST COMPLETE!                             ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    
    while (true) {
        tight_loop_contents();
    }
    
    return 0;
}