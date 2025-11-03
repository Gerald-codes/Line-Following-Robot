/**
 * motor_test.c
 * Simple motor test to verify wiring and orientation
 */

#include "pico/stdlib.h"
#include "motor.h"
#include "pin_definitions.h"
#include <stdio.h>

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║              MOTOR ORIENTATION TEST                          ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    
    // Initialize motors
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    printf("✓ Motors initialized\n");
    printf("  Left motor: GP%d, GP%d\n", M1A, M1B);
    printf("  Right motor: GP%d, GP%d\n\n", M2A, M2B);
    
    printf("INSTRUCTIONS:\n");
    printf("  Watch the robot and verify each movement\n");
    printf("  Robot should be on the ground/track\n\n");
    
    sleep_ms(2000);
    
    // Test 1: Both motors forward
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("TEST 1: STRAIGHT FORWARD\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("Both motors at +50 power\n");
    printf("Expected: Robot moves STRAIGHT FORWARD\n");
    printf("Starting in 2 seconds...\n\n");
    sleep_ms(2000);
    
    motor_drive(M1A, M1B,-50);
    motor_drive(M2A, M2B, -52);
    printf("▶ RUNNING (3 seconds)...\n");
    sleep_ms(3000);
    
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    printf("■ STOPPED\n\n");
    printf("Did robot move STRAIGHT FORWARD? (y/n): ");
    sleep_ms(3000);
    printf("\n\n");
    
    // Test 2: Turn left
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("TEST 2: TURN LEFT\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("Left motor: +30, Right motor: +70\n");
    printf("Expected: Robot turns LEFT (counterclockwise)\n");
    printf("Starting in 2 seconds...\n\n");
    sleep_ms(2000);
    
    motor_drive(M1A, M1B, -70);   // right faster
    motor_drive(M2A, M2B, -30);   // left slower
    printf("▶ RUNNING (3 seconds)...\n");
    sleep_ms(3000);
    
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    printf("■ STOPPED\n\n");
    printf("Did robot turn LEFT? (y/n): ");
    sleep_ms(3000);
    printf("\n\n");
    
    // Test 3: Turn right
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("TEST 3: TURN RIGHT\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("Left motor: +70, Right motor: +30\n");
    printf("Expected: Robot turns RIGHT (clockwise)\n");
    printf("Starting in 2 seconds...\n\n");
    sleep_ms(2000);
    
    motor_drive(M1A, M1B, -30);   // right slower
    motor_drive(M2A, M2B, -70);   // left faster
    printf("▶ RUNNING (3 seconds)...\n");
    sleep_ms(3000);
    
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    printf("■ STOPPED\n\n");
    printf("Did robot turn RIGHT? (y/n): ");
    sleep_ms(3000);
    printf("\n\n");
    
    // Test 4: Spin left (in place)
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("TEST 4: SPIN LEFT (in place)\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("Left motor: -50, Right motor: +50\n");
    printf("Expected: Robot spins LEFT in place\n");
    printf("Starting in 2 seconds...\n\n");
    sleep_ms(2000);
    
    motor_drive(M1A, M1B, -50);  // right forward
    motor_drive(M2A, M2B, 50);   // left backward
    printf("▶ RUNNING (2 seconds)...\n");
    sleep_ms(2000);
    
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    printf("■ STOPPED\n\n");
    printf("Did robot spin LEFT? (y/n): ");
    sleep_ms(3000);
    printf("\n\n");
    
    // Test 5: Spin right (in place)
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("TEST 5: SPIN RIGHT (in place)\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("Left motor: +50, Right motor: -50\n");
    printf("Expected: Robot spins RIGHT in place\n");
    printf("Starting in 2 seconds...\n\n");
    sleep_ms(2000);
    
    motor_drive(M1A, M1B, 50);   // right backwards
    motor_drive(M2A, M2B, -50);  // left forwards
    printf("▶ RUNNING (2 seconds)...\n");
    sleep_ms(2000);
    
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    printf("■ STOPPED\n\n");
    printf("Did robot spin RIGHT? (y/n): ");
    sleep_ms(3000);
    printf("\n\n");
    
    // Summary
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST COMPLETE - DIAGNOSIS\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    printf("POSSIBLE ISSUES:\n\n");
    
    printf("1. If robot moved BACKWARD instead of forward:\n");
    printf("   → BOTH motors wired backwards\n");
    printf("   → Fix: Swap motor wires OR add minus signs in code\n\n");
    
    printf("2. If robot turned RIGHT when expecting LEFT:\n");
    printf("   → Motors are swapped (left/right reversed)\n");
    printf("   → Fix: Swap M1 ↔ M2 in code OR physically swap motors\n\n");
    
    printf("3. If robot turned LEFT when expecting LEFT but wobbled:\n");
    printf("   → One motor stronger than other\n");
    printf("   → Fix: Tune LEFT_MOTOR_CORRECTION / RIGHT_MOTOR_CORRECTION\n\n");
    
    printf("4. If one motor didn't move:\n");
    printf("   → Motor disconnected or driver issue\n");
    printf("   → Fix: Check wiring and motor driver\n\n");
    
    printf("5. If everything worked correctly:\n");
    printf("   → Motors are wired correctly! ✓\n");
    printf("   → Problem is in PID/sensor logic\n\n");
    
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    return 0;
}