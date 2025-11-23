/**
 * test_motor.c
 * 
 * Unit tests for motor driver
 * 
 * Test Cases:
 * 1. Motor Forward/Reverse Test (100 iterations)
 * 2. Motor Speed Consistency Test (100 iterations)
 * 3. Motor Stop Test (100 iterations)
 */

#include "pico/stdlib.h"
#include "motor.h"
#include "pin_definitions.h"
#include <stdio.h>

// Test configuration
#define TEST_ITERATIONS 100
#define TEST_DELAY_MS 50

// Test result tracking
typedef struct {
    int passed;
    int failed;
    int total;
} TestResults;

// Color codes for terminal output
#define COLOR_GREEN "\033[32m"
#define COLOR_RED "\033[31m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_BLUE "\033[34m"
#define COLOR_RESET "\033[0m"

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static void print_test_header(const char* test_name) {
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  %s\n", test_name);
    printf("═══════════════════════════════════════════════════════════════\n");
}

static void print_test_result(TestResults* results) {
    printf("\n");
    printf("───────────────────────────────────────────────────────────────\n");
    printf("  Results: ");
    
    if (results->failed == 0) {
        printf(COLOR_GREEN "✓ ALL PASSED" COLOR_RESET);
    } else {
        printf(COLOR_RED "✗ SOME FAILED" COLOR_RESET);
    }
    
    printf("\n");
    printf("  Passed:  %s%d/%d%s\n", COLOR_GREEN, results->passed, results->total, COLOR_RESET);
    printf("  Failed:  %s%d/%d%s\n", COLOR_RED, results->failed, results->total, COLOR_RESET);
    printf("  Success: %.1f%%\n", (results->passed * 100.0f) / results->total);
    printf("───────────────────────────────────────────────────────────────\n");
}

static void print_progress(int iteration, int total) {
    if (iteration % 10 == 0 || iteration == total) {
        printf("  Progress: [%d/%d] ", iteration, total);
        
        // Progress bar
        int bar_width = 30;
        int filled = (iteration * bar_width) / total;
        printf("[");
        for (int i = 0; i < bar_width; i++) {
            if (i < filled) printf("█");
            else printf("░");
        }
        printf("] %.0f%%\r", (iteration * 100.0f) / total);
        fflush(stdout);
    }
}

// ============================================================================
// TEST CASE 1: MOTOR FORWARD/REVERSE TEST
// ============================================================================

/**
 * Test: Verify motor can switch between forward and reverse directions
 * 
 * Expected: Motor should respond to positive and negative power values
 * Success Criteria: Direction changes are applied without errors
 */
static void test_motor_forward_reverse(void) {
    print_test_header("TEST 1: Motor Forward/Reverse Direction");
    
    TestResults results = {0, 0, TEST_ITERATIONS};
    
    printf("  Testing motor direction changes...\n");
    printf("  Motor 1 (Left):  M1A=GP%d, M1B=GP%d\n", M1A, M1B);
    printf("  Motor 2 (Right): M2A=GP%d, M2B=GP%d\n", M2A, M2B);
    printf("\n");
    
    for (int i = 0; i < TEST_ITERATIONS; i++) {
        bool test_passed = true;
        
        // Test forward direction
        motor_drive(M1A, M1B, -50);  // Left forward
        motor_drive(M2A, M2B, -50);  // Right forward
        sleep_ms(TEST_DELAY_MS);
        
        // Verify pins are set (basic check - both motors should be active)
        bool m1_active = gpio_get(M1A) != gpio_get(M1B);  // Pins should differ
        bool m2_active = gpio_get(M2A) != gpio_get(M2B);
        
        if (!m1_active || !m2_active) {
            test_passed = false;
        }
        
        // Test reverse direction
        motor_drive(M1A, M1B, 50);   // Left reverse
        motor_drive(M2A, M2B, 50);   // Right reverse
        sleep_ms(TEST_DELAY_MS);
        
        // Verify pins changed
        bool m1_active_rev = gpio_get(M1A) != gpio_get(M1B);
        bool m2_active_rev = gpio_get(M2A) != gpio_get(M2B);
        
        if (!m1_active_rev || !m2_active_rev) {
            test_passed = false;
        }
        
        // Test stop
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        sleep_ms(TEST_DELAY_MS);
        
        if (test_passed) {
            results.passed++;
        } else {
            results.failed++;
        }
        
        print_progress(i + 1, TEST_ITERATIONS);
    }
    
    printf("\n");
    print_test_result(&results);
}

// ============================================================================
// TEST CASE 2: MOTOR SPEED CONSISTENCY TEST
// ============================================================================

/**
 * Test: Verify motor maintains consistent behavior at different speeds
 * 
 * Expected: Motors should respond to different power levels
 * Success Criteria: All speed levels applied without errors
 */
static void test_motor_speed_consistency(void) {
    print_test_header("TEST 2: Motor Speed Consistency");
    
    TestResults results = {0, 0, TEST_ITERATIONS};
    
    printf("  Testing multiple speed levels (25, 50, 75, 100)...\n");
    printf("  Each iteration tests 4 speed levels\n");
    printf("\n");
    
    int speeds[] = {25, 50, 75, 100};
    int num_speeds = sizeof(speeds) / sizeof(speeds[0]);
    
    for (int i = 0; i < TEST_ITERATIONS; i++) {
        bool test_passed = true;
        
        for (int s = 0; s < num_speeds; s++) {
            int speed = speeds[s];
            
            // Apply speed to both motors
            motor_drive(M1A, M1B, -speed);
            motor_drive(M2A, M2B, -speed);
            sleep_ms(TEST_DELAY_MS);
            
            // Check motors are active
            bool m1_active = gpio_get(M1A) != gpio_get(M1B);
            bool m2_active = gpio_get(M2A) != gpio_get(M2B);
            
            if (!m1_active || !m2_active) {
                test_passed = false;
                break;
            }
        }
        
        // Stop motors
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        sleep_ms(TEST_DELAY_MS);
        
        if (test_passed) {
            results.passed++;
        } else {
            results.failed++;
        }
        
        print_progress(i + 1, TEST_ITERATIONS);
    }
    
    printf("\n");
    print_test_result(&results);
}

// ============================================================================
// TEST CASE 3: MOTOR STOP TEST
// ============================================================================

/**
 * Test: Verify motor stop function reliably stops motors
 * 
 * Expected: After motor_stop(), both motor pins should be LOW
 * Success Criteria: Both pins LOW after stop command
 */
static void test_motor_stop(void) {
    print_test_header("TEST 3: Motor Stop Reliability");
    
    TestResults results = {0, 0, TEST_ITERATIONS};
    
    printf("  Testing motor stop from various speeds...\n");
    printf("  Verifying both pins go LOW after stop\n");
    printf("\n");
    
    for (int i = 0; i < TEST_ITERATIONS; i++) {
        bool test_passed = true;
        
        // Start motor at random speed (25-100)
        int speed = 25 + (i % 76);  // Varies from 25 to 100
        motor_drive(M1A, M1B, -speed);
        motor_drive(M2A, M2B, -speed);
        sleep_ms(TEST_DELAY_MS);
        
        // Stop motors
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        sleep_ms(10);  // Brief delay for pins to settle
        
        // Verify both pins are LOW
        bool m1a_low = !gpio_get(M1A);
        bool m1b_low = !gpio_get(M1B);
        bool m2a_low = !gpio_get(M2A);
        bool m2b_low = !gpio_get(M2B);
        
        if (!m1a_low || !m1b_low || !m2a_low || !m2b_low) {
            test_passed = false;
        }
        
        if (test_passed) {
            results.passed++;
        } else {
            results.failed++;
        }
        
        print_progress(i + 1, TEST_ITERATIONS);
    }
    
    printf("\n");
    print_test_result(&results);
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(2000);
    
    // Print test suite header
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║              MOTOR DRIVER UNIT TEST SUITE                    ║\n");
    printf("║                                                               ║\n");
    printf("║  Test Configuration:                                         ║\n");
    printf("║    - Iterations per test: %d                                ║\n", TEST_ITERATIONS);
    printf("║    - Test delay: %dms                                       ║\n", TEST_DELAY_MS);
    printf("║    - Total tests: 3                                          ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    
    // Initialize motor driver
    printf("\n[INIT] Initializing motor driver...\n");
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    printf("[INIT] ✓ Motor driver initialized\n");
    
    // Run all tests
    printf("\n");
    printf(COLOR_BLUE "Starting test execution..." COLOR_RESET "\n");
    
    test_motor_forward_reverse();
    test_motor_speed_consistency();
    test_motor_stop();
    
    // Print final summary
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                   TEST SUITE COMPLETE                        ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf(COLOR_GREEN "All motor tests completed!" COLOR_RESET "\n");
    printf("Check individual test results above.\n");
    printf("\n");
    
    // Cleanup
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    return 0;
}