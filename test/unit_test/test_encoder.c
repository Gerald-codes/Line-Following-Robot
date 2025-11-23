/**
 * test_encoder.c
 * 
 * Unit tests for encoder driver
 * Compatible with common encoder API patterns
 */

#include "pico/stdlib.h"
#include "encoder.h"
#include "motor.h"
#include "pin_definitions.h"
#include <stdio.h>
#include <stdlib.h>

// Test configuration
#define TEST_ITERATIONS 100
#define MOTOR_RUN_TIME_MS 200
#define MOTOR_SPEED 60

// Test result tracking
typedef struct {
    int passed;
    int failed;
    int total;
} TestResults;

// Color codes
#define COLOR_GREEN "\033[32m"
#define COLOR_RED "\033[31m"
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
// ENCODER API WRAPPER - Matched to your encoder.h
// ============================================================================

static inline void reset_left_encoder(void) {
    encoder_reset();  // Resets both encoders
}

static inline void reset_right_encoder(void) {
    // Do nothing - encoder_reset() already reset both
}

static inline int32_t get_left_count(void) {
    return get_left_encoder();
}

static inline int32_t get_right_count(void) {
    return get_right_encoder();
}

// ============================================================================
// TEST CASE 1: ENCODER COUNT INCREMENT TEST
// ============================================================================

static void test_encoder_count_increment(void) {
    print_test_header("TEST 1: Encoder Count Increment");
    
    TestResults results = {0, 0, TEST_ITERATIONS};
    
    printf("  Testing encoder counting during motor operation...\n");
    printf("  Motor run time: %dms per iteration\n", MOTOR_RUN_TIME_MS);
    printf("  Motor speed: %d\n", MOTOR_SPEED);
    printf("\n");
    
    for (int i = 0; i < TEST_ITERATIONS; i++) {
        bool test_passed = true;
        
        // Reset encoders
        reset_left_encoder();
        reset_right_encoder();
        
        // Get initial counts (should be 0)
        int32_t left_start = get_left_count();
        int32_t right_start = get_right_count();
        
        if (left_start != 0 || right_start != 0) {
            test_passed = false;
        }
        
        // Run motors forward
        motor_drive(M1A, M1B, -MOTOR_SPEED);
        motor_drive(M2A, M2B, -MOTOR_SPEED);
        sleep_ms(MOTOR_RUN_TIME_MS);
        
        // Stop motors
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        sleep_ms(50);
        
        // Get final counts
        int32_t left_end = get_left_count();
        int32_t right_end = get_right_count();
        
        // Verify counts increased
        if (abs(left_end) == 0 || abs(right_end) == 0) {
            test_passed = false;
        }
        
        if (test_passed) {
            results.passed++;
        } else {
            results.failed++;
            if (results.failed <= 3) {
                printf("\n  [FAIL #%d] Left: %ld, Right: %ld (expected non-zero)\n", 
                       results.failed, left_end, right_end);
            }
        }
        
        print_progress(i + 1, TEST_ITERATIONS);
    }
    
    printf("\n");
    
    int32_t sample_left = get_left_count();
    int32_t sample_right = get_right_count();
    printf("  Sample counts (last iteration): Left=%ld, Right=%ld\n", sample_left, sample_right);
    
    print_test_result(&results);
}

// ============================================================================
// TEST CASE 2: ENCODER RESET TEST
// ============================================================================

static void test_encoder_reset(void) {
    print_test_header("TEST 2: Encoder Reset Functionality");
    
    TestResults results = {0, 0, TEST_ITERATIONS};
    
    printf("  Testing encoder reset after accumulating counts...\n");
    printf("\n");
    
    for (int i = 0; i < TEST_ITERATIONS; i++) {
        bool test_passed = true;
        
        // Run motors to accumulate counts
        motor_drive(M1A, M1B, -MOTOR_SPEED);
        motor_drive(M2A, M2B, -MOTOR_SPEED);
        sleep_ms(MOTOR_RUN_TIME_MS);
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        sleep_ms(50);
        
        // Reset encoders
        reset_left_encoder();
        reset_right_encoder();
        
        // Verify reset worked
        int32_t left_after = get_left_count();
        int32_t right_after = get_right_count();
        
        if (left_after != 0 || right_after != 0) {
            test_passed = false;
            if (results.failed < 3) {
                printf("\n  [FAIL #%d] After reset: Left=%ld, Right=%ld (expected 0)\n", 
                       results.failed + 1, left_after, right_after);
            }
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
// TEST CASE 3: ENCODER DIRECTION TEST
// ============================================================================

static void test_encoder_direction(void) {
    print_test_header("TEST 3: Encoder Direction Detection");
    
    TestResults results = {0, 0, TEST_ITERATIONS};
    
    printf("  Testing encoder direction for forward vs reverse motion...\n");
    printf("\n");
    
    for (int i = 0; i < TEST_ITERATIONS; i++) {
        bool test_passed = true;
        
        // === TEST FORWARD ===
        reset_left_encoder();
        reset_right_encoder();
        
        motor_drive(M1A, M1B, -MOTOR_SPEED);
        motor_drive(M2A, M2B, -MOTOR_SPEED);
        sleep_ms(MOTOR_RUN_TIME_MS);
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        sleep_ms(50);
        
        int32_t left_forward = get_left_count();
        int32_t right_forward = get_right_count();
        
        // === TEST REVERSE ===
        reset_left_encoder();
        reset_right_encoder();
        
        motor_drive(M1A, M1B, MOTOR_SPEED);
        motor_drive(M2A, M2B, MOTOR_SPEED);
        sleep_ms(MOTOR_RUN_TIME_MS);
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        sleep_ms(50);
        
        int32_t left_reverse = get_left_count();
        int32_t right_reverse = get_right_count();
        
        // Verify directions are opposite
        bool left_opposite = (left_forward > 0 && left_reverse < 0) || 
                             (left_forward < 0 && left_reverse > 0);
        bool right_opposite = (right_forward > 0 && right_reverse < 0) || 
                              (right_forward < 0 && right_reverse > 0);
        
        if (!left_opposite || !right_opposite) {
            test_passed = false;
            if (results.failed < 3) {
                printf("\n  [FAIL #%d] Left: F=%ld R=%ld | Right: F=%ld R=%ld\n", 
                       results.failed + 1, 
                       left_forward, left_reverse, 
                       right_forward, right_reverse);
            }
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
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║             ENCODER DRIVER UNIT TEST SUITE                   ║\n");
    printf("║                                                               ║\n");
    printf("║  Test Configuration:                                         ║\n");
    printf("║    - Iterations per test: %d                                ║\n", TEST_ITERATIONS);
    printf("║    - Motor run time: %dms                                   ║\n", MOTOR_RUN_TIME_MS);
    printf("║    - Motor speed: %d                                        ║\n", MOTOR_SPEED);
    printf("║    - Total tests: 3                                          ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    
    printf("\n[INIT] Initializing drivers...\n");
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    encoder_init();
    printf("[INIT] ✓ Drivers initialized\n");
    
    printf("\n");
    printf(COLOR_BLUE "Starting test execution..." COLOR_RESET "\n");
    
    test_encoder_count_increment();
    test_encoder_reset();
    test_encoder_direction();
    
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                   TEST SUITE COMPLETE                        ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf(COLOR_GREEN "All encoder tests completed!" COLOR_RESET "\n");
    printf("\n");
    
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    return 0;
}