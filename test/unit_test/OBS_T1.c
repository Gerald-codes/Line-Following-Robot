/**
 * OBS-T1.c
 * 
 * Test Case ID: OBS-T1
 * Description: Verify ultrasonic distance measurement accuracy within detection range
 * Tests: Distance measurement functionality (FR-24, NFR-09)
 * 
 * Test Method:
 * - Position flat obstacle at known distances: 10, 15, 20, 25, 30 cm
 * - Take 10 consecutive readings at each position
 * - Compare measured vs actual distance
 * - Test with different materials (wood, plastic, cardboard)
 * 
 * Success Criteria:
 * - Absolute error: ≤2 cm at all test distances
 * - Reading consistency: Variation within ±2 cm across 10 readings
 * - Success rate: ≥95% valid readings (no timeouts)
 * - Material independence: Results consistent across materials
 */

#include "pico/stdlib.h"
#include "ultrasonic.h"
#include "servo.h"
#include <stdio.h>
#include <math.h>

// Test configuration
#define NUM_TEST_DISTANCES 5
#define READINGS_PER_DISTANCE 10
#define MAX_ABSOLUTE_ERROR_CM 2
#define MAX_VARIATION_CM 2
#define MIN_SUCCESS_RATE 0.95

// Color codes
#define COLOR_GREEN "\033[32m"
#define COLOR_RED "\033[31m"
#define COLOR_BLUE "\033[34m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RESET "\033[0m"

// Test result structure
typedef struct {
    int distance_cm;
    const char* material;
    uint64_t readings[READINGS_PER_DISTANCE];
    int valid_readings;
    float mean;
    float min;
    float max;
    float variation;
    float absolute_error;
    bool passed;
} DistanceTestResult;

typedef struct {
    int total_tests;
    int passed_tests;
    int failed_tests;
} TestSummary;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static void print_test_header(void) {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║           OBS-T1: DISTANCE MEASUREMENT ACCURACY               ║\n");
    printf("║                                                               ║\n");
    printf("║  Test: Ultrasonic distance measurement accuracy              ║\n");
    printf("║  Requirement: FR-24, NFR-09                                   ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

static void calculate_statistics(DistanceTestResult* result) {
    if (result->valid_readings == 0) {
        result->mean = 0;
        result->min = 0;
        result->max = 0;
        result->variation = 0;
        return;
    }
    
    // Calculate mean
    uint64_t sum = 0;
    result->min = result->readings[0];
    result->max = result->readings[0];
    
    for (int i = 0; i < result->valid_readings; i++) {
        sum += result->readings[i];
        if (result->readings[i] < result->min) result->min = result->readings[i];
        if (result->readings[i] > result->max) result->max = result->readings[i];
    }
    
    result->mean = (float)sum / result->valid_readings;
    result->variation = result->max - result->min;
    result->absolute_error = fabsf(result->mean - result->distance_cm);
}

static void print_test_result(DistanceTestResult* result) {
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ Test Distance: %d cm | Material: %-10s             │\n", 
           result->distance_cm, result->material);
    printf("├─────────────────────────────────────────────────────────────┤\n");
    
    // Print all readings
    printf("│ Readings (cm): ");
    for (int i = 0; i < result->valid_readings; i++) {
        printf("%llu ", result->readings[i]);
    }
    printf("\n");
    
    printf("│ Valid readings: %d/%d (%.0f%%)                              \n", 
           result->valid_readings, READINGS_PER_DISTANCE,
           (result->valid_readings * 100.0f) / READINGS_PER_DISTANCE);
    printf("│ Mean: %.2f cm                                               \n", result->mean);
    printf("│ Range: %.0f - %.0f cm                                       \n", result->min, result->max);
    printf("│ Variation: %.2f cm (limit: ±%d cm)                         \n", 
           result->variation, MAX_VARIATION_CM);
    printf("│ Absolute Error: %.2f cm (limit: ≤%d cm)                    \n", 
           result->absolute_error, MAX_ABSOLUTE_ERROR_CM);
    printf("├─────────────────────────────────────────────────────────────┤\n");
    
    // Check pass/fail criteria
    bool error_ok = result->absolute_error <= MAX_ABSOLUTE_ERROR_CM;
    bool variation_ok = result->variation <= MAX_VARIATION_CM;
    bool success_rate_ok = ((float)result->valid_readings / READINGS_PER_DISTANCE) >= MIN_SUCCESS_RATE;
    
    result->passed = error_ok && variation_ok && success_rate_ok;
    
    printf("│ Absolute Error:  %s%-6s%s                                     \n",
           error_ok ? COLOR_GREEN : COLOR_RED,
           error_ok ? "✓ PASS" : "✗ FAIL",
           COLOR_RESET);
    printf("│ Variation:       %s%-6s%s                                     \n",
           variation_ok ? COLOR_GREEN : COLOR_RED,
           variation_ok ? "✓ PASS" : "✗ FAIL",
           COLOR_RESET);
    printf("│ Success Rate:    %s%-6s%s                                     \n",
           success_rate_ok ? COLOR_GREEN : COLOR_RED,
           success_rate_ok ? "✓ PASS" : "✗ FAIL",
           COLOR_RESET);
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

static void print_final_summary(TestSummary* summary) {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    FINAL TEST SUMMARY                         ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("  Total Tests:  %d\n", summary->total_tests);
    printf("  Passed:       %s%d%s\n", COLOR_GREEN, summary->passed_tests, COLOR_RESET);
    printf("  Failed:       %s%d%s\n", COLOR_RED, summary->failed_tests, COLOR_RESET);
    printf("  Success Rate: %.1f%%\n", 
           (summary->passed_tests * 100.0f) / summary->total_tests);
    printf("\n");
    
    if (summary->failed_tests == 0) {
        printf("  %s✓ OBS-T1: ALL TESTS PASSED%s\n", COLOR_GREEN, COLOR_RESET);
    } else {
        printf("  %s✗ OBS-T1: SOME TESTS FAILED%s\n", COLOR_RED, COLOR_RESET);
    }
    printf("\n");
}

// ============================================================================
// MAIN TEST FUNCTION
// ============================================================================

static DistanceTestResult test_distance_accuracy(int target_distance, const char* material) {
    DistanceTestResult result = {0};
    result.distance_cm = target_distance;
    result.material = material;
    
    printf("\n%sTesting %d cm with %s...%s\n", 
           COLOR_BLUE, target_distance, material, COLOR_RESET);
    printf("Please position obstacle at %d cm and press Enter to start...", target_distance);
    getchar();
    
    // Set servo to center position
    servo_set_angle(75);
    sleep_ms(100);
    
    // Take readings
    for (int i = 0; i < READINGS_PER_DISTANCE; i++) {
        uint64_t distance;
        int status = ultrasonic_get_distance(TRIG_PIN, ECHO_PIN, &distance);
        
        if (status == SUCCESS) {
            result.readings[result.valid_readings] = distance;
            result.valid_readings++;
            printf("  Reading %d: %llu cm\n", i + 1, distance);
        } else {
            printf("  Reading %d: TIMEOUT\n", i + 1);
        }
        
        sleep_ms(100);
    }
    
    calculate_statistics(&result);
    print_test_result(&result);
    
    return result;
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    print_test_header();
    
    // Initialize hardware
    printf("[INIT] Initializing ultrasonic sensor and servo...\n");
    ultrasonic_init(TRIG_PIN, ECHO_PIN);
    servo_init(SERVO_PIN);
    printf("[INIT] ✓ Hardware initialized\n");
    
    // Test distances
    int test_distances[NUM_TEST_DISTANCES] = {10, 15, 20, 25, 30};
    const char* materials[] = {"Cardboard", "Wood", "Plastic"};
    int num_materials = 3;
    
    TestSummary summary = {0};
    DistanceTestResult all_results[NUM_TEST_DISTANCES * 3];  // 5 distances × 3 materials
    
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST INSTRUCTIONS:\n");
    printf("  1. Position obstacle perpendicular to sensor at marked distance\n");
    printf("  2. Use ruler for accurate placement\n");
    printf("  3. Press Enter when ready for each measurement\n");
    printf("  4. Test will be repeated with different materials\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    
    // Run tests for each material
    for (int m = 0; m < num_materials; m++) {
        printf("\n\n");
        printf("═══════════════════════════════════════════════════════════════\n");
        printf("  TESTING WITH MATERIAL: %s\n", materials[m]);
        printf("═══════════════════════════════════════════════════════════════\n");
        
        for (int d = 0; d < NUM_TEST_DISTANCES; d++) {
            DistanceTestResult result = test_distance_accuracy(
                test_distances[d], 
                materials[m]
            );
            
            all_results[summary.total_tests] = result;
            summary.total_tests++;
            
            if (result.passed) {
                summary.passed_tests++;
            } else {
                summary.failed_tests++;
            }
            
            sleep_ms(500);
        }
    }
    
    // Print final summary
    print_final_summary(&summary);
    
    // Material independence check
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  MATERIAL INDEPENDENCE ANALYSIS\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    
    for (int d = 0; d < NUM_TEST_DISTANCES; d++) {
        printf("\nDistance %d cm:\n", test_distances[d]);
        for (int m = 0; m < num_materials; m++) {
            DistanceTestResult* result = &all_results[m * NUM_TEST_DISTANCES + d];
            printf("  %s: Mean=%.2f cm, Error=%.2f cm\n", 
                   result->material, result->mean, result->absolute_error);
        }
    }
    
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  OBS-T1 TEST COMPLETE\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    
    return (summary.failed_tests == 0) ? 0 : 1;
}