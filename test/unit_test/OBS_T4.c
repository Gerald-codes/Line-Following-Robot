/**
 * test_obs_t4_distance_consistency.c
 * 
 * Test Case ID: OBS-T4
 * Description: Verify width measurement consistency across different distances
 * Tests: Distance-independent width calculation (FR-29, NFR-10)
 * 
 * Test Method:
 * - Use single 20 cm wide obstacle
 * - Position at distances: 15, 18, 23, 30 cm from sensor
 * - Ensure obstacle remains centered and perpendicular
 * - Perform full scan at each distance
 * - Calculate coefficient of variation
 * 
 * Success Criteria:
 * - Mean width across distances: 18-22 cm (within ±2 cm of 20 cm actual)
 * - Coefficient of variation: ≤10%
 * - Width stability: No systematic increase/decrease with distance
 * - Angle span behavior: Span increases as distance decreases
 */

#include "pico/stdlib.h"
#include "obstacle_scanner.h"
#include <stdio.h>
#include <math.h>

// Test configuration
#define NUM_TEST_DISTANCES 4
#define ACTUAL_OBSTACLE_WIDTH_CM 20
#define MIN_ACCEPTABLE_WIDTH_CM 18
#define MAX_ACCEPTABLE_WIDTH_CM 22
#define MAX_COEFFICIENT_OF_VARIATION 0.10  // 10%

// Color codes
#define COLOR_GREEN "\033[32m"
#define COLOR_RED "\033[31m"
#define COLOR_BLUE "\033[34m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RESET "\033[0m"

// Test result structure
typedef struct {
    int test_distance_cm;
    float measured_width;
    int angle_span;
    uint64_t min_distance;
} DistanceTestPoint;

typedef struct {
    DistanceTestPoint measurements[NUM_TEST_DISTANCES];
    float mean_width;
    float std_dev;
    float coefficient_of_variation;
    bool mean_in_range;
    bool cv_acceptable;
    bool width_stable;
    bool span_behavior_correct;
    bool passed;
} ConsistencyTestResult;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static void print_test_header(void) {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║       OBS-T4: WIDTH CONSISTENCY ACROSS DISTANCES              ║\n");
    printf("║                                                               ║\n");
    printf("║  Test: Distance-independent width calculation                ║\n");
    printf("║  Requirement: FR-29, NFR-10                                   ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

static void calculate_statistics(ConsistencyTestResult* result) {
    // Calculate mean
    float sum = 0;
    for (int i = 0; i < NUM_TEST_DISTANCES; i++) {
        sum += result->measurements[i].measured_width;
    }
    result->mean_width = sum / NUM_TEST_DISTANCES;
    
    // Calculate standard deviation
    float variance_sum = 0;
    for (int i = 0; i < NUM_TEST_DISTANCES; i++) {
        float diff = result->measurements[i].measured_width - result->mean_width;
        variance_sum += diff * diff;
    }
    result->std_dev = sqrtf(variance_sum / NUM_TEST_DISTANCES);
    
    // Calculate coefficient of variation
    result->coefficient_of_variation = result->std_dev / result->mean_width;
    
    // Check criteria
    result->mean_in_range = (result->mean_width >= MIN_ACCEPTABLE_WIDTH_CM && 
                             result->mean_width <= MAX_ACCEPTABLE_WIDTH_CM);
    result->cv_acceptable = (result->coefficient_of_variation <= MAX_COEFFICIENT_OF_VARIATION);
    
    // Check width stability (no systematic trend)
    float mean_distance = 0;
    for (int i = 0; i < NUM_TEST_DISTANCES; i++) {
        mean_distance += result->measurements[i].test_distance_cm;
    }
    mean_distance /= NUM_TEST_DISTANCES;
    
    float covariance = 0;
    float distance_variance = 0;
    for (int i = 0; i < NUM_TEST_DISTANCES; i++) {
        float dist_diff = result->measurements[i].test_distance_cm - mean_distance;
        float width_diff = result->measurements[i].measured_width - result->mean_width;
        covariance += dist_diff * width_diff;
        distance_variance += dist_diff * dist_diff;
    }
    
    float correlation = fabsf(covariance / sqrtf(distance_variance * variance_sum));
    result->width_stable = (correlation < 0.5);  // Low correlation = stable
    
    // Check angle span behavior
    bool spans_decreasing = true;
    for (int i = 0; i < NUM_TEST_DISTANCES - 1; i++) {
        if (result->measurements[i].angle_span <= result->measurements[i+1].angle_span) {
            spans_decreasing = false;
            break;
        }
    }
    result->span_behavior_correct = spans_decreasing;
    
    // Overall pass
    result->passed = result->mean_in_range && result->cv_acceptable && result->width_stable;
}

static void print_measurements_table(ConsistencyTestResult* result) {
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ MEASUREMENTS AT DIFFERENT DISTANCES                         │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Distance │ Width   │ Span  │ Min Dist │ Error          │\n");
    printf("│   (cm)   │  (cm)   │  (°)  │   (cm)   │  (cm)          │\n");
    printf("├──────────┼─────────┼───────┼──────────┼────────────────┤\n");
    
    for (int i = 0; i < NUM_TEST_DISTANCES; i++) {
        float error = result->measurements[i].measured_width - ACTUAL_OBSTACLE_WIDTH_CM;
        printf("│   %2d     │  %5.2f  │  %3d  │   %3llu    │  %+5.2f        │\n",
               result->measurements[i].test_distance_cm,
               result->measurements[i].measured_width,
               result->measurements[i].angle_span,
               result->measurements[i].min_distance,
               error);
    }
    printf("└──────────┴─────────┴───────┴──────────┴────────────────┘\n");
}

static void print_test_result(ConsistencyTestResult* result) {
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ STATISTICAL ANALYSIS                                        │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Actual Width:           %d cm                               │\n", 
           ACTUAL_OBSTACLE_WIDTH_CM);
    printf("│ Mean Width:             %.2f cm ", result->mean_width);
    if (result->mean_in_range) {
        printf("%s✓%s                         │\n", COLOR_GREEN, COLOR_RESET);
    } else {
        printf("%s✗%s                         │\n", COLOR_RED, COLOR_RESET);
    }
    printf("│ Coefficient Variation:  %.1f%% ", 
           result->coefficient_of_variation * 100);
    if (result->cv_acceptable) {
        printf("%s✓%s                            │\n", COLOR_GREEN, COLOR_RESET);
    } else {
        printf("%s✗%s                            │\n", COLOR_RED, COLOR_RESET);
    }
    printf("│ Width Stability:        ");
    if (result->width_stable) {
        printf("%s✓ STABLE%s                             │\n", COLOR_GREEN, COLOR_RESET);
    } else {
        printf("%s✗ UNSTABLE%s                           │\n", COLOR_RED, COLOR_RESET);
    }
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Overall Result: ");
    if (result->passed) {
        printf("%s✓ PASS%s                                    │\n",
               COLOR_GREEN, COLOR_RESET);
    } else {
        printf("%s✗ FAIL%s                                    │\n",
               COLOR_RED, COLOR_RESET);
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

// ============================================================================
// MAIN TEST FUNCTION
// ============================================================================

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    print_test_header();
    
    printf("[INIT] Initializing scanner system...\n");
    scanner_init();
    printf("[INIT] ✓ Scanner initialized\n");
    
    int test_distances[NUM_TEST_DISTANCES] = {15, 18, 23, 30};
    ConsistencyTestResult result = {0};
    
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST INSTRUCTIONS:\n");
    printf("  1. Prepare ONE obstacle exactly %d cm wide\n", ACTUAL_OBSTACLE_WIDTH_CM);
    printf("  2. Position obstacle at each test distance\n");
    printf("  3. Ensure obstacle remains CENTERED and PERPENDICULAR\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    
    for (int i = 0; i < NUM_TEST_DISTANCES; i++) {
        int distance = test_distances[i];
        
        printf("\n%sTest %d/%d: Position obstacle at %d cm%s\n",
               COLOR_BLUE, i + 1, NUM_TEST_DISTANCES, distance, COLOR_RESET);
        printf("Press Enter to scan...");
        getchar();
        
        ScanResult scan = scanner_perform_scan();
        
        if (scan.obstacle_count > 0) {
            result.measurements[i].test_distance_cm = distance;
            result.measurements[i].measured_width = scan.obstacles[0].width;
            result.measurements[i].angle_span = scan.obstacles[0].angle_span;
            result.measurements[i].min_distance = scan.obstacles[0].min_distance;
            
            printf("  Measured width: %.2f cm\n", scan.obstacles[0].width);
        } else {
            printf("  %sWARNING: No obstacle detected!%s\n", COLOR_YELLOW, COLOR_RESET);
        }
    }
    
    calculate_statistics(&result);
    print_measurements_table(&result);
    print_test_result(&result);
    
    printf("\n");
    if (result.passed) {
        printf("  %s✓ OBS-T4: TEST PASSED%s\n", COLOR_GREEN, COLOR_RESET);
    } else {
        printf("  %s✗ OBS-T4: TEST FAILED%s\n", COLOR_RED, COLOR_RESET);
    }
    printf("\n");
    
    return result.passed ? 0 : 1;
}