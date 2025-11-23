/**
 * OBS_T6.c
 * 
 * Test Case ID: OBS-T6
 * Description: Verify multiple obstacle detection
 * Tests: Multi-obstacle tracking capability (FR-28, FR-31, NFR-11, NFR-12)
 * 
 * UPDATED FOR 60° SCAN RANGE (50° to 110°, centered at 80°)
 * 
 * Test Method:
 * - Setup 3 obstacles at different positions within 60° span
 * - Verify detection and parameter accuracy
 * - Progressively increase to 10 obstacles
 * - Verify array limit enforcement
 * 
 * Success Criteria:
 * - Detection accuracy: obstacle_count = actual number (up to 10)
 * - Angle accuracy: ±3° tolerance
 * - Distance accuracy: ±2 cm tolerance
 * - No merging of distinct obstacles
 * - Max 10 obstacles stored, no crashes beyond limit
 */

#include "pico/stdlib.h"
#include "obstacle_scanner.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// Test configuration - UPDATED FOR YOUR ROBOT
#define MAX_TEST_OBSTACLES 10
#define ANGLE_TOLERANCE 3
#define DISTANCE_TOLERANCE 2
#define MIN_SEPARATION_ANGLE 10  // Reduced from 15° to fit 60° span

// Your robot's scan configuration
#define SCAN_CENTER 80
#define SCAN_MIN 50   // ANGLE_CENTER - 30
#define SCAN_MAX 110  // ANGLE_CENTER + 30
#define SCAN_RANGE 60 // Total scan span

// Color codes
#define COLOR_GREEN "\033[32m"
#define COLOR_RED "\033[31m"
#define COLOR_BLUE "\033[34m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_CYAN "\033[36m"
#define COLOR_RESET "\033[0m"

// Expected obstacle definition
typedef struct {
    int expected_angle_start;
    int expected_angle_end;
    int expected_distance;
} ExpectedObstacle;

// Test result structure
typedef struct {
    int expected_count;
    int detected_count;
    bool count_match;
    int obstacles_matched;
    int obstacles_missed;
    int false_positives;
    bool passed;
} MultiObstacleTestResult;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static void print_test_header(void) {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║          OBS-T7: MULTIPLE OBSTACLE DETECTION                  ║\n");
    printf("║                                                               ║\n");
    printf("║  Test: Multi-obstacle tracking capability                    ║\n");
    printf("║  Scan Range: %d° to %d° (60° total, centered at %d°)          ║\n", 
           SCAN_MIN, SCAN_MAX, SCAN_CENTER);
    printf("║  Requirement: FR-28, FR-31, NFR-11, NFR-12                    ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

static bool angles_match(int detected, int expected) {
    return abs(detected - expected) <= ANGLE_TOLERANCE;
}

static bool distance_matches(uint64_t detected, int expected) {
    return abs((int)detected - expected) <= DISTANCE_TOLERANCE;
}

static bool obstacle_matches(Obstacle* detected, ExpectedObstacle* expected) {
    bool start_ok = angles_match(detected->angle_start, expected->expected_angle_start);
    bool end_ok = angles_match(detected->angle_end, expected->expected_angle_end);
    bool dist_ok = distance_matches(detected->min_distance, expected->expected_distance);
    
    return start_ok && end_ok && dist_ok;
}

static void print_obstacle_comparison(Obstacle* detected, ExpectedObstacle* expected, bool matched) {
    printf("    Expected: %3d°-%3d° at %3d cm → ",
           expected->expected_angle_start,
           expected->expected_angle_end,
           expected->expected_distance);
    
    printf("Detected: %3d°-%3d° at %3llu cm ",
           detected->angle_start,
           detected->angle_end,
           detected->min_distance);
    
    if (matched) {
        printf("%s✓ MATCH%s\n", COLOR_GREEN, COLOR_RESET);
    } else {
        printf("%s✗ NO MATCH%s\n", COLOR_RED, COLOR_RESET);
    }
}

static void print_test_result(MultiObstacleTestResult* result) {
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ MULTI-OBSTACLE TEST RESULTS                                 │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Expected Obstacles:  %2d                                     │\n", result->expected_count);
    printf("│ Detected Obstacles:  %2d ", result->detected_count);
    if (result->count_match) {
        printf("%s✓ MATCH%s                            │\n", COLOR_GREEN, COLOR_RESET);
    } else {
        printf("%s✗ MISMATCH%s                         │\n", COLOR_RED, COLOR_RESET);
    }
    printf("│                                                             │\n");
    printf("│ Obstacles Matched:   %2d                                     │\n", result->obstacles_matched);
    printf("│ Obstacles Missed:    %2d                                     │\n", result->obstacles_missed);
    printf("│ False Positives:     %2d                                     │\n", result->false_positives);
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Result: ");
    if (result->passed) {
        printf("%s✓ PASS%s                                            │\n",
               COLOR_GREEN, COLOR_RESET);
    } else {
        printf("%s✗ FAIL%s                                            │\n",
               COLOR_RED, COLOR_RESET);
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

// ============================================================================
// TEST FUNCTIONS
// ============================================================================

static MultiObstacleTestResult test_multi_obstacle_detection(
    ExpectedObstacle* expected, 
    int num_expected,
    const char* test_name) {
    
    MultiObstacleTestResult result = {0};
    result.expected_count = num_expected;
    
    printf("\n%s%s%s\n", COLOR_BLUE, test_name, COLOR_RESET);
    printf("Expected configuration (within 60° scan range):\n");
    for (int i = 0; i < num_expected; i++) {
        printf("  Obstacle %d: %d°-%d° at %d cm\n",
               i + 1,
               expected[i].expected_angle_start,
               expected[i].expected_angle_end,
               expected[i].expected_distance);
    }
    printf("\nPress Enter to scan...");
    getchar();
    
    // Perform scan
    ScanResult scan = scanner_perform_scan();
    result.detected_count = scan.obstacle_count;
    result.count_match = (scan.obstacle_count == num_expected);
    
    // Match detected obstacles to expected
    bool expected_matched[MAX_TEST_OBSTACLES] = {false};
    bool detected_matched[MAX_TEST_OBSTACLES] = {false};
    
    printf("\nMatching obstacles:\n");
    
    // Try to match each expected obstacle
    for (int e = 0; e < num_expected; e++) {
        bool found_match = false;
        
        for (int d = 0; d < scan.obstacle_count && d < 10; d++) {
            if (!detected_matched[d]) {
                if (obstacle_matches(&scan.obstacles[d], &expected[e])) {
                    expected_matched[e] = true;
                    detected_matched[d] = true;
                    found_match = true;
                    result.obstacles_matched++;
                    
                    printf("  Expected #%d: ", e + 1);
                    print_obstacle_comparison(&scan.obstacles[d], &expected[e], true);
                    break;
                }
            }
        }
        
        if (!found_match) {
            result.obstacles_missed++;
            printf("  Expected #%d: %3d°-%3d° at %3d cm %s✗ NOT DETECTED%s\n",
                   e + 1,
                   expected[e].expected_angle_start,
                   expected[e].expected_angle_end,
                   expected[e].expected_distance,
                   COLOR_YELLOW, COLOR_RESET);
        }
    }
    
    // Count false positives
    for (int d = 0; d < scan.obstacle_count && d < 10; d++) {
        if (!detected_matched[d]) {
            result.false_positives++;
            printf("  %sFalse Positive: %d°-%d° at %llu cm%s\n",
                   COLOR_RED,
                   scan.obstacles[d].angle_start,
                   scan.obstacles[d].angle_end,
                   scan.obstacles[d].min_distance,
                   COLOR_RESET);
        }
    }
    
    // Test passes if all obstacles matched and no false positives
    result.passed = (result.obstacles_matched == num_expected) && 
                   (result.false_positives == 0);
    
    print_test_result(&result);
    
    return result;
}

// ============================================================================
// TEST SCENARIOS - UPDATED FOR 60° SCAN
// ============================================================================

static void test_three_obstacles(int* total_passed, int* total_failed) {
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST 1: THREE OBSTACLE DETECTION (60° Scan)\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    
    // Adjusted for 60° scan (50° to 110°)
    // Obstacles positioned with ~10° separation to fit in range
    ExpectedObstacle obstacles[3] = {
        {.expected_angle_start = 53, .expected_angle_end = 65, .expected_distance = 15},  // Left side
        {.expected_angle_start = 74, .expected_angle_end = 86, .expected_distance = 20},  // Center
        {.expected_angle_start = 95, .expected_angle_end = 107, .expected_distance = 25}  // Right side
    };
    
    MultiObstacleTestResult result = test_multi_obstacle_detection(
        obstacles, 3, "Testing 3 obstacles at different positions within 60° scan"
    );
    
    if (result.passed) (*total_passed)++;
    else (*total_failed)++;
}

static void test_limit_enforcement(int* total_passed, int* total_failed) {
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST 2: ARRAY LIMIT ENFORCEMENT (10 OBSTACLES)\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    printf("This test verifies the system handles up to 10 obstacles\n");
    printf("Setup as many obstacles as possible within the 60° scan\n");
    printf("System should detect maximum 10 without crashing\n");
    printf("\nPress Enter when ready...");
    getchar();
    
    // Perform scan
    ScanResult scan = scanner_perform_scan();
    
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ LIMIT ENFORCEMENT TEST                                      │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Detected Obstacles:  %2d                                     │\n", scan.obstacle_count);
    printf("│ Max Limit:           10                                     │\n");
    printf("│ Scan Range:          60° (50°-110°)                         │\n");
    printf("│                                                             │\n");
    
    bool passed = (scan.obstacle_count <= 10);
    
    if (passed) {
        printf("│ Result: %s✓ PASS%s (within limit, no crash)                  │\n",
               COLOR_GREEN, COLOR_RESET);
        (*total_passed)++;
    } else {
        printf("│ Result: %s✗ FAIL%s (exceeded limit!)                         │\n",
               COLOR_RED, COLOR_RESET);
        (*total_failed)++;
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

static void test_separation(int* total_passed, int* total_failed) {
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST 3: OBSTACLE SEPARATION (NO MERGING)\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    printf("Place 2 obstacles with ≥10° separation within 60° scan\n");
    printf("System should detect them as separate obstacles\n");
    printf("\nPress Enter when ready...");
    getchar();
    
    ScanResult scan = scanner_perform_scan();
    
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ SEPARATION TEST (60° Scan Range)                           │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Detected Obstacles:  %2d                                     │\n", scan.obstacle_count);
    printf("│                                                             │\n");
    
    // Check if obstacles are properly separated
    bool properly_separated = true;
    if (scan.obstacle_count >= 2) {
        for (int i = 0; i < scan.obstacle_count - 1; i++) {
            int gap = scan.obstacles[i+1].angle_start - scan.obstacles[i].angle_end;
            printf("│ Gap between obstacle %d and %d: %3d°                      │\n",
                   i + 1, i + 2, gap);
            if (gap < MIN_SEPARATION_ANGLE) {
                properly_separated = false;
            }
        }
    }
    
    printf("│                                                             │\n");
    bool passed = (scan.obstacle_count == 2) && properly_separated;
    
    if (passed) {
        printf("│ Result: %s✓ PASS%s (2 obstacles, properly separated)         │\n",
               COLOR_GREEN, COLOR_RESET);
        (*total_passed)++;
    } else {
        printf("│ Result: %s✗ FAIL%s (obstacles merged or wrong count)        │\n",
               COLOR_RED, COLOR_RESET);
        (*total_failed)++;
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    print_test_header();
    
    // Initialize hardware
    printf("[INIT] Initializing scanner system...\n");
    scanner_init();
    printf("[INIT] ✓ Scanner initialized\n");
    
    // Instructions
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST INSTRUCTIONS:\n");
    printf("  1. Your robot scans 60° (from 50° to 110°, center at 80°)\n");
    printf("  2. Position obstacles WITHIN this 60° scan range\n");
    printf("  3. Use ≥10° separation between obstacles (reduced for 60°)\n");
    printf("  4. Use ruler/protractor for accurate placement\n");
    printf("  5. Follow specific setup for each test\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\nPress Enter to begin tests...");
    getchar();
    
    // Run test scenarios
    int total_passed = 0;
    int total_failed = 0;
    
    test_three_obstacles(&total_passed, &total_failed);
    test_limit_enforcement(&total_passed, &total_failed);
    test_separation(&total_passed, &total_failed);
    
    // Final summary
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    FINAL TEST SUMMARY                         ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("  Total Tests:  %d\n", total_passed + total_failed);
    printf("  Passed:       %s%d%s\n", COLOR_GREEN, total_passed, COLOR_RESET);
    printf("  Failed:       %s%d%s\n", COLOR_RED, total_failed, COLOR_RESET);
    printf("\n");
    
    if (total_failed == 0) {
        printf("  %s✓ OBS-T7: ALL TESTS PASSED%s\n", COLOR_GREEN, COLOR_RESET);
    } else {
        printf("  %s✗ OBS-T7: SOME TESTS FAILED%s\n", COLOR_RED, COLOR_RESET);
    }
    printf("\n");
    
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  OBS-T7 TEST COMPLETE\n");
    printf("  Scan Configuration: 60° range (50° to 110°)\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    
    return (total_failed == 0) ? 0 : 1;
}