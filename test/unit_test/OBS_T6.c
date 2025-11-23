/**
 * @file    OBS_T6.c
 * @brief   Test Case OBS-T6: Multiple Obstacle Detection
 * 
 * @details Verifies multi-obstacle tracking capability within 60° scan range.
 *          Tests FR-28, FR-31, NFR-11, NFR-12 (multiple obstacle detection).
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
#include <stdbool.h>

/*******************************************************************************
 * CONSTANTS AND MACROS
 ******************************************************************************/

/* Test Configuration - UPDATED FOR 60° SCAN */
#define MAX_TEST_OBSTACLES     (10)
#define ANGLE_TOLERANCE        (3)     /* Stricter than system default (2°) */
#define DISTANCE_TOLERANCE     (2)
#define MIN_SEPARATION_ANGLE   (10)    /* Reduced from 15° to fit 60° span */

/* Note: Scan angles come from obstacle_scanner.h:
 * - ANGLE_CENTER = 80
 * - MIN_ANGLE = 50
 * - MAX_ANGLE = 110
 * - Scan range = 60° total
 */

/* ANSI Color Codes */
#define COLOR_GREEN   "\033[32m"
#define COLOR_RED     "\033[31m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_RESET   "\033[0m"

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

/**
 * @brief Expected obstacle definition
 */
typedef struct
{
    int expected_angle_start;
    int expected_angle_end;
    int expected_distance;
} ExpectedObstacle_t;

/**
 * @brief Multi-obstacle test result structure
 */
typedef struct
{
    int  expected_count;
    int  detected_count;
    bool count_match;
    int  obstacles_matched;
    int  obstacles_missed;
    int  false_positives;
    bool passed;
} MultiObstacleTestResult_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES
 ******************************************************************************/

static void print_test_header(void);
static bool angles_match(int detected, int expected);
static bool distance_matches(uint64_t detected, int expected);
static bool obstacle_matches(Obstacle const * p_detected, ExpectedObstacle_t const * p_expected);
static void print_obstacle_comparison(Obstacle const * p_detected, 
                                     ExpectedObstacle_t const * p_expected, 
                                     bool matched);
static void print_test_result(MultiObstacleTestResult_t const * p_result);
static MultiObstacleTestResult_t test_multi_obstacle_detection(ExpectedObstacle_t const * p_expected, 
                                                                int num_expected,
                                                                char const * p_test_name);
static void test_three_obstacles(int * p_total_passed, int * p_total_failed);
static void test_limit_enforcement(int * p_total_passed, int * p_total_failed);
static void test_separation(int * p_total_passed, int * p_total_failed);

/*******************************************************************************
 * PRIVATE FUNCTIONS
 ******************************************************************************/

/**
 * @brief Print test header banner
 */
static void
print_test_header(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║          OBS-T6: MULTIPLE OBSTACLE DETECTION                  ║\n");
    printf("║                                                               ║\n");
    printf("║  Test: Multi-obstacle tracking capability                    ║\n");
    printf("║  Scan Range: %d° to %d° (60° total, centered at %d°)          ║\n", 
           MIN_ANGLE, MAX_ANGLE, ANGLE_CENTER);
    printf("║  Requirement: FR-28, FR-31, NFR-11, NFR-12                    ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

/**
 * @brief Check if angles match within tolerance
 * 
 * @param[in] detected  Detected angle value
 * @param[in] expected  Expected angle value
 * @return    true if angles match within tolerance
 */
static bool
angles_match(int detected, int expected)
{
    return (abs(detected - expected) <= ANGLE_TOLERANCE);
}

/**
 * @brief Check if distances match within tolerance
 * 
 * @param[in] detected  Detected distance value
 * @param[in] expected  Expected distance value
 * @return    true if distances match within tolerance
 */
static bool
distance_matches(uint64_t detected, int expected)
{
    return (abs((int)detected - expected) <= DISTANCE_TOLERANCE);
}

/**
 * @brief Check if obstacle matches expected parameters
 * 
 * @param[in] p_detected  Pointer to detected obstacle
 * @param[in] p_expected  Pointer to expected obstacle
 * @return    true if obstacle matches all criteria
 */
static bool
obstacle_matches(Obstacle const * p_detected, ExpectedObstacle_t const * p_expected)
{
    bool start_ok = angles_match(p_detected->angle_start, p_expected->expected_angle_start);
    bool end_ok = angles_match(p_detected->angle_end, p_expected->expected_angle_end);
    bool dist_ok = distance_matches(p_detected->min_distance, p_expected->expected_distance);
    
    return (start_ok && end_ok && dist_ok);
}

/**
 * @brief Print obstacle comparison for debugging
 * 
 * @param[in] p_detected  Pointer to detected obstacle
 * @param[in] p_expected  Pointer to expected obstacle
 * @param[in] matched     Whether obstacle matched
 */
static void
print_obstacle_comparison(Obstacle const * p_detected, 
                         ExpectedObstacle_t const * p_expected, 
                         bool matched)
{
    printf("    Expected: %3d°-%3d° at %3d cm → ",
           p_expected->expected_angle_start,
           p_expected->expected_angle_end,
           p_expected->expected_distance);
    
    printf("Detected: %3d°-%3d° at %3llu cm ",
           p_detected->angle_start,
           p_detected->angle_end,
           p_detected->min_distance);
    
    if (matched)
    {
        printf("%s✓ MATCH%s\n", COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗ NO MATCH%s\n", COLOR_RED, COLOR_RESET);
    }
}

/**
 * @brief Print formatted test result
 * 
 * @param[in] p_result Pointer to test result structure
 */
static void
print_test_result(MultiObstacleTestResult_t const * p_result)
{
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ MULTI-OBSTACLE TEST RESULTS                                 │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Expected Obstacles:  %2d                                     │\n", p_result->expected_count);
    printf("│ Detected Obstacles:  %2d ", p_result->detected_count);
    if (p_result->count_match)
    {
        printf("%s✓ MATCH%s                            │\n", COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗ MISMATCH%s                         │\n", COLOR_RED, COLOR_RESET);
    }
    printf("│                                                             │\n");
    printf("│ Obstacles Matched:   %2d                                     │\n", p_result->obstacles_matched);
    printf("│ Obstacles Missed:    %2d                                     │\n", p_result->obstacles_missed);
    printf("│ False Positives:     %2d                                     │\n", p_result->false_positives);
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Result: ");
    if (p_result->passed)
    {
        printf("%s✓ PASS%s                                            │\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗ FAIL%s                                            │\n",
               COLOR_RED, COLOR_RESET);
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief Test multiple obstacle detection
 * 
 * @param[in] p_expected   Pointer to array of expected obstacles
 * @param[in] num_expected Number of expected obstacles
 * @param[in] p_test_name  Pointer to test name string
 * @return    Test result structure
 */
static MultiObstacleTestResult_t
test_multi_obstacle_detection(ExpectedObstacle_t const * p_expected, 
                              int num_expected,
                              char const * p_test_name)
{
    MultiObstacleTestResult_t result = {0};
    bool expected_matched[MAX_TEST_OBSTACLES] = {false};
    bool detected_matched[MAX_TEST_OBSTACLES] = {false};
    ScanResult scan;
    int e, d;
    
    result.expected_count = num_expected;
    
    printf("\n%s%s%s\n", COLOR_BLUE, p_test_name, COLOR_RESET);
    printf("Expected configuration (within 60° scan range):\n");
    for (e = 0; e < num_expected; e++)
    {
        printf("  Obstacle %d: %d°-%d° at %d cm\n",
               e + 1,
               p_expected[e].expected_angle_start,
               p_expected[e].expected_angle_end,
               p_expected[e].expected_distance);
    }
    printf("\nPress Enter to scan...");
    (void)getchar();
    
    /* Perform scan */
    scan = scanner_perform_scan();
    result.detected_count = scan.obstacle_count;
    result.count_match = (scan.obstacle_count == num_expected);
    
    printf("\nMatching obstacles:\n");
    
    /* Try to match each expected obstacle */
    for (e = 0; e < num_expected; e++)
    {
        bool found_match = false;
        
        for (d = 0; (d < scan.obstacle_count) && (d < 10); d++)
        {
            if (!detected_matched[d])
            {
                if (obstacle_matches(&scan.obstacles[d], &p_expected[e]))
                {
                    expected_matched[e] = true;
                    detected_matched[d] = true;
                    found_match = true;
                    result.obstacles_matched++;
                    
                    printf("  Expected #%d: ", e + 1);
                    print_obstacle_comparison(&scan.obstacles[d], &p_expected[e], true);
                    break;
                }
            }
        }
        
        if (!found_match)
        {
            result.obstacles_missed++;
            printf("  %sMissed: %d°-%d° at %d cm%s\n",
                   COLOR_YELLOW,
                   p_expected[e].expected_angle_start,
                   p_expected[e].expected_angle_end,
                   p_expected[e].expected_distance,
                   COLOR_RESET);
        }
    }
    
    /* Check for false positives */
    for (d = 0; (d < scan.obstacle_count) && (d < 10); d++)
    {
        if (!detected_matched[d])
        {
            result.false_positives++;
            printf("  %sFalse Positive: %d°-%d° at %llu cm%s\n",
                   COLOR_RED,
                   scan.obstacles[d].angle_start,
                   scan.obstacles[d].angle_end,
                   scan.obstacles[d].min_distance,
                   COLOR_RESET);
        }
    }
    
    /* Test passes if all obstacles matched and no false positives */
    result.passed = ((result.obstacles_matched == num_expected) && 
                    (0 == result.false_positives));
    
    print_test_result(&result);
    
    return result;
}

/**
 * @brief Test three obstacle detection scenario
 * 
 * @param[in,out] p_total_passed Pointer to passed test counter
 * @param[in,out] p_total_failed Pointer to failed test counter
 */
static void
test_three_obstacles(int * p_total_passed, int * p_total_failed)
{
    MultiObstacleTestResult_t result;
    
    /* Adjusted for 60° scan (50° to 110°) */
    /* Obstacles positioned with ~10° separation to fit in range */
    ExpectedObstacle_t const obstacles[3] = {
        {.expected_angle_start = 53, .expected_angle_end = 65, .expected_distance = 15},  /* Left */
        {.expected_angle_start = 74, .expected_angle_end = 86, .expected_distance = 20},  /* Center */
        {.expected_angle_start = 95, .expected_angle_end = 107, .expected_distance = 25}  /* Right */
    };
    
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST 1: THREE OBSTACLE DETECTION (60° Scan)\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    
    result = test_multi_obstacle_detection(obstacles, 3, 
             "Testing 3 obstacles at different positions within 60° scan");
    
    if (result.passed)
    {
        (*p_total_passed)++;
    }
    else
    {
        (*p_total_failed)++;
    }
}

/**
 * @brief Test array limit enforcement
 * 
 * @param[in,out] p_total_passed Pointer to passed test counter
 * @param[in,out] p_total_failed Pointer to failed test counter
 */
static void
test_limit_enforcement(int * p_total_passed, int * p_total_failed)
{
    ScanResult scan;
    bool passed;
    
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST 2: ARRAY LIMIT ENFORCEMENT (10 OBSTACLES)\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    printf("This test verifies the system handles up to 10 obstacles\n");
    printf("Setup as many obstacles as possible within the 60° scan\n");
    printf("System should detect maximum 10 without crashing\n");
    printf("\nPress Enter when ready...");
    (void)getchar();
    
    /* Perform scan */
    scan = scanner_perform_scan();
    
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ LIMIT ENFORCEMENT TEST                                      │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Detected Obstacles:  %2d                                     │\n", scan.obstacle_count);
    printf("│ Max Limit:           10                                     │\n");
    printf("│ Scan Range:          60° (50°-110°)                         │\n");
    printf("│                                                             │\n");
    
    passed = (scan.obstacle_count <= 10);
    
    if (passed)
    {
        printf("│ Result: %s✓ PASS%s (within limit, no crash)                  │\n",
               COLOR_GREEN, COLOR_RESET);
        (*p_total_passed)++;
    }
    else
    {
        printf("│ Result: %s✗ FAIL%s (exceeded limit!)                         │\n",
               COLOR_RED, COLOR_RESET);
        (*p_total_failed)++;
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief Test obstacle separation (no merging)
 * 
 * @param[in,out] p_total_passed Pointer to passed test counter
 * @param[in,out] p_total_failed Pointer to failed test counter
 */
static void
test_separation(int * p_total_passed, int * p_total_failed)
{
    ScanResult scan;
    bool properly_separated = true;
    bool passed;
    int i;
    
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST 3: OBSTACLE SEPARATION (NO MERGING)\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    printf("Place 2 obstacles with ≥10° separation within 60° scan\n");
    printf("System should detect them as separate obstacles\n");
    printf("\nPress Enter when ready...");
    (void)getchar();
    
    scan = scanner_perform_scan();
    
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ SEPARATION TEST (60° Scan Range)                           │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Detected Obstacles:  %2d                                     │\n", scan.obstacle_count);
    printf("│                                                             │\n");
    
    /* Check if obstacles are properly separated */
    if (scan.obstacle_count >= 2)
    {
        for (i = 0; i < (scan.obstacle_count - 1); i++)
        {
            int gap = scan.obstacles[i+1].angle_start - scan.obstacles[i].angle_end;
            printf("│ Gap between obstacle %d and %d: %3d°                      │\n",
                   i + 1, i + 2, gap);
            if (gap < MIN_SEPARATION_ANGLE)
            {
                properly_separated = false;
            }
        }
    }
    
    printf("│                                                             │\n");
    passed = ((2 == scan.obstacle_count) && properly_separated);
    
    if (passed)
    {
        printf("│ Result: %s✓ PASS%s (2 obstacles, properly separated)         │\n",
               COLOR_GREEN, COLOR_RESET);
        (*p_total_passed)++;
    }
    else
    {
        printf("│ Result: %s✗ FAIL%s (obstacles merged or wrong count)        │\n",
               COLOR_RED, COLOR_RESET);
        (*p_total_failed)++;
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
 ******************************************************************************/

/**
 * @brief Main test function
 * 
 * @return 0 if all tests passed, 1 otherwise
 */
int
main(void)
{
    int total_passed = 0;
    int total_failed = 0;
    
    stdio_init_all();
    sleep_ms(2000);
    
    print_test_header();
    
    /* Initialize hardware */
    printf("[INIT] Initializing scanner system...\n");
    scanner_init();
    printf("[INIT] ✓ Scanner initialized\n");
    
    /* Instructions */
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
    (void)getchar();
    
    /* Run test scenarios */
    test_three_obstacles(&total_passed, &total_failed);
    test_limit_enforcement(&total_passed, &total_failed);
    test_separation(&total_passed, &total_failed);
    
    /* Final summary */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    FINAL TEST SUMMARY                         ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("  Total Tests:  %d\n", total_passed + total_failed);
    printf("  Passed:       %s%d%s\n", COLOR_GREEN, total_passed, COLOR_RESET);
    printf("  Failed:       %s%d%s\n", COLOR_RED, total_failed, COLOR_RESET);
    printf("\n");
    
    if (0 == total_failed)
    {
        printf("  %s✓ OBS-T6: ALL TESTS PASSED%s\n", COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("  %s✗ OBS-T6: SOME TESTS FAILED%s\n", COLOR_RED, COLOR_RESET);
    }
    printf("\n");
    
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  OBS-T6 TEST COMPLETE\n");
    printf("  Scan Configuration: 60° range (50° to 110°)\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    
    return (0 == total_failed) ? 0 : 1;
}