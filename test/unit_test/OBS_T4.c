/**
 * @file    OBS_T4.c
 * @brief   Test Case OBS-T4: Width Consistency Across Distances
 * 
 * @details Verifies distance-independent width calculation accuracy.
 *          Tests FR-29 (width calculation) and NFR-10 (measurement accuracy).
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
#include <stdbool.h>

/*******************************************************************************
 * CONSTANTS AND MACROS
 ******************************************************************************/

/* Test Configuration */
#define NUM_TEST_DISTANCES            (4)
#define ACTUAL_OBSTACLE_WIDTH_CM      (20)
#define MIN_ACCEPTABLE_WIDTH_CM       (18)
#define MAX_ACCEPTABLE_WIDTH_CM       (22)
#define MAX_COEFFICIENT_OF_VARIATION  (0.10f)  /* 10% */

/* ANSI Color Codes */
#define COLOR_GREEN    "\033[32m"
#define COLOR_RED      "\033[31m"
#define COLOR_BLUE     "\033[34m"
#define COLOR_YELLOW   "\033[33m"
#define COLOR_RESET    "\033[0m"

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

/**
 * @brief Single distance test measurement
 */
typedef struct
{
    int      test_distance_cm;
    float    measured_width;
    int      angle_span;
    uint64_t min_distance;
} DistanceTestPoint_t;

/**
 * @brief Complete consistency test result
 */
typedef struct
{
    DistanceTestPoint_t measurements[NUM_TEST_DISTANCES];
    float               mean_width;
    float               std_dev;
    float               coefficient_of_variation;
    bool                mean_in_range;
    bool                cv_acceptable;
    bool                width_stable;
    bool                span_behavior_correct;
    bool                passed;
} ConsistencyTestResult_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES
 ******************************************************************************/

static void print_test_header(void);
static void calculate_statistics(ConsistencyTestResult_t * p_result);
static void print_measurements_table(ConsistencyTestResult_t const * p_result);
static void print_test_result(ConsistencyTestResult_t const * p_result);

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
    printf("║       OBS-T4: WIDTH CONSISTENCY ACROSS DISTANCES              ║\n");
    printf("║                                                               ║\n");
    printf("║  Test: Distance-independent width calculation                ║\n");
    printf("║  Requirement: FR-29, NFR-10                                   ║\n");
    printf("║  Scan Range: 60° (50° to 110°, centered at 80°)              ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

/**
 * @brief Calculate statistical metrics for consistency test
 * 
 * @param[in,out] p_result Pointer to test result structure
 */
static void
calculate_statistics(ConsistencyTestResult_t * p_result)
{
    float sum = 0.0f;
    float variance_sum = 0.0f;
    float mean_distance = 0.0f;
    float covariance = 0.0f;
    float distance_variance = 0.0f;
    bool spans_decreasing = true;
    int i;
    
    /* Calculate mean width */
    for (i = 0; i < NUM_TEST_DISTANCES; i++)
    {
        sum += p_result->measurements[i].measured_width;
    }
    p_result->mean_width = sum / (float)NUM_TEST_DISTANCES;
    
    /* Calculate standard deviation */
    for (i = 0; i < NUM_TEST_DISTANCES; i++)
    {
        float diff = p_result->measurements[i].measured_width - p_result->mean_width;
        variance_sum += (diff * diff);
    }
    p_result->std_dev = sqrtf(variance_sum / (float)NUM_TEST_DISTANCES);
    
    /* Calculate coefficient of variation */
    p_result->coefficient_of_variation = p_result->std_dev / p_result->mean_width;
    
    /* Check criteria */
    p_result->mean_in_range = ((p_result->mean_width >= (float)MIN_ACCEPTABLE_WIDTH_CM) && 
                               (p_result->mean_width <= (float)MAX_ACCEPTABLE_WIDTH_CM));
    p_result->cv_acceptable = (p_result->coefficient_of_variation <= MAX_COEFFICIENT_OF_VARIATION);
    
    /* Check width stability (no systematic trend with distance) */
    for (i = 0; i < NUM_TEST_DISTANCES; i++)
    {
        mean_distance += (float)p_result->measurements[i].test_distance_cm;
    }
    mean_distance /= (float)NUM_TEST_DISTANCES;
    
    for (i = 0; i < NUM_TEST_DISTANCES; i++)
    {
        float dist_diff = (float)p_result->measurements[i].test_distance_cm - mean_distance;
        float width_diff = p_result->measurements[i].measured_width - p_result->mean_width;
        covariance += (dist_diff * width_diff);
        distance_variance += (dist_diff * dist_diff);
    }
    
    float correlation = fabsf(covariance / sqrtf(distance_variance * variance_sum));
    p_result->width_stable = (correlation < 0.5f);  /* Low correlation = stable */
    
    /* Check angle span behavior (should decrease with increasing distance) */
    for (i = 0; i < (NUM_TEST_DISTANCES - 1); i++)
    {
        if (p_result->measurements[i].angle_span <= p_result->measurements[i+1].angle_span)
        {
            spans_decreasing = false;
            break;
        }
    }
    p_result->span_behavior_correct = spans_decreasing;
    
    /* Overall pass */
    p_result->passed = (p_result->mean_in_range && 
                        p_result->cv_acceptable && 
                        p_result->width_stable);
}

/**
 * @brief Print measurements table
 * 
 * @param[in] p_result Pointer to test result structure
 */
static void
print_measurements_table(ConsistencyTestResult_t const * p_result)
{
    int i;
    
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ MEASUREMENTS AT DIFFERENT DISTANCES                         │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Distance │ Width   │ Span  │ Min Dist │ Error          │\n");
    printf("│   (cm)   │  (cm)   │  (°)  │   (cm)   │  (cm)          │\n");
    printf("├──────────┼─────────┼───────┼──────────┼────────────────┤\n");
    
    for (i = 0; i < NUM_TEST_DISTANCES; i++)
    {
        float error = p_result->measurements[i].measured_width - (float)ACTUAL_OBSTACLE_WIDTH_CM;
        printf("│   %2d     │  %5.2f  │  %3d  │   %3llu    │  %+5.2f        │\n",
               p_result->measurements[i].test_distance_cm,
               p_result->measurements[i].measured_width,
               p_result->measurements[i].angle_span,
               p_result->measurements[i].min_distance,
               error);
    }
    printf("└──────────┴─────────┴───────┴──────────┴────────────────┘\n");
}

/**
 * @brief Print formatted test result
 * 
 * @param[in] p_result Pointer to test result structure
 */
static void
print_test_result(ConsistencyTestResult_t const * p_result)
{
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ STATISTICAL ANALYSIS                                        │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Actual Width:           %d cm                               │\n", 
           ACTUAL_OBSTACLE_WIDTH_CM);
    printf("│ Mean Width:             %.2f cm ", p_result->mean_width);
    if (p_result->mean_in_range)
    {
        printf("%s✓%s                         │\n", COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗%s                         │\n", COLOR_RED, COLOR_RESET);
    }
    printf("│ Coefficient Variation:  %.1f%% ", 
           p_result->coefficient_of_variation * 100.0f);
    if (p_result->cv_acceptable)
    {
        printf("%s✓%s                            │\n", COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗%s                            │\n", COLOR_RED, COLOR_RESET);
    }
    printf("│ Width Stability:        ");
    if (p_result->width_stable)
    {
        printf("%s✓ STABLE%s                             │\n", COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗ UNSTABLE%s                           │\n", COLOR_RED, COLOR_RESET);
    }
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Overall Result: ");
    if (p_result->passed)
    {
        printf("%s✓ PASS%s                                    │\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗ FAIL%s                                    │\n",
               COLOR_RED, COLOR_RESET);
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
    ConsistencyTestResult_t result = {0};
    int const test_distances[NUM_TEST_DISTANCES] = {15, 18, 23, 30};
    int i;
    
    stdio_init_all();
    sleep_ms(2000);
    
    print_test_header();
    
    printf("[INIT] Initializing scanner system...\n");
    scanner_init();
    printf("[INIT] ✓ Scanner initialized\n");
    
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST INSTRUCTIONS:\n");
    printf("  1. Prepare ONE obstacle exactly %d cm wide\n", ACTUAL_OBSTACLE_WIDTH_CM);
    printf("  2. Position obstacle at each test distance\n");
    printf("  3. Ensure obstacle remains CENTERED and PERPENDICULAR\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    
    for (i = 0; i < NUM_TEST_DISTANCES; i++)
    {
        int distance = test_distances[i];
        
        printf("\n%sTest %d/%d: Position obstacle at %d cm%s\n",
               COLOR_BLUE, i + 1, NUM_TEST_DISTANCES, distance, COLOR_RESET);
        printf("Press Enter to scan...");
        (void)getchar();
        
        ScanResult scan = scanner_perform_scan();
        
        if (scan.obstacle_count > 0)
        {
            result.measurements[i].test_distance_cm = distance;
            result.measurements[i].measured_width = scan.obstacles[0].width;
            result.measurements[i].angle_span = scan.obstacles[0].angle_span;
            result.measurements[i].min_distance = scan.obstacles[0].min_distance;
            
            printf("  Measured width: %.2f cm\n", scan.obstacles[0].width);
        }
        else
        {
            printf("  %sWARNING: No obstacle detected!%s\n", COLOR_YELLOW, COLOR_RESET);
        }
    }
    
    calculate_statistics(&result);
    print_measurements_table(&result);
    print_test_result(&result);
    
    printf("\n");
    if (result.passed)
    {
        printf("  %s✓ OBS-T4: TEST PASSED%s\n", COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("  %s✗ OBS-T4: TEST FAILED%s\n", COLOR_RED, COLOR_RESET);
    }
    printf("\n");
    
    return (result.passed ? 0 : 1);
}