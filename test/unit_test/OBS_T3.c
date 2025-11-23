/**
 * @file    OBS_T3.c
 * @brief   Test Case OBS-T3: Width Calculation Accuracy
 * 
 * @details Verifies trigonometric width calculation using obstacles of known widths.
 *          Tests FR-29 (width calculation) and NFR-10 (measurement accuracy).
 * 
 * Test Method:
 * - Use obstacles with known widths: 10, 15, 20, 25 cm
 * - Position each at 20 cm distance from sensor
 * - Perform full scan for each obstacle
 * - Repeat measurement 5 times per obstacle
 * 
 * Success Criteria:
 * - Absolute width error: ≤3 cm for all obstacles
 * - Relative error: ≤15% of actual width
 * - Repeatability: Width variation ≤2 cm across 5 trials
 * - Consistency: Width stable across different distances
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
#define NUM_OBSTACLES            (4)
#define TRIALS_PER_OBSTACLE      (5)
#define TEST_DISTANCE_CM         (20)
#define MAX_ABSOLUTE_ERROR_CM    (3)
#define MAX_RELATIVE_ERROR       (0.15f)   /* 15% */
#define MAX_REPEATABILITY_CM     (2)

/* ANSI Color Codes */
#define COLOR_GREEN              "\033[32m"
#define COLOR_RED                "\033[31m"
#define COLOR_BLUE               "\033[34m"
#define COLOR_YELLOW             "\033[33m"
#define COLOR_RESET              "\033[0m"

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

/**
 * @brief Width test result structure
 */
typedef struct
{
    int   actual_width_cm;
    float measured_widths[TRIALS_PER_OBSTACLE];
    float mean_width;
    float min_width;
    float max_width;
    float variation;
    float absolute_error;
    float relative_error;
    bool  error_ok;
    bool  relative_ok;
    bool  repeatability_ok;
    bool  passed;
} WidthTestResult_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES
 ******************************************************************************/

static void print_test_header(void);
static void calculate_statistics(WidthTestResult_t * p_result);
static void print_test_result(WidthTestResult_t const * p_result);
static WidthTestResult_t test_obstacle_width(int actual_width);

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
    printf("║          OBS-T3: WIDTH CALCULATION ACCURACY                   ║\n");
    printf("║                                                               ║\n");
    printf("║  Test: Trigonometric width calculation                       ║\n");
    printf("║  Requirement: FR-29, NFR-10                                   ║\n");
    printf("║  Scan Range: 60° (50° to 110°, centered at 80°)              ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

/**
 * @brief Calculate statistics for width test results
 * 
 * @param[in,out] p_result Pointer to test result structure
 */
static void
calculate_statistics(WidthTestResult_t * p_result)
{
    float sum = 0.0f;
    int i;
    
    /* Initialize min and max with first measurement */
    p_result->min_width = p_result->measured_widths[0];
    p_result->max_width = p_result->measured_widths[0];
    
    /* Calculate mean and find min/max */
    for (i = 0; i < TRIALS_PER_OBSTACLE; i++)
    {
        sum += p_result->measured_widths[i];
        
        if (p_result->measured_widths[i] < p_result->min_width)
        {
            p_result->min_width = p_result->measured_widths[i];
        }
        
        if (p_result->measured_widths[i] > p_result->max_width)
        {
            p_result->max_width = p_result->measured_widths[i];
        }
    }
    
    p_result->mean_width = sum / (float)TRIALS_PER_OBSTACLE;
    p_result->variation = p_result->max_width - p_result->min_width;
    p_result->absolute_error = fabsf(p_result->mean_width - (float)p_result->actual_width_cm);
    p_result->relative_error = p_result->absolute_error / (float)p_result->actual_width_cm;
    
    /* Check criteria */
    p_result->error_ok = (p_result->absolute_error <= (float)MAX_ABSOLUTE_ERROR_CM);
    p_result->relative_ok = (p_result->relative_error <= MAX_RELATIVE_ERROR);
    p_result->repeatability_ok = (p_result->variation <= (float)MAX_REPEATABILITY_CM);
    p_result->passed = (p_result->error_ok && p_result->relative_ok && p_result->repeatability_ok);
}

/**
 * @brief Print formatted test result
 * 
 * @param[in] p_result Pointer to test result structure
 */
static void
print_test_result(WidthTestResult_t const * p_result)
{
    int i;
    
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ Actual Width: %d cm                                          │\n", 
           p_result->actual_width_cm);
    printf("├─────────────────────────────────────────────────────────────┤\n");
    
    /* Print all measurements */
    printf("│ Measurements (cm): ");
    for (i = 0; i < TRIALS_PER_OBSTACLE; i++)
    {
        printf("%.2f ", p_result->measured_widths[i]);
    }
    printf("              │\n");
    
    printf("│                                                             │\n");
    printf("│ Mean Width:        %.2f cm                                  │\n", 
           p_result->mean_width);
    printf("│ Range:             %.2f - %.2f cm                           │\n", 
           p_result->min_width, p_result->max_width);
    printf("│ Variation:         %.2f cm (limit: ≤%d cm)                 │\n",
           p_result->variation, MAX_REPEATABILITY_CM);
    printf("│ Absolute Error:    %.2f cm (limit: ≤%d cm)                 │\n",
           p_result->absolute_error, MAX_ABSOLUTE_ERROR_CM);
    printf("│ Relative Error:    %.1f%% (limit: ≤%.0f%%)                  │\n",
           p_result->relative_error * 100.0f, MAX_RELATIVE_ERROR * 100.0f);
    printf("├─────────────────────────────────────────────────────────────┤\n");
    
    printf("│ Absolute Error:    %s%-6s%s                                  │\n",
           p_result->error_ok ? COLOR_GREEN : COLOR_RED,
           p_result->error_ok ? "✓ PASS" : "✗ FAIL",
           COLOR_RESET);
    printf("│ Relative Error:    %s%-6s%s                                  │\n",
           p_result->relative_ok ? COLOR_GREEN : COLOR_RED,
           p_result->relative_ok ? "✓ PASS" : "✗ FAIL",
           COLOR_RESET);
    printf("│ Repeatability:     %s%-6s%s                                  │\n",
           p_result->repeatability_ok ? COLOR_GREEN : COLOR_RED,
           p_result->repeatability_ok ? "✓ PASS" : "✗ FAIL",
           COLOR_RESET);
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief Test obstacle width measurement
 * 
 * @param[in] actual_width Actual width of obstacle in cm
 * @return    Test result structure
 */
static WidthTestResult_t
test_obstacle_width(int actual_width)
{
    WidthTestResult_t result = {0};
    int trial;
    
    result.actual_width_cm = actual_width;
    
    printf("\n%sTesting obstacle: %d cm width%s\n", 
           COLOR_BLUE, actual_width, COLOR_RESET);
    printf("Position obstacle at %d cm distance, centered in scan area\n", 
           TEST_DISTANCE_CM);
    printf("Press Enter to start measurements...");
    (void)getchar();
    
    /* Perform trials */
    for (trial = 0; trial < TRIALS_PER_OBSTACLE; trial++)
    {
        printf("\n  Trial %d/%d: Scanning...\n", 
               trial + 1, TRIALS_PER_OBSTACLE);
        
        ScanResult scan = scanner_perform_scan();
        
        /* Find the obstacle and extract width */
        if (scan.obstacle_count > 0)
        {
            /* Use the first detected obstacle */
            float width = scan.obstacles[0].width;
            result.measured_widths[trial] = width;
            
            printf("    Measured width: %.2f cm\n", width);
            printf("    (Angle span: %d°, Min distance: %llu cm)\n",
                   scan.obstacles[0].angle_span,
                   scan.obstacles[0].min_distance);
        }
        else
        {
            printf("    %sWARNING: No obstacle detected!%s\n", 
                   COLOR_YELLOW, COLOR_RESET);
            result.measured_widths[trial] = 0.0f;
        }
        
        sleep_ms(500);
    }
    
    calculate_statistics(&result);
    print_test_result(&result);
    
    return result;
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
    WidthTestResult_t results[NUM_OBSTACLES];
    int const test_widths[NUM_OBSTACLES] = {10, 15, 20, 25};
    int passed = 0;
    int failed = 0;
    int i;
    
    stdio_init_all();
    sleep_ms(2000);
    
    print_test_header();
    
    /* Initialize hardware */
    printf("[INIT] Initializing scanner system...\n");
    scanner_init();
    printf("[INIT] ✓ Scanner initialized\n");
    
    /* Print test instructions */
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST INSTRUCTIONS:\n");
    printf("  1. Prepare obstacles with exact widths: 10, 15, 20, 25 cm\n");
    printf("  2. Use ruler to verify obstacle dimensions\n");
    printf("  3. Position each obstacle at %d cm from sensor\n", TEST_DISTANCE_CM);
    printf("  4. Ensure obstacle is perpendicular and centered\n");
    printf("  5. Each obstacle will be scanned %d times\n", TRIALS_PER_OBSTACLE);
    printf("═══════════════════════════════════════════════════════════════\n");
    
    /* Run tests */
    for (i = 0; i < NUM_OBSTACLES; i++)
    {
        results[i] = test_obstacle_width(test_widths[i]);
        
        if (results[i].passed)
        {
            passed++;
        }
        else
        {
            failed++;
        }
        
        sleep_ms(1000);
    }
    
    /* Final summary */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    FINAL TEST SUMMARY                         ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    printf("  Width Test Results:\n");
    for (i = 0; i < NUM_OBSTACLES; i++)
    {
        printf("    %d cm: ", results[i].actual_width_cm);
        if (results[i].passed)
        {
            printf("%s✓ PASS%s (mean: %.2f cm, error: %.2f cm)\n",
                   COLOR_GREEN, COLOR_RESET,
                   results[i].mean_width, results[i].absolute_error);
        }
        else
        {
            printf("%s✗ FAIL%s (mean: %.2f cm, error: %.2f cm)\n",
                   COLOR_RED, COLOR_RESET,
                   results[i].mean_width, results[i].absolute_error);
        }
    }
    
    printf("\n");
    printf("  Total Tests:  %d\n", NUM_OBSTACLES);
    printf("  Passed:       %s%d%s\n", COLOR_GREEN, passed, COLOR_RESET);
    printf("  Failed:       %s%d%s\n", COLOR_RED, failed, COLOR_RESET);
    printf("\n");
    
    if (0 == failed)
    {
        printf("  %s✓ OBS-T3: ALL TESTS PASSED%s\n", COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("  %s✗ OBS-T3: SOME TESTS FAILED%s\n", COLOR_RED, COLOR_RESET);
    }
    printf("\n");
    
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  OBS-T3 TEST COMPLETE\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    
    return (0 == failed) ? 0 : 1;
}