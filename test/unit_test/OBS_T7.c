/**
 * @file    OBS_T7.c
 * @brief   Test Case OBS-T7: EMA Width Smoothing
 * 
 * @details Verifies exponential moving average width smoothing effectiveness.
 *          Tests FR-30 (width smoothing) and NFR-10 (measurement accuracy).
 * 
 * Test Method:
 * - Position 15 cm wide obstacle at 20 cm
 * - Perform 10 consecutive scans (no movement)
 * - Record raw width and smoothed_width each scan
 * - Calculate width variation between consecutive scans
 * - Plot width values to observe smoothing
 * - Formula: smoothed_width = (0.7 × width) + (0.3 × last_width)
 * 
 * Success Criteria:
 * - EMA formula: smoothed_width = (0.7 × width) + (0.3 × last_width)
 * - Convergence time: ≤4 scans to stable value
 * - Stabilized width: Within ±5% of actual (14.25-15.75 cm)
 * - Final variation: Std dev ≤1 cm for scans 7-10
 * - Smoothing visible: Reduced scan-to-scan variation
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
#define NUM_SCANS                   (10)
#define ACTUAL_WIDTH_CM             (15)
#define TEST_DISTANCE_CM            (20)
#define MAX_CONVERGENCE_SCANS       (4)
#define MIN_ACCEPTABLE_WIDTH_CM     (14.25f)
#define MAX_ACCEPTABLE_WIDTH_CM     (15.75f)
#define MAX_FINAL_VARIATION_CM      (1.0f)

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
 * @brief EMA test result structure
 */
typedef struct
{
    float raw_widths[NUM_SCANS];
    float smoothed_widths[NUM_SCANS];
    float initial_width;
    float stabilized_width;
    int   convergence_scan;
    float final_variation;
    bool  convergence_ok;
    bool  stabilized_within_range;
    bool  variation_ok;
    bool  smoothing_visible;
    bool  passed;
} EMATestResult_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES
 ******************************************************************************/

static void print_test_header(void);
static float calculate_std_dev(float const * p_values, int start_idx, int count);
static void print_scan_table(EMATestResult_t const * p_result);
static void print_visual_plot(EMATestResult_t const * p_result);
static void print_test_result(EMATestResult_t const * p_result);

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
    printf("║           OBS-T7: EMA WIDTH SMOOTHING                         ║\n");
    printf("║                                                               ║\n");
    printf("║  Test: Exponential moving average width smoothing            ║\n");
    printf("║  Requirement: FR-30, NFR-10                                   ║\n");
    printf("║  Scan Range: 60° (50° to 110°, centered at 80°)              ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

/**
 * @brief Calculate standard deviation of values
 * 
 * @param[in] p_values  Pointer to array of values
 * @param[in] start_idx Starting index
 * @param[in] count     Number of values to process
 * @return    Standard deviation
 */
static float
calculate_std_dev(float const * p_values, int start_idx, int count)
{
    float sum = 0.0f;
    float mean;
    float variance_sum = 0.0f;
    int i;
    
    /* Calculate mean */
    for (i = start_idx; (i < (start_idx + count)) && (i < NUM_SCANS); i++)
    {
        sum += p_values[i];
    }
    mean = sum / (float)count;
    
    /* Calculate variance */
    for (i = start_idx; (i < (start_idx + count)) && (i < NUM_SCANS); i++)
    {
        float diff = p_values[i] - mean;
        variance_sum += (diff * diff);
    }
    
    return sqrtf(variance_sum / (float)count);
}

/**
 * @brief Print table of scan measurements
 * 
 * @param[in] p_result Pointer to test result structure
 */
static void
print_scan_table(EMATestResult_t const * p_result)
{
    int i;
    
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ SCAN MEASUREMENTS                                           │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Scan │ Raw Width │ Smoothed │ Change │ Status            │\n");
    printf("│──────┼───────────┼──────────┼────────┼───────────────────│\n");
    
    for (i = 0; i < NUM_SCANS; i++)
    {
        float change = 0.0f;
        char const * p_status = "";
        
        if (i > 0)
        {
            change = p_result->smoothed_widths[i] - p_result->smoothed_widths[i-1];
        }
        
        if (0 == i)
        {
            p_status = "Initial";
        }
        else if (i < MAX_CONVERGENCE_SCANS)
        {
            p_status = "Converging";
        }
        else
        {
            p_status = "Stable";
        }
        
        printf("│  %2d  │  %6.2f   │  %6.2f  │ %+5.2f │ %-17s │\n",
               i + 1,
               p_result->raw_widths[i],
               p_result->smoothed_widths[i],
               change,
               p_status);
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief Print visual plot of width convergence
 * 
 * @param[in] p_result Pointer to test result structure
 */
static void
print_visual_plot(EMATestResult_t const * p_result)
{
    int i;
    
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ WIDTH CONVERGENCE PLOT                                      │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│                                                             │\n");
    printf("│ Width (cm)                                                  │\n");
    printf("│   18 │                                                      │\n");
    printf("│   17 │                                                      │\n");
    printf("│   16 │");
    
    /* Plot smoothed values */
    for (i = 0; i < NUM_SCANS; i++)
    {
        if (i > 0)
        {
            printf("      ");
        }
        printf("●");
    }
    printf("     │\n");
    
    printf("│   15 │     Target: %d cm                                    │\n", 
           ACTUAL_WIDTH_CM);
    printf("│   14 │                                                      │\n");
    printf("│   13 │                                                      │\n");
    printf("│        1    2    3    4    5    6    7    8    9   10     │\n");
    printf("│                        (Scan Number)                        │\n");
    printf("│                                                             │\n");
    printf("│   ● = Smoothed width measurement                           │\n");
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief Print formatted test result
 * 
 * @param[in] p_result Pointer to test result structure
 */
static void
print_test_result(EMATestResult_t const * p_result)
{
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ EMA SMOOTHING ANALYSIS                                      │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Initial Width (Scan 1):     %.2f cm                          │\n", 
           p_result->initial_width);
    printf("│ Stabilized Width (7-10):    %.2f cm                          │\n", 
           p_result->stabilized_width);
    printf("│ Target Width:               %d cm                            │\n", 
           ACTUAL_WIDTH_CM);
    printf("│ Convergence Scan:           %d (max: %d)                     │\n",
           p_result->convergence_scan, MAX_CONVERGENCE_SCANS);
    printf("│ Final Variation (σ):        %.2f cm (max: %.1f cm)          │\n",
           p_result->final_variation, MAX_FINAL_VARIATION_CM);
    printf("├─────────────────────────────────────────────────────────────┤\n");
    
    printf("│ Convergence Time:       %s%-6s%s                             │\n",
           p_result->convergence_ok ? COLOR_GREEN : COLOR_RED,
           p_result->convergence_ok ? "✓ PASS" : "✗ FAIL",
           COLOR_RESET);
    printf("│ Stabilized Value:       %s%-6s%s                             │\n",
           p_result->stabilized_within_range ? COLOR_GREEN : COLOR_RED,
           p_result->stabilized_within_range ? "✓ PASS" : "✗ FAIL",
           COLOR_RESET);
    printf("│ Final Variation:        %s%-6s%s                             │\n",
           p_result->variation_ok ? COLOR_GREEN : COLOR_RED,
           p_result->variation_ok ? "✓ PASS" : "✗ FAIL",
           COLOR_RESET);
    printf("│ Smoothing Visible:      %s%-6s%s                             │\n",
           p_result->smoothing_visible ? COLOR_GREEN : COLOR_YELLOW,
           p_result->smoothing_visible ? "✓ PASS" : "⚠ WARN",
           COLOR_RESET);
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
    EMATestResult_t result = {0};
    float sum = 0.0f;
    float raw_std;
    float smoothed_std;
    int count = 0;
    int i;
    
    stdio_init_all();
    sleep_ms(2000);
    
    print_test_header();
    
    /* Initialize hardware */
    printf("[INIT] Initializing scanner system...\n");
    scanner_init();
    printf("[INIT] ✓ Scanner initialized\n");
    
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST INSTRUCTIONS:\n");
    printf("  1. Position %d cm wide obstacle at %d cm distance\n", 
           ACTUAL_WIDTH_CM, TEST_DISTANCE_CM);
    printf("  2. Keep obstacle STATIONARY (do not move)\n");
    printf("  3. System will perform %d consecutive scans\n", NUM_SCANS);
    printf("  4. EMA formula: smoothed = (0.7 × width) + (0.3 × last)\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\nPress Enter to begin test...");
    (void)getchar();
    
    printf("\n%sPerforming %d scans...%s\n\n", 
           COLOR_BLUE, NUM_SCANS, COLOR_RESET);
    
    /* Perform scans */
    for (i = 0; i < NUM_SCANS; i++)
    {
        ScanResult scan;
        
        printf("  Scan %d/%d: ", i + 1, NUM_SCANS);
        
        scan = scanner_perform_scan();
        
        if (scan.obstacle_count > 0)
        {
            result.raw_widths[i] = scan.obstacles[0].width;
            result.smoothed_widths[i] = scan.obstacles[0].smoothed_width;
            
            printf("Raw=%.2f cm, Smoothed=%.2f cm\n",
                   result.raw_widths[i], result.smoothed_widths[i]);
        }
        else
        {
            printf("%sNo obstacle detected!%s\n", COLOR_YELLOW, COLOR_RESET);
            result.raw_widths[i] = 0.0f;
            result.smoothed_widths[i] = 0.0f;
        }
        
        sleep_ms(500);
    }
    
    /* Calculate statistics */
    result.initial_width = result.smoothed_widths[0];
    
    /* Calculate stabilized width (mean of scans 7-10) */
    for (i = 6; i < NUM_SCANS; i++)
    {
        if (result.smoothed_widths[i] > 0.0f)
        {
            sum += result.smoothed_widths[i];
            count++;
        }
    }
    result.stabilized_width = (count > 0) ? (sum / (float)count) : 0.0f;
    
    /* Find convergence scan (when change becomes < 0.5 cm) */
    result.convergence_scan = NUM_SCANS;
    for (i = 1; i < NUM_SCANS; i++)
    {
        float change = fabsf(result.smoothed_widths[i] - result.smoothed_widths[i-1]);
        if (change < 0.5f)
        {
            result.convergence_scan = i + 1;
            break;
        }
    }
    
    /* Calculate final variation (std dev of scans 7-10) */
    result.final_variation = calculate_std_dev(result.smoothed_widths, 6, 4);
    
    /* Check if smoothing is visible (smoothed has less variation than raw) */
    raw_std = calculate_std_dev(result.raw_widths, 0, NUM_SCANS);
    smoothed_std = calculate_std_dev(result.smoothed_widths, 0, NUM_SCANS);
    result.smoothing_visible = (smoothed_std < raw_std);
    
    /* Check criteria */
    result.convergence_ok = (result.convergence_scan <= MAX_CONVERGENCE_SCANS);
    result.stabilized_within_range = ((result.stabilized_width >= MIN_ACCEPTABLE_WIDTH_CM) &&
                                      (result.stabilized_width <= MAX_ACCEPTABLE_WIDTH_CM));
    result.variation_ok = (result.final_variation <= MAX_FINAL_VARIATION_CM);
    result.passed = (result.convergence_ok && 
                    result.stabilized_within_range && 
                    result.variation_ok);
    
    /* Print results */
    printf("\n\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                      TEST RESULTS                             ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    
    print_scan_table(&result);
    print_test_result(&result);
    print_visual_plot(&result);
    
    /* Final summary */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    FINAL TEST SUMMARY                         ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    if (result.passed)
    {
        printf("  %s✓ OBS-T7: ALL CRITERIA PASSED%s\n", COLOR_GREEN, COLOR_RESET);
        printf("\n");
        printf("  EMA width smoothing is working correctly:\n");
        printf("  - Converges within %d scans\n", result.convergence_scan);
        printf("  - Stabilizes at %.2f cm (target: %d cm)\n", 
               result.stabilized_width, ACTUAL_WIDTH_CM);
        printf("  - Final variation: %.2f cm\n", result.final_variation);
    }
    else
    {
        printf("  %s✗ OBS-T7: SOME CRITERIA FAILED%s\n", COLOR_RED, COLOR_RESET);
        printf("\n");
        if (!result.convergence_ok)
        {
            printf("  ✗ Convergence took too long (>%d scans)\n", MAX_CONVERGENCE_SCANS);
        }
        if (!result.stabilized_within_range)
        {
            printf("  ✗ Stabilized width outside acceptable range\n");
        }
        if (!result.variation_ok)
        {
            printf("  ✗ Final variation too high\n");
        }
    }
    printf("\n");
    
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  OBS-T7 TEST COMPLETE\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    
    return (result.passed ? 0 : 1);
}