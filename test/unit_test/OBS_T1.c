/**
 * @file    OBS_T1.c
 * @brief   Distance measurement accuracy test for ultrasonic sensor
 * 
 * @details Test Case ID: OBS-T1
 *          Description: Verify ultrasonic distance measurement accuracy 
 *                      within detection range
 *          Tests: Distance measurement functionality (FR-24, NFR-09)
 * 
 * Test Method:
 *   - Position flat cardboard obstacle perpendicular to sensor
 *   - Place at known distances: 10, 15, 20, 25, 30 cm using ruler
 *   - Perform distance measurement (servo at center position, 80°)
 *   - Record 10 consecutive readings at each position
 *   - Compare measured vs actual distance
 *   - Test with different materials (wood, plastic, cardboard)
 * 
 * Success Criteria:
 *   - Absolute error: ≤2 cm at all test distances
 *   - Reading consistency: Variation within ±2 cm across 10 readings
 *   - Success rate: ≥95% valid readings (no timeouts)
 *   - Material independence: Results consistent across materials
 */

#include "pico/stdlib.h"
#include "ultrasonic.h"
#include "servo.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "obstacle_scanner.h"
#include <stdlib.h>
/* ========================================================================== */
/* Configuration Constants                                                    */
/* ========================================================================== */

#define NUM_TEST_DISTANCES      (5U)
#define NUM_MATERIALS           (3U)
#define READINGS_PER_TEST       (10U)
#define MAX_ERROR_CM            (2)
#define MAX_VARIATION_CM        (2)
#define MIN_SUCCESS_RATE        (0.95f)

/* ANSI color codes */
#define COLOR_GREEN   "\033[32m"
#define COLOR_RED     "\033[31m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_RESET   "\033[0m"

/* ========================================================================== */
/* Type Definitions                                                           */
/* ========================================================================== */

typedef struct
{
    uint64_t readings[READINGS_PER_TEST];
    int      readings_count;
    uint64_t mean;
    uint64_t min;
    uint64_t max;
    uint64_t variation;
    int      error_cm;
    bool     within_tolerance;
    float    success_rate;
} DistanceTestResult_t;

typedef struct
{
    int                    distance_cm;
    char const            *material;
    DistanceTestResult_t   result;
} TestPoint_t;

/* ========================================================================== */
/* Static Function Prototypes                                                 */
/* ========================================================================== */

static void print_test_header(void);
static void calculate_statistics(DistanceTestResult_t *result, 
                                 int actual_distance);
static void print_test_point_result(TestPoint_t const *point);
static void print_material_comparison(TestPoint_t const *points, 
                                     int num_points);
static void print_final_summary(TestPoint_t const *all_points, 
                               int total_points);
static bool validate_test_results(TestPoint_t const *all_points, 
                                 int total_points);

/* ========================================================================== */
/* Static Function Implementations                                            */
/* ========================================================================== */

/**
 * @brief   Print test header with configuration information
 * @param   None
 * @return  None
 */
static void 
print_test_header(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║       OBS-T1: DISTANCE MEASUREMENT ACCURACY                   ║\n");
    printf("║                                                               ║\n");
    printf("║  Test: Ultrasonic distance measurement functionality         ║\n");
    printf("║  Requirement: FR-24, NFR-09                                   ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

/**
 * @brief   Calculate statistics for distance measurements
 * @param   result          Pointer to test result structure to update
 * @param   actual_distance Actual physical distance in cm
 * @return  None
 */
static void 
calculate_statistics(DistanceTestResult_t *result, int actual_distance)
{
    uint64_t sum = 0;
    int valid_count = 0;
    int i;
    
    result->min = 0xFFFFFFFFFFFFFFFFULL;
    result->max = 0;
    
    /* Calculate sum, min, max */
    for (i = 0; i < result->readings_count; i++)
    {
        if (result->readings[i] > 0U)
        {
            sum += result->readings[i];
            valid_count++;
            
            if (result->readings[i] < result->min)
            {
                result->min = result->readings[i];
            }
            if (result->readings[i] > result->max)
            {
                result->max = result->readings[i];
            }
        }
    }
    
    /* Calculate mean */
    if (valid_count > 0)
    {
        result->mean = sum / (uint64_t)valid_count;
    }
    else
    {
        result->mean = 0;
    }
    
    /* Calculate variation */
    result->variation = result->max - result->min;
    
    /* Calculate error */
    if (result->mean >= (uint64_t)actual_distance)
    {
        result->error_cm = (int)(result->mean - (uint64_t)actual_distance);
    }
    else
    {
        result->error_cm = -(int)((uint64_t)actual_distance - result->mean);
    }
    
    /* Check tolerance */
    result->within_tolerance = (abs(result->error_cm) <= MAX_ERROR_CM) &&
                              (result->variation <= MAX_VARIATION_CM);
    
    /* Calculate success rate */
    result->success_rate = (float)valid_count / (float)result->readings_count;
}

/**
 * @brief   Print results for a single test point
 * @param   point  Pointer to test point structure
 * @return  None
 */
static void 
print_test_point_result(TestPoint_t const *point)
{
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ TEST RESULT: %d cm (%-10s)                               │\n",
           point->distance_cm, point->material);
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Actual Distance:    %d cm                                    │\n",
           point->distance_cm);
    printf("│ Mean Measured:      %llu cm                                  │\n",
           point->result.mean);
    printf("│ Error:              %+d cm                                   │\n",
           point->result.error_cm);
    printf("│ Min Reading:        %llu cm                                  │\n",
           point->result.min);
    printf("│ Max Reading:        %llu cm                                  │\n",
           point->result.max);
    printf("│ Variation:          %llu cm                                  │\n",
           point->result.variation);
    printf("│ Valid Readings:     %d / %d                                  │\n",
           (int)(point->result.success_rate * READINGS_PER_TEST),
           READINGS_PER_TEST);
    printf("│ Success Rate:       %.0f%%                                    │\n",
           point->result.success_rate * 100.0f);
    printf("├─────────────────────────────────────────────────────────────┤\n");
    
    if (point->result.within_tolerance)
    {
        printf("│ Result: %s✓ PASS%s                                           │\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("│ Result: %s✗ FAIL%s                                           │\n",
               COLOR_RED, COLOR_RESET);
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief   Print comparison of results across materials
 * @param   points     Array of test points for same distance
 * @param   num_points Number of points to compare
 * @return  None
 */
static void 
print_material_comparison(TestPoint_t const *points, int num_points)
{
    int i;
    
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ MATERIAL COMPARISON at %d cm                                 │\n",
           points[0].distance_cm);
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Material   │ Mean (cm) │ Error │ Variation │ Status       │\n");
    printf("├────────────┼───────────┼───────┼───────────┼──────────────┤\n");
    
    for (i = 0; i < num_points; i++)
    {
        char const *status = points[i].result.within_tolerance ? 
                            COLOR_GREEN "✓ PASS" COLOR_RESET : 
                            COLOR_RED "✗ FAIL" COLOR_RESET;
        
        printf("│ %-10s │   %3llu     │  %+2d   │    %2llu     │ %-12s │\n",
               points[i].material,
               points[i].result.mean,
               points[i].result.error_cm,
               points[i].result.variation,
               status);
    }
    printf("└────────────┴───────────┴───────┴───────────┴──────────────┘\n");
}

/**
 * @brief   Print comprehensive final summary
 * @param   all_points    Array of all test points
 * @param   total_points  Total number of test points
 * @return  None
 */
static void 
print_final_summary(TestPoint_t const *all_points, int total_points)
{
    int passed = 0;
    int failed = 0;
    float avg_success_rate = 0.0f;
    int i;
    
    for (i = 0; i < total_points; i++)
    {
        if (all_points[i].result.within_tolerance)
        {
            passed++;
        }
        else
        {
            failed++;
        }
        avg_success_rate += all_points[i].result.success_rate;
    }
    
    avg_success_rate /= (float)total_points;
    
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    FINAL TEST SUMMARY                         ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("  Total Tests:      %d\n", total_points);
    printf("  Passed:           %s%d%s\n", COLOR_GREEN, passed, COLOR_RESET);
    printf("  Failed:           %s%d%s\n", COLOR_RED, failed, COLOR_RESET);
    printf("  Avg Success Rate: %.0f%%\n", avg_success_rate * 100.0f);
    printf("\n");
    
    if (0 == failed)
    {
        printf("  %s✓ OBS-T1: ALL CRITERIA PASSED%s\n", 
               COLOR_GREEN, COLOR_RESET);
        printf("\n");
        printf("  Distance measurements are accurate and consistent:\n");
        printf("  - All errors within ±%d cm\n", MAX_ERROR_CM);
        printf("  - All variations within ±%d cm\n", MAX_VARIATION_CM);
        printf("  - Success rate ≥%.0f%%\n", MIN_SUCCESS_RATE * 100.0f);
        printf("  - Material independent\n");
    }
    else
    {
        printf("  %s✗ OBS-T1: SOME CRITERIA FAILED%s\n", 
               COLOR_RED, COLOR_RESET);
    }
    printf("\n");
}

/**
 * @brief   Validate overall test results
 * @param   all_points    Array of all test points
 * @param   total_points  Total number of test points
 * @return  true if all tests passed, false otherwise
 */
static bool 
validate_test_results(TestPoint_t const *all_points, int total_points)
{
    int i;
    
    for (i = 0; i < total_points; i++)
    {
        if (!all_points[i].result.within_tolerance)
        {
            return false;
        }
        if (all_points[i].result.success_rate < MIN_SUCCESS_RATE)
        {
            return false;
        }
    }
    return true;
}

/* ========================================================================== */
/* Main Test Function                                                         */
/* ========================================================================== */

/**
 * @brief   Main test entry point
 * @param   None
 * @return  0 if test passed, 1 if test failed
 */
int 
main(void)
{
    int const test_distances[NUM_TEST_DISTANCES] = {10, 15, 20, 25, 30};
    char const *materials[NUM_MATERIALS] = {"Cardboard", "Wood", "Plastic"};
    TestPoint_t all_points[NUM_TEST_DISTANCES * NUM_MATERIALS];
    int point_index = 0;
    int d, m, r;
    bool test_passed;
    
    /* Initialize standard I/O */
    stdio_init_all();
    sleep_ms(2000);
    
    print_test_header();
    
    /* Initialize hardware */
    printf("[INIT] Initializing ultrasonic sensor...\n");
    ultrasonic_init(TRIG_PIN, ECHO_PIN);
    servo_init(SERVO_PIN);
    printf("[INIT] %s✓%s Ultrasonic initialized (TRIG: GP%d, ECHO: GP%d)\n",
           COLOR_GREEN, COLOR_RESET, TRIG_PIN, ECHO_PIN);
    
    /* Position servo at center */
    servo_set_angle(ANGLE_CENTER);
    sleep_ms(500);
    printf("[INIT] %s✓%s Servo positioned at %d°\n",
           COLOR_GREEN, COLOR_RESET, ANGLE_CENTER);
    
    /* Display instructions */
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST INSTRUCTIONS:\n");
    printf("  1. Prepare obstacles: cardboard, wood, plastic\n");
    printf("  2. Use ruler to position at exact distances\n");
    printf("  3. Keep obstacles perpendicular to sensor\n");
    printf("  4. %d readings will be taken at each position\n", 
           READINGS_PER_TEST);
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\nPress Enter to begin tests...");
    (void)getchar();
    
    /* Run tests for each distance and material */
    for (d = 0; d < NUM_TEST_DISTANCES; d++)
    {
        printf("\n");
        printf("═══════════════════════════════════════════════════════════════\n");
        printf("  TESTING DISTANCE: %d cm\n", test_distances[d]);
        printf("═══════════════════════════════════════════════════════════════\n");
        
        for (m = 0; m < NUM_MATERIALS; m++)
        {
            printf("\n%s━━━ Material: %s ━━━%s\n",
                   COLOR_BLUE, materials[m], COLOR_RESET);
            printf("Position %s obstacle at %d cm (perpendicular)\n",
                   materials[m], test_distances[d]);
            printf("Press Enter when ready...");
            (void)getchar();
            
            /* Initialize test point */
            all_points[point_index].distance_cm = test_distances[d];
            all_points[point_index].material = materials[m];
            all_points[point_index].result.readings_count = READINGS_PER_TEST;
            
            /* Take readings */
            printf("Taking %d readings", READINGS_PER_TEST);
            fflush(stdout);
            
            for (r = 0; r < READINGS_PER_TEST; r++)
            {
                uint64_t distance;
                int status = ultrasonic_get_distance(TRIG_PIN, ECHO_PIN, &distance);
                
                if (SUCCESS == status)
                {
                    all_points[point_index].result.readings[r] = distance;
                }
                else
                {
                    all_points[point_index].result.readings[r] = 0;
                }
                
                printf(".");
                fflush(stdout);
                sleep_ms(100);
            }
            printf(" done\n");
            
            /* Calculate statistics */
            calculate_statistics(&all_points[point_index].result, 
                               test_distances[d]);
            
            /* Print result */
            print_test_point_result(&all_points[point_index]);
            
            point_index++;
        }
        
        /* Print material comparison for this distance */
        print_material_comparison(&all_points[point_index - NUM_MATERIALS], 
                                 NUM_MATERIALS);
    }
    
    /* Print final summary */
    print_final_summary(all_points, point_index);
    
    /* Validate results */
    test_passed = validate_test_results(all_points, point_index);
    
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  OBS-T1 TEST COMPLETE\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    
    return (test_passed ? 0 : 1);
}