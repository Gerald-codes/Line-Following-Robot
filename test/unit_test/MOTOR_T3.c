/**
 * @file    MOTOR_T3.c
 * @brief   Encoder Synchronization Test
 * @details Verifies left/right encoder counts match within 10%
 */

#include "pico/stdlib.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "pin_definitions.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Configuration */
#define TEST_ITERATIONS         100

/* ANSI Colors */
#define COLOR_GREEN             "\033[32m"
#define COLOR_RED               "\033[31m"
#define COLOR_YELLOW            "\033[33m"
#define COLOR_BLUE              "\033[34m"
#define COLOR_CYAN              "\033[36m"
#define COLOR_RESET             "\033[0m"

/* Test Results Structure */
typedef struct
{
    int   passed_count;
    int   failed_count;
    int   total_count;
    float minimum_value;
    float maximum_value;
    float average_value;
    float sum_of_values;
} TestResults;

/* Helper Functions */
static void print_test_header(char const *test_id, char const *test_name);
static void print_results_table(TestResults *results);
static void update_statistics(TestResults *results, float value);
static void print_progress_bar(int current_iteration, int total_iterations);

/**
 * @brief Print formatted test header
 */
static void 
print_test_header(char const *test_id, char const *test_name)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║ %s%-60s%s║\n", COLOR_CYAN, test_id, COLOR_RESET);
    printf("║ %s\n", test_name);
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
}

/**
 * @brief Print formatted results table
 */
static void 
print_results_table(TestResults *results)
{
    float success_rate = (results->passed_count * 100.0f) / results->total_count;
    
    printf("\n");
    printf("┌────────────────────────────────────────────────────────────┐\n");
    printf("│                      TEST RESULTS                          │\n");
    printf("├────────────────────────────────────────────────────────────┤\n");
    printf("│ Total Tests:    %s%-43d%s│\n", 
           COLOR_BLUE, results->total_count, COLOR_RESET);
    printf("│ Passed:         %s%-43d%s│\n", 
           COLOR_GREEN, results->passed_count, COLOR_RESET);
    printf("│ Failed:         %s%-43d%s│\n", 
           COLOR_RED, results->failed_count, COLOR_RESET);
    printf("│ Success Rate:   %s%.1f%%%s                                     │\n", 
           (results->failed_count == 0) ? COLOR_GREEN : COLOR_YELLOW,
           success_rate, COLOR_RESET);
    printf("├────────────────────────────────────────────────────────────┤\n");
    printf("│ Min Value:      %-43.2f│\n", results->minimum_value);
    printf("│ Max Value:      %-43.2f│\n", results->maximum_value);
    printf("│ Avg Value:      %-43.2f│\n", results->average_value);
    printf("└────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief Update running statistics
 */
static void 
update_statistics(TestResults *results, float value)
{
    results->sum_of_values += value;
    
    if (value < results->minimum_value)
    {
        results->minimum_value = value;
    }
    
    if (value > results->maximum_value)
    {
        results->maximum_value = value;
    }
    
    results->average_value = results->sum_of_values / results->total_count;
}

/**
 * @brief Print progress bar
 */
static void 
print_progress_bar(int current_iteration, int total_iterations)
{
    int const bar_width = 40;
    int       filled_segments;
    
    if ((current_iteration % 10 == 0) || (current_iteration == total_iterations))
    {
        filled_segments = (current_iteration * bar_width) / total_iterations;
        
        printf("  [%3d/%3d] [", current_iteration, total_iterations);
        
        for (int i = 0; i < bar_width; i++)
        {
            printf((i < filled_segments) ? "█" : "░");
        }
        
        printf("] %.0f%%\r", (current_iteration * 100.0f) / total_iterations);
        fflush(stdout);
    }
}

static void test_encoder_synchronization(void)
{
    TestResults results = {
        .passed_count   = 0,
        .failed_count   = 0,
        .total_count    = TEST_ITERATIONS,
        .minimum_value  = 999999.0f,
        .maximum_value  = -999999.0f,
        .average_value  = 0.0f,
        .sum_of_values  = 0.0f
    };
    
    int32_t left_count;
    int32_t right_count;
    float   difference_percentage;
    bool    test_passed;
    int     i;
    
    print_test_header("MOTOR-T3", "Encoder Count Matching Between Motors");
    
    printf("\nTesting: Both motors at same speed (50) for 1 second\n");
    printf("Measuring: Difference in encoder counts (must be <10%%)\n\n");
    
    for (i = 0; i < TEST_ITERATIONS; i++)
    {
        encoder_reset();
        
        /* Run motors at same speed */
        motor_drive(M1A, M1B, -50);
        motor_drive(M2A, M2B, -50);
        sleep_ms(1000);
        
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        sleep_ms(100);
        
        /* Get encoder counts */
        left_count  = abs(get_left_encoder());
        right_count = abs(get_right_encoder());
        
        /* Calculate percentage difference relative to total counts */
        difference_percentage = 0.0f;
        
        if ((left_count + right_count) > 0)
        {
            difference_percentage = fabsf((float)(left_count - right_count)) / 
                                   ((float)(left_count + right_count)) * 100.0f;
        }
        
        /* Difference must be less than 10% */
        test_passed = (difference_percentage < 10.0f);
        
        if (test_passed)
        {
            results.passed_count++;
        }
        else
        {
            results.failed_count++;
        }
        
        update_statistics(&results, difference_percentage);
        print_progress_bar(i + 1, TEST_ITERATIONS);
    }
    
    printf("\n");
    print_results_table(&results);
    printf("\nAverage difference: %.2f%% (threshold: <10%%)\n", results.average_value);
    
    if (results.failed_count == 0)
    {
        printf("\n%s✓ MOTOR-T3 PASSED:%s Encoder counts match within acceptable range\n", 
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("\n%s✗ MOTOR-T3 FAILED:%s Encoder mismatch exceeds 10%% tolerance\n", 
               COLOR_RED, COLOR_RESET);
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n╔═══════════════════════════════════════════════════════╗\n");
    printf("║            MOTOR-T3: Encoder Synchronization          ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n");
    
    test_encoder_synchronization();
    
    return 0;
}