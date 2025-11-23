/**
 * @file    MOTOR_T4.c
 * @brief   PID Calculation Accuracy Test
 * @details Verifies PID algorithm calculates correctly
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

static void test_pid_calculation(void)
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
    
    PIDController pid_proportional;
    PIDController pid_integral;
    float         p_output;
    float         i_output;
    float         expected_integral_output;
    int           checks_passed;
    bool          test_passed;
    int           i;
    int           j;
    
    print_test_header("MOTOR-T4", "PID Algorithm Calculation Verification");
    
    printf("\nTesting: PID calculations with known inputs\n");
    printf("Test 1: Kp=1.0, Ki=0, Kd=0, error=10 → output should be 10\n");
    printf("Test 2: Kp=0, Ki=0.5, Kd=0, error=10 for 5 cycles → integral check\n\n");
    
    for (i = 0; i < TEST_ITERATIONS; i++)
    {
        checks_passed = 0;
        
        /* Test 1: Pure proportional control (P-only) */
        pid_init(&pid_proportional, 1.0f, 0.0f, 0.0f, -100.0f, 100.0f);
        pid_set_target(&pid_proportional, 10.0f);  /* Target = 10 */
        p_output = pid_compute(&pid_proportional, 0.0f, 0.1f);  /* Measured = 0 */
        
        /* Output should be Kp * error = 1.0 * 10 = 10.0 */
        if (fabsf(p_output - 10.0f) < 0.1f)
        {
            checks_passed++;
        }
        
        /* Test 2: Pure integral control (I-only) */
        pid_init(&pid_integral, 0.0f, 0.5f, 0.0f, -100.0f, 100.0f);
        pid_set_target(&pid_integral, 10.0f);  /* Target = 10 */
        
        /* Accumulate integral over 5 cycles */
        expected_integral_output = 0.0f;
        
        for (j = 0; j < 5; j++)
        {
            i_output = pid_compute(&pid_integral, 0.0f, 0.1f);
            expected_integral_output += 10.0f * 0.1f * 0.5f;  /* Ki * error * dt */
        }
        
        /* Compute one more time to get final output */
        i_output = pid_compute(&pid_integral, 0.0f, 0.1f);
        expected_integral_output += 10.0f * 0.1f * 0.5f;
        
        /* Final output should match accumulated integral */
        if (fabsf(i_output - expected_integral_output) < 0.5f)
        {
            checks_passed++;
        }
        
        /* Both checks must pass */
        test_passed = (checks_passed == 2);
        
        if (test_passed)
        {
            results.passed_count++;
        }
        else
        {
            results.failed_count++;
        }
        
        update_statistics(&results, (float)checks_passed);
        print_progress_bar(i + 1, TEST_ITERATIONS);
    }
    
    printf("\n");
    print_results_table(&results);
    
    if (results.failed_count == 0)
    {
        printf("\n%s✓ MOTOR-T4 PASSED:%s PID calculations are accurate\n", 
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("\n%s✗ MOTOR-T4 FAILED:%s PID calculation errors detected\n", 
               COLOR_RED, COLOR_RESET);
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n╔═══════════════════════════════════════════════════════╗\n");
    printf("║            MOTOR-T4: PID Calculation                  ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n");
    
    test_pid_calculation();
    
    return 0;
}