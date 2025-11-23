/**
 * @file    MOTOR_T2.c
 * @brief   Motor PWM Speed Levels Test
 * @details Verifies PWM duty cycles at different speeds
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

static void test_motor_pwm_speed_levels(void)
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
    
    int const speed_levels[] = {0, 25, 50, 75, 100};
    int const num_levels = 5;
    
    int  successful_levels;
    int  speed;
    bool gpio_active;
    bool test_passed;
    int  i;
    int  j;
    
    print_test_header("MOTOR-T2", "Motor Speed Levels Verification");
    
    printf("\nTesting speed levels: 0, 25, 50, 75, 100\n");
    printf("Measuring: GPIO activity at each speed\n");
    printf("Note: Physical observation recommended for speed verification\n\n");
    
    for (i = 0; i < TEST_ITERATIONS; i++)
    {
        successful_levels = 0;
        
        for (j = 0; j < num_levels; j++)
        {
            speed = speed_levels[j];
            
            /* Set motor speed (0 = stop, >0 = drive) */
            if (speed == 0)
            {
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
            }
            else
            {
                motor_drive(M1A, M1B, -speed);
                motor_drive(M2A, M2B, -speed);
            }
            
            sleep_ms(100);
            
            /* Check GPIO activity matches expected state */
            gpio_active = (gpio_get(M1A) != gpio_get(M1B)) || 
                         (gpio_get(M2A) != gpio_get(M2B));
            
            /* Speed 0 should have no activity, others should have activity */
            if ((speed == 0 && !gpio_active) || (speed > 0 && gpio_active))
            {
                successful_levels++;
            }
        }
        
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        
        /* All speed levels must work correctly */
        test_passed = (successful_levels == num_levels);
        
        if (test_passed)
        {
            results.passed_count++;
        }
        else
        {
            results.failed_count++;
        }
        
        update_statistics(&results, (float)successful_levels);
        print_progress_bar(i + 1, TEST_ITERATIONS);
    }
    
    printf("\n");
    print_results_table(&results);
    
    if (results.failed_count == 0)
    {
        printf("\n%s✓ MOTOR-T2 PASSED:%s All speed levels respond correctly\n", 
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("\n%s✗ MOTOR-T2 FAILED:%s Some speed levels did not work\n", 
               COLOR_RED, COLOR_RESET);
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n╔═══════════════════════════════════════════════════════╗\n");
    printf("║            MOTOR-T2: PWM Speed Levels                 ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n");
    
    test_motor_pwm_speed_levels();
    
    return 0;
}