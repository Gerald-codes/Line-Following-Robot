/**
 * @file    motor_test_t1_through_t5.c
 * @brief   Comprehensive motor subsystem test suite (Tests 1-5)
 * @details Validates motor initialization, speed control, encoder tracking,
 *          and PID controller functionality across 100 iterations per test.
 *
 * @note    Barr C Coding Standard compliant
 * @author  Your Name
 * @date    November 23, 2025
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

/*******************************************************************************
 * Configuration Constants
 ******************************************************************************/
#define TEST_ITERATIONS         100

/* ANSI color codes for terminal output */
#define COLOR_GREEN             "\033[32m"
#define COLOR_RED               "\033[31m"
#define COLOR_YELLOW            "\033[33m"
#define COLOR_BLUE              "\033[34m"
#define COLOR_CYAN              "\033[36m"
#define COLOR_RESET             "\033[0m"

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/**
 * @brief Test result statistics structure
 */
typedef struct
{
    int   passed_count;      /* Number of tests that passed */
    int   failed_count;      /* Number of tests that failed */
    int   total_count;       /* Total number of tests run */
    float minimum_value;     /* Minimum value recorded */
    float maximum_value;     /* Maximum value recorded */
    float average_value;     /* Average of all values */
    float sum_of_values;     /* Running sum of values */
} TestResults;

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
static void print_test_header(char const *test_id, char const *test_name);
static void print_results_table(TestResults *results);
static void update_statistics(TestResults *results, float value);
static void print_progress_bar(int current_iteration, int total_iterations);

/*******************************************************************************
 * Helper Functions - Output Formatting
 ******************************************************************************/

/**
 * @brief Print formatted test header
 * @param test_id   Test identifier (e.g., "MOTOR-T1")
 * @param test_name Descriptive test name
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
 * @param results Pointer to test results structure
 */
static void 
print_results_table(TestResults *results)
{
    float success_rate;
    
    /* Calculate success rate percentage */
    success_rate = (results->passed_count * 100.0f) / results->total_count;
    
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
 * @brief Update running statistics with new value
 * @param results Pointer to test results structure
 * @param value   New value to include in statistics
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
    
    /* Calculate new average */
    results->average_value = results->sum_of_values / results->total_count;
}

/**
 * @brief Print progress bar for long-running tests
 * @param current_iteration Current iteration number
 * @param total_iterations  Total number of iterations
 */
static void 
print_progress_bar(int current_iteration, int total_iterations)
{
    int const bar_width = 40;
    int       filled_segments;
    int       i;
    
    /* Only update display every 10 iterations or on completion */
    if ((current_iteration % 10 == 0) || (current_iteration == total_iterations))
    {
        filled_segments = (current_iteration * bar_width) / total_iterations;
        
        printf("  [%3d/%3d] ", current_iteration, total_iterations);
        printf("[");
        
        for (i = 0; i < bar_width; i++)
        {
            printf((i < filled_segments) ? "█" : "░");
        }
        
        printf("] %.0f%%\r", (current_iteration * 100.0f) / total_iterations);
        fflush(stdout);
    }
}

/*******************************************************************************
 * Test Functions
 ******************************************************************************/

/**
 * @brief MOTOR-T1: Motor initialization and direction control test
 * @details Validates forward, backward, and stop commands by checking GPIO states
 */
static void 
test_motor_initialization_and_direction(void)
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
    
    int  checks_passed;
    bool motor1_active;
    bool motor2_active;
    bool motor1_stopped;
    bool motor2_stopped;
    bool test_passed;
    int  i;
    
    print_test_header("MOTOR-T1", "Motor Initialization and Direction Control");
    
    printf("\nTesting: Forward, Backward, Stop commands\n");
    printf("Measuring: GPIO state changes\n\n");
    
    for (i = 0; i < TEST_ITERATIONS; i++)
    {
        checks_passed = 0;
        
        /* Test forward direction - motors should be active */
        motor_drive(M1A, M1B, -50);
        motor_drive(M2A, M2B, -50);
        sleep_ms(100);
        
        motor1_active = (gpio_get(M1A) != gpio_get(M1B));
        motor2_active = (gpio_get(M2A) != gpio_get(M2B));
        
        if (motor1_active && motor2_active)
        {
            checks_passed++;
        }
        
        /* Test backward direction - motors should be active */
        motor_drive(M1A, M1B, 50);
        motor_drive(M2A, M2B, 50);
        sleep_ms(100);
        
        motor1_active = (gpio_get(M1A) != gpio_get(M1B));
        motor2_active = (gpio_get(M2A) != gpio_get(M2B));
        
        if (motor1_active && motor2_active)
        {
            checks_passed++;
        }
        
        /* Test stop command - both pins should be low */
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        sleep_ms(50);
        
        motor1_stopped = (!gpio_get(M1A) && !gpio_get(M1B));
        motor2_stopped = (!gpio_get(M2A) && !gpio_get(M2B));
        
        if (motor1_stopped && motor2_stopped)
        {
            checks_passed++;
        }
        
        /* All three checks must pass */
        test_passed = (checks_passed == 3);
        
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
        printf("\n%s✓ MOTOR-T1 PASSED:%s All direction commands working correctly\n", 
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("\n%s✗ MOTOR-T1 FAILED:%s Some commands did not respond correctly\n", 
               COLOR_RED, COLOR_RESET);
    }
}

/**
 * @brief MOTOR-T2: Motor speed level verification test
 * @details Tests PWM control at 0%, 25%, 50%, 75%, and 100% duty cycles
 * @note Physical observation recommended for actual speed verification
 */
static void 
test_motor_speed_levels(void)
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

/**
 * @brief MOTOR-T3: Encoder count matching test
 * @details Verifies left and right encoders track symmetrically within 10% tolerance
 */
static void 
test_encoder_count_matching(void)
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

/**
 * @brief MOTOR-T4: PID algorithm calculation verification
 * @details Tests P-only and I-only control calculations with known inputs
 */
static void 
test_pid_calculation_accuracy(void)
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

/**
 * @brief MOTOR-T5: PID output limiting test
 * @details Verifies PID output clamping to prevent motor over-drive
 */
static void 
test_pid_output_limits(void)
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
    
    PIDController pid;
    float         output_positive_clamp;
    float         output_negative_clamp;
    float         output_no_clamp;
    int           checks_passed;
    bool          test_passed;
    int           i;
    
    print_test_header("MOTOR-T5", "PID Output Limits for Safety");
    
    printf("\nTesting: PID output clamping to [-100, +100]\n");
    printf("Test cases: error=50, error=-50, error=5\n\n");
    
    for (i = 0; i < TEST_ITERATIONS; i++)
    {
        checks_passed = 0;
        
        pid_init(&pid, 10.0f, 0.0f, 0.0f, -100.0f, 100.0f);
        
        /* Test 1: Large positive error (should clamp to +100) */
        pid_set_target(&pid, 50.0f);
        output_positive_clamp = pid_compute(&pid, 0.0f, 0.1f);  /* Error = 50 */
        
        if (output_positive_clamp == 100.0f)
        {
            checks_passed++;
        }
        
        pid_reset(&pid);
        
        /* Test 2: Large negative error (should clamp to -100) */
        pid_set_target(&pid, -50.0f);
        output_negative_clamp = pid_compute(&pid, 0.0f, 0.1f);  /* Error = -50 */
        
        if (output_negative_clamp == -100.0f)
        {
            checks_passed++;
        }
        
        pid_reset(&pid);
        
        /* Test 3: Small error (should not clamp, output = 50) */
        pid_set_target(&pid, 5.0f);
        output_no_clamp = pid_compute(&pid, 0.0f, 0.1f);  /* Error = 5 */
        
        if (output_no_clamp == 50.0f)
        {
            checks_passed++;
        }
        
        /* All three checks must pass */
        test_passed = (checks_passed == 3);
        
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
        printf("\n%s✓ MOTOR-T5 PASSED:%s PID limits working correctly\n", 
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("\n%s✗ MOTOR-T5 FAILED:%s PID limiting not working as expected\n", 
               COLOR_RED, COLOR_RESET);
    }
}

/*******************************************************************************
 * Main Test Runner
 ******************************************************************************/

/**
 * @brief Main entry point - runs complete motor test suite
 * @return Exit code (0 = success)
 */
int 
main(void)
{
    /* Initialize standard I/O for USB serial */
    stdio_init_all();
    sleep_ms(2000);
    
    /* Print test suite header */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║        COMPREHENSIVE MOTOR TEST SUITE (100 iterations)       ║\n");
    printf("║                                                               ║\n");
    printf("║  Tests: MOTOR-T1 through MOTOR-T5                            ║\n");
    printf("║  Each test runs 100 times with statistics                    ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    
    /* Initialize hardware */
    printf("\n[INIT] Initializing hardware...\n");
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    encoder_init();
    printf("[INIT] ✓ Hardware initialized\n");
    
    printf("\n%sStarting test execution...%s\n", COLOR_BLUE, COLOR_RESET);
    
    /* Execute all tests sequentially */
    test_motor_initialization_and_direction();
    test_motor_speed_levels();
    test_encoder_count_matching();
    test_pid_calculation_accuracy();
    test_pid_output_limits();
    
    /* Print final summary */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    ALL TESTS COMPLETE                        ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    /* Ensure motors are stopped */
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    return 0;
}