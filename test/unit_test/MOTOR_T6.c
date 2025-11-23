/**
 * @file    MOTOR_T6.c
 * @brief   RPM Stability Test (30 seconds)
 * @details Verifies PID maintains 50 RPM within ±5%
 */

#include "pico/stdlib.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "pin_definitions.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/*******************************************************************************
 * Configuration Constants
 ******************************************************************************/

/* ANSI color codes for terminal output */
#define COLOR_GREEN             "\033[32m"
#define COLOR_RED               "\033[31m"
#define COLOR_YELLOW            "\033[33m"
#define COLOR_CYAN              "\033[36m"
#define COLOR_RESET             "\033[0m"

/* Robot physical parameters */
#define WHEEL_DIAMETER_MM       65.0f    /* Wheel diameter in millimeters */
#define PULSES_PER_REVOLUTION   20       /* Encoder pulses per wheel rotation */

/* Calculated conversion factor */
#define MM_PER_PULSE            ((WHEEL_DIAMETER_MM * 3.14159f) / PULSES_PER_REVOLUTION)

/* Test parameters */
#define RPM_TEST_DURATION_SEC   30       /* T6: Test duration in seconds */
#define RPM_TARGET              50.0f    /* T6: Target RPM */
#define RPM_TOLERANCE_PERCENT   5.0f     /* T6: Allowed deviation (±5%) */
#define RPM_MIN_ACCEPTABLE      (RPM_TARGET * (1.0f - RPM_TOLERANCE_PERCENT / 100.0f))
#define RPM_MAX_ACCEPTABLE      (RPM_TARGET * (1.0f + RPM_TOLERANCE_PERCENT / 100.0f))

#define DISTANCE_TARGET_CM      50.0f    /* T7: Target distance in centimeters */
#define DISTANCE_TARGET_MM      (DISTANCE_TARGET_CM * 10.0f)
#define DISTANCE_TOLERANCE_MM   10.0f    /* T7: Allowed error in millimeters */
#define DISTANCE_TEST_RUNS      10       /* T7: Number of test runs */


static void test_rpm_stability(void)
{
    PIDController rpm_controller;
    float         target_rpm;
    int           total_samples;
    int           samples_in_tolerance;
    int           samples_out_tolerance;
    float         rpm_minimum;
    float         rpm_maximum;
    float         rpm_sum;
    float         rpm_average;
    float         success_rate;
    
    uint32_t      start_time_ms;
    uint32_t      elapsed_time_ms;
    int32_t       left_pulse_count;
    int32_t       right_pulse_count;
    float         left_rpm;
    float         right_rpm;
    float         average_rpm;
    bool          within_tolerance;
    int           second;
    
    /* Print test header */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║ %sMOTOR-T6%s                                                      ║\n", 
           COLOR_CYAN, COLOR_RESET);
    printf("║ RPM Stability Test (30 seconds)                              ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    
    printf("\nTarget RPM: %.0f\n", RPM_TARGET);
    printf("Tolerance: ±%.0f%% (%.1f - %.1f RPM)\n", 
           RPM_TOLERANCE_PERCENT, RPM_MIN_ACCEPTABLE, RPM_MAX_ACCEPTABLE);
    printf("Duration: %d seconds\n", RPM_TEST_DURATION_SEC);
    printf("Sampling: Every 1 second\n\n");
    
    /* Initialize PID controller for RPM control */
    pid_init(&rpm_controller, 2.0f, 0.5f, 0.1f, -100.0f, 100.0f);
    
    /* Initialize test variables */
    target_rpm              = RPM_TARGET;
    total_samples           = RPM_TEST_DURATION_SEC;
    samples_in_tolerance    = 0;
    samples_out_tolerance   = 0;
    rpm_minimum             = 999999.0f;
    rpm_maximum             = 0.0f;
    rpm_sum                 = 0.0f;
    
    /* Print table header */
    printf("┌──────┬────────────┬────────────┬────────────┬────────┐\n");
    printf("│ Time │  Left RPM  │ Right RPM  │  Avg RPM   │ Status │\n");
    printf("├──────┼────────────┼────────────┼────────────┼────────┤\n");
    
    /* Run test for specified duration */
    for (second = 0; second < total_samples; second++)
    {
        encoder_reset();
        start_time_ms = to_ms_since_boot(get_absolute_time());
        
        /* Drive motors at constant speed for 1 second */
        motor_drive(M1A, M1B, -50);
        motor_drive(M2A, M2B, -50);
        sleep_ms(1000);
        
        /* Calculate elapsed time */
        elapsed_time_ms = to_ms_since_boot(get_absolute_time()) - start_time_ms;
        
        /* Read encoder counts */
        left_pulse_count  = abs(get_left_encoder());
        right_pulse_count = abs(get_right_encoder());
        
        /* Calculate RPM for each wheel */
        /* RPM = (pulses * 60000 ms/min) / (pulses_per_rev * elapsed_ms) */
        left_rpm  = (left_pulse_count * 60000.0f) / 
                    (PULSES_PER_REVOLUTION * elapsed_time_ms);
        right_rpm = (right_pulse_count * 60000.0f) / 
                    (PULSES_PER_REVOLUTION * elapsed_time_ms);
        average_rpm = (left_rpm + right_rpm) / 2.0f;
        
        /* Check if within tolerance */
        within_tolerance = (average_rpm >= RPM_MIN_ACCEPTABLE) && 
                          (average_rpm <= RPM_MAX_ACCEPTABLE);
        
        if (within_tolerance)
        {
            samples_in_tolerance++;
        }
        else
        {
            samples_out_tolerance++;
        }
        
        /* Update statistics */
        if (average_rpm < rpm_minimum)
        {
            rpm_minimum = average_rpm;
        }
        
        if (average_rpm > rpm_maximum)
        {
            rpm_maximum = average_rpm;
        }
        
        rpm_sum += average_rpm;
        
        /* Print table row */
        printf("│ %2ds   │ %8.2f   │ %8.2f   │ %8.2f   │ %s%-6s%s │\n",
               second + 1, left_rpm, right_rpm, average_rpm,
               within_tolerance ? COLOR_GREEN : COLOR_RED,
               within_tolerance ? "OK" : "OUT",
               COLOR_RESET);
    }
    
    /* Stop motors */
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    /* Print table footer */
    printf("└──────┴────────────┴────────────┴────────────┴────────┘\n");
    
    /* Calculate final statistics */
    rpm_average  = rpm_sum / total_samples;
    success_rate = (samples_in_tolerance * 100.0f) / total_samples;
    
    /* Print results */
    printf("\n");
    printf("┌────────────────────────────────────────────────────────────┐\n");
    printf("│                      TEST RESULTS                          │\n");
    printf("├────────────────────────────────────────────────────────────┤\n");
    printf("│ Samples in tolerance:    %s%2d/%d (%.1f%%)%s                 │\n",
           (success_rate >= 90.0f) ? COLOR_GREEN : COLOR_RED,
           samples_in_tolerance, total_samples, success_rate, COLOR_RESET);
    printf("│ Samples out of tolerance: %d/%d                            │\n", 
           samples_out_tolerance, total_samples);
    printf("├────────────────────────────────────────────────────────────┤\n");
    printf("│ Min RPM:                 %-8.2f                           │\n", rpm_minimum);
    printf("│ Max RPM:                 %-8.2f                           │\n", rpm_maximum);
    printf("│ Avg RPM:                 %-8.2f                           │\n", rpm_average);
    printf("│ Deviation:               ±%-7.2f                          │\n", 
           rpm_maximum - rpm_minimum);
    printf("└────────────────────────────────────────────────────────────┘\n");
    
    /* Print pass/fail result */
    if (success_rate >= 90.0f)
    {
        printf("\n%s✓ MOTOR-T6 PASSED:%s RPM stability maintained >90%% of time\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("\n%s✗ MOTOR-T6 FAILED:%s RPM stability below 90%% threshold\n",
               COLOR_RED, COLOR_RESET);
        printf("Consider: PID tuning or mechanical issues\n");
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n╔═══════════════════════════════════════════════════════╗\n");
    printf("║            MOTOR-T6: RPM Stability (30s)              ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n");
    
    test_rpm_stability();
    
    return 0;
}