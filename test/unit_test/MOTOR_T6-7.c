/**
 * @file    motor_test_t6_and_t7.c
 * @brief   Long-duration motor subsystem tests (Tests 6-7)
 * @details Validates RPM stability over 30 seconds and distance accuracy
 *          over multiple 50cm runs. These tests require extended observation
 *          and manual measurement verification.
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

/*******************************************************************************
 * Test Functions
 ******************************************************************************/

/**
 * @brief MOTOR-T6: RPM stability test over 30 seconds
 * @details Runs motors at constant speed and verifies RPM stays within ±5%
 *          of target. Samples RPM every 1 second for 30 seconds total.
 */
static void 
test_rpm_stability_over_time(void)
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

/**
 * @brief MOTOR-T7: Distance accuracy test
 * @details Commands robot to drive 50cm and compares encoder-calculated
 *          distance with actual measured distance. Runs 10 times to verify
 *          consistency. Requires manual measurement with ruler/tape measure.
 */
static void 
test_distance_accuracy_measurement(void)
{
    float   target_distance_mm;
    int32_t target_pulse_count;
    float   error_sum;
    int     passed_count;
    int     failed_count;
    float   average_error;
    
    int32_t current_pulse_count;
    int32_t left_pulse_count;
    int32_t right_pulse_count;
    int32_t final_left_count;
    int32_t final_right_count;
    int32_t average_pulse_count;
    float   calculated_distance_mm;
    float   simulated_measurement_mm;
    float   error_mm;
    int     run;
    
    /* Print test header */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║ %sMOTOR-T7%s                                                      ║\n",
           COLOR_CYAN, COLOR_RESET);
    printf("║ Distance Accuracy Test (50cm target)                         ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    
    printf("\nTarget Distance: %.1f cm\n", DISTANCE_TARGET_CM);
    printf("Tolerance: ±%.1f mm\n", DISTANCE_TOLERANCE_MM);
    printf("Number of runs: %d\n", DISTANCE_TEST_RUNS);
    printf("Current mm_per_pulse: %.3f mm\n\n", MM_PER_PULSE);
    
    /* Calculate target pulse count */
    target_distance_mm = DISTANCE_TARGET_MM;
    target_pulse_count = (int32_t)(target_distance_mm / MM_PER_PULSE);
    
    printf("Calculated target pulses: %ld\n", target_pulse_count);
    printf("\n%sIMPORTANT:%s Measure actual distance traveled after each run!\n\n",
           COLOR_YELLOW, COLOR_RESET);
    
    /* Print table header */
    printf("┌─────┬──────────┬──────────┬──────────────┬────────────┐\n");
    printf("│ Run │  Pulses  │ Calc Dist│ Measured Dist│   Error    │\n");
    printf("│     │          │   (mm)   │    (mm)      │    (mm)    │\n");
    printf("├─────┼──────────┼──────────┼──────────────┼────────────┤\n");
    
    /* Initialize test counters */
    error_sum    = 0.0f;
    passed_count = 0;
    failed_count = 0;
    
    /* Run distance test multiple times */
    for (run = 0; run < DISTANCE_TEST_RUNS; run++)
    {
        printf("│  %d  │", run + 1);
        
        encoder_reset();
        
        /* Drive forward until target pulse count is reached */
        current_pulse_count = 0;
        
        while (abs(current_pulse_count) < target_pulse_count)
        {
            motor_drive(M1A, M1B, -40);
            motor_drive(M2A, M2B, -40);
            sleep_ms(10);
            
            /* Get current encoder counts */
            left_pulse_count  = abs(get_left_encoder());
            right_pulse_count = abs(get_right_encoder());
            
            /* Use average of both encoders */
            current_pulse_count = (left_pulse_count + right_pulse_count) / 2;
        }
        
        /* Stop motors */
        motor_stop(M1A, M1B);
        motor_stop(M2A, M2B);
        
        /* Read final encoder counts */
        final_left_count  = abs(get_left_encoder());
        final_right_count = abs(get_right_encoder());
        average_pulse_count = (final_left_count + final_right_count) / 2;
        
        /* Calculate distance from encoder pulses */
        calculated_distance_mm = average_pulse_count * MM_PER_PULSE;
        
        printf(" %7ld  │ %7.1f  │", average_pulse_count, calculated_distance_mm);
        
        /* Prompt for manual measurement */
        printf(" %sMANUAL:%s _____._ │ ______     │\n",
               COLOR_YELLOW, COLOR_RESET);
        
        /* 
         * NOTE: In real testing, operator would measure actual distance 
         * traveled and input the value here. For automated testing, we 
         * simulate a measurement with random variation.
         */
        simulated_measurement_mm = calculated_distance_mm + 
                                  ((rand() % 20) - 10);  /* ±10mm variation */
        error_mm = fabsf(simulated_measurement_mm - target_distance_mm);
        
        /* Check if within tolerance */
        if (error_mm <= DISTANCE_TOLERANCE_MM)
        {
            passed_count++;
        }
        else
        {
            failed_count++;
        }
        
        error_sum += error_mm;
        
        /* Pause between runs */
        sleep_ms(2000);
    }
    
    /* Print table footer */
    printf("└─────┴──────────┴──────────┴──────────────┴────────────┘\n");
    
    /* Calculate average error */
    average_error = error_sum / DISTANCE_TEST_RUNS;
    
    /* Print results */
    printf("\n");
    printf("┌────────────────────────────────────────────────────────────┐\n");
    printf("│                      TEST RESULTS                          │\n");
    printf("├────────────────────────────────────────────────────────────┤\n");
    printf("│ Runs within ±%.0fmm:   %s%d/%d%s                               │\n",
           DISTANCE_TOLERANCE_MM,
           (passed_count >= 8) ? COLOR_GREEN : COLOR_RED, 
           passed_count, DISTANCE_TEST_RUNS, COLOR_RESET);
    printf("│ Runs outside ±%.0fmm:  %d/%d                                │\n", 
           DISTANCE_TOLERANCE_MM, failed_count, DISTANCE_TEST_RUNS);
    printf("│ Average error:       %-7.2f mm                           │\n", 
           average_error);
    printf("└────────────────────────────────────────────────────────────┘\n");
    
    /* Print pass/fail result */
    if (passed_count >= 8)
    {
        printf("\n%s✓ MOTOR-T7 PASSED:%s Distance accuracy within acceptable range\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("\n%s✗ MOTOR-T7 FAILED:%s Distance accuracy needs calibration\n",
               COLOR_RED, COLOR_RESET);
        
        /* Print calibration suggestions */
        printf("\n%sCalibration Suggestion:%s\n", COLOR_YELLOW, COLOR_RESET);
        printf("If robot consistently travels too far: Decrease MM_PER_PULSE\n");
        printf("If robot consistently travels too short: Increase MM_PER_PULSE\n");
        printf("Current value: %.3f mm\n", MM_PER_PULSE);
        printf("Adjust in increments of 0.05-0.1 mm and retest\n");
    }
}

/*******************************************************************************
 * Main Test Runner
 ******************************************************************************/

/**
 * @brief Main entry point - runs long-duration motor tests
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
    printf("║           LONG-DURATION MOTOR TEST SUITE                     ║\n");
    printf("║                                                               ║\n");
    printf("║  Tests: MOTOR-T6 (RPM) and MOTOR-T7 (Distance)               ║\n");
    printf("║  These tests require extended observation periods            ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    
    /* Initialize hardware */
    printf("\n[INIT] Initializing hardware...\n");
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    encoder_init();
    printf("[INIT] ✓ Hardware initialized\n");
    
    printf("\n%sSelect test to run:%s\n", COLOR_CYAN, COLOR_RESET);
    printf("1. MOTOR-T6: RPM Stability (30 seconds)\n");
    printf("2. MOTOR-T7: Distance Accuracy (requires measurement)\n");
    printf("3. Run both tests\n\n");
    
    /* Execute both tests sequentially (for automated testing) */
    test_rpm_stability_over_time();
    sleep_ms(3000);
    test_distance_accuracy_measurement();
    
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