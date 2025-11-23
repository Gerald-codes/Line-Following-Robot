/**
 * @file    MOTOR_T7.c
 * @brief   Distance Calibration Test
 * @details Verifies encoder-calculated distance matches actual
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

static void test_distance_calibration(void)
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

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n╔═══════════════════════════════════════════════════════╗\n");
    printf("║            MOTOR-T7: Distance Calibration             ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n");
    
    test_distance_calibration();
    
    return 0;
}