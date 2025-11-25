/**
 * @file    INTEG_T4.c
 * @brief   Heading Keeping Mode Test
 * @details Tests the robot's ability to maintain a target heading while driving
 *          using IMU feedback and motor corrections.
 *
 */

#include "pico/stdlib.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "pid.h"
#include "heading_control.h"
#include "config.h"
#include "pin_definitions.h"
#include "calibration.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Test parameters */
#define TEST_DURATION_SEC       10      /* Duration for stability test */
#define HEADING_TOLERANCE_DEG   3.0f    /* Acceptable heading deviation */
#define SAMPLE_PERIOD_MS        100     /* IMU sampling period */
#define PRINT_PERIOD_MS         500     /* Status print interval */

/* Motor control parameters */
#define BASE_SPEED_LEFT         42      /* Left motor base speed (0-100) */
#define BASE_SPEED_RIGHT        40      /* Right motor base speed (0-100) */
#define DEADBAND_DEGREES        3.0f    /* Heading error deadband */
#define CORRECTION_PER_DEGREE   1.0f    /* Correction gain */

/* Test threshold */
#define MIN_STABILITY_PERCENT   85.0f   /* Minimum time in tolerance (%) */

/* ANSI color codes */
#define COLOR_GREEN             "\033[32m"
#define COLOR_RED               "\033[31m"
#define COLOR_YELLOW            "\033[33m"
#define COLOR_CYAN              "\033[36m"
#define COLOR_RESET             "\033[0m"

/**
 * @brief Heading statistics structure
 */
typedef struct
{
    float   minimum_heading;       /* Minimum heading observed */
    float   maximum_heading;       /* Maximum heading observed */
    float   average_error;         /* Average heading error */
    int     samples_in_tolerance;  /* Samples within tolerance */
    int     samples_out_tolerance; /* Samples outside tolerance */
    int     total_samples;         /* Total samples collected */
} HeadingStatistics;

/* Global Variables */

static IMU              g_imu;
static HeadingController g_heading_ctrl;
static PIDController    g_left_motor_pid;
static PIDController    g_right_motor_pid;

/*  Hardware Initialization */

/**
 * @brief Initialize hardware for testing
 */
static void 
initialize_hardware(void)
{
    printf("[INIT] Initializing hardware...\n");
    
    /* Initialize motors */
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    printf("[INIT]   ✓ Motors initialized\n");
    
    /* Initialize encoders */
    encoder_init();
    printf("[INIT]   ✓ Encoders initialized\n");
    
    /* Initialize IMU */
    imu_init(&g_imu);
    printf("[INIT]   ✓ IMU initialized\n");
    
    /* Initialize calibration button */
    calibration_init();
    printf("[INIT]   ✓ Button initialized (GP20)\n");
    
    /* Initialize PID controllers */
    pid_init(&g_left_motor_pid,
             MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD,
             MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);
    
    pid_init(&g_right_motor_pid,
             MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD,
             MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);
    
    printf("[INIT]   ✓ Motor PIDs initialized\n");
    
    /* Initialize heading controller */
    heading_control_init(&g_heading_ctrl,
                        HEADING_PID_KP_DEMO1,
                        HEADING_PID_KI_DEMO1,
                        HEADING_PID_KD_DEMO1,
                        HEADING_CORRECTION_MAX);
    
    printf("[INIT]   ✓ Heading controller initialized\n");
    printf("[INIT] Hardware initialization complete\n\n");
}


/* Main Test */

/**
 * @brief Main entry point - runs heading keeping test
 * @return Exit code (0 = success)
 */
int 
main(void)
{
    /* Initialize USB serial */
    stdio_init_all();
    sleep_ms(2000);
    
    /* Print test header */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                                                               ║\n");
    printf("║                    HEADING KEEPING TEST                       ║\n");
    printf("║                                                               ║\n");
    printf("║  Tests the robot's ability to maintain a target heading      ║\n");
    printf("║  while driving using IMU feedback and motor corrections.     ║\n");
    printf("║                                                               ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    
    /* Initialize hardware */
    printf("\n");
    initialize_hardware();
    
    printf("Test Configuration:\n");
    printf("  Duration:        %d seconds\n", TEST_DURATION_SEC);
    printf("  Tolerance:       ±%.1f°\n", HEADING_TOLERANCE_DEG);
    printf("  Sample Period:   %dms\n", SAMPLE_PERIOD_MS);
    printf("  Base Speed L/R:  %d/%d\n", BASE_SPEED_LEFT, BASE_SPEED_RIGHT);
    printf("\n");
    
    /* Calibrate IMU */
    printf("Calibrating IMU (keep robot still)...\n");
    sleep_ms(1000);
    imu_calibrate(&g_imu);
    printf("  ✓ IMU calibrated\n\n");
    
    /* Get initial heading as target */
    imu_update(&g_imu);
    float target_heading = imu_get_heading(&g_imu);
    
    printf("Target heading: %+.1f°\n", target_heading);
    printf("Starting test in 2 seconds...\n\n");
    sleep_ms(2000);
    
    /* Initialize statistics */
    HeadingStatistics stats = {
        .minimum_heading       = 999.0f,
        .maximum_heading       = -999.0f,
        .average_error         = 0.0f,
        .samples_in_tolerance  = 0,
        .samples_out_tolerance = 0,
        .total_samples         = 0
    };
    
    float error_sum = 0.0f;
    
    /* Print table header */
    printf("┌──────┬─────────┬───────┬────────┐\n");
    printf("│ Time │ Heading │ Error │ Status │\n");
    printf("├──────┼─────────┼───────┼────────┤\n");
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_sample = start_time;
    uint32_t last_print = start_time;
    
    /* Run test for specified duration */
    while ((to_ms_since_boot(get_absolute_time()) - start_time) < 
           (TEST_DURATION_SEC * 1000))
    {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        /* Sample every 100ms */
        if (current_time - last_sample >= SAMPLE_PERIOD_MS)
        {
            imu_update(&g_imu);
            float current_heading = imu_get_heading(&g_imu);
            float error = normalize_angle(target_heading - current_heading);
            
            /* Update statistics */
            if (current_heading < stats.minimum_heading)
            {
                stats.minimum_heading = current_heading;
            }
            
            if (current_heading > stats.maximum_heading)
            {
                stats.maximum_heading = current_heading;
            }
            
            error_sum += fabsf(error);
            stats.total_samples++;
            
            if (fabsf(error) <= HEADING_TOLERANCE_DEG)
            {
                stats.samples_in_tolerance++;
            }
            else
            {
                stats.samples_out_tolerance++;
            }
            
            /* Calculate motor powers with correction */
            int left_power  = BASE_SPEED_LEFT;
            int right_power = BASE_SPEED_RIGHT;
            
            if (fabsf(error) > DEADBAND_DEGREES)
            {
                int correction = (int)(error * CORRECTION_PER_DEGREE * 0.75f);
                left_power  += correction;
                right_power -= correction;
                
                /* Clamp to valid range */
                if (left_power  > 100) left_power  = 100;
                if (left_power  <   0) left_power  =   0;
                if (right_power > 100) right_power = 100;
                if (right_power <   0) right_power =   0;
            }
            
            /* Drive motors */
            motor_drive(M1A, M1B, -left_power);
            motor_drive(M2A, M2B, -right_power);
            
            last_sample = current_time;
        }
        
        /* Print status every 500ms */
        if (current_time - last_print >= PRINT_PERIOD_MS)
        {
            imu_update(&g_imu);
            float current_heading = imu_get_heading(&g_imu);
            float error = normalize_angle(target_heading - current_heading);
            float elapsed_sec = (current_time - start_time) / 1000.0f;
            
            bool in_tolerance = (fabsf(error) <= HEADING_TOLERANCE_DEG);
            
            printf("│ %.1fs │ %+6.1f  │ %+5.1f │ %s%-6s%s │\n",
                   elapsed_sec, current_heading, error,
                   in_tolerance ? COLOR_GREEN : COLOR_YELLOW,
                   in_tolerance ? "OK" : "OUT",
                   COLOR_RESET);
            
            last_print = current_time;
        }
        
        sleep_ms(10);
    }
    
    printf("└──────┴─────────┴───────┴────────┘\n");
    
    /* Stop motors */
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    /* Calculate final statistics */
    stats.average_error = (stats.total_samples > 0) ? 
                         (error_sum / stats.total_samples) : 0.0f;
    
    float stability_percent = (stats.total_samples > 0) ?
        (stats.samples_in_tolerance * 100.0f / stats.total_samples) : 0.0f;
    
    float heading_range = stats.maximum_heading - stats.minimum_heading;
    
    /* Print test results */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                        TEST RESULTS                           ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    printf("Test ID:           HEADING-T1\n");
    printf("Description:       Heading Keeping Mode\n");
    printf("Duration:          %d seconds\n\n", TEST_DURATION_SEC);
    
    printf("Statistics:\n");
    printf("  Total samples:     %d\n", stats.total_samples);
    printf("  In tolerance:      %d (%.1f%%)\n", 
           stats.samples_in_tolerance, stability_percent);
    printf("  Out of tolerance:  %d\n", stats.samples_out_tolerance);
    printf("  Average error:     %.2f°\n", stats.average_error);
    printf("  Heading range:     %.2f° (%.1f° to %.1f°)\n\n",
           heading_range, stats.minimum_heading, stats.maximum_heading);
    
    /* Determine pass/fail */
    bool passed = (stability_percent >= MIN_STABILITY_PERCENT);
    
    printf("Expected Results:\n");
    printf("  Stability:         %.1f%% (required ≥ %.1f%%)\n",
           stability_percent, MIN_STABILITY_PERCENT);
    printf("  Status:            %s%s%s\n\n",
           passed ? COLOR_GREEN : COLOR_RED,
           passed ? "✓ PASSED" : "✗ FAILED",
           COLOR_RESET);
    
    if (passed)
    {
        printf("%s╔════════════════════════════════════════════════════════════╗%s\n", 
               COLOR_GREEN, COLOR_RESET);
        printf("%s║              TEST PASSED - HEADING KEEPING OK              ║%s\n", 
               COLOR_GREEN, COLOR_RESET);
        printf("%s╚════════════════════════════════════════════════════════════╝%s\n", 
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s╔════════════════════════════════════════════════════════════╗%s\n", 
               COLOR_RED, COLOR_RESET);
        printf("%s║     TEST FAILED - STABILITY BELOW REQUIRED %.0f%%            ║%s\n", 
               COLOR_RED, MIN_STABILITY_PERCENT, COLOR_RESET);
        printf("%s╚════════════════════════════════════════════════════════════╝%s\n", 
               COLOR_RED, COLOR_RESET);
    }
    
    printf("\n");
    
    return passed ? 0 : 1;
}