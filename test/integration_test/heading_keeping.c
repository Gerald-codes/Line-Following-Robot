/**
 * @file    integration_test_heading_keeping.c
 * @brief   Integration test for IMU-Motor heading keeping mode
 * @details Tests the integrated system's ability to maintain a target heading
 *          while driving. Validates IMU readings, motor corrections, and
 *          closed-loop heading control performance.
 *
 * @note    Barr C Coding Standard compliant
 * @author  Your Name
 * @date    November 23, 2025
 *
 * Test Scenarios:
 *   - INT-T1: IMU initialization and calibration
 *   - INT-T2: Heading error calculation
 *   - INT-T3: Motor correction response
 *   - INT-T4: Heading stability over time
 *   - INT-T5: Recovery from disturbances
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

/*******************************************************************************
 * Configuration Constants
 ******************************************************************************/

/* Test parameters */
#define TEST_DURATION_SEC       10      /* Duration for stability tests */
#define HEADING_TOLERANCE_DEG   3.0f    /* Acceptable heading deviation */
#define SAMPLE_PERIOD_MS        100     /* IMU sampling period */
#define PRINT_PERIOD_MS         500     /* Status print interval */

/* Motor control parameters */
#define BASE_SPEED_LEFT         42      /* Left motor base speed (0-100) */
#define BASE_SPEED_RIGHT        40      /* Right motor base speed (0-100) */
#define DEADBAND_DEGREES        3.0f    /* Heading error deadband */
#define CORRECTION_PER_DEGREE   1.0f    /* Correction gain */

/* Test thresholds */
#define MIN_STABILITY_PERCENT   85.0f   /* Minimum time in tolerance (%) */
#define MAX_OVERSHOOT_DEG       10.0f   /* Maximum heading overshoot */
#define MAX_STEADY_STATE_ERR    2.0f    /* Maximum steady-state error */

/* ANSI color codes */
#define COLOR_GREEN             "\033[32m"
#define COLOR_RED               "\033[31m"
#define COLOR_YELLOW            "\033[33m"
#define COLOR_CYAN              "\033[36m"
#define COLOR_BLUE              "\033[34m"
#define COLOR_RESET             "\033[0m"

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/**
 * @brief Integration test result structure
 */
typedef struct
{
    char const *test_id;           /* Test identifier (e.g., "INT-T1") */
    char const *test_name;         /* Descriptive test name */
    bool        passed;            /* Test result */
    float       measured_value;    /* Primary measured value */
    float       expected_value;    /* Expected value */
    float       tolerance;         /* Acceptable tolerance */
    char        notes[128];        /* Additional notes or failure reasons */
} IntegrationTestResult;

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

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

static IMU              g_imu;
static HeadingController g_heading_ctrl;
static PIDController    g_left_motor_pid;
static PIDController    g_right_motor_pid;

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

/**
 * @brief Normalize angle to [-180, +180] range
 * @param angle_deg Angle in degrees
 * @return Normalized angle in degrees
 */
static float 
normalize_angle(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    
    return angle_deg;
}

/**
 * @brief Print test header
 * @param test_id   Test identifier
 * @param test_name Test description
 */
static void 
print_test_header(char const *test_id, char const *test_name)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║ %s%-60s%s║\n", COLOR_CYAN, test_id, COLOR_RESET);
    printf("║ %s\n", test_name);
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
}

/**
 * @brief Print test result
 * @param result Pointer to test result structure
 */
static void 
print_test_result(IntegrationTestResult const *result)
{
    printf("\n");
    printf("┌────────────────────────────────────────────────────────────┐\n");
    printf("│                      TEST RESULT                           │\n");
    printf("├────────────────────────────────────────────────────────────┤\n");
    printf("│ Test ID:        %-42s │\n", result->test_id);
    printf("│ Status:         %s%-42s%s │\n",
           result->passed ? COLOR_GREEN : COLOR_RED,
           result->passed ? "✓ PASSED" : "✗ FAILED",
           COLOR_RESET);
    printf("├────────────────────────────────────────────────────────────┤\n");
    printf("│ Measured:       %-42.2f │\n", result->measured_value);
    printf("│ Expected:       %-42.2f │\n", result->expected_value);
    printf("│ Tolerance:      ±%-41.2f │\n", result->tolerance);
    
    if (!result->passed && result->notes[0] != '\0')
    {
        printf("├────────────────────────────────────────────────────────────┤\n");
        printf("│ Notes:          %-42s │\n", result->notes);
    }
    
    printf("└────────────────────────────────────────────────────────────┘\n");
}

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

/*******************************************************************************
 * Integration Tests
 ******************************************************************************/

/**
 * @brief INT-T1: IMU initialization and calibration test
 * @details Verifies IMU can be initialized and calibrated successfully
 * @return Test result structure
 */
static IntegrationTestResult 
test_imu_initialization_and_calibration(void)
{
    IntegrationTestResult result = {
        .test_id        = "INT-T1",
        .test_name      = "IMU Initialization and Calibration",
        .passed         = false,
        .measured_value = 0.0f,
        .expected_value = 0.0f,
        .tolerance      = 5.0f,
        .notes          = ""
    };
    
    print_test_header(result.test_id, result.test_name);
    
    printf("Step 1: Calibrating IMU (keep robot still)...\n");
    sleep_ms(1000);
    
    imu_calibrate(&g_imu);
    printf("  ✓ IMU calibrated\n\n");
    
    printf("Step 2: Reading heading (10 samples)...\n");
    
    float heading_sum = 0.0f;
    int   valid_samples = 0;
    
    for (int i = 0; i < 10; i++)
    {
        imu_update(&g_imu);
        float heading = imu_get_heading(&g_imu);
        
        printf("  Sample %2d: %+7.2f°\n", i + 1, heading);
        
        /* Check if heading is reasonable (not NaN or infinity) */
        if (!isnan(heading) && !isinf(heading))
        {
            heading_sum += heading;
            valid_samples++;
        }
        
        sleep_ms(100);
    }
    
    /* Calculate average heading */
    float average_heading = 0.0f;
    
    if (valid_samples > 0)
    {
        average_heading = heading_sum / valid_samples;
    }
    
    result.measured_value = average_heading;
    result.expected_value = 0.0f;  /* After calibration, should be near 0 */
    
    /* Test passes if average is within tolerance and we got valid samples */
    result.passed = (valid_samples >= 8) && 
                   (fabsf(average_heading) < result.tolerance);
    
    if (!result.passed)
    {
        if (valid_samples < 8)
        {
            snprintf(result.notes, sizeof(result.notes),
                    "Only %d/10 valid samples", valid_samples);
        }
        else
        {
            snprintf(result.notes, sizeof(result.notes),
                    "Heading drift too large");
        }
    }
    
    print_test_result(&result);
    
    /* Stop motors before continuing */
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    return result;
}

/**
 * @brief INT-T2: Heading error calculation test
 * @details Verifies heading error is calculated correctly for various targets
 * @return Test result structure
 */
static IntegrationTestResult 
test_heading_error_calculation(void)
{
    IntegrationTestResult result = {
        .test_id        = "INT-T2",
        .test_name      = "Heading Error Calculation",
        .passed         = true,
        .measured_value = 0.0f,
        .expected_value = 0.0f,
        .tolerance      = 0.1f,
        .notes          = ""
    };
    
    print_test_header(result.test_id, result.test_name);
    
    printf("Testing angle normalization and error calculation...\n\n");
    
    /* Test cases: (current, target, expected_error) */
    struct {
        float current_heading;
        float target_heading;
        float expected_error;
    } test_cases[] = {
        {0.0f, 0.0f, 0.0f},           /* No error */
        {10.0f, 20.0f, 10.0f},        /* Simple positive error */
        {20.0f, 10.0f, -10.0f},       /* Simple negative error */
        {350.0f, 10.0f, 20.0f},       /* Wrap-around positive */
        {10.0f, 350.0f, -20.0f},      /* Wrap-around negative */
        {180.0f, -180.0f, 0.0f},      /* Boundary case */
    };
    
    int const num_cases = sizeof(test_cases) / sizeof(test_cases[0]);
    int cases_passed = 0;
    
    printf("┌──────────┬──────────┬──────────┬─────────┐\n");
    printf("│ Current  │  Target  │ Expected │  Status │\n");
    printf("├──────────┼──────────┼──────────┼─────────┤\n");
    
    for (int i = 0; i < num_cases; i++)
    {
        float error = normalize_angle(test_cases[i].target_heading - 
                                     test_cases[i].current_heading);
        
        float error_diff = fabsf(error - test_cases[i].expected_error);
        bool  case_passed = (error_diff < result.tolerance);
        
        if (case_passed)
        {
            cases_passed++;
        }
        
        printf("│ %+7.1f  │ %+7.1f  │ %+7.1f  │ %s%-7s%s │\n",
               test_cases[i].current_heading,
               test_cases[i].target_heading,
               error,
               case_passed ? COLOR_GREEN : COLOR_RED,
               case_passed ? "✓ PASS" : "✗ FAIL",
               COLOR_RESET);
    }
    
    printf("└──────────┴──────────┴──────────┴─────────┘\n");
    
    result.measured_value = (float)cases_passed;
    result.expected_value = (float)num_cases;
    result.passed = (cases_passed == num_cases);
    
    if (!result.passed)
    {
        snprintf(result.notes, sizeof(result.notes),
                "%d/%d test cases failed", num_cases - cases_passed, num_cases);
    }
    
    print_test_result(&result);
    
    return result;
}

/**
 * @brief INT-T3: Motor correction response test
 * @details Verifies motors respond correctly to heading errors
 * @return Test result structure
 */
static IntegrationTestResult 
test_motor_correction_response(void)
{
    IntegrationTestResult result = {
        .test_id        = "INT-T3",
        .test_name      = "Motor Correction Response",
        .passed         = false,
        .measured_value = 0.0f,
        .expected_value = 0.0f,
        .tolerance      = 5.0f,
        .notes          = ""
    };
    
    print_test_header(result.test_id, result.test_name);
    
    printf("Testing motor power adjustments based on heading errors...\n\n");
    
    /* Calibrate IMU */
    printf("Calibrating IMU...\n");
    imu_calibrate(&g_imu);
    sleep_ms(1000);
    
    imu_update(&g_imu);
    float initial_heading = imu_get_heading(&g_imu);
    
    printf("Initial heading: %+.1f°\n", initial_heading);
    printf("Target heading:  %+.1f° (+10° offset for test)\n\n", 
           initial_heading + 10.0f);
    
    float target_heading = initial_heading + 10.0f;
    
    printf("Running motors with correction for 3 seconds...\n");
    printf("┌──────┬─────────┬───────┬────────┬────────┐\n");
    printf("│ Time │ Heading │ Error │ L Pwr  │ R Pwr  │\n");
    printf("├──────┼─────────┼───────┼────────┼────────┤\n");
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_print = start_time;
    
    float error_sum = 0.0f;
    int   samples = 0;
    
    /* Run for 3 seconds */
    while ((to_ms_since_boot(get_absolute_time()) - start_time) < 3000)
    {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        /* Read heading */
        imu_update(&g_imu);
        float current_heading = imu_get_heading(&g_imu);
        
        /* Calculate error */
        float error = normalize_angle(target_heading - current_heading);
        
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
        
        /* Accumulate statistics */
        error_sum += fabsf(error);
        samples++;
        
        /* Print status every 500ms */
        if (current_time - last_print >= 500)
        {
            float elapsed_sec = (current_time - start_time) / 1000.0f;
            
            printf("│ %.1fs │ %+6.1f  │ %+5.1f │  %3d   │  %3d   │\n",
                   elapsed_sec, current_heading, error, left_power, right_power);
            
            last_print = current_time;
        }
        
        sleep_ms(10);
    }
    
    printf("└──────┴─────────┴───────┴────────┴────────┘\n");
    
    /* Stop motors */
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    /* Calculate average error */
    float average_error = (samples > 0) ? (error_sum / samples) : 999.0f;
    
    result.measured_value = average_error;
    result.expected_value = 5.0f;  /* Target average error < 5° */
    result.passed = (average_error < result.expected_value);
    
    if (!result.passed)
    {
        snprintf(result.notes, sizeof(result.notes),
                "Average error too high: %.1f°", average_error);
    }
    
    printf("\nAverage heading error: %.2f°\n", average_error);
    
    print_test_result(&result);
    
    return result;
}

/**
 * @brief INT-T4: Heading stability over time test
 * @details Runs system for extended period and measures stability
 * @return Test result structure
 */
static IntegrationTestResult 
test_heading_stability_over_time(void)
{
    IntegrationTestResult result = {
        .test_id        = "INT-T4",
        .test_name      = "Heading Stability Over Time",
        .passed         = false,
        .measured_value = 0.0f,
        .expected_value = MIN_STABILITY_PERCENT,
        .tolerance      = 0.0f,
        .notes          = ""
    };
    
    HeadingStatistics stats = {
        .minimum_heading       = 999.0f,
        .maximum_heading       = -999.0f,
        .average_error         = 0.0f,
        .samples_in_tolerance  = 0,
        .samples_out_tolerance = 0,
        .total_samples         = 0
    };
    
    print_test_header(result.test_id, result.test_name);
    
    printf("Testing heading stability for %d seconds...\n\n", TEST_DURATION_SEC);
    
    /* Calibrate IMU */
    printf("Calibrating IMU...\n");
    imu_calibrate(&g_imu);
    sleep_ms(1000);
    
    imu_update(&g_imu);
    float target_heading = imu_get_heading(&g_imu);
    
    printf("Target heading: %+.1f°\n", target_heading);
    printf("Tolerance: ±%.1f°\n\n", HEADING_TOLERANCE_DEG);
    
    printf("┌──────┬─────────┬───────┬────────┐\n");
    printf("│ Time │ Heading │ Error │ Status │\n");
    printf("├──────┼─────────┼───────┼────────┤\n");
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_sample = start_time;
    uint32_t last_print = start_time;
    
    float error_sum = 0.0f;
    
    /* Run for specified duration */
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
            
            /* Calculate motor powers */
            int left_power  = BASE_SPEED_LEFT;
            int right_power = BASE_SPEED_RIGHT;
            
            if (fabsf(error) > DEADBAND_DEGREES)
            {
                int correction = (int)(error * CORRECTION_PER_DEGREE * 0.75f);
                left_power  += correction;
                right_power -= correction;
                
                if (left_power  > 100) left_power  = 100;
                if (left_power  <   0) left_power  =   0;
                if (right_power > 100) right_power = 100;
                if (right_power <   0) right_power =   0;
            }
            
            motor_drive(M1A, M1B, -left_power);
            motor_drive(M2A, M2B, -right_power);
            
            last_sample = current_time;
        }
        
        /* Print every 500ms */
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
    
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    /* Calculate final statistics */
    stats.average_error = (stats.total_samples > 0) ? 
                         (error_sum / stats.total_samples) : 999.0f;
    
    float stability_percent = (stats.total_samples > 0) ?
        (stats.samples_in_tolerance * 100.0f / stats.total_samples) : 0.0f;
    
    float heading_range = stats.maximum_heading - stats.minimum_heading;
    
    printf("\nStatistics:\n");
    printf("  Total samples:     %d\n", stats.total_samples);
    printf("  In tolerance:      %d (%.1f%%)\n", 
           stats.samples_in_tolerance, stability_percent);
    printf("  Out of tolerance:  %d\n", stats.samples_out_tolerance);
    printf("  Average error:     %.2f°\n", stats.average_error);
    printf("  Heading range:     %.2f° (%.1f to %.1f)\n",
           heading_range, stats.minimum_heading, stats.maximum_heading);
    
    result.measured_value = stability_percent;
    result.expected_value = MIN_STABILITY_PERCENT;
    result.passed = (stability_percent >= MIN_STABILITY_PERCENT);
    
    if (!result.passed)
    {
        snprintf(result.notes, sizeof(result.notes),
                "Stability %.1f%% < required %.1f%%",
                stability_percent, MIN_STABILITY_PERCENT);
    }
    
    print_test_result(&result);
    
    return result;
}

/**
 * @brief INT-T5: Recovery from disturbances test
 * @details Simulates external disturbance and measures recovery time
 * @return Test result structure
 */
static IntegrationTestResult 
test_recovery_from_disturbances(void)
{
    IntegrationTestResult result = {
        .test_id        = "INT-T5",
        .test_name      = "Recovery from Disturbances",
        .passed         = false,
        .measured_value = 0.0f,
        .expected_value = 2.0f,  /* Should recover within 2 seconds */
        .tolerance      = 1.0f,
        .notes          = ""
    };
    
    print_test_header(result.test_id, result.test_name);
    
    printf("Simulating external disturbances...\n\n");
    
    /* Calibrate */
    printf("Calibrating IMU...\n");
    imu_calibrate(&g_imu);
    sleep_ms(1000);
    
    imu_update(&g_imu);
    float target_heading = imu_get_heading(&g_imu);
    
    printf("Target heading: %+.1f°\n", target_heading);
    printf("Will simulate disturbance by varying motor speeds\n\n");
    
    /* Test 3 disturbances */
    int const num_disturbances = 3;
    float total_recovery_time = 0.0f;
    int successful_recoveries = 0;
    
    for (int disturbance = 0; disturbance < num_disturbances; disturbance++)
    {
        printf("Disturbance #%d:\n", disturbance + 1);
        printf("  1. Stabilizing first...\n");
        
        /* Stabilize for 2 seconds */
        uint32_t stabilize_start = to_ms_since_boot(get_absolute_time());
        
        while ((to_ms_since_boot(get_absolute_time()) - stabilize_start) < 2000)
        {
            imu_update(&g_imu);
            float current_heading = imu_get_heading(&g_imu);
            float error = normalize_angle(target_heading - current_heading);
            
            int left_power  = BASE_SPEED_LEFT;
            int right_power = BASE_SPEED_RIGHT;
            
            if (fabsf(error) > DEADBAND_DEGREES)
            {
                int correction = (int)(error * CORRECTION_PER_DEGREE * 0.75f);
                left_power  += correction;
                right_power -= correction;
                
                if (left_power  > 100) left_power  = 100;
                if (left_power  <   0) left_power  =   0;
                if (right_power > 100) right_power = 100;
                if (right_power <   0) right_power =   0;
            }
            
            motor_drive(M1A, M1B, -left_power);
            motor_drive(M2A, M2B, -right_power);
            
            sleep_ms(10);
        }
        
        printf("  2. Applying disturbance (unbalanced motors for 500ms)...\n");
        
        /* Apply disturbance: drive one motor faster */
        uint32_t disturbance_start = to_ms_since_boot(get_absolute_time());
        
        while ((to_ms_since_boot(get_absolute_time()) - disturbance_start) < 500)
        {
            motor_drive(M1A, M1B, -70);  /* Left faster */
            motor_drive(M2A, M2B, -30);  /* Right slower */
            sleep_ms(10);
        }
        
        printf("  3. Measuring recovery time...\n");
        
        /* Measure recovery time */
        uint32_t recovery_start = to_ms_since_boot(get_absolute_time());
        bool recovered = false;
        int stable_samples = 0;
        
        while ((to_ms_since_boot(get_absolute_time()) - recovery_start) < 5000)
        {
            uint32_t current_time = to_ms_since_boot(get_absolute_time());
            
            imu_update(&g_imu);
            float current_heading = imu_get_heading(&g_imu);
            float error = normalize_angle(target_heading - current_heading);
            
            /* Normal control */
            int left_power  = BASE_SPEED_LEFT;
            int right_power = BASE_SPEED_RIGHT;
            
            if (fabsf(error) > DEADBAND_DEGREES)
            {
                int correction = (int)(error * CORRECTION_PER_DEGREE * 0.75f);
                left_power  += correction;
                right_power -= correction;
                
                if (left_power  > 100) left_power  = 100;
                if (left_power  <   0) left_power  =   0;
                if (right_power > 100) right_power = 100;
                if (right_power <   0) right_power =   0;
            }
            
            motor_drive(M1A, M1B, -left_power);
            motor_drive(M2A, M2B, -right_power);
            
            /* Check if stable (within tolerance for 500ms) */
            if (fabsf(error) <= HEADING_TOLERANCE_DEG)
            {
                stable_samples++;
                
                if (stable_samples >= 50)  /* 50 samples * 10ms = 500ms */
                {
                    float recovery_time_sec = 
                        (current_time - recovery_start) / 1000.0f;
                    
                    printf("  ✓ Recovered in %.2f seconds\n\n", recovery_time_sec);
                    
                    total_recovery_time += recovery_time_sec;
                    successful_recoveries++;
                    recovered = true;
                    break;
                }
            }
            else
            {
                stable_samples = 0;
            }
            
            sleep_ms(10);
        }
        
        if (!recovered)
        {
            printf("  ✗ Failed to recover within 5 seconds\n\n");
        }
        
        sleep_ms(500);
    }
    
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    if (successful_recoveries > 0)
    {
        float average_recovery = total_recovery_time / successful_recoveries;
        
        result.measured_value = average_recovery;
        result.passed = (average_recovery <= result.expected_value + result.tolerance);
        
        printf("Summary:\n");
        printf("  Successful recoveries: %d/%d\n", 
               successful_recoveries, num_disturbances);
        printf("  Average recovery time: %.2f seconds\n", average_recovery);
    }
    else
    {
        result.measured_value = 999.0f;
        result.passed = false;
        snprintf(result.notes, sizeof(result.notes),
                "No successful recoveries");
    }
    
    print_test_result(&result);
    
    return result;
}

/*******************************************************************************
 * Main Test Runner
 ******************************************************************************/

/**
 * @brief Main entry point - runs complete integration test suite
 * @return Exit code (0 = success)
 */
int 
main(void)
{
    IntegrationTestResult results[5];
    int tests_passed = 0;
    int tests_failed = 0;
    
    /* Initialize USB serial */
    stdio_init_all();
    sleep_ms(2000);
    
    /* Print test suite header */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                                                               ║\n");
    printf("║       INTEGRATION TEST: IMU-MOTOR HEADING KEEPING MODE       ║\n");
    printf("║                                                               ║\n");
    printf("║  Tests the integrated system's ability to maintain heading  ║\n");
    printf("║  using closed-loop control with IMU feedback.                ║\n");
    printf("║                                                               ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    
    /* Initialize hardware */
    printf("\n");
    initialize_hardware();
    
    printf("Starting integration tests...\n");
    printf("Press CTRL+C to abort at any time.\n\n");
    
    sleep_ms(2000);
    
    /* Run all tests */
    results[0] = test_imu_initialization_and_calibration();
    sleep_ms(2000);
    
    results[1] = test_heading_error_calculation();
    sleep_ms(2000);
    
    results[2] = test_motor_correction_response();
    sleep_ms(2000);
    
    results[3] = test_heading_stability_over_time();
    sleep_ms(2000);
    
    results[4] = test_recovery_from_disturbances();
    
    /* Count results */
    for (int i = 0; i < 5; i++)
    {
        if (results[i].passed)
        {
            tests_passed++;
        }
        else
        {
            tests_failed++;
        }
    }
    
    /* Print final summary */
    printf("\n\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                   INTEGRATION TEST SUMMARY                   ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    printf("┌──────────────┬────────────────────────────────────────┬────────┐\n");
    printf("│   Test ID    │              Description               │ Result │\n");
    printf("├──────────────┼────────────────────────────────────────┼────────┤\n");
    
    for (int i = 0; i < 5; i++)
    {
        printf("│ %-12s │ %-38s │ %s%-6s%s │\n",
               results[i].test_id,
               results[i].test_name,
               results[i].passed ? COLOR_GREEN : COLOR_RED,
               results[i].passed ? "PASS" : "FAIL",
               COLOR_RESET);
    }
    
    printf("└──────────────┴────────────────────────────────────────┴────────┘\n\n");
    
    printf("Overall Results:\n");
    printf("  Tests Passed:  %s%d/5%s\n", 
           COLOR_GREEN, tests_passed, COLOR_RESET);
    printf("  Tests Failed:  %s%d/5%s\n", 
           tests_failed > 0 ? COLOR_RED : COLOR_GREEN, tests_failed, COLOR_RESET);
    printf("  Success Rate:  %.1f%%\n\n", (tests_passed * 100.0f / 5.0f));
    
    if (tests_failed == 0)
    {
        printf("%s╔════════════════════════════════════════════════════════════╗%s\n", 
               COLOR_GREEN, COLOR_RESET);
        printf("%s║  ALL INTEGRATION TESTS PASSED - SYSTEM READY FOR USE      ║%s\n", 
               COLOR_GREEN, COLOR_RESET);
        printf("%s╚════════════════════════════════════════════════════════════╝%s\n", 
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s╔════════════════════════════════════════════════════════════╗%s\n", 
               COLOR_YELLOW, COLOR_RESET);
        printf("%s║  SOME TESTS FAILED - REVIEW RESULTS AND TUNE PARAMETERS   ║%s\n", 
               COLOR_YELLOW, COLOR_RESET);
        printf("%s╚════════════════════════════════════════════════════════════╝%s\n", 
               COLOR_YELLOW, COLOR_RESET);
    }
    
    printf("\n");
    
    /* Ensure motors are stopped */
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    return (tests_failed == 0) ? 0 : 1;
}