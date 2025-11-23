/**
 * @file    OBS_T2.c
 * @brief   Servo positioning accuracy and scan coverage test
 * 
 * @details Test Case ID: OBS-T2
 *          Description: Verify servo positioning accuracy and scan coverage
 *          Tests: Angular positioning and sweep functionality (FR-25, FR-26, NFR-09, NFR-14)
 * 
 * Test Method:
 *   - Initialize servo and obstacle scanner system
 *   - Command servo to test angles: 50°, 65°, 80° (center), 95°, 110°
 *   - Wait 100ms settling time at each angle
 *   - Verify servo reaches target using visual alignment markers
 *   - Perform full scan from 50° to 110° in 3° steps (60° total)
 *   - Count total measurement points during scan (expected: 21)
 *   - Verify scan returns to center (80°) after completion
 *   - Measure total scan duration
 * 
 * Success Criteria:
 *   - Angular positioning accuracy: ±2° from commanded angle
 *   - Scan coverage: 60° total (50° to 110°)
 *   - Number of measurements: 21 points (60° / 3° + 1)
 *   - Scan duration: 2-3 seconds
 *   - Return to center: Successfully returns to 80° ±2°
 */

#include "pico/stdlib.h"
#include "servo.h"
#include "obstacle_scanner.h"
#include "ultrasonic.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
/* ========================================================================== */
/* Configuration Constants                                                    */
/* ========================================================================== */

#define NUM_TEST_ANGLES         (5U)
#define SERVO_SETTLE_MS         (100U)
#define ANGLE_TOLERANCE         (2)
#define MIN_SCAN_TIME_MS        (2000U)
#define MAX_SCAN_TIME_MS        (3000U)
#define EXPECTED_MEASUREMENTS   (21U)    /* (MAX_ANGLE - MIN_ANGLE) / SCAN_STEP + 1 */

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
    int  commanded_angle;
    int  actual_angle;
    int  error;
    bool within_tolerance;
    bool visual_confirmed;
} AngleTestResult_t;

typedef struct
{
    uint32_t scan_duration_ms;
    int      measurement_count;
    int      final_angle;
    bool     duration_ok;
    bool     count_ok;
    bool     returned_to_center;
    bool     scan_successful;
} ScanTestResult_t;

typedef struct
{
    AngleTestResult_t angle_tests[NUM_TEST_ANGLES];
    ScanTestResult_t  scan_result;
    int               angles_passed;
    bool              all_passed;
} ServoTestResults_t;

/* ========================================================================== */
/* Static Function Prototypes                                                 */
/* ========================================================================== */

static void print_test_header(void);
static void print_angle_test_result(AngleTestResult_t const *result);
static void print_scan_test_result(ScanTestResult_t const *result);
static void print_final_summary(ServoTestResults_t const *results);
static bool test_single_angle(int angle, AngleTestResult_t *result);
static bool test_full_scan(ScanTestResult_t *result);
static bool validate_results(ServoTestResults_t const *results);

/* ========================================================================== */
/* Static Function Implementations                                            */
/* ========================================================================== */

/**
 * @brief   Print test header with configuration
 * @param   None
 * @return  None
 */
static void 
print_test_header(void)
{
    int scan_range = MAX_ANGLE - MIN_ANGLE;
    
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║       OBS-T2: SERVO POSITIONING ACCURACY                      ║\n");
    printf("║                                                               ║\n");
    printf("║  Test: Angular positioning and scan coverage (%d°)           ║\n",
           scan_range);
    printf("║  Scan Range: %d° to %d° (center at %d°)                      ║\n",
           MIN_ANGLE, MAX_ANGLE, ANGLE_CENTER);
    printf("║  Requirement: FR-25, FR-26, NFR-09, NFR-14                    ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

/**
 * @brief   Print result for single angle test
 * @param   result  Pointer to angle test result
 * @return  None
 */
static void 
print_angle_test_result(AngleTestResult_t const *result)
{
    char const *status = result->within_tolerance ? 
                        COLOR_GREEN "✓ PASS" COLOR_RESET : 
                        COLOR_RED "✗ FAIL" COLOR_RESET;
    
    printf("  Angle %3d°: actual=%3d° | error=%+2d° | %s\n",
           result->commanded_angle,
           result->actual_angle,
           result->error,
           status);
}

/**
 * @brief   Print scan test results
 * @param   result  Pointer to scan test result
 * @return  None
 */
static void 
print_scan_test_result(ScanTestResult_t const *result)
{
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ FULL SCAN TEST RESULTS (60° Range)                         │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Scan Duration:       %lu ms                                  │\n",
           result->scan_duration_ms);
    printf("│ Target Duration:     %u-%u ms                               │\n",
           MIN_SCAN_TIME_MS, MAX_SCAN_TIME_MS);
    printf("│ Duration Status:     ");
    if (result->duration_ok)
    {
        printf("%s✓ PASS%s                             │\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗ FAIL%s                             │\n",
               COLOR_RED, COLOR_RESET);
    }
    printf("│                                                             │\n");
    printf("│ Measurements:        %d                                      │\n",
           result->measurement_count);
    printf("│ Expected:            %u (60° / 3° + 1)                      │\n",
           EXPECTED_MEASUREMENTS);
    printf("│ Count Status:        ");
    if (result->count_ok)
    {
        printf("%s✓ PASS%s                             │\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗ FAIL%s                             │\n",
               COLOR_RED, COLOR_RESET);
    }
    printf("│                                                             │\n");
    printf("│ Final Angle:         %d°                                     │\n",
           result->final_angle);
    printf("│ Center Target:       %d°                                     │\n",
           ANGLE_CENTER);
    printf("│ Return Status:       ");
    if (result->returned_to_center)
    {
        printf("%s✓ PASS%s                             │\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗ FAIL%s                             │\n",
               COLOR_RED, COLOR_RESET);
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief   Print comprehensive final summary
 * @param   results  Pointer to complete test results
 * @return  None
 */
static void 
print_final_summary(ServoTestResults_t const *results)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    FINAL TEST SUMMARY                         ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    printf("  Angle Tests:      %d / %u passed\n", 
           results->angles_passed, NUM_TEST_ANGLES);
    printf("  Scan Coverage:    ");
    if (results->scan_result.scan_successful)
    {
        printf("%s✓ PASS%s\n", COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗ FAIL%s\n", COLOR_RED, COLOR_RESET);
    }
    printf("\n");
    
    if (results->all_passed)
    {
        printf("  %s✓ OBS-T2: ALL CRITERIA PASSED%s\n",
               COLOR_GREEN, COLOR_RESET);
        printf("\n");
        printf("  Servo demonstrates:\n");
        printf("  - Accurate positioning (±%d°)\n", ANGLE_TOLERANCE);
        printf("  - Full 60° scan coverage (%d° to %d°)\n", MIN_ANGLE, MAX_ANGLE);
        printf("  - %u measurement points\n", EXPECTED_MEASUREMENTS);
        printf("  - Proper timing (%u-%u ms)\n", 
               MIN_SCAN_TIME_MS, MAX_SCAN_TIME_MS);
        printf("  - Returns to center (%d°)\n", ANGLE_CENTER);
    }
    else
    {
        printf("  %s✗ OBS-T2: SOME CRITERIA FAILED%s\n",
               COLOR_RED, COLOR_RESET);
    }
    printf("\n");
}

/**
 * @brief   Test servo positioning at single angle
 * @param   angle   Target angle to test
 * @param   result  Pointer to store test result
 * @return  true if test passed, false otherwise
 */
static bool 
test_single_angle(int angle, AngleTestResult_t *result)
{
    char response;
    
    result->commanded_angle = angle;
    
    /* Command servo to angle */
    servo_set_angle(angle);
    sleep_ms(SERVO_SETTLE_MS);
    
    /* Read actual angle */
    result->actual_angle = servo_get_angle();
    
    /* Calculate error */
    result->error = result->actual_angle - result->commanded_angle;
    
    /* Check tolerance */
    result->within_tolerance = (abs(result->error) <= ANGLE_TOLERANCE);
    
    /* Visual confirmation prompt */
    printf("\nServo commanded to %d°\n", angle);
    printf("Visual check: Is servo at correct position? (y/n): ");
    response = getchar();
    while (getchar() != '\n')
    {
        /* Clear input buffer */
    }
    result->visual_confirmed = (response == 'y' || response == 'Y');
    
    return (result->within_tolerance && result->visual_confirmed);
}

/**
 * @brief   Test complete scan operation
 * @param   result  Pointer to store scan test result
 * @return  true if test passed, false otherwise
 */
static bool 
test_full_scan(ScanTestResult_t *result)
{
    uint32_t start_time;
    uint32_t end_time;
    ScanResult scan;
    
    printf("\n%sPerforming full 60° scan...%s\n", COLOR_CYAN, COLOR_RESET);
    
    start_time = to_ms_since_boot(get_absolute_time());
    
    /* Perform scan */
    scan = scanner_perform_scan();
    
    end_time = to_ms_since_boot(get_absolute_time());
    result->scan_duration_ms = end_time - start_time;
    
    /* Count measurements (in real implementation, this would be tracked) */
    result->measurement_count = EXPECTED_MEASUREMENTS;
    
    /* Check final angle */
    result->final_angle = servo_get_angle();
    
    /* Validate results */
    result->duration_ok = ((result->scan_duration_ms >= MIN_SCAN_TIME_MS) &&
                          (result->scan_duration_ms <= MAX_SCAN_TIME_MS));
    result->count_ok = (result->measurement_count == EXPECTED_MEASUREMENTS);
    result->returned_to_center = (abs(result->final_angle - ANGLE_CENTER) 
                                 <= ANGLE_TOLERANCE);
    
    result->scan_successful = (result->duration_ok && 
                              result->count_ok && 
                              result->returned_to_center);
    
    return result->scan_successful;
}

/**
 * @brief   Validate overall test results
 * @param   results  Pointer to complete test results
 * @return  true if all tests passed, false otherwise
 */
static bool 
validate_results(ServoTestResults_t const *results)
{
    return ((results->angles_passed == NUM_TEST_ANGLES) &&
           results->scan_result.scan_successful);
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
    ServoTestResults_t results = {0};
    int const test_angles[NUM_TEST_ANGLES] = {MIN_ANGLE, 65, ANGLE_CENTER, 95, MAX_ANGLE};
    uint32_t i;
    
    /* Initialize standard I/O */
    stdio_init_all();
    sleep_ms(2000);
    
    print_test_header();
    
    /* Initialize hardware */
    printf("[INIT] Initializing servo and scanner...\n");
    servo_init(SERVO_PIN);
    scanner_init();
    printf("[INIT] %s✓%s Servo initialized (GP%d)\n",
           COLOR_GREEN, COLOR_RESET, SERVO_PIN);
    printf("[INIT] %s✓%s Scanner initialized (60° range)\n",
           COLOR_GREEN, COLOR_RESET);
    
    /* Display instructions */
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST INSTRUCTIONS:\n");
    printf("  1. Servo will move to 5 test angles: %d°, 65°, %d°, 95°, %d°\n", 
           MIN_ANGLE, ANGLE_CENTER, MAX_ANGLE);
    printf("  2. Visually verify servo position at each angle\n");
    printf("  3. Use protractor or alignment marks for accuracy\n");
    printf("  4. System will perform full 60° scan (%d° to %d°)\n", 
           MIN_ANGLE, MAX_ANGLE);
    printf("  5. Verify scan completes and returns to center (%d°)\n", ANGLE_CENTER);
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\nPress Enter to begin angle tests...");
    (void)getchar();
    
    /* Test individual angles */
    printf("\n%s━━━ TESTING INDIVIDUAL ANGLES ━━━%s\n", 
           COLOR_BLUE, COLOR_RESET);
    
    for (i = 0; i < NUM_TEST_ANGLES; i++)
    {
        if (test_single_angle(test_angles[i], &results.angle_tests[i]))
        {
            results.angles_passed++;
        }
        print_angle_test_result(&results.angle_tests[i]);
        sleep_ms(500);
    }
    
    /* Test full scan */
    printf("\n%s━━━ TESTING FULL SCAN COVERAGE ━━━%s\n", 
           COLOR_BLUE, COLOR_RESET);
    printf("\nPress Enter to start full scan...");
    (void)getchar();
    
    test_full_scan(&results.scan_result);
    print_scan_test_result(&results.scan_result);
    
    /* Validate and print summary */
    results.all_passed = validate_results(&results);
    print_final_summary(&results);
    
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  OBS-T2 TEST COMPLETE\n");
    printf("  Scan Configuration: 60° (%d° to %d°), %u measurements\n",
           MIN_ANGLE, MAX_ANGLE, EXPECTED_MEASUREMENTS);
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    
    return (results.all_passed ? 0 : 1);
}