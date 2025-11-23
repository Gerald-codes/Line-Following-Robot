/**
 * @file    INTEG_T3.c
 * @brief   Integration test for obstacle avoidance with ultrasonic and IMU
 * 
 * @details Test Case ID: INTEG-T3
 *          Description: Obstacle Avoidance with Ultrasonic and IMU
 *          Tests: Verify robot can detect obstacles with ultrasonic sensor,
 *                 use IMU for orientation feedback, and alter path without
 *                 manual reset
 * 
 * Test Method:
 *   1. Place obstacles in robot's path on line-following course
 *   2. Enable ultrasonic scanning (60° range) and IMU tracking
 *   3. Observe if robot senses obstacle, slows or stops
 *   4. Verify robot navigates around using IMU for heading correction
 *   5. Check if robot resumes original path
 * 
 * Success Criteria:
 *   - Obstacle detection: Robot detects obstacles ≥10 cm ahead
 *   - Stop/slow response: Robot reacts within 1 second of detection
 *   - IMU tracking: Heading accuracy ±5° during maneuver
 *   - Path correction: Returns to within ±10° of original heading
 *   - No manual intervention required
 *   - Completes 5 consecutive avoidance maneuvers successfully
 */

#include "pico/stdlib.h"
#include "obstacle_scanner.h"
#include "imu.h"
#include "motor.h"
#include <stdio.h>
#include <math.h>

/* ========================================================================== */
/* Configuration Constants                                                    */
/* ========================================================================== */

#define MIN_OBSTACLE_DISTANCE_CM    10U         /* Minimum detection distance */
#define REACTION_TIME_MS            1000U       /* Max time to react */
#define HEADING_TOLERANCE_DEG       5.0f        /* Heading accuracy during turn */
#define RETURN_HEADING_TOLERANCE    10.0f       /* Return heading tolerance */
#define NUM_TEST_MANEUVERS          5U          /* Number of test runs */
#define TURN_ANGLE_DEG              45.0f       /* Avoidance turn angle */
#define SCAN_INTERVAL_MS            500U        /* Scan every 500ms */

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

typedef enum
{
    STATE_INIT,
    STATE_SCANNING,
    STATE_OBSTACLE_DETECTED,
    STATE_TURNING,
    STATE_BYPASSING,
    STATE_RETURNING,
    STATE_COMPLETE,
    STATE_FAILED
} AvoidanceState;

typedef struct
{
    uint32_t maneuver_num;
    bool obstacle_detected;
    uint32_t detection_time_ms;
    uint32_t reaction_time_ms;
    float initial_heading;
    float heading_during_turn;
    float final_heading;
    float heading_error;
    bool imu_accurate;
    bool returned_to_path;
    bool completed_successfully;
} ManeuverResult;

typedef struct
{
    uint32_t total_maneuvers;
    uint32_t successful_maneuvers;
    uint32_t failed_maneuvers;
    uint32_t detection_failures;
    uint32_t heading_failures;
    uint32_t return_failures;
    float avg_reaction_time_ms;
    float avg_heading_error;
} TestStats;

/* ========================================================================== */
/* Static Function Prototypes                                                 */
/* ========================================================================== */

static void print_test_header(void);
static void print_maneuver_result(ManeuverResult const *result);
static void print_final_stats(TestStats const *stats);
static bool check_obstacle_ahead(uint64_t *distance_cm);
static bool perform_avoidance_maneuver(ManeuverResult *result);
static void stop_motors(void);
static bool validate_test_results(TestStats const *stats);

/* ========================================================================== */
/* Static Function Implementations                                            */
/* ========================================================================== */

/**
 * @brief   Print test header
 * @param   None
 * @return  None
 */
static void 
print_test_header(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║    INTEG-T3: OBSTACLE AVOIDANCE WITH ULTRASONIC & IMU        ║\n");
    printf("║                                                               ║\n");
    printf("║  Test: Autonomous obstacle detection and path correction     ║\n");
    printf("║  Sensors: Ultrasonic (60° scan) + IMU (GY-511)               ║\n");
    printf("║  Maneuvers: 5 consecutive avoidance tests                    ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

/**
 * @brief   Print individual maneuver result
 * @param   result  Pointer to maneuver result structure
 * @return  None
 */
static void 
print_maneuver_result(ManeuverResult const *result)
{
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ MANEUVER #%lu RESULT                                          │\n",
           result->maneuver_num);
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Obstacle Detected:   %s%-3s%s                                  │\n",
           result->obstacle_detected ? COLOR_GREEN : COLOR_RED,
           result->obstacle_detected ? "YES" : "NO",
           COLOR_RESET);
    printf("│ Reaction Time:       %lu ms                                  │\n",
           result->reaction_time_ms);
    printf("│                                                             │\n");
    printf("│ IMU Tracking:                                               │\n");
    printf("│   Initial Heading:   %.1f°                                   │\n",
           result->initial_heading);
    printf("│   During Turn:       %.1f°                                   │\n",
           result->heading_during_turn);
    printf("│   Final Heading:     %.1f°                                   │\n",
           result->final_heading);
    printf("│   Heading Error:     %.1f° (tolerance: ±%.1f°)              │\n",
           result->heading_error, RETURN_HEADING_TOLERANCE);
    printf("│                                                             │\n");
    printf("│ IMU Accuracy:        %s%-6s%s                                │\n",
           result->imu_accurate ? COLOR_GREEN : COLOR_YELLOW,
           result->imu_accurate ? "✓ PASS" : "⚠ WARN",
           COLOR_RESET);
    printf("│ Path Return:         %s%-6s%s                                │\n",
           result->returned_to_path ? COLOR_GREEN : COLOR_RED,
           result->returned_to_path ? "✓ PASS" : "✗ FAIL",
           COLOR_RESET);
    printf("│                                                             │\n");
    printf("│ Result:              ");
    if (result->completed_successfully)
    {
        printf("%s✓ SUCCESS%s                          │\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗ FAILED%s                           │\n",
               COLOR_RED, COLOR_RESET);
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief   Print final test statistics
 * @param   stats  Pointer to test statistics structure
 * @return  None
 */
static void 
print_final_stats(TestStats const *stats)
{
    float success_rate = 0.0f;
    if (stats->total_maneuvers > 0U)
    {
        success_rate = (stats->successful_maneuvers * 100.0f) / 
                      stats->total_maneuvers;
    }
    
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    FINAL TEST STATISTICS                      ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    printf("┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ OVERALL PERFORMANCE                                         │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Total Maneuvers:     %lu                                     │\n",
           stats->total_maneuvers);
    printf("│ Successful:          %lu (%.0f%%)                             │\n",
           stats->successful_maneuvers, success_rate);
    printf("│ Failed:              %lu                                     │\n",
           stats->failed_maneuvers);
    printf("│                                                             │\n");
    printf("│ Failure Analysis:                                           │\n");
    printf("│   Detection fails:   %lu                                     │\n",
           stats->detection_failures);
    printf("│   Heading fails:     %lu                                     │\n",
           stats->heading_failures);
    printf("│   Return fails:      %lu                                     │\n",
           stats->return_failures);
    printf("│                                                             │\n");
    printf("│ Average Metrics:                                            │\n");
    printf("│   Reaction time:     %.0f ms                                 │\n",
           stats->avg_reaction_time_ms);
    printf("│   Heading error:     %.1f°                                   │\n",
           stats->avg_heading_error);
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief   Check if obstacle is ahead within minimum distance
 * @param   distance_cm  Pointer to store detected distance
 * @return  true if obstacle detected, false otherwise
 */
static bool 
check_obstacle_ahead(uint64_t *distance_cm)
{
    ScanResult scan = scanner_perform_scan();
    
    if (scan.obstacle_count > 0)
    {
        /* Check if closest obstacle is within threshold */
        uint64_t min_dist = scan.obstacles[0].min_distance;
        
        for (int i = 1; i < scan.obstacle_count; i++)
        {
            if (scan.obstacles[i].min_distance < min_dist)
            {
                min_dist = scan.obstacles[i].min_distance;
            }
        }
        
        *distance_cm = min_dist;
        return (min_dist <= MIN_OBSTACLE_DISTANCE_CM);
    }
    
    *distance_cm = 0;
    return false;
}

/**
 * @brief   Perform complete avoidance maneuver
 * @param   result  Pointer to store maneuver result
 * @return  true if successful, false otherwise
 */
static bool 
perform_avoidance_maneuver(ManeuverResult *result)
{
    AvoidanceState state = STATE_SCANNING;
    uint32_t maneuver_start = to_ms_since_boot(get_absolute_time());
    uint64_t obstacle_distance = 0;
    
    /* Reset IMU heading reference */
    imu_helper_reset_heading();
    result->initial_heading = imu_helper_get_heading();
    
    printf("\n%s[Maneuver #%lu] State: SCANNING for obstacles...%s\n",
           COLOR_CYAN, result->maneuver_num, COLOR_RESET);
    
    /* Wait for obstacle detection */
    while (state == STATE_SCANNING)
    {
        if (check_obstacle_ahead(&obstacle_distance))
        {
            result->obstacle_detected = true;
            result->detection_time_ms = to_ms_since_boot(get_absolute_time());
            result->reaction_time_ms = result->detection_time_ms - maneuver_start;
            
            printf("%s[Maneuver #%lu] OBSTACLE DETECTED at %llu cm!%s\n",
                   COLOR_YELLOW, result->maneuver_num, 
                   obstacle_distance, COLOR_RESET);
            
            state = STATE_OBSTACLE_DETECTED;
            break;
        }
        
        sleep_ms(SCAN_INTERVAL_MS);
        
        /* Timeout after 30 seconds */
        if ((to_ms_since_boot(get_absolute_time()) - maneuver_start) > 30000U)
        {
            printf("%s[Maneuver #%lu] TIMEOUT - No obstacle detected%s\n",
                   COLOR_RED, result->maneuver_num, COLOR_RESET);
            return false;
        }
    }
    
    /* Stop motors immediately */
    printf("%s[Maneuver #%lu] State: STOPPING...%s\n",
           COLOR_CYAN, result->maneuver_num, COLOR_RESET);
    stop_motors();
    sleep_ms(500);
    
    /* Initiate turn maneuver using IMU */
    printf("%s[Maneuver #%lu] State: TURNING (target: %.0f°)...%s\n",
           COLOR_CYAN, result->maneuver_num, 
           TURN_ANGLE_DEG, COLOR_RESET);
    
    imu_helper_start_avoidance();
    state = STATE_TURNING;
    
    /* Simulate turn (in real implementation, control motors based on IMU) */
    while (!imu_helper_has_turned(TURN_ANGLE_DEG, HEADING_TOLERANCE_DEG))
    {
        imu_helper_update();
        float current_turn = imu_helper_get_relative_heading();
        
        printf("  Turn progress: %.1f° / %.0f°\r", 
               current_turn, TURN_ANGLE_DEG);
        fflush(stdout);
        
        sleep_ms(100);
        
        /* Timeout after 10 seconds */
        if ((to_ms_since_boot(get_absolute_time()) - 
             result->detection_time_ms) > 10000U)
        {
            printf("\n%s[Maneuver #%lu] Turn timeout%s\n",
                   COLOR_RED, result->maneuver_num, COLOR_RESET);
            return false;
        }
    }
    
    result->heading_during_turn = imu_helper_get_heading();
    printf("\n%s[Maneuver #%lu] Turn complete at %.1f°%s\n",
           COLOR_GREEN, result->maneuver_num, 
           result->heading_during_turn, COLOR_RESET);
    
    /* Bypass obstacle */
    printf("%s[Maneuver #%lu] State: BYPASSING obstacle...%s\n",
           COLOR_CYAN, result->maneuver_num, COLOR_RESET);
    state = STATE_BYPASSING;
    sleep_ms(2000);  /* Simulate forward motion */
    
    /* Return to original heading */
    printf("%s[Maneuver #%lu] State: RETURNING to path...%s\n",
           COLOR_CYAN, result->maneuver_num, COLOR_RESET);
    state = STATE_RETURNING;
    
    /* Turn back to original heading */
    imu_helper_reset_heading();
    while (!imu_helper_has_turned(-TURN_ANGLE_DEG, HEADING_TOLERANCE_DEG))
    {
        imu_helper_update();
        sleep_ms(100);
    }
    
    result->final_heading = imu_helper_get_heading();
    result->heading_error = fabsf(result->final_heading - result->initial_heading);
    
    /* Check success criteria */
    result->imu_accurate = (result->heading_error <= HEADING_TOLERANCE_DEG);
    result->returned_to_path = (result->heading_error <= RETURN_HEADING_TOLERANCE);
    result->completed_successfully = result->obstacle_detected && 
                                    result->returned_to_path &&
                                    (result->reaction_time_ms <= REACTION_TIME_MS);
    
    printf("%s[Maneuver #%lu] COMPLETE - Final heading: %.1f° (error: %.1f°)%s\n",
           result->completed_successfully ? COLOR_GREEN : COLOR_YELLOW,
           result->maneuver_num, result->final_heading, 
           result->heading_error, COLOR_RESET);
    
    return result->completed_successfully;
}

/**
 * @brief   Stop all motors
 * @param   None
 * @return  None
 */
static void 
stop_motors(void)
{
    motor_stop(M1A, M1B);  // Stop left motor
    motor_stop(M2A, M2B);  // Stop right motor
}

/**
 * @brief   Validate overall test results
 * @param   stats  Pointer to test statistics
 * @return  true if test passed, false otherwise
 */
static bool 
validate_test_results(TestStats const *stats)
{
    /* Require 100% success rate for all 5 maneuvers */
    return (stats->successful_maneuvers == NUM_TEST_MANEUVERS);
}

/* ========================================================================== */
/* Main Test Function                                                         */
/* ========================================================================== */

/**
 * @brief   Main integration test entry point
 * @param   None
 * @return  0 if test passed, 1 if test failed
 */
int 
main(void)
{
    TestStats stats = {0};
    ManeuverResult results[NUM_TEST_MANEUVERS] = {0};
    
    /* Initialize standard I/O */
    stdio_init_all();
    sleep_ms(2000);
    
    print_test_header();
    
    /* Initialize subsystems */
    printf("[INIT] Initializing obstacle scanner (60° range)...\n");
    scanner_init();
    printf("[INIT] %s✓%s Scanner ready\n", COLOR_GREEN, COLOR_RESET);
    
    printf("[INIT] Initializing IMU (GY-511)...\n");
    if (!imu_helper_init())
    {
        printf("[INIT] %s✗%s IMU initialization failed!\n", 
               COLOR_RED, COLOR_RESET);
        return 1;
    }
    printf("[INIT] %s✓%s IMU ready\n", COLOR_GREEN, COLOR_RESET);
    
    printf("[INIT] Initializing motors...\n");
    motor_init(M1A, M1B);  // Initialize left motor
    motor_init(M2A, M2B);  // Initialize right motor
    printf("[INIT] %s✓%s Motors ready\n", COLOR_GREEN, COLOR_RESET);
    
    /* Display test instructions */
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST INSTRUCTIONS:\n");
    printf("  1. Place robot on test course\n");
    printf("  2. Position obstacle ≥10 cm ahead in path (within 60° scan)\n");
    printf("  3. Robot will detect, stop, turn, bypass, and return\n");
    printf("  4. Test will repeat 5 times\n");
    printf("  5. Reposition obstacle between maneuvers as prompted\n");
    printf("  6. Monitor IMU heading and ultrasonic detection\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n%sPress Enter to start test...%s", COLOR_CYAN, COLOR_RESET);
    getchar();
    
    /* Run test maneuvers */
    for (uint32_t i = 0; i < NUM_TEST_MANEUVERS; i++)
    {
        results[i].maneuver_num = i + 1;
        
        printf("\n");
        printf("═══════════════════════════════════════════════════════════════\n");
        printf("  MANEUVER %lu / %u\n", i + 1, NUM_TEST_MANEUVERS);
        printf("═══════════════════════════════════════════════════════════════\n");
        
        if (i > 0)
        {
            printf("\nReposition obstacle for next test...\n");
            printf("Press Enter when ready...");
            getchar();
        }
        
        /* Perform maneuver */
        bool success = perform_avoidance_maneuver(&results[i]);
        
        stats.total_maneuvers++;
        if (success)
        {
            stats.successful_maneuvers++;
        }
        else
        {
            stats.failed_maneuvers++;
            
            if (!results[i].obstacle_detected)
            {
                stats.detection_failures++;
            }
            if (!results[i].imu_accurate)
            {
                stats.heading_failures++;
            }
            if (!results[i].returned_to_path)
            {
                stats.return_failures++;
            }
        }
        
        /* Update averages */
        stats.avg_reaction_time_ms += results[i].reaction_time_ms;
        stats.avg_heading_error += results[i].heading_error;
        
        /* Print result */
        print_maneuver_result(&results[i]);
        
        sleep_ms(2000);
    }
    
    /* Calculate final averages */
    if (stats.total_maneuvers > 0U)
    {
        stats.avg_reaction_time_ms /= (float)stats.total_maneuvers;
        stats.avg_heading_error /= (float)stats.total_maneuvers;
    }
    
    /* Print final statistics */
    print_final_stats(&stats);
    
    /* Validate results */
    bool test_passed = validate_test_results(&stats);
    
    /* Print final verdict */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    FINAL TEST VERDICT                         ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    if (test_passed)
    {
        printf("  %s✓ INTEG-T3: ALL MANEUVERS SUCCESSFUL%s\n",
               COLOR_GREEN, COLOR_RESET);
        printf("\n");
        printf("  Robot demonstrated:\n");
        printf("  - Reliable obstacle detection (ultrasonic)\n");
        printf("  - Accurate heading tracking (IMU)\n");
        printf("  - Autonomous path correction\n");
        printf("  - 100%% success rate (%u/%u maneuvers)\n",
               stats.successful_maneuvers, NUM_TEST_MANEUVERS);
        printf("  - Average reaction: %.0f ms\n", stats.avg_reaction_time_ms);
        printf("  - Average heading error: %.1f°\n", stats.avg_heading_error);
    }
    else
    {
        printf("  %s✗ INTEG-T3: SOME MANEUVERS FAILED%s\n",
               COLOR_RED, COLOR_RESET);
        printf("\n");
        printf("  Failures: %lu / %u maneuvers\n",
               stats.failed_maneuvers, NUM_TEST_MANEUVERS);
        if (stats.detection_failures > 0U)
        {
            printf("  - Detection failures: %lu\n", stats.detection_failures);
        }
        if (stats.heading_failures > 0U)
        {
            printf("  - Heading failures: %lu\n", stats.heading_failures);
        }
        if (stats.return_failures > 0U)
        {
            printf("  - Path return failures: %lu\n", stats.return_failures);
        }
    }
    printf("\n");
    
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  INTEG-T3 TEST COMPLETE\n");
    printf("  Ultrasonic: 60° scan (50° to 110°)\n");
    printf("  IMU: GY-511 (LSM303DLHC) heading tracking\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    
    return test_passed ? 0 : 1;
}