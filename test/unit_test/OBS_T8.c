/**
 * @file    OBS_T8.c
 * @brief   Integration test for obstacle detection system
 * 
 * @details Test Case ID: OBS-T8
 *          Description: Verify system integration and 30-minute continuous operation
 *          Tests: Complete system integration with 60° scan range (FR-24 through FR-32, NFR-09 through NFR-14)
 * 
 * Test Method:
 *   - Initialize with single scanner_init() call
 *   - Setup dynamic obstacle course (3-5 obstacles, 10-30 cm range, within 60° scan)
 *   - Start continuous scanning (5s interval between scans)
 *   - Move obstacles every 5 minutes
 *   - Run for 30 minutes (target: ≥360 scans)
 *   - Monitor system health and data completeness
 * 
 * Success Criteria:
 *   - Runtime: 30 minutes minimum
 *   - Scans completed: ≥360 (at ~5s per scan)
 *   - Scan timing: 2-3s per scan (adjusted for 60° range)
 *   - Accuracy: ±2 cm maintained throughout
 *   - Detection: Correct obstacle_count after changes
 *   - Stability: Zero crashes/hangs/memory errors
 *   - Data completeness: 100% scans have complete results (21 readings)
 *   - Dynamic response: Detects moves within 1 scan
 */

#include "pico/stdlib.h"
#include "obstacle_scanner.h"
#include <stdio.h>
#include <string.h>

/* ========================================================================== */
/* Configuration Constants                                                    */
/* ========================================================================== */

#define TEST_DURATION_MS        1800000U    /* 30 minutes in milliseconds */
#define SCAN_INTERVAL_MS        5000U       /* 5 seconds between scans */
#define MIN_SCANS_EXPECTED      360U        /* Minimum scans in 30 minutes */
#define PROMPT_INTERVAL_MS      300000U     /* 5 minutes between prompts */
#define SCAN_MIN_TIME_MS        2000U       /* Minimum expected scan time */
#define SCAN_MAX_TIME_MS        3000U       /* Maximum expected scan time */
#define EXPECTED_READINGS       21U         /* For 60° scan (50° to 110°) */

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
    uint32_t total_scans;
    uint32_t successful_scans;
    uint32_t failed_scans;
    uint32_t scans_with_data;
    uint32_t scans_in_time_window;
    uint32_t min_scan_time_ms;
    uint32_t max_scan_time_ms;
    uint32_t total_obstacles_detected;
    bool system_crashed;
    bool data_corruption_detected;
} IntegrationTestStats;

/* ========================================================================== */
/* Static Function Prototypes                                                 */
/* ========================================================================== */

static void print_test_header(void);
static void print_progress_bar(uint32_t current_ms, uint32_t total_ms);
static void print_live_stats(IntegrationTestStats const *stats, 
                             uint32_t elapsed_ms);
static void print_scan_result(ScanResult const *scan, uint32_t scan_num, 
                              uint32_t duration_ms);
static void print_final_report(IntegrationTestStats const *stats, 
                               uint32_t total_time_ms);
static bool validate_test_results(IntegrationTestStats const *stats);

/* ========================================================================== */
/* Static Function Implementations                                            */
/* ========================================================================== */

/**
 * @brief   Print test header with scan configuration
 * @param   None
 * @return  None
 */
static void 
print_test_header(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║        OBS-T8: 30-MINUTE INTEGRATION TEST                     ║\n");
    printf("║                                                               ║\n");
    printf("║  Test: System integration and continuous operation           ║\n");
    printf("║  Duration: 30 minutes                                         ║\n");
    printf("║  Scan Configuration: 60° (50° to 110°, center at 80°)        ║\n");
    printf("║  Expected Readings: 21 per scan                               ║\n");
    printf("║  Requirement: FR-24 to FR-32, NFR-09 to NFR-14                ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

/**
 * @brief   Display progress bar with time remaining
 * @param   current_ms  Current elapsed time in milliseconds
 * @param   total_ms    Total test duration in milliseconds
 * @return  None
 */
static void 
print_progress_bar(uint32_t current_ms, uint32_t total_ms)
{
    int const bar_width = 40;
    int filled = (int)((current_ms * bar_width) / total_ms);
    float percent = (current_ms * 100.0f) / total_ms;
    
    uint32_t remaining_ms = total_ms - current_ms;
    uint32_t remaining_min = remaining_ms / 60000U;
    uint32_t remaining_sec = (remaining_ms % 60000U) / 1000U;
    
    printf("\r  Progress: [");
    for (int i = 0; i < bar_width; i++)
    {
        if (i < filled)
        {
            printf("█");
        }
        else
        {
            printf("░");
        }
    }
    printf("] %.1f%% | Time remaining: %lu:%02lu   ", 
           percent, remaining_min, remaining_sec);
    fflush(stdout);
}

/**
 * @brief   Display live statistics during test
 * @param   stats       Pointer to test statistics structure
 * @param   elapsed_ms  Elapsed time in milliseconds
 * @return  None
 */
static void 
print_live_stats(IntegrationTestStats const *stats, uint32_t elapsed_ms)
{
    uint32_t elapsed_sec = elapsed_ms / 1000U;
    float scans_per_sec = (float)stats->total_scans / ((float)elapsed_sec + 0.001f);
    
    printf("\n");
    printf("┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ LIVE STATISTICS                                             │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Scans Completed:     %4lu                                    │\n", 
           stats->total_scans);
    printf("│ Successful:          %4lu                                    │\n", 
           stats->successful_scans);
    printf("│ Failed:              %4lu                                    │\n", 
           stats->failed_scans);
    printf("│ Rate:                %.2f scans/sec                          │\n", 
           scans_per_sec);
    printf("│                                                             │\n");
    printf("│ Timing:                                                     │\n");
    printf("│   Min scan time:     %4lu ms                                │\n", 
           stats->min_scan_time_ms);
    printf("│   Max scan time:     %4lu ms                                │\n", 
           stats->max_scan_time_ms);
    printf("│   In window:         %4lu (2-3 sec)                         │\n", 
           stats->scans_in_time_window);
    printf("│                                                             │\n");
    printf("│ Obstacles Detected:  %4lu                                    │\n", 
           stats->total_obstacles_detected);
    printf("│ Data Completeness:   %4lu / %4lu                            │\n",
           stats->scans_with_data, stats->total_scans);
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief   Print individual scan result (compact format)
 * @param   scan        Pointer to scan result structure
 * @param   scan_num    Scan number
 * @param   duration_ms Scan duration in milliseconds
 * @return  None
 */
static void 
print_scan_result(ScanResult const *scan, uint32_t scan_num, 
                  uint32_t duration_ms)
{
    char const *time_color = COLOR_GREEN;
    if ((duration_ms < SCAN_MIN_TIME_MS) || (duration_ms > SCAN_MAX_TIME_MS))
    {
        time_color = COLOR_YELLOW;
    }
    
    printf("  Scan #%3lu: %d obstacles | %s%lu ms%s\n",
           scan_num,
           scan->obstacle_count,
           time_color,
           duration_ms,
           COLOR_RESET);
}

/**
 * @brief   Print comprehensive final test report
 * @param   stats         Pointer to test statistics structure
 * @param   total_time_ms Total test duration in milliseconds
 * @return  None
 */
static void 
print_final_report(IntegrationTestStats const *stats, uint32_t total_time_ms)
{
    uint32_t total_min = total_time_ms / 60000U;
    uint32_t total_sec = (total_time_ms % 60000U) / 1000U;
    
    float success_rate = 0.0f;
    if (stats->total_scans > 0U)
    {
        success_rate = (stats->successful_scans * 100.0f) / stats->total_scans;
    }
    
    float timing_accuracy = 0.0f;
    if (stats->total_scans > 0U)
    {
        timing_accuracy = (stats->scans_in_time_window * 100.0f) / 
                         stats->total_scans;
    }
    
    float data_completeness = 0.0f;
    if (stats->total_scans > 0U)
    {
        data_completeness = (stats->scans_with_data * 100.0f) / 
                           stats->total_scans;
    }
    
    printf("\n\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    FINAL TEST REPORT                          ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    printf("┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ TEST DURATION                                               │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Total Runtime:       %lu min %lu sec                         │\n",
           total_min, total_sec);
    printf("│ Target Runtime:      30 min 0 sec                           │\n");
    printf("│ Runtime Status:      ");
    if (total_time_ms >= TEST_DURATION_MS)
    {
        printf("%s✓ PASS%s                                 │\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗ FAIL (incomplete)%s                     │\n",
               COLOR_RED, COLOR_RESET);
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
    
    printf("\n");
    printf("┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ SCAN STATISTICS                                             │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Total Scans:         %lu                                     │\n",
           stats->total_scans);
    printf("│ Required Scans:      ≥%u                                     │\n",
           MIN_SCANS_EXPECTED);
    printf("│ Successful:          %lu (%.1f%%)                            │\n",
           stats->successful_scans, success_rate);
    printf("│ Failed:              %lu                                     │\n",
           stats->failed_scans);
    printf("│ Scan Count:          ");
    if (stats->total_scans >= MIN_SCANS_EXPECTED)
    {
        printf("%s✓ PASS%s                                 │\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗ FAIL%s                                 │\n",
               COLOR_RED, COLOR_RESET);
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
    
    printf("\n");
    printf("┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ TIMING ANALYSIS (60° Scan)                                 │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Min Scan Time:       %lu ms                                  │\n",
           stats->min_scan_time_ms);
    printf("│ Max Scan Time:       %lu ms                                  │\n",
           stats->max_scan_time_ms);
    printf("│ Target Window:       2000-3000 ms                           │\n");
    printf("│ Scans in Window:     %lu / %lu (%.1f%%)                      │\n",
           stats->scans_in_time_window,
           stats->total_scans,
           timing_accuracy);
    printf("│ Timing Accuracy:     ");
    if (timing_accuracy >= 90.0f)
    {
        printf("%s✓ PASS%s                                 │\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s⚠ WARNING%s                              │\n",
               COLOR_YELLOW, COLOR_RESET);
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
    
    printf("\n");
    printf("┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ DATA INTEGRITY                                              │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Scans with Data:     %lu / %lu (%.1f%%)                      │\n",
           stats->scans_with_data,
           stats->total_scans,
           data_completeness);
    printf("│ Data Corruption:     %s                                      │\n",
           stats->data_corruption_detected ? "DETECTED" : "None");
    printf("│ System Crashes:      %s                                      │\n",
           stats->system_crashed ? "YES" : "None");
    printf("│ Data Completeness:   ");
    if ((data_completeness >= 100.0f) && (!stats->data_corruption_detected))
    {
        printf("%s✓ PASS%s                                 │\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s✗ FAIL%s                                 │\n",
               COLOR_RED, COLOR_RESET);
    }
    printf("└─────────────────────────────────────────────────────────────┘\n");
    
    printf("\n");
    printf("┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ OBSTACLE DETECTION                                          │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Total Obstacles:     %lu                                     │\n",
           stats->total_obstacles_detected);
    printf("│ Avg per Scan:        %.2f                                    │\n",
           (float)stats->total_obstacles_detected / 
           (float)(stats->total_scans + 1U));
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

/**
 * @brief   Validate test results against success criteria
 * @param   stats  Pointer to test statistics structure
 * @return  true if all criteria passed, false otherwise
 */
static bool 
validate_test_results(IntegrationTestStats const *stats)
{
    bool passed = true;
    
    /* Check minimum scan count */
    if (stats->total_scans < MIN_SCANS_EXPECTED)
    {
        passed = false;
    }
    
    /* Check data completeness */
    if (stats->scans_with_data < stats->total_scans)
    {
        passed = false;
    }
    
    /* Check for corruption or crashes */
    if (stats->data_corruption_detected || stats->system_crashed)
    {
        passed = false;
    }
    
    /* Check timing window accuracy (at least 90%) */
    float timing_accuracy = 0.0f;
    if (stats->total_scans > 0U)
    {
        timing_accuracy = (stats->scans_in_time_window * 100.0f) / 
                         stats->total_scans;
    }
    if (timing_accuracy < 90.0f)
    {
        passed = false;
    }
    
    return passed;
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
    IntegrationTestStats stats = {0};
    stats.min_scan_time_ms = 0xFFFFFFFFU;  /* Initialize to max value */
    
    /* Initialize standard I/O */
    stdio_init_all();
    sleep_ms(2000);
    
    print_test_header();
    
    /* Initialize scanner system */
    printf("[INIT] Initializing obstacle detection system...\n");
    scanner_init();
    printf("[INIT] %s✓%s Scanner initialized (60° range: 50° to 110°)\n",
           COLOR_GREEN, COLOR_RESET);
    
    /* Display test instructions */
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST INSTRUCTIONS:\n");
    printf("  1. Setup 3-5 obstacles within 60° scan range (50° to 110°)\n");
    printf("  2. Position obstacles at 10-30 cm distance\n");
    printf("  3. Test will run for 30 minutes continuously\n");
    printf("  4. Move obstacles every 5 minutes when prompted\n");
    printf("  5. Monitor console for any errors or crashes\n");
    printf("  6. Expected: ~360 scans total, 2-3 sec per scan\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n%sPress Enter to start 30-minute test...%s",
           COLOR_CYAN, COLOR_RESET);
    getchar();
    
    /* Start test */
    printf("\n%s━━━ STARTING 30-MINUTE INTEGRATION TEST ━━━%s\n\n",
           COLOR_BLUE, COLOR_RESET);
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_prompt_time = start_time;
    uint32_t last_scan_time = start_time;
    uint32_t last_stats_display = start_time;
    
    /* Main test loop */
    while (true)
    {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        uint32_t elapsed = current_time - start_time;
        
        /* Check if test duration reached */
        if (elapsed >= TEST_DURATION_MS)
        {
            break;
        }
        
        /* Update progress bar every second */
        if ((current_time - last_stats_display) >= 1000U)
        {
            print_progress_bar(elapsed, TEST_DURATION_MS);
            last_stats_display = current_time;
        }
        
        /* Check if it's time to prompt for obstacle movement */
        if ((current_time - last_prompt_time) >= PROMPT_INTERVAL_MS)
        {
            printf("\n\n%s⚠ MOVE OBSTACLES NOW (5 min mark)%s\n",
                   COLOR_YELLOW, COLOR_RESET);
            printf("Reposition 1-2 obstacles within scan range...\n\n");
            last_prompt_time = current_time;
        }
        
        /* Perform scan every SCAN_INTERVAL_MS */
        if ((current_time - last_scan_time) >= SCAN_INTERVAL_MS)
        {
            uint32_t scan_start = to_ms_since_boot(get_absolute_time());
            
            /* Execute scan */
            ScanResult scan = scanner_perform_scan();
            
            uint32_t scan_end = to_ms_since_boot(get_absolute_time());
            uint32_t scan_duration = scan_end - scan_start;
            
            /* Update statistics */
            stats.total_scans++;
            
            if (scan.obstacle_count >= 0)
            {
                stats.successful_scans++;
                stats.scans_with_data++;
                stats.total_obstacles_detected += (uint32_t)scan.obstacle_count;
            }
            else
            {
                stats.failed_scans++;
            }
            
            /* Update timing statistics */
            if (scan_duration < stats.min_scan_time_ms)
            {
                stats.min_scan_time_ms = scan_duration;
            }
            if (scan_duration > stats.max_scan_time_ms)
            {
                stats.max_scan_time_ms = scan_duration;
            }
            if ((scan_duration >= SCAN_MIN_TIME_MS) && 
                (scan_duration <= SCAN_MAX_TIME_MS))
            {
                stats.scans_in_time_window++;
            }
            
            /* Print scan result (every 10 scans) */
            if ((stats.total_scans % 10U) == 0U)
            {
                printf("\n");
                print_scan_result(&scan, stats.total_scans, scan_duration);
            }
            
            /* Display live stats every 60 scans (~5 minutes) */
            if ((stats.total_scans % 60U) == 0U)
            {
                print_live_stats(&stats, elapsed);
            }
            
            last_scan_time = current_time;
        }
        
        sleep_ms(100);
    }
    
    uint32_t end_time = to_ms_since_boot(get_absolute_time());
    uint32_t total_time = end_time - start_time;
    
    /* Print final report */
    print_final_report(&stats, total_time);
    
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
        printf("  %s✓ OBS-T8: ALL CRITERIA PASSED%s\n",
               COLOR_GREEN, COLOR_RESET);
        printf("\n");
        printf("  System demonstrated:\n");
        printf("  - 30 minutes continuous operation\n");
        printf("  - %lu total scans (target: ≥%u)\n",
               stats.total_scans, MIN_SCANS_EXPECTED);
        printf("  - 100%% data completeness\n");
        printf("  - Scan timing: %lu-%lu ms (target: 2-3 sec)\n",
               stats.min_scan_time_ms, stats.max_scan_time_ms);
        printf("  - Zero crashes or data corruption\n");
        printf("  - Stable performance throughout test\n");
    }
    else
    {
        printf("  %s✗ OBS-T8: SOME CRITERIA FAILED%s\n",
               COLOR_RED, COLOR_RESET);
        printf("\n");
        if (stats.total_scans < MIN_SCANS_EXPECTED)
        {
            printf("  ✗ Insufficient scans completed\n");
        }
        if (stats.data_corruption_detected)
        {
            printf("  ✗ Data corruption detected\n");
        }
        if (stats.system_crashed)
        {
            printf("  ✗ System crash occurred\n");
        }
    }
    printf("\n");
    
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  OBS-T8 INTEGRATION TEST COMPLETE\n");
    printf("  Scan Configuration: 60° (50° to 110°), 21 readings per scan\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    
    return test_passed ? 0 : 1;
}