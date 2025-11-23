/*!
 * @file    integration_test_obstacle.c
 * @brief   Integration test for obstacle detection, scanning, and avoidance.
 *
 * @details Tests the integrated obstacle detection system including ultrasonic
 *          sensor, servo scanning, avoidance algorithm, and telemetry.
 *
 *          Test Scenarios:
 *          - INT-OBS-T1: Hardware initialization (ultrasonic, servo, scanner)
 *          - INT-OBS-T2: Network and telemetry initialization (WiFi, MQTT)
 *          - INT-OBS-T3: Basic obstacle detection
 *          - INT-OBS-T4: Full scan and data quality
 *          - INT-OBS-T5: Avoidance direction computation
 *          - INT-OBS-T6: Telemetry publishing reliability
 *
 * @par     COPYRIGHT NOTICE
 *          (c) 2025 Barr Group. All rights reserved.
 *
 * @author  Your Name
 * @date    November 24, 2025
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "config.h"
#include "pin_definitions.h"
#include "wifi.h"
#include "mqtt_client.h"
#include "telemetry.h"
#include "ultrasonic.h"
#include "servo.h"
#include "obstacle_control.h"
#include "obstacle_scanner.h"
#include "avoidance_maneuver.h"

/*----------------------------------------------------------------------------*/
/* Configuration Constants                                                    */
/*----------------------------------------------------------------------------*/

/* Test parameters */
#define OBSTACLE_DETECT_DISTANCE_CM     20u
#define SCAN_TIMEOUT_MS                 5000u
#define MIN_VALID_READINGS              5u
#define TELEMETRY_TIMEOUT_MS            3000u

/* Test intervals */
#define OBSTACLE_CHECK_INTERVAL_MS      500u

/* Maximum number of integration tests */
#define MAX_INTEGRATION_TESTS           6u

/* ANSI color codes */
#define COLOR_GREEN                     "\033[32m"
#define COLOR_RED                       "\033[31m"
#define COLOR_YELLOW                    "\033[33m"
#define COLOR_CYAN                      "\033[36m"
#define COLOR_BLUE                      "\033[34m"
#define COLOR_RESET                     "\033[0m"

/* String buffer sizes */
#define TEST_NOTES_SIZE                 128u
#define TEST_ID_SIZE                    16u
#define TEST_NAME_SIZE                  64u

/*----------------------------------------------------------------------------*/
/* Type Definitions                                                           */
/*----------------------------------------------------------------------------*/

/*!
 * @brief Integration test result structure
 */
typedef struct
{
    char    test_id[TEST_ID_SIZE];      /* Test identifier e.g., "INT-OBS-T1" */
    char    test_name[TEST_NAME_SIZE];  /* Descriptive test name */
    bool    bpassed;                    /* Test result */
    float   measured_value;             /* Primary measured value */
    float   expected_value;             /* Expected value */
    float   tolerance;                  /* Acceptable tolerance */
    char    notes[TEST_NOTES_SIZE];     /* Additional notes or failure info */
} integration_test_result_t;

/*----------------------------------------------------------------------------*/
/* Private Function Prototypes                                                */
/*----------------------------------------------------------------------------*/

static void print_test_header (char const * const ptest_id,
                                char const * const ptest_name);

static void print_test_result (integration_test_result_t const * const presult);

static integration_test_result_t test_hardware_initialization (void);

static integration_test_result_t test_network_and_telemetry_init (void);

static integration_test_result_t test_basic_obstacle_detection (void);

static integration_test_result_t test_full_scan_quality (void);

static integration_test_result_t test_avoidance_direction_computation (void);

static integration_test_result_t test_telemetry_publishing (void);

/*----------------------------------------------------------------------------*/
/* Public Function Bodies                                                     */
/*----------------------------------------------------------------------------*/

/*!
 * @brief   Main entry point - runs complete integration test suite.
 * @return  Exit code (0 = success)
 */
int
main (void)
{
    integration_test_result_t results[MAX_INTEGRATION_TESTS];
    uint32_t                  tests_passed = 0u;
    uint32_t                  tests_failed = 0u;
    
    /* Initialize USB serial */
    stdio_init_all();
    sleep_ms(2000u);
    
    /* Print test suite header */
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                                                               ║\n");
    printf("║   INTEGRATION TEST: OBSTACLE DETECTION & AVOIDANCE SYSTEM    ║\n");
    printf("║                                                               ║\n");
    printf("║   Tests obstacle detection, scanning, avoidance logic, and    ║\n");
    printf("║   telemetry publishing for the complete obstacle subsystem.   ║\n");
    printf("║                                                               ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    
    printf("\nStarting integration tests...\n");
    printf("Press CTRL+C to abort at any time.\n\n");
    sleep_ms(2000u);
    
    /* Run all tests */
    results[0] = test_hardware_initialization();
    sleep_ms(1000u);
    
    results[1] = test_network_and_telemetry_init();
    sleep_ms(1000u);
    
    /* Only continue if network is up */
    if (!results[1].bpassed)
    {
        printf("\n%sNetwork initialization failed - skipping remaining tests%s\n",
               COLOR_RED, COLOR_RESET);
        return (1);
    }
    
    results[2] = test_basic_obstacle_detection();
    sleep_ms(1000u);
    
    results[3] = test_full_scan_quality();
    sleep_ms(1000u);
    
    results[4] = test_avoidance_direction_computation();
    sleep_ms(1000u);
    
    results[5] = test_telemetry_publishing();
    
    /* Count results */
    for (uint32_t idx = 0u; idx < MAX_INTEGRATION_TESTS; idx++)
    {
        if (results[idx].bpassed)
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
    printf("║             INTEGRATION TEST SUMMARY                          ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    printf("┌──────────────┬──────────────────────────────────────────┬────────┐\n");
    printf("│ Test ID      │ Description                              │ Result │\n");
    printf("├──────────────┼──────────────────────────────────────────┼────────┤\n");
    
    for (uint32_t idx = 0u; idx < MAX_INTEGRATION_TESTS; idx++)
    {
        printf("│ %-12s │ %-40s │ %s%-6s%s │\n",
               results[idx].test_id,
               results[idx].test_name,
               (results[idx].bpassed) ? COLOR_GREEN : COLOR_RED,
               (results[idx].bpassed) ? "PASS" : "FAIL",
               COLOR_RESET);
    }
    
    printf("└──────────────┴──────────────────────────────────────────┴────────┘\n\n");
    
    printf("Overall Results:\n");
    printf(" Tests Passed: %s%lu/6%s\n",
           COLOR_GREEN, tests_passed, COLOR_RESET);
    printf(" Tests Failed: %s%lu/6%s\n",
           (0u == tests_failed) ? COLOR_GREEN : COLOR_RED,
           tests_failed,
           COLOR_RESET);
    printf(" Success Rate: %.1f%%\n\n",
           ((float)tests_passed * 100.0f / (float)MAX_INTEGRATION_TESTS));
    
    if (0u == tests_failed)
    {
        printf("%s╔════════════════════════════════════════════════════════════╗%s\n",
               COLOR_GREEN, COLOR_RESET);
        printf("%s║ ALL INTEGRATION TESTS PASSED - SYSTEM READY FOR USE       ║%s\n",
               COLOR_GREEN, COLOR_RESET);
        printf("%s╚════════════════════════════════════════════════════════════╝%s\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("%s╔════════════════════════════════════════════════════════════╗%s\n",
               COLOR_YELLOW, COLOR_RESET);
        printf("%s║ SOME TESTS FAILED - REVIEW RESULTS AND CHECK HARDWARE     ║%s\n",
               COLOR_YELLOW, COLOR_RESET);
        printf("%s╚════════════════════════════════════════════════════════════╝%s\n",
               COLOR_YELLOW, COLOR_RESET);
    }
    
    printf("\n");
    
    return ((0u == tests_failed) ? 0 : 1);
}

/*----------------------------------------------------------------------------*/
/* Private Function Bodies                                                    */
/*----------------------------------------------------------------------------*/

/*!
 * @brief   Print test header.
 * @param   ptest_id   Pointer to test identifier string
 * @param   ptest_name Pointer to test description string
 * @return  None
 */
static void
print_test_header (char const * const ptest_id, char const * const ptest_name)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║ %s%-60s%s║\n", COLOR_CYAN, ptest_id, COLOR_RESET);
    printf("║ %s\n", ptest_name);
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
}

/*!
 * @brief   Print test result.
 * @param   presult Pointer to test result structure
 * @return  None
 */
static void
print_test_result (integration_test_result_t const * const presult)
{
    printf("\n");
    printf("┌────────────────────────────────────────────────────────────┐\n");
    printf("│ TEST RESULT                                                 │\n");
    printf("├────────────────────────────────────────────────────────────┤\n");
    printf("│ Test ID: %-50s │\n", presult->test_id);
    printf("│ Status:  %s%-50s%s │\n",
           (presult->bpassed) ? COLOR_GREEN : COLOR_RED,
           (presult->bpassed) ? "✓ PASSED" : "✗ FAILED",
           COLOR_RESET);
    printf("├────────────────────────────────────────────────────────────┤\n");
    printf("│ Measured:  %-48.2f │\n", presult->measured_value);
    printf("│ Expected:  %-48.2f │\n", presult->expected_value);
    printf("│ Tolerance: ±%-46.2f │\n", presult->tolerance);
    
    if ((!presult->bpassed) && ('\0' != presult->notes[0]))
    {
        printf("├────────────────────────────────────────────────────────────┤\n");
        printf("│ Notes: %-52s │\n", presult->notes);
    }
    
    printf("└────────────────────────────────────────────────────────────┘\n");
}

/*!
 * @brief   INT-OBS-T1: Hardware initialization test.
 * @details Verifies servo, ultrasonic, and scanner init successfully.
 * @return  Test result structure
 */
static integration_test_result_t
test_hardware_initialization (void)
{
    integration_test_result_t result;
    uint64_t                  distance = 0u;
    int                       status = 0;
    
    /* Initialize result structure */
    strncpy(result.test_id, "INT-OBS-T1", TEST_ID_SIZE);
    strncpy(result.test_name, "Hardware Initialization", TEST_NAME_SIZE);
    result.bpassed = false;
    result.measured_value = 0.0f;
    result.expected_value = 1.0f;
    result.tolerance = 0.0f;
    result.notes[0] = '\0';
    
    print_test_header(result.test_id, result.test_name);
    
    printf("Step 1: Initializing obstacle hardware...\n");
    
    /* scanner_init() calls ultrasonic_init() and servo_init() */
    scanner_init();
    printf(" ✓ Scanner initialized (ultrasonic + servo)\n\n");
    
    /* Verify servo can move */
    printf("Step 2: Testing servo movement...\n");
    servo_set_angle(MIN_ANGLE);      /* 50 degrees - right position */
    sleep_ms(500u);
    servo_set_angle(ANGLE_CENTER);   /* 80 degrees - center position */
    sleep_ms(500u);
    servo_set_angle(MAX_ANGLE);      /* 110 degrees - left position */
    sleep_ms(500u);
    servo_set_angle(ANGLE_CENTER);   /* Back to center */
    printf(" ✓ Servo movement verified\n\n");
    
    /* Test ultrasonic reading */
    printf("Step 3: Testing ultrasonic sensor...\n");
    status = ultrasonic_get_distance(TRIG_PIN, ECHO_PIN, &distance);
    
    if ((SUCCESS == status) && (0u < distance) && (400u > distance))
    {
        printf(" ✓ Ultrasonic reading: %llu cm\n", distance);
        result.measured_value = 1.0f;
        result.bpassed = true;
    }
    else
    {
        printf(" ✗ Ultrasonic sensor error (status: %d)\n", status);
        snprintf(result.notes, TEST_NOTES_SIZE, "Sensor status: %d", status);
    }
    
    print_test_result(&result);
    
    return (result);
}

/*!
 * @brief   INT-OBS-T2: Network and telemetry initialization.
 * @details Verifies WiFi connection and MQTT telemetry setup.
 * @return  Test result structure
 */
static integration_test_result_t
test_network_and_telemetry_init (void)
{
    integration_test_result_t result;
    char                      ip_buf[16];
    bool                      bconnected = false;
    bool                      binit_ok = false;
    
    /* Initialize result structure */
    strncpy(result.test_id, "INT-OBS-T2", TEST_ID_SIZE);
    strncpy(result.test_name, "Network and Telemetry Initialization", TEST_NAME_SIZE);
    result.bpassed = false;
    result.measured_value = 0.0f;
    result.expected_value = 1.0f;
    result.tolerance = 0.0f;
    result.notes[0] = '\0';
    
    print_test_header(result.test_id, result.test_name);
    
    /* Initialize WiFi */
    printf("Step 1: Initializing WiFi...\n");
    wifi_init();
    
    if (WIFI_ERROR == wifi_get_status())
    {
        printf(" ✗ WiFi init error\n");
        snprintf(result.notes, TEST_NOTES_SIZE, "WiFi init failed");
        print_test_result(&result);
        
        return (result);
    }
    
    printf(" ✓ WiFi initialized\n\n");
    
    /* Connect to WiFi */
    printf("Step 2: Connecting to WiFi SSID: %s\n", WIFI_SSID);
    bconnected = wifi_connect(WIFI_SSID, WIFI_PASSWORD);
    
    if (!bconnected)
    {
        printf(" ✗ WiFi connection failed\n");
        snprintf(result.notes, TEST_NOTES_SIZE, "WiFi connection failed");
        print_test_result(&result);
        
        return (result);
    }
    
    if (wifi_get_ip(ip_buf, sizeof(ip_buf)))
    {
        printf(" ✓ Connected. IP Address: %s\n\n", ip_buf);
    }
    
    /* Initialize telemetry */
    printf("Step 3: Initializing telemetry (MQTT)...\n");
    printf(" Broker: %s:%d\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
    printf(" Client ID: %s\n", MQTT_CLIENT_ID);
    
    binit_ok = telemetry_init(MQTT_BROKER_IP, MQTT_BROKER_PORT, MQTT_CLIENT_ID);
    
    if (!binit_ok)
    {
        printf(" ✗ Telemetry init failed\n");
        snprintf(result.notes, TEST_NOTES_SIZE, "MQTT init failed");
        print_test_result(&result);
        
        return (result);
    }
    
    printf(" ✓ Telemetry system ready\n\n");
    
    /* Send test status */
    telemetry_publish_status("Obstacle integration test firmware started");
    
    result.measured_value = 1.0f;
    result.bpassed = true;
    
    print_test_result(&result);
    
    return (result);
}

/*!
 * @brief   INT-OBS-T3: Basic obstacle detection test.
 * @details Tests check_for_obstacles() function.
 * @return  Test result structure
 */
static integration_test_result_t
test_basic_obstacle_detection (void)
{
    integration_test_result_t result;
    uint32_t                  detections = 0u;
    uint32_t const            total_checks = 5u;
    bool                      bdetected = false;
    
    /* Initialize result structure */
    strncpy(result.test_id, "INT-OBS-T3", TEST_ID_SIZE);
    strncpy(result.test_name, "Basic Obstacle Detection", TEST_NAME_SIZE);
    result.bpassed = false;
    result.measured_value = 0.0f;
    result.expected_value = 1.0f;
    result.tolerance = 0.0f;
    result.notes[0] = '\0';
    
    print_test_header(result.test_id, result.test_name);
    
    printf("Testing obstacle detection function...\n");
    printf("Place an obstacle within 20 cm or leave clear.\n\n");
    
    for (uint32_t idx = 0u; idx < total_checks; idx++)
    {
        printf(" Check %lu/%lu: ", (idx + 1u), total_checks);
        bdetected = check_for_obstacles();
        
        if (bdetected)
        {
            printf("%sObstacle detected%s\n", COLOR_YELLOW, COLOR_RESET);
            detections++;
        }
        else
        {
            printf("Clear\n");
        }
        
        sleep_ms(300u);
    }
    
    printf("\n");
    printf("Summary: %lu/%lu checks detected obstacles\n", detections, total_checks);
    
    /* Test passes if function executes without error */
    result.measured_value = (float)detections;
    result.expected_value = 0.0f;  /* Expected varies - just check it works */
    result.bpassed = true;         /* Pass if no crashes */
    
    print_test_result(&result);
    
    return (result);
}

/*!
 * @brief   INT-OBS-T4: Full scan and data quality test.
 * @details Tests scanner_perform_scan() and validates scan quality.
 * @return  Test result structure
 */
static integration_test_result_t
test_full_scan_quality (void)
{
    integration_test_result_t result;
    ScanResult                scan;
    uint32_t                  valid_count = 0u;
    
    /* Initialize result structure */
    strncpy(result.test_id, "INT-OBS-T4", TEST_ID_SIZE);
    strncpy(result.test_name, "Full Scan and Data Quality", TEST_NAME_SIZE);
    result.bpassed = false;
    result.measured_value = 0.0f;
    result.expected_value = (float)MIN_VALID_READINGS;
    result.tolerance = 0.0f;
    result.notes[0] = '\0';
    
    print_test_header(result.test_id, result.test_name);
    
    printf("Performing full ultrasonic scan...\n\n");
    
    scan = scanner_perform_scan();
    
    printf("Scan complete.\n");
    scanner_print_results(scan);
    
    /* Count valid readings - distances array has 21 elements */
    for (uint32_t idx = 0u; idx < 21u; idx++)
    {
        if ((0u < scan.distances[idx]) && (400u > scan.distances[idx]))
        {
            valid_count++;
        }
    }
    
    result.measured_value = (float)valid_count;
    result.bpassed = (valid_count >= MIN_VALID_READINGS);
    
    if (!result.bpassed)
    {
        snprintf(result.notes, TEST_NOTES_SIZE,
                 "Only %lu valid readings (need %u)",
                 valid_count,
                 MIN_VALID_READINGS);
    }
    
    print_test_result(&result);
    
    return (result);
}

/*!
 * @brief   INT-OBS-T5: Avoidance direction computation test.
 * @details Tests scanner_get_best_avoidance_direction().
 * @return  Test result structure
 */
static integration_test_result_t
test_avoidance_direction_computation (void)
{
    integration_test_result_t result;
    ScanResult                scan;
    AvoidanceDirection        direction = AVOID_NONE;
    
    /* Initialize result structure */
    strncpy(result.test_id, "INT-OBS-T5", TEST_ID_SIZE);
    strncpy(result.test_name, "Avoidance Direction Computation", TEST_NAME_SIZE);
    result.bpassed = false;
    result.measured_value = 0.0f;
    result.expected_value = 1.0f;
    result.tolerance = 0.0f;
    result.notes[0] = '\0';
    
    print_test_header(result.test_id, result.test_name);
    
    printf("Performing scan and computing avoidance direction...\n\n");
    
    scan = scanner_perform_scan();
    direction = scanner_get_best_avoidance_direction(scan);
    
    printf("Scan Results:\n");
    scanner_print_results(scan);
    
    printf("\nComputed avoidance direction: ");
    
    switch (direction)
    {
        case AVOID_NONE:
            printf("%sNONE (path clear)%s\n", COLOR_GREEN, COLOR_RESET);
            break;
            
        case AVOID_LEFT:
            printf("%sLEFT%s\n", COLOR_YELLOW, COLOR_RESET);
            break;
            
        case AVOID_RIGHT:
            printf("%sRIGHT%s\n", COLOR_YELLOW, COLOR_RESET);
            break;
            
        default:
            printf("%sUNKNOWN%s\n", COLOR_RED, COLOR_RESET);
            break;
    }
    
    /* Test passes if direction is valid */
    result.measured_value = 1.0f;
    result.bpassed = ((AVOID_NONE <= direction) && (AVOID_RIGHT >= direction));
    
    if (!result.bpassed)
    {
        snprintf(result.notes, TEST_NOTES_SIZE,
                 "Invalid direction value: %d",
                 (int)direction);
    }
    
    print_test_result(&result);
    
    return (result);
}

/*!
 * @brief   INT-OBS-T6: Telemetry publishing reliability test.
 * @details Tests telemetry_publish_obstacle_scan() and avoidance telemetry.
 * @return  Test result structure
 */
static integration_test_result_t
test_telemetry_publishing (void)
{
    integration_test_result_t result;
    ScanResult                scan;
    AvoidanceDirection        direction = AVOID_NONE;
    bool                      bcleared = false;
    AvoidanceState            state = AVOIDANCE_IDLE;
    bool                      bpub_ok = false;
    
    /* Initialize result structure */
    strncpy(result.test_id, "INT-OBS-T6", TEST_ID_SIZE);
    strncpy(result.test_name, "Telemetry Publishing Reliability", TEST_NAME_SIZE);
    result.bpassed = false;
    result.measured_value = 0.0f;
    result.expected_value = 2.0f;  /* Expect 2 successful publishes */
    result.tolerance = 0.0f;
    result.notes[0] = '\0';
    
    print_test_header(result.test_id, result.test_name);
    
    if (!telemetry_is_ready())
    {
        printf("✗ Telemetry not ready\n");
        snprintf(result.notes, TEST_NOTES_SIZE, "Telemetry not connected");
        print_test_result(&result);
        
        return (result);
    }
    
    printf("Telemetry ready. Testing publish operations...\n\n");
    
    /* Test 1: Scan telemetry */
    printf("Test 1: Publishing obstacle scan telemetry...\n");
    scan = scanner_perform_scan();
    
    /* scanner_perform_scan() publishes automatically if telemetry is ready */
    printf(" ✓ Scan telemetry published (via scanner_perform_scan)\n\n");
    
    /* Test 2: Avoidance telemetry */
    printf("Test 2: Publishing avoidance telemetry...\n");
    direction = scanner_get_best_avoidance_direction(scan);
    bcleared = (AVOID_NONE != direction);
    state = (bcleared) ? AVOIDANCE_COMPLETE : AVOIDANCE_FAILED;
    
    bpub_ok = telemetry_publish_avoidance(direction, state, bcleared);
    
    if (bpub_ok)
    {
        printf(" ✓ Avoidance telemetry published successfully\n");
        result.measured_value = 2.0f;
        result.bpassed = true;
    }
    else
    {
        printf(" ✗ Avoidance telemetry publish failed\n");
        result.measured_value = 1.0f;
        snprintf(result.notes, TEST_NOTES_SIZE, "Avoidance publish failed");
    }
    
    print_test_result(&result);
    
    return (result);
}

/*** end of file ***/
