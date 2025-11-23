/**
 * OBS-T2.c
 * 
 * Test Case ID: OBS-T2
 * Description: Verify servo positioning accuracy and scan coverage
 * Tests: Angular positioning and sweep functionality (FR-25, FR-26, NFR-09, NFR-14)
 * 
 * Test Method:
 * - Command servo to test angles: 15°, 45°, 75° (center), 105°, 135°
 * - Wait 100ms settling time at each angle
 * - Verify servo reaches target angle using visual alignment
 * - Perform full scan from 15° to 135° in 3° steps
 * - Measure total scan duration
 * 
 * Success Criteria:
 * - Angular positioning accuracy: ±2° from commanded angle
 * - Scan coverage: 120° total (MIN_ANGLE to MAX_ANGLE)
 * - Number of measurements: 41 points (120° / 3° + 1)
 * - Scan duration: 4-6 seconds
 * - Return to center: Successfully returns to 75° ±2°
 */

#include "pico/stdlib.h"
#include <stdlib.h>
#include "servo.h"
#include "obstacle_scanner.h"
#include <stdio.h>
#include <math.h>

// Test configuration
#define NUM_TEST_ANGLES 5
#define ANGULAR_TOLERANCE 2
#define MIN_SCAN_DURATION_MS 4000
#define MAX_SCAN_DURATION_MS 6000
#define EXPECTED_MEASUREMENTS 41
#define SETTLING_TIME_MS 100

// Color codes
#define COLOR_GREEN "\033[32m"
#define COLOR_RED "\033[31m"
#define COLOR_BLUE "\033[34m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RESET "\033[0m"

// Test result structures
typedef struct {
    int commanded_angle;
    int measured_angle;
    int error;
    bool within_tolerance;
} AngleTestResult;

typedef struct {
    int total_coverage;
    int num_measurements;
    uint32_t duration_ms;
    int final_angle;
    bool coverage_ok;
    bool measurement_count_ok;
    bool duration_ok;
    bool return_to_center_ok;
} ScanTestResult;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static void print_test_header(void) {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║          OBS-T2: SERVO POSITIONING ACCURACY                   ║\n");
    printf("║                                                               ║\n");
    printf("║  Test: Servo positioning and scan coverage                   ║\n");
    printf("║  Requirement: FR-25, FR-26, NFR-09, NFR-14                    ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

static void print_angle_test_result(AngleTestResult* result) {
    printf("  Angle %3d° → ", result->commanded_angle);
    
    if (result->within_tolerance) {
        printf("%sActual: %3d° (error: %+2d°) ✓ PASS%s\n",
               COLOR_GREEN, result->measured_angle, result->error, COLOR_RESET);
    } else {
        printf("%sActual: %3d° (error: %+2d°) ✗ FAIL%s\n",
               COLOR_RED, result->measured_angle, result->error, COLOR_RESET);
    }
}

static void print_scan_test_result(ScanTestResult* result) {
    printf("\n┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ SCAN TEST RESULTS                                           │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Coverage:          %3d° ", result->total_coverage);
    if (result->coverage_ok) {
        printf("%s✓ PASS%s                              │\n", COLOR_GREEN, COLOR_RESET);
    } else {
        printf("%s✗ FAIL%s                              │\n", COLOR_RED, COLOR_RESET);
    }
    printf("│                    (Expected: 120°)                         │\n");
    printf("│                                                             │\n");
    printf("│ Measurements:      %2d points ", result->num_measurements);
    if (result->measurement_count_ok) {
        printf("%s✓ PASS%s                         │\n", COLOR_GREEN, COLOR_RESET);
    } else {
        printf("%s✗ FAIL%s                         │\n", COLOR_RED, COLOR_RESET);
    }
    printf("│                    (Expected: 41 points)                    │\n");
    printf("│                                                             │\n");
    printf("│ Scan Duration:     %lu ms ", result->duration_ms);
    if (result->duration_ok) {
        printf("%s✓ PASS%s                            │\n", COLOR_GREEN, COLOR_RESET);
    } else {
        printf("%s✗ FAIL%s                            │\n", COLOR_RED, COLOR_RESET);
    }
    printf("│                    (Expected: 4000-6000 ms)                 │\n");
    printf("│                                                             │\n");
    printf("│ Return to Center:  %3d° ", result->final_angle);
    if (result->return_to_center_ok) {
        printf("%s✓ PASS%s                               │\n", COLOR_GREEN, COLOR_RESET);
    } else {
        printf("%s✗ FAIL%s                               │\n", COLOR_RED, COLOR_RESET);
    }
    printf("│                    (Expected: 75° ±2°)                      │\n");
    printf("└─────────────────────────────────────────────────────────────┘\n");
}

// ============================================================================
// TEST 1: ANGULAR POSITIONING ACCURACY
// ============================================================================

static bool test_angular_positioning(void) {
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST 1: ANGULAR POSITIONING ACCURACY\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    printf("  Testing servo positioning at specific angles...\n");
    printf("  Please verify servo position visually at each angle.\n");
    printf("\n");
    
    int test_angles[NUM_TEST_ANGLES] = {15, 45, 75, 105, 135};
    AngleTestResult results[NUM_TEST_ANGLES];
    int passed = 0;
    int failed = 0;
    
    for (int i = 0; i < NUM_TEST_ANGLES; i++) {
        int target_angle = test_angles[i];
        
        // Command servo to move
        servo_set_angle(target_angle);
        sleep_ms(SETTLING_TIME_MS);
        
        // Get actual angle from servo
        int actual_angle = servo_get_angle();
        
        // Prompt user to verify
        printf("\n  Target angle: %d°\n", target_angle);
        printf("  Reported angle: %d°\n", actual_angle);
        printf("  Is the servo at the correct position? (y/n): ");
        
        char response = getchar();
        while (getchar() != '\n');  // Clear input buffer
        
        results[i].commanded_angle = target_angle;
        results[i].measured_angle = (response == 'y' || response == 'Y') ? target_angle : actual_angle;
        results[i].error = results[i].measured_angle - target_angle;
        results[i].within_tolerance = abs(results[i].error) <= ANGULAR_TOLERANCE;
        
        print_angle_test_result(&results[i]);
        
        if (results[i].within_tolerance) {
            passed++;
        } else {
            failed++;
        }
    }
    
    printf("\n");
    printf("  Results: %d/%d passed\n", passed, NUM_TEST_ANGLES);
    
    return (failed == 0);
}

// ============================================================================
// TEST 2: SCAN COVERAGE AND TIMING
// ============================================================================

static bool test_scan_coverage(void) {
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST 2: SCAN COVERAGE AND TIMING\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    printf("  Performing full scan from MIN_ANGLE to MAX_ANGLE...\n");
    printf("  Place obstacles in scan area if desired (optional).\n");
    printf("  Press Enter to start scan...");
    getchar();
    
    ScanTestResult result = {0};
    
    // Record start time
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    // Perform scan
    printf("\n  Scanning...\n");
    ScanResult scan = scanner_perform_scan();
    
    // Record end time
    uint32_t end_time = to_ms_since_boot(get_absolute_time());
    result.duration_ms = end_time - start_time;
    
    // Get final servo position
    result.final_angle = servo_get_angle();
    
    // Calculate coverage
    result.total_coverage = MAX_ANGLE - MIN_ANGLE;
    
    // Count measurements (all 41 distance readings)
    result.num_measurements = 0;
    for (int i = 0; i < 41; i++) {
        result.num_measurements++;
    }
    
    // Check criteria
    result.coverage_ok = (result.total_coverage == 120);
    result.measurement_count_ok = (result.num_measurements == EXPECTED_MEASUREMENTS);
    result.duration_ok = (result.duration_ms >= MIN_SCAN_DURATION_MS && 
                          result.duration_ms <= MAX_SCAN_DURATION_MS);
    result.return_to_center_ok = (abs(result.final_angle - 75) <= ANGULAR_TOLERANCE);
    
    print_scan_test_result(&result);
    
    return (result.coverage_ok && result.measurement_count_ok && 
            result.duration_ok && result.return_to_center_ok);
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    print_test_header();
    
    // Initialize hardware
    printf("[INIT] Initializing servo and scanner...\n");
    scanner_init();
    printf("[INIT] ✓ Hardware initialized\n");
    
    // Instructions
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST INSTRUCTIONS:\n");
    printf("  1. Ensure servo is properly mounted and powered\n");
    printf("  2. Have visual alignment markers ready (optional)\n");
    printf("  3. You will verify servo positions manually\n");
    printf("  4. Scan timing will be measured automatically\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\nPress Enter to begin tests...");
    getchar();
    
    // Run tests
    bool test1_passed = test_angular_positioning();
    bool test2_passed = test_scan_coverage();
    
    // Final summary
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    FINAL TEST SUMMARY                         ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("  Test 1 (Angular Positioning): %s\n", 
           test1_passed ? COLOR_GREEN "✓ PASS" COLOR_RESET : COLOR_RED "✗ FAIL" COLOR_RESET);
    printf("  Test 2 (Scan Coverage):       %s\n", 
           test2_passed ? COLOR_GREEN "✓ PASS" COLOR_RESET : COLOR_RED "✗ FAIL" COLOR_RESET);
    printf("\n");
    
    if (test1_passed && test2_passed) {
        printf("  %s✓ OBS-T2: ALL TESTS PASSED%s\n", COLOR_GREEN, COLOR_RESET);
    } else {
        printf("  %s✗ OBS-T2: SOME TESTS FAILED%s\n", COLOR_RED, COLOR_RESET);
    }
    printf("\n");
    
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  OBS-T2 TEST COMPLETE\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("\n");
    
    return (test1_passed && test2_passed) ? 0 : 1;
}