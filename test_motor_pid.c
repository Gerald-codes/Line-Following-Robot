#include "test_motor_pid.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// Configuration
#define WHEEL_DIAMETER_MM 65.0f
#define PULSES_PER_REVOLUTION 360
#define PID_UPDATE_INTERVAL_MS 20

// PID Controllers
static PIDController left_pid;
static PIDController right_pid;

// Motor metrics tracking
typedef struct {
    int32_t last_count;
    float distance_mm;
    float speed_mm_per_sec;
    uint32_t last_update_time;
} MotorMetrics;

static MotorMetrics left_metrics = {0};
static MotorMetrics right_metrics = {0};

// Test results storage
#define MAX_TEST_RESULTS 20
static TestResult test_results[MAX_TEST_RESULTS];
static int test_count = 0;

// Helper: Convert pulses to distance
static float pulses_to_distance_mm(int32_t pulses) {
    float wheel_circumference = WHEEL_DIAMETER_MM * 3.14159f;
    return (pulses / (float)PULSES_PER_REVOLUTION) * wheel_circumference;
}

// Helper: Update motor speed calculation
static void update_motor_speed(MotorMetrics *metrics, int32_t new_count, uint32_t current_time) {
    uint32_t time_diff_ms = current_time - metrics->last_update_time;
    
    if (time_diff_ms >= PID_UPDATE_INTERVAL_MS) {
        int32_t pulse_diff = new_count - metrics->last_count;
        float distance_interval = pulses_to_distance_mm(pulse_diff);
        float time_diff_sec = time_diff_ms / 1000.0f;
        
        metrics->speed_mm_per_sec = distance_interval / time_diff_sec;
        metrics->distance_mm += distance_interval;
        
        metrics->last_count = new_count;
        metrics->last_update_time = current_time;
    }
}

// Helper: Update PID and motors
static void update_pid_control(void) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // Get encoder counts
    int32_t left_count = get_left_encoder();
    int32_t right_count = get_right_encoder();
    
    // Update speed measurements
    update_motor_speed(&left_metrics, left_count, current_time);
    update_motor_speed(&right_metrics, right_count, current_time);
    
    // Compute PID
    float dt = PID_UPDATE_INTERVAL_MS / 1000.0f;
    float left_output = pid_compute(&left_pid, left_metrics.speed_mm_per_sec, dt);
    float right_output = pid_compute(&right_pid, right_metrics.speed_mm_per_sec, dt);
    
    // Apply to motors (inverted)
    motor_drive(M1A, M1B, -(int)left_output);
    motor_drive(M2A, M2B, -(int)right_output);
}

// Helper: Set target speed
static void set_target_speed(float left_speed, float right_speed) {
    pid_set_target(&left_pid, left_speed);
    pid_set_target(&right_pid, right_speed);
}

// Helper: Reset metrics
static void reset_metrics(void) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    left_metrics.last_count = get_left_encoder();
    left_metrics.distance_mm = 0;
    left_metrics.speed_mm_per_sec = 0;
    left_metrics.last_update_time = current_time;
    
    right_metrics.last_count = get_right_encoder();
    right_metrics.distance_mm = 0;
    right_metrics.speed_mm_per_sec = 0;
    right_metrics.last_update_time = current_time;
    
    encoder_reset();
}

void test_init(void) {
    test_count = 0;
    memset(test_results, 0, sizeof(test_results));
    
    // Initialize hardware
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    encoder_init();
    
    // Initialize PID
    pid_init(&left_pid, 0.3, 0.05, 0.01, -100, 100);
    pid_init(&right_pid, 0.3, 0.05, 0.01, -100, 100);
    
    reset_metrics();
    
    printf("\n========================================\n");
    printf("Motor PID Test Suite Initialized\n");
    printf("========================================\n\n");
}

// Helper: Record test result
static void record_test(TestResult result) {
    if (test_count < MAX_TEST_RESULTS) {
        test_results[test_count++] = result;
    }
}

// ====================
// UNIT TESTS
// ====================

TestResult test_encoder_counting(void) {
    TestResult result;
    strcpy(result.test_name, "Encoder Counting");
    
    printf("\n[UNIT TEST] Encoder Counting\n");
    printf("  -> Manually spin wheels for 5 seconds...\n");
    
    encoder_reset();
    sleep_ms(5000);
    
    int left = get_left_encoder();
    int right = get_right_encoder();
    
    result.measured_value = (left + right) / 2.0f;
    result.expected_value = 100;
    result.tolerance = 0;
    
    result.passed = (left > 50 && right > 50);
    
    if (result.passed) {
        snprintf(result.notes, sizeof(result.notes), 
                "Left: %d, Right: %d counts", left, right);
    } else {
        snprintf(result.notes, sizeof(result.notes), 
                "FAIL: Left: %d, Right: %d (too low)", left, right);
    }
    
    printf("  Result: %s\n", result.passed ? "PASS" : "FAIL");
    record_test(result);
    return result;
}

TestResult test_motor_direction(void) {
    TestResult result;
    strcpy(result.test_name, "Motor Direction");
    
    printf("\n[UNIT TEST] Motor Direction\n");
    
    // Test forward
    printf("  -> Testing FORWARD for 2 seconds...\n");
    encoder_reset();
    motor_drive(M1A, M1B, -50);
    motor_drive(M2A, M2B, -50);
    sleep_ms(2000);
    
    int left_fwd = get_left_encoder();
    int right_fwd = get_right_encoder();
    
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    sleep_ms(500);
    
    // Test backward
    printf("  -> Testing BACKWARD for 2 seconds...\n");
    encoder_reset();
    motor_drive(M1A, M1B, 50);
    motor_drive(M2A, M2B, 50);
    sleep_ms(2000);
    
    int left_bwd = get_left_encoder();
    int right_bwd = get_right_encoder();
    
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    result.passed = (left_fwd > 50 && right_fwd > 50 && left_bwd > 50 && right_bwd > 50);
    result.measured_value = (left_fwd + right_fwd) / 2.0f;
    result.expected_value = 100;
    
    snprintf(result.notes, sizeof(result.notes), 
            "FWD: L:%d R:%d, BWD: L:%d R:%d", 
            left_fwd, right_fwd, left_bwd, right_bwd);
    
    printf("  Result: %s\n", result.passed ? "PASS" : "FAIL");
    record_test(result);
    return result;
}

TestResult test_pid_computation(void) {
    TestResult result;
    strcpy(result.test_name, "PID Computation");
    
    printf("\n[UNIT TEST] PID Computation\n");
    
    // Create test PID controller
    PIDController test_pid;
    pid_init(&test_pid, 1.0, 0.1, 0.05, -100, 100);
    pid_set_target(&test_pid, 100);
    
    // Test P term
    float output1 = pid_compute(&test_pid, 50, 0.02);  // Error = 50
    
    // Output should be positive (needs to increase)
    result.passed = (output1 > 40 && output1 < 60);
    result.measured_value = output1;
    result.expected_value = 50;
    result.tolerance = 10;
    
    snprintf(result.notes, sizeof(result.notes), 
            "PID output: %.2f (expected ~50)", output1);
    
    printf("  Result: %s\n", result.passed ? "PASS" : "FAIL");
    record_test(result);
    return result;
}

TestResult test_speed_measurement(void) {
    TestResult result;
    strcpy(result.test_name, "Speed Measurement");
    
    printf("\n[UNIT TEST] Speed Measurement\n");
    printf("  -> Running motor at 50%% for 3 seconds...\n");
    
    reset_metrics();
    motor_drive(M1A, M1B, -50);
    motor_drive(M2A, M2B, -50);
    
    sleep_ms(500);  // Warmup
    
    float speed_sum = 0;
    int samples = 0;
    
    for (int i = 0; i < 50; i++) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        int32_t left_count = get_left_encoder();
        update_motor_speed(&left_metrics, left_count, current_time);
        
        speed_sum += left_metrics.speed_mm_per_sec;
        samples++;
        sleep_ms(50);
    }
    
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    float avg_speed = speed_sum / samples;
    
    result.passed = (avg_speed > 50);
    result.measured_value = avg_speed;
    result.expected_value = 150;
    result.tolerance = 50;
    
    snprintf(result.notes, sizeof(result.notes), 
            "Average speed: %.1f mm/s", avg_speed);
    
    printf("  Result: %s\n", result.passed ? "PASS" : "FAIL");
    record_test(result);
    return result;
}

// ====================
// INTEGRATION TESTS
// ====================

TestResult test_speed_control_steady_state(float target_speed) {
    TestResult result;
    strcpy(result.test_name, "Speed Control Steady State");
    
    printf("\n[INTEGRATION TEST] Speed Control Steady State\n");
    printf("  -> Target: %.1f mm/s\n", target_speed);
    
    reset_metrics();
    set_target_speed(target_speed, target_speed);
    
    // Let it stabilize
    printf("  -> Stabilizing for 3 seconds...\n");
    for (int i = 0; i < 150; i++) {
        update_pid_control();
        sleep_ms(20);
    }
    
    // Measure steady state
    float speed_sum = 0;
    int samples = 0;
    
    printf("  -> Measuring steady state for 2 seconds...\n");
    for (int i = 0; i < 100; i++) {
        update_pid_control();
        float avg_speed = (left_metrics.speed_mm_per_sec + right_metrics.speed_mm_per_sec) / 2.0f;
        speed_sum += avg_speed;
        samples++;
        
        if (i % 10 == 0) {
            printf("    Sample %d: %.1f mm/s\n", i+1, avg_speed);
        }
        sleep_ms(20);
    }
    
    set_target_speed(0, 0);
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    float avg_speed = speed_sum / samples;
    float error_percent = fabs(avg_speed - target_speed) / target_speed * 100.0f;
    
    result.measured_value = avg_speed;
    result.expected_value = target_speed;
    result.tolerance = target_speed * 0.1;
    result.passed = (error_percent < 10.0f);
    
    snprintf(result.notes, sizeof(result.notes), 
            "Avg: %.1f mm/s (Error: %.1f%%)", avg_speed, error_percent);
    
    printf("  Result: %s\n", result.passed ? "PASS" : "FAIL");
    record_test(result);
    return result;
}

TestResult test_speed_response_time(float target_speed, float max_time_ms) {
    TestResult result;
    strcpy(result.test_name, "Speed Response Time");
    
    printf("\n[INTEGRATION TEST] Speed Response Time\n");
    printf("  -> Target: %.1f mm/s, Max time: %.0f ms\n", target_speed, max_time_ms);
    
    reset_metrics();
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    set_target_speed(target_speed, target_speed);
    
    uint32_t response_time = 0;
    bool reached = false;
    
    while (to_ms_since_boot(get_absolute_time()) - start_time < max_time_ms) {
        update_pid_control();
        float avg_speed = (left_metrics.speed_mm_per_sec + right_metrics.speed_mm_per_sec) / 2.0f;
        
        if (!reached && fabs(avg_speed - target_speed) < target_speed * 0.05) {
            response_time = to_ms_since_boot(get_absolute_time()) - start_time;
            reached = true;
            break;
        }
        sleep_ms(20);
    }
    
    set_target_speed(0, 0);
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    result.measured_value = response_time;
    result.expected_value = max_time_ms;
    result.tolerance = max_time_ms * 0.2;
    result.passed = reached && (response_time < max_time_ms);
    
    snprintf(result.notes, sizeof(result.notes), 
            "Response time: %lu ms", response_time);
    
    printf("  Result: %s\n", result.passed ? "PASS" : "FAIL");
    record_test(result);
    return result;
}

TestResult test_speed_accuracy(float target_speed, float tolerance_percent) {
    TestResult result;
    strcpy(result.test_name, "Speed Accuracy");
    
    printf("\n[INTEGRATION TEST] Speed Accuracy\n");
    
    reset_metrics();
    set_target_speed(target_speed, target_speed);
    
    for (int i = 0; i < 100; i++) {
        update_pid_control();
        sleep_ms(20);
    }
    
    float left_speed = left_metrics.speed_mm_per_sec;
    float right_speed = right_metrics.speed_mm_per_sec;
    float avg_speed = (left_speed + right_speed) / 2.0f;
    
    set_target_speed(0, 0);
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    float error_percent = fabs(avg_speed - target_speed) / target_speed * 100.0f;
    
    result.measured_value = avg_speed;
    result.expected_value = target_speed;
    result.tolerance = target_speed * (tolerance_percent / 100.0f);
    result.passed = (error_percent <= tolerance_percent);
    
    snprintf(result.notes, sizeof(result.notes), 
            "L: %.1f, R: %.1f, Avg: %.1f (Error: %.1f%%)", 
            left_speed, right_speed, avg_speed, error_percent);
    
    printf("  Result: %s\n", result.passed ? "PASS" : "FAIL");
    record_test(result);
    return result;
}

TestResult test_distance_accuracy(float target_distance, float tolerance_mm) {
    TestResult result;
    strcpy(result.test_name, "Distance Accuracy");
    
    printf("\n[INTEGRATION TEST] Distance Accuracy\n");
    printf("  -> Target distance: %.1f mm\n", target_distance);
    
    reset_metrics();
    set_target_speed(200, 200);
    
    while (left_metrics.distance_mm < target_distance) {
        update_pid_control();
        sleep_ms(20);
    }
    
    set_target_speed(0, 0);
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    sleep_ms(500);
    
    float left_dist = left_metrics.distance_mm;
    float right_dist = right_metrics.distance_mm;
    float avg_dist = (left_dist + right_dist) / 2.0f;
    
    float error = fabs(avg_dist - target_distance);
    
    result.measured_value = avg_dist;
    result.expected_value = target_distance;
    result.tolerance = tolerance_mm;
    result.passed = (error <= tolerance_mm);
    
    snprintf(result.notes, sizeof(result.notes), 
            "L: %.1f, R: %.1f, Avg: %.1f mm (Error: %.1f mm)", 
            left_dist, right_dist, avg_dist, error);
    
    printf("  Result: %s\n", result.passed ? "PASS" : "FAIL");
    record_test(result);
    return result;
}

TestResult test_motor_synchronization(float target_speed) {
    TestResult result;
    strcpy(result.test_name, "Motor Synchronization");
    
    printf("\n[INTEGRATION TEST] Motor Synchronization\n");
    
    reset_metrics();
    set_target_speed(target_speed, target_speed);
    
    for (int i = 0; i < 150; i++) {
        update_pid_control();
        sleep_ms(20);
    }
    
    float left_speed = left_metrics.speed_mm_per_sec;
    float right_speed = right_metrics.speed_mm_per_sec;
    
    set_target_speed(0, 0);
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    float speed_diff_percent = fabs(left_speed - right_speed) / target_speed * 100.0f;
    
    result.measured_value = speed_diff_percent;
    result.expected_value = 0;
    result.tolerance = 5;
    result.passed = (speed_diff_percent < 5.0f);
    
    snprintf(result.notes, sizeof(result.notes), 
            "L: %.1f, R: %.1f mm/s (Diff: %.1f%%)", 
            left_speed, right_speed, speed_diff_percent);
    
    printf("  Result: %s\n", result.passed ? "PASS" : "FAIL");
    record_test(result);
    return result;
}

TestResult test_pid_stability(float target_speed) {
    TestResult result;
    strcpy(result.test_name, "PID Stability");
    
    printf("\n[INTEGRATION TEST] PID Stability (Oscillation Check)\n");
    
    reset_metrics();
    set_target_speed(target_speed, target_speed);
    
    // Stabilize
    for (int i = 0; i < 100; i++) {
        update_pid_control();
        sleep_ms(20);
    }
    
    // Measure oscillation
    float speeds[20];
    for (int i = 0; i < 20; i++) {
        update_pid_control();
        speeds[i] = left_metrics.speed_mm_per_sec;
        sleep_ms(50);
    }
    
    set_target_speed(0, 0);
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    // Calculate variation
    float max_speed = speeds[0];
    float min_speed = speeds[0];
    for (int i = 1; i < 20; i++) {
        if (speeds[i] > max_speed) max_speed = speeds[i];
        if (speeds[i] < min_speed) min_speed = speeds[i];
    }
    
    float oscillation = max_speed - min_speed;
    float oscillation_percent = oscillation / target_speed * 100.0f;
    
    result.measured_value = oscillation_percent;
    result.expected_value = 0;
    result.tolerance = 15;
    result.passed = (oscillation_percent < 15.0f);
    
    snprintf(result.notes, sizeof(result.notes), 
            "Oscillation: %.1f mm/s (%.1f%%)", oscillation, oscillation_percent);
    
    printf("  Result: %s\n", result.passed ? "PASS" : "FAIL");
    record_test(result);
    return result;
}

// ====================
// TEST SUITE RUNNERS
// ====================

void run_unit_tests(void) {
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║       UNIT TEST SUITE                  ║\n");
    printf("╚════════════════════════════════════════╝\n");
    
    test_encoder_counting();
    test_motor_direction();
    test_pid_computation();
    test_speed_measurement();
}

void run_integration_tests(void) {
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║    INTEGRATION TEST SUITE              ║\n");
    printf("╚════════════════════════════════════════╝\n");
    
    test_speed_control_steady_state(200);
    test_speed_response_time(200, 2000);
    test_speed_accuracy(200, 10);
    test_distance_accuracy(1000, 50);
    test_motor_synchronization(200);
    test_pid_stability(200);
}

void run_all_tests(void) {
    test_init();
    run_unit_tests();
    run_integration_tests();
    print_test_report(test_results, test_count);
}

void print_test_report(TestResult *results, int num_tests) {
    int passed = 0;
    int failed = 0;
    
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════╗\n");
    printf("║                      TEST REPORT                               ║\n");
    printf("╚════════════════════════════════════════════════════════════════╝\n\n");
    
    for (int i = 0; i < num_tests; i++) {
        printf("[%s] %s\n", 
               results[i].passed ? "PASS" : "FAIL", 
               results[i].test_name);
        printf("  Measured: %.2f, Expected: %.2f ±%.2f\n", 
               results[i].measured_value,
               results[i].expected_value,
               results[i].tolerance);
        printf("  Notes: %s\n\n", results[i].notes);
        
        if (results[i].passed) passed++;
        else failed++;
    }
    
    printf("════════════════════════════════════════════════════════════════\n");
    printf("TOTAL: %d tests | PASSED: %d | FAILED: %d\n", 
           num_tests, passed, failed);
    printf("SUCCESS RATE: %.1f%%\n", (passed / (float)num_tests) * 100.0f);
    printf("════════════════════════════════════════════════════════════════\n\n");
    
    if (failed == 0) {
        printf("✅ ALL TESTS PASSED! Motor PID system is ready.\n\n");
    } else {
        printf("❌ SOME TESTS FAILED. Please review and tune PID parameters.\n\n");
    }
}