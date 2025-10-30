#ifndef TEST_MOTOR_PID_H
#define TEST_MOTOR_PID_H

#include "pico/stdlib.h"

// Test result structure
typedef struct {
    char test_name[64];
    bool passed;
    float measured_value;
    float expected_value;
    float tolerance;
    char notes[128];
} TestResult;

// Initialize test system
void test_init(void);

// ====================
// UNIT TESTS (Individual Components)
// ====================

// Test encoder counting
TestResult test_encoder_counting(void);

// Test motor basic operation (forward/backward)
TestResult test_motor_direction(void);

// Test PID computation (mathematical correctness)
TestResult test_pid_computation(void);

// Test motor speed measurement
TestResult test_speed_measurement(void);

// ====================
// INTEGRATION TESTS (System Working Together)
// ====================

// Test 1: Speed Control - Can motor reach and hold target speed?
TestResult test_speed_control_steady_state(float target_speed_mm_s);

// Test 2: Speed Response Time - How fast does it reach target?
TestResult test_speed_response_time(float target_speed_mm_s, float max_time_ms);

// Test 3: Speed Accuracy - How close to target speed?
TestResult test_speed_accuracy(float target_speed_mm_s, float tolerance_percent);

// Test 4: Distance Control - Can it travel exact distance?
TestResult test_distance_accuracy(float target_distance_mm, float tolerance_mm);

// Test 5: Both Motors Match - Do left and right move at same speed?
TestResult test_motor_synchronization(float target_speed_mm_s);

// Test 6: PID Tuning - Is PID stable (no oscillation)?
TestResult test_pid_stability(float target_speed_mm_s);

// ====================
// TEST SUITE RUNNERS
// ====================

// Run all unit tests
void run_unit_tests(void);

// Run all integration tests
void run_integration_tests(void);

// Run complete test suite
void run_all_tests(void);

// Print test report
void print_test_report(TestResult *results, int num_tests);

#endif // TEST_MOTOR_PID_H