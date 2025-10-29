#include "pico/stdlib.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include <stdio.h>
#include <math.h>

// Configuration
#define WHEEL_DIAMETER_MM 65.0f
#define PULSES_PER_REVOLUTION 360
#define PID_UPDATE_INTERVAL_MS 20  // Update PID every 20ms (50Hz)

// PID Controllers - one for each motor
PIDController left_motor_pid;
PIDController right_motor_pid;

// Motor metrics
typedef struct {
    int32_t last_count;
    int32_t current_count;
    float distance_mm;
    float speed_mm_per_sec;
    uint32_t last_measurement_time;
} MotorMetrics;

MotorMetrics left_metrics = {0};
MotorMetrics right_metrics = {0};

// Convert pulses to distance
float pulses_to_distance_mm(int32_t pulses) {
    float wheel_circumference = WHEEL_DIAMETER_MM * 3.14159f;
    return (pulses / (float)PULSES_PER_REVOLUTION) * wheel_circumference;
}

// Update motor speed measurement
void update_motor_speed(MotorMetrics *metrics, int32_t new_count, uint32_t current_time) {
    metrics->current_count = new_count;
    
    uint32_t time_diff_ms = current_time - metrics->last_measurement_time;
    
    if (time_diff_ms >= PID_UPDATE_INTERVAL_MS) {
        // Calculate pulse difference
        int32_t pulse_diff = metrics->current_count - metrics->last_count;
        
        // Calculate speed in mm/s
        float distance_interval = pulses_to_distance_mm(pulse_diff);
        float time_diff_sec = time_diff_ms / 1000.0f;
        metrics->speed_mm_per_sec = distance_interval / time_diff_sec;
        
        // Update distance
        metrics->distance_mm += distance_interval;
        
        // Update for next iteration
        metrics->last_count = metrics->current_count;
        metrics->last_measurement_time = current_time;
    }
}

// Reset metrics
void reset_metrics() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    left_metrics.last_count = get_left_encoder();
    left_metrics.current_count = left_metrics.last_count;
    left_metrics.distance_mm = 0;
    left_metrics.speed_mm_per_sec = 0;
    left_metrics.last_measurement_time = current_time;
    
    right_metrics.last_count = get_right_encoder();
    right_metrics.current_count = right_metrics.last_count;
    right_metrics.distance_mm = 0;
    right_metrics.speed_mm_per_sec = 0;
    right_metrics.last_measurement_time = current_time;
    
    encoder_reset();
}

// Print status
void print_status() {
    printf("L: %6.1f mm/s (target: %6.1f) | %7.1f mm  |  ",
           left_metrics.speed_mm_per_sec,
           left_motor_pid.setpoint,
           left_metrics.distance_mm);
    
    printf("R: %6.1f mm/s (target: %6.1f) | %7.1f mm\n",
           right_metrics.speed_mm_per_sec,
           right_motor_pid.setpoint,
           right_metrics.distance_mm);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n========================================\n");
    printf("PID Motor Speed Control\n");
    printf("========================================\n\n");
    
    // Initialize hardware
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    encoder_init();
    
    // Initialize PID controllers for speed control
    // pid_init(pid, Kp, Ki, Kd, output_min, output_max)
    // Start with conservative values - you'll need to tune these!
    pid_init(&left_motor_pid, 0.3, 0.05, 0.01, -100, 100);
    pid_init(&right_motor_pid, 0.3, 0.05, 0.01, -100, 100);
    
    printf("PID Parameters:\n");
    printf("Kp=%.2f, Ki=%.3f, Kd=%.3f\n\n", 
           left_motor_pid.Kp, left_motor_pid.Ki, left_motor_pid.Kd);
    
    // Initialize metrics
    reset_metrics();
    
    printf("Starting PID control tests...\n\n");
    
    uint32_t last_pid_update = to_ms_since_boot(get_absolute_time());
    uint32_t last_print = last_pid_update;
    uint32_t test_start = last_pid_update;
    int test_phase = 0;
    
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        // Get encoder counts
        int32_t left_count = get_left_encoder();
        int32_t right_count = get_right_encoder();
        
        // Update speed measurements
        update_motor_speed(&left_metrics, left_count, current_time);
        update_motor_speed(&right_metrics, right_count, current_time);
        
        // Update PID at fixed interval
        if (current_time - last_pid_update >= PID_UPDATE_INTERVAL_MS) {
            float dt = (current_time - last_pid_update) / 1000.0f;
            
            // Compute PID outputs
            float left_motor_output = pid_compute(&left_motor_pid, 
                                                   left_metrics.speed_mm_per_sec, 
                                                   dt);
            float right_motor_output = pid_compute(&right_motor_pid, 
                                                    right_metrics.speed_mm_per_sec, 
                                                    dt);
            
            // Apply to motors
            motor_drive(M1A, M1B, -(int)left_motor_output);
            motor_drive(M2A, M2B, -(int)right_motor_output);
            
            last_pid_update = current_time;
        }
        
        // Change test phase every 5 seconds
        if (current_time - test_start >= 5000) {
            test_start = current_time;
            test_phase = (test_phase + 1) % 5;
            
            // Reset PIDs when changing targets
            pid_reset(&left_motor_pid);
            pid_reset(&right_motor_pid);
            reset_metrics();
            
            switch(test_phase) {
                case 0:
                    printf("\n=== Test 1: Forward 200 mm/s ===\n");
                    pid_set_target(&left_motor_pid, 200.0f);
                    pid_set_target(&right_motor_pid, 200.0f);
                    break;
                    
                case 1:
                    printf("\n=== Test 2: Forward 400 mm/s ===\n");
                    pid_set_target(&left_motor_pid, 400.0f);
                    pid_set_target(&right_motor_pid, 400.0f);
                    break;
                    
                case 2:
                    printf("\n=== Test 3: Slow Forward 100 mm/s ===\n");
                    pid_set_target(&left_motor_pid, 100.0f);
                    pid_set_target(&right_motor_pid, 100.0f);
                    break;
                    
                case 3:
                    printf("\n=== Test 4: Turn (L:300, R:150) ===\n");
                    pid_set_target(&left_motor_pid, 300.0f);
                    pid_set_target(&right_motor_pid, 150.0f);
                    break;
                    
                case 4:
                    printf("\n=== Test 5: Stop ===\n");
                    pid_set_target(&left_motor_pid, 0.0f);
                    pid_set_target(&right_motor_pid, 0.0f);
                    break;
            }
        }
        
        // Print status every 200ms
        if (current_time - last_print >= 200) {
            print_status();
            last_print = current_time;
        }
        
        sleep_ms(5);
    }
    
    return 0;
}