#include "pico/stdlib.h"
#include "motor.h"
#include "encoder.h"
#include <stdio.h>
#include <math.h>

// Configuration constants - ADJUST THESE FOR YOUR ROBOT
#define WHEEL_DIAMETER_MM 65.0f      // Your wheel diameter in mm
#define PULSES_PER_REVOLUTION 360    // Encoder pulses per wheel revolution
#define MEASUREMENT_INTERVAL_MS 100  // Speed calculation interval

// Structure to track motor metrics
typedef struct {
    int32_t last_count;
    int32_t current_count;
    float distance_mm;
    float speed_mm_per_sec;
    uint32_t last_measurement_time;
} MotorMetrics;

MotorMetrics motor1_metrics = {0};
MotorMetrics motor2_metrics = {0};

// Convert encoder pulses to distance in millimeters
float pulses_to_distance_mm(int32_t pulses) {
    float wheel_circumference = WHEEL_DIAMETER_MM * 3.14159f;
    return (pulses / (float)PULSES_PER_REVOLUTION) * wheel_circumference;
}

// Update metrics for one motor (calculate speed and distance)
void update_motor_metrics(MotorMetrics *metrics, int32_t new_count, uint32_t current_time) {
    metrics->current_count = new_count;
    
    uint32_t time_diff_ms = current_time - metrics->last_measurement_time;
    
    if (time_diff_ms >= MEASUREMENT_INTERVAL_MS) {
        // Calculate how many pulses since last measurement
        int32_t pulse_diff = metrics->current_count - metrics->last_count;
        
        // Calculate distance traveled in this interval
        float distance_interval = pulses_to_distance_mm(pulse_diff);
        metrics->distance_mm += distance_interval;
        
        // Calculate speed in mm/s
        float time_diff_sec = time_diff_ms / 1000.0f;
        metrics->speed_mm_per_sec = distance_interval / time_diff_sec;
        
        // Update for next calculation
        metrics->last_count = metrics->current_count;
        metrics->last_measurement_time = current_time;
    }
}

// Reset all metrics to zero
void reset_metrics() {
    motor1_metrics.last_count = 0;
    motor1_metrics.current_count = 0;
    motor1_metrics.distance_mm = 0;
    motor1_metrics.speed_mm_per_sec = 0;
    motor1_metrics.last_measurement_time = to_ms_since_boot(get_absolute_time());
    
    motor2_metrics = motor1_metrics;
    encoder_reset();
}

// Print current speed and distance for both motors
void print_metrics() {
    printf("LEFT:  %7ld pulses | %7.1f mm | %6.1f mm/s  |  ", 
           motor1_metrics.current_count,
           motor1_metrics.distance_mm,
           motor1_metrics.speed_mm_per_sec);
    
    printf("RIGHT: %7ld pulses | %7.1f mm | %6.1f mm/s\n",
           motor2_metrics.current_count,
           motor2_metrics.distance_mm,
           motor2_metrics.speed_mm_per_sec);
}

int main() {
    stdio_init_all();

    
    // Wait for USB serial connection
    sleep_ms(2000);
    printf("\n========================================\n");
    printf("Motor Speed & Distance Measurement\n");
    printf("========================================\n");
    printf("Wheel diameter: %.1f mm\n", WHEEL_DIAMETER_MM);
    printf("Pulses per revolution: %d\n", PULSES_PER_REVOLUTION);
    printf("========================================\n\n");
    
    // Initialize hardware
    printf("Initializing motors and encoders...\n");
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    encoder_init();
    printf("Ready!\n\n");
    
    // Main loop
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        // Get current encoder counts from encoder.c functions
        int32_t left_count = get_left_encoder();
        int32_t right_count = get_right_encoder();
        
        // Update metrics for both motors
        update_motor_metrics(&motor1_metrics, left_count, current_time);
        update_motor_metrics(&motor2_metrics, right_count, current_time);
        
        // Test sequence controller
        static uint32_t test_start_time = 0;
        static int test_phase = 0;
        static uint32_t phase_duration = 0;
        
        if (test_start_time == 0) {
            test_start_time = current_time;
            phase_duration = current_time;
        }
        
        // Change test phase every 3 seconds
        if (current_time - phase_duration >= 3000) {
            phase_duration = current_time;
            test_phase = (test_phase + 1) % 5;
            
            switch(test_phase) {
                case 0:
                    printf("\n=== FORWARD (Speed 40) ===\n");
                    reset_metrics();
                    motor_drive(M1A, M1B, 40);
                    motor_drive(M2A, M2B, 40);
                    break;
                    
                case 1:
                    printf("\n=== FORWARD FAST (Speed 70) ===\n");
                    reset_metrics();
                    motor_drive(M1A, M1B, 70);
                    motor_drive(M2A, M2B, 70);
                    break;
                    
                case 2:
                    printf("\n=== TURN RIGHT ===\n");
                    reset_metrics();
                    motor_drive(M1A, M1B, 60);
                    motor_drive(M2A, M2B, -60);
                    break;
                    
                case 3:
                    printf("\n=== TURN LEFT ===\n");
                    reset_metrics();
                    motor_drive(M1A, M1B, -60);
                    motor_drive(M2A, M2B, 60);
                    break;
                    
                case 4:
                    printf("\n=== STOPPED ===\n");
                    motor_stop(M1A, M1B);
                    motor_stop(M2A, M2B);
                    printf("Final measurements:\n");
                    print_metrics();
                    break;
            }
        }
        
        // Print metrics every 200ms while moving
        static uint32_t last_print = 0;
        if (test_phase != 4 && current_time - last_print >= 200) {
            print_metrics();
            last_print = current_time;
        }
        
        sleep_ms(10);
    }
    
    return 0;
}