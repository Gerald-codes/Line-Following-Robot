/**
 * demo2_main.c
 * SIMPLIFIED line following with single PID controller
 * - Clean, simple control loop
 * - Motor speed PIDs for smooth control
 * - Easy to tune with config.h
 */

#include "pico/stdlib.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "ir_sensor.h"
#include "line_following.h"
#include "pid.h"
#include "config.h"
#include "pin_definitions.h"
#include "calibration.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// Motor speed tracking
typedef struct {
    int32_t last_count;
    int32_t current_count;
    float distance_mm;
    float speed_mm_per_sec;
    uint32_t last_measurement_time;
} MotorMetrics;

static PIDController left_motor_pid;
static PIDController right_motor_pid;
static IMU imu;
static MotorMetrics left_metrics = {0};
static MotorMetrics right_metrics = {0};

#define MOTOR_MIN_POWER 30

// Function prototypes
static void init_hardware(void);
static void update_motor_speed(MotorMetrics *metrics, int32_t new_count, uint32_t current_time);
static void reset_metrics(void);
static float pulses_to_distance_mm(int32_t pulses);
static int apply_deadband(float output);

float demo2_base_speed_mm_s = 55.0f;  // Base forward speed

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n");
    printf("========================================\n");
    printf("  DEMO 2: Simple PID Line Following\n");
    printf("========================================\n\n");
    
    init_hardware();
    
    // Calibration sequence
    printf("Press button for calibration (or wait 5s)...\n");
    
    uint32_t cal_start = to_ms_since_boot(get_absolute_time());
    bool do_cal = false;
    while (to_ms_since_boot(get_absolute_time()) - cal_start < 5000) {
        if (calibration_button_pressed()) {
            do_cal = true;
            break;
        }
        sleep_ms(10);
    }
    
    if (do_cal) {
        calibration_run_sequence();
        printf("\nPress button to continue...\n");
        while (!calibration_button_pressed()) sleep_ms(10);
    }
    
    // IMU calibration
    printf("\nCalibrating IMU...\n");
    imu_calibrate(&imu);
    
    // Initialize motor PIDs
    pid_init(&left_motor_pid, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, 
             MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);
    pid_init(&right_motor_pid, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, 
             MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);
    
    // Initialize line following controller
    line_following_init();
    
    printf("\n");
    printf("========================================\n");
    printf("  READY TO START\n");
    printf("========================================\n");
    printf("  Base speed: %.0f mm/s\n", demo2_base_speed_mm_s);
    printf("  Line PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", 
           LINE_PID_KP, LINE_PID_KI, LINE_PID_KD);
    printf("  Motor PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", 
           MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD);
    printf("========================================\n\n");
    
    printf("Place robot on line edge, then press button...\n");
    while (!calibration_button_pressed()) sleep_ms(10);
    
    reset_metrics();
    
    printf("\nSTARTING LINE FOLLOWING!\n\n");
    printf("Time | State      | L_tgt | R_tgt | L_act | R_act | IR   | Pos\n");
    printf("-----+------------+-------+-------+-------+-------+------+------\n");
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_print = start_time;
    uint32_t last_pid = start_time;
    
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        // Read encoders
        int32_t left_count = get_left_encoder();
        int32_t right_count = get_right_encoder();
        
        // Update speed measurements
        update_motor_speed(&left_metrics, left_count, current_time);
        update_motor_speed(&right_metrics, right_count, current_time);
        
        // PID update every 100ms
        if (current_time - last_pid >= 100) {
            float dt = (current_time - last_pid) / 1000.0f;
            
            // Get steering correction from line following PID
            float steering = line_following_update(dt);
            
            // Calculate target speeds: base speed ± steering
            float left_target_speed = demo2_base_speed_mm_s - steering;
            float right_target_speed = demo2_base_speed_mm_s + steering;
            
            // Clamp targets to reasonable range
            if (left_target_speed < 0) left_target_speed = 0;
            if (right_target_speed < 0) right_target_speed = 0;
            if (left_target_speed > 200) left_target_speed = 200;
            if (right_target_speed > 200) right_target_speed = 200;
            
            // Motor speed PIDs - compute motor power needed
            pid_set_target(&left_motor_pid, left_target_speed);
            pid_set_target(&right_motor_pid, right_target_speed);
            
            float left_motor_output = pid_compute(&left_motor_pid, 
                                                  left_metrics.speed_mm_per_sec, dt);
            float right_motor_output = pid_compute(&right_motor_pid, 
                                                   right_metrics.speed_mm_per_sec, dt);
            
            int left_power = apply_deadband(left_motor_output);
            int right_power = apply_deadband(right_motor_output);
            
            // Apply to motors (M1=RIGHT, M2=LEFT, negative=forward)
            motor_drive(M1A, M1B, -right_power);
            motor_drive(M2A, M2B, -(left_power + LEFT_MOTOR_OFFSET));
            
            // Print status every 500ms
            if (current_time - last_print >= 500) {
                uint16_t ir_reading = ir_read_line_sensor();
                int32_t position = ir_get_line_position();
                LineFollowState state = line_following_get_state();
                
                printf("%4lus | %-10s | %5.0f | %5.0f | %5.0f | %5.0f | %4d | %+5ld\n",
                       (current_time - start_time) / 1000,
                       line_state_to_string(state),
                       left_target_speed,
                       right_target_speed,
                       left_metrics.speed_mm_per_sec,
                       right_metrics.speed_mm_per_sec,
                       ir_reading,
                       position);
                
                last_print = current_time;
            }
            
            last_pid = current_time;
        }
        
        sleep_ms(5);
    }
    
    return 0;
}

static void init_hardware(void) {
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    encoder_init();
    ir_sensor_init();
    calibration_init();
    imu_init(&imu);
    printf("Hardware initialized\n");
}

static void update_motor_speed(MotorMetrics *metrics, int32_t new_count, uint32_t current_time) {
    metrics->current_count = new_count;
    uint32_t time_diff_ms = current_time - metrics->last_measurement_time;
    
    if (time_diff_ms >= 100) {
        int32_t pulse_diff = metrics->current_count - metrics->last_count;
        float distance_interval = pulses_to_distance_mm(pulse_diff);
        float time_diff_sec = time_diff_ms / 1000.0f;
        float new_speed = distance_interval / time_diff_sec;
        
        // Smooth speed measurement with low-pass filter
        metrics->speed_mm_per_sec = 0.8f * metrics->speed_mm_per_sec + 0.2f * new_speed;
        
        metrics->distance_mm += distance_interval;
        metrics->last_count = metrics->current_count;
        metrics->last_measurement_time = current_time;
    }
}

static void reset_metrics(void) {
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

static float pulses_to_distance_mm(int32_t pulses) {
    // Use average of left and right
    float avg_mm_per_pulse = (LEFT_MM_PER_PULSE + RIGHT_MM_PER_PULSE) / 2.0f;
    return (pulses * avg_mm_per_pulse) / 4.0f;
}

static int apply_deadband(float output) {
    if (output > 0 && output < MOTOR_MIN_POWER) {
        return MOTOR_MIN_POWER;
    } else if (output < 0 && output > -MOTOR_MIN_POWER) {
        return -MOTOR_MIN_POWER;
    }
    return (int)output;
}