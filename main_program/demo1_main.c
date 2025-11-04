
#include "pico/stdlib.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "pid.h"
#include "heading_control.h"
#include "telemetry.h"
#include "config.h"
#include "pin_definitions.h"
#include <stdio.h>
#include <math.h>

// Motor deadband - minimum power to overcome friction
#define MOTOR_MIN_POWER 25

typedef struct {
    int32_t last_count;
    int32_t current_count;
    float distance_mm;
    float speed_mm_per_sec;
    uint32_t last_measurement_time;
    float mm_per_pulse;  // ADDED: Each motor has its own conversion factor
} MotorMetrics;

static PIDController left_motor_pid;
static PIDController right_motor_pid;
static HeadingController heading_ctrl;
static IMU imu;
static MotorMetrics left_metrics = {0};
static MotorMetrics right_metrics = {0};

static void init_hardware(void);
static void init_controllers(void);
static void update_motor_speed(MotorMetrics *metrics, int32_t new_count, uint32_t current_time);
static void reset_metrics(void);
static void print_status_line(void);
static void control_loop(void);
static int apply_deadband(float output);

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║          DEMO 1: BASIC MOTION & SENSING INTEGRATION          ║\n");
    printf("║              (FIXED: Separate Encoder Values)                 ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    // IMPORTANT: Set conversion factors for each encoder
    left_metrics.mm_per_pulse = LEFT_MM_PER_PULSE;
    right_metrics.mm_per_pulse = RIGHT_MM_PER_PULSE;
    
    printf("Encoder Configuration:\n");
    printf("  Left:  %d PPR → %.3f mm/pulse\n", LEFT_PULSES_PER_REV, LEFT_MM_PER_PULSE);
    printf("  Right: %d PPR → %.3f mm/pulse\n", RIGHT_PULSES_PER_REV, RIGHT_MM_PER_PULSE);
    printf("\n");
    
    init_hardware();
    init_controllers();
    
    printf("\n⚠️  CALIBRATING IMU - Keep robot STILL for 3 seconds!\n");
    sleep_ms(1000);
    imu_calibrate(&imu);
    
    printf("\nSetting heading reference...\n");
    sleep_ms(500);
    imu_update(&imu);
    float initial_heading = imu_get_heading(&imu);
    heading_control_set_target(&heading_ctrl, initial_heading);
    printf("✓ Initial heading: %.1f°\n", initial_heading);
    
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  Target Speed: %.0f mm/s\n", DEMO1_BASE_SPEED_MM_S);
    printf("  Motor deadband: %d (prevents beeping)\n", MOTOR_MIN_POWER);
    printf("  Robot will drive straight using IMU feedback\n");
    printf("  Press Ctrl+C to stop\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    printf("Starting in 5 seconds...\n");
    sleep_ms(5000);
    
    reset_metrics();
    
    printf("\n");
    printf("Time     | Speed (mm/s)  | Distance (mm) | Heading     | Status\n");
    printf("─────────┼───────────────┼───────────────┼─────────────┼────────\n");
    
    control_loop();
    
    return 0;
}

static void init_hardware(void) {
    printf("Initializing hardware...\n");
    
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    printf("  ✓ Motors (GP%d-%d)\n", M1A, M2B);
    
    encoder_init();
    printf("  ✓ Encoders (GP%d, GP%d)\n", LEFT_ENCODER, RIGHT_ENCODER);
    
    imu_init(&imu);
    
    telemetry_init();
}

static void init_controllers(void) {
    printf("\nInitializing PID controllers...\n");
    
    pid_init(&left_motor_pid, 
             MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, 
             MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);
    
    pid_init(&right_motor_pid, 
             MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, 
             MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);
    
    printf("  ✓ Motor PIDs: Kp=%.2f, Ki=%.3f, Kd=%.3f\n",
           MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD);
    
    heading_control_init(&heading_ctrl,
                        HEADING_PID_KP_DEMO1,
                        HEADING_PID_KI_DEMO1,
                        HEADING_PID_KD_DEMO1,
                        HEADING_CORRECTION_MAX);
    
    printf("Heading PID: Kp=%.2f, Ki=%.3f, Kd=%.3f\n",
           HEADING_PID_KP_DEMO1, HEADING_PID_KI_DEMO1, HEADING_PID_KD_DEMO1);
}

// Apply deadband: if output is too low, set to minimum power
static int apply_deadband(float output) {
    if (output > 0 && output < MOTOR_MIN_POWER) {
        return MOTOR_MIN_POWER;  // Minimum forward power
    } else if (output < 0 && output > -MOTOR_MIN_POWER) {
        return -MOTOR_MIN_POWER;  // Minimum reverse power
    }
    return (int)output;
}

static void control_loop(void) {
    uint32_t last_pid_update = to_ms_since_boot(get_absolute_time());
    uint32_t last_print = last_pid_update;
    uint32_t start_time = last_pid_update;
    
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        imu_update(&imu);
        float current_heading = imu_get_heading(&imu);
        
        int32_t left_count = get_left_encoder();
        int32_t right_count = get_right_encoder();
        
        update_motor_speed(&left_metrics, left_count, current_time);
        update_motor_speed(&right_metrics, right_count, current_time);
        
        if (current_time - last_pid_update >= PID_UPDATE_INTERVAL_MS) {
            float dt = (current_time - last_pid_update) / 1000.0f;
            
            float heading_correction = heading_control_update(&heading_ctrl, current_heading);
            
            // REMOVED motor correction factors - now handled by correct encoder values
            float left_target_speed = DEMO1_BASE_SPEED_MM_S - heading_correction;
            float right_target_speed = DEMO1_BASE_SPEED_MM_S + heading_correction;
            
            pid_set_target(&left_motor_pid, left_target_speed);
            pid_set_target(&right_motor_pid, right_target_speed);
            
            float left_motor_output = pid_compute(&left_motor_pid, 
                                                  left_metrics.speed_mm_per_sec, 
                                                  dt);
            float right_motor_output = pid_compute(&right_motor_pid, 
                                                   right_metrics.speed_mm_per_sec, 
                                                   dt);
            
            // Apply deadband to prevent beeping
            int left_power = apply_deadband(left_motor_output);
            int right_power = apply_deadband(right_motor_output);
            
            // Drive motors (negative because positive was backward)
            motor_drive(M1A, M1B, -left_power);
            motor_drive(M2A, M2B, -right_power);
            
            last_pid_update = current_time;
        }
        
        if (current_time - last_print >= 500) {
            print_status_line();
            last_print = current_time;
        }
        
        sleep_ms(5);
    }
}

static void update_motor_speed(MotorMetrics *metrics, int32_t new_count, uint32_t current_time) {
    metrics->current_count = new_count;
    
    uint32_t time_diff_ms = current_time - metrics->last_measurement_time;
    
    if (time_diff_ms >= PID_UPDATE_INTERVAL_MS) {
        int32_t pulse_diff = metrics->current_count - metrics->last_count;
        
        // FIXED: Use the correct mm_per_pulse for THIS encoder
        float distance_interval = pulse_diff * metrics->mm_per_pulse;
        
        float time_diff_sec = time_diff_ms / 1000.0f;
        metrics->speed_mm_per_sec = distance_interval / time_diff_sec;
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
    // mm_per_pulse already set in main()
    
    right_metrics.last_count = get_right_encoder();
    right_metrics.current_count = right_metrics.last_count;
    right_metrics.distance_mm = 0;
    right_metrics.speed_mm_per_sec = 0;
    right_metrics.last_measurement_time = current_time;
    // mm_per_pulse already set in main()
    
    encoder_reset();
}

static void print_status_line(void) {
    static uint32_t start_time = 0;
    if (start_time == 0) {
        start_time = to_ms_since_boot(get_absolute_time());
    }
    
    uint32_t elapsed_sec = (to_ms_since_boot(get_absolute_time()) - start_time) / 1000;
    
    float avg_distance = (left_metrics.distance_mm + right_metrics.distance_mm) / 2.0f;
    float heading = imu_get_heading(&imu);
    float heading_error = heading_get_error(&heading_ctrl);
    
    printf("%4lu s   | L:%5.0f R:%5.0f | %7.0f mm    | %+6.1f° (%+4.1f°) |",
           elapsed_sec,
           left_metrics.speed_mm_per_sec,
           right_metrics.speed_mm_per_sec,
           avg_distance,
           heading,
           heading_error);
    
    if (heading_ctrl.at_target) {
        printf(" ✓ On track");
    } else {
        if (fabsf(heading_error) > 10.0f) {
            printf(" ⚠ Large error");
        } else {
            printf(" → Correcting");
        }
    }
    
    printf("\n");
}