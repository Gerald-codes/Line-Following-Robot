/**
 * demo2_main.c
 * Demo 2: Line Following with Barcode Detection
 * Uses single IR sensor edge detection + separate barcode sensor
 */

#include "pico/stdlib.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "ir_sensor.h"
#include "barcode.h"
#include "pid.h"
#include "heading_control.h"
#include "line_following.h"
#include "telemetry.h"
#include "config.h"
#include "pin_definitions.h"
#include <stdio.h>
#include <math.h>

typedef struct {
    int32_t last_count;
    int32_t current_count;
    float distance_mm;
    float speed_mm_per_sec;
    uint32_t last_measurement_time;
} MotorMetrics;

static PIDController left_motor_pid;
static PIDController right_motor_pid;
static HeadingController heading_ctrl;
static IMU imu;
static MotorMetrics left_metrics = {0};
static MotorMetrics right_metrics = {0};
static RobotState robot_state = ROBOT_STATE_IDLE;
static float target_heading = 0.0f;

#define MOTOR_MIN_POWER 25

static void init_hardware(void);
static void init_controllers(void);
static void update_motor_speed(MotorMetrics *metrics, int32_t new_count, uint32_t current_time);
static void reset_metrics(void);
static float pulses_to_distance_mm(int32_t pulses);
static void print_status_line(void);
static void control_loop(void);
static int apply_deadband(float output);
static void execute_barcode_command(BarcodeCommand cmd);
static void state_machine(void);

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║     DEMO 2: LINE FOLLOWING WITH BARCODE DETECTION            ║\n");
    printf("║         Single IR Sensor Edge Detection Method               ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    
    init_hardware();
    init_controllers();
    
    printf("\n⚠️  IR SENSOR CALIBRATION\n");
    printf("This is CRITICAL for edge detection!\n\n");
    ir_calibrate();
    
    printf("\n⚠️  CALIBRATING IMU - Keep robot STILL!\n");
    sleep_ms(1000);
    imu_calibrate(&imu);
    
    printf("\nSetting initial heading...\n");
    sleep_ms(500);
    imu_update(&imu);
    target_heading = imu_get_heading(&imu);
    heading_control_set_target(&heading_ctrl, target_heading);
    printf("✓ Initial heading: %.1f°\n", target_heading);
    
    printf("\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  READY TO START\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  Line Speed: %.0f mm/s\n", DEMO2_BASE_SPEED_MM_S);
    printf("  Turn Speed: %.0f mm/s\n", DEMO2_TURN_SPEED_MM_S);
    printf("\n  Barcode Commands:\n");
    printf("    Short-Short-Long-Long → TURN LEFT\n");
    printf("    Long-Long-Short-Short → TURN RIGHT\n");
    printf("    Short-Long-Short-Long → STOP\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    printf("Place robot on line edge, then press Enter to start...\n");
    getchar();
    
    reset_metrics();
    robot_state = ROBOT_STATE_FOLLOWING;
    
    printf("\n");
    printf("Time | Speed (mm/s) | IR Reading | Line State    | Barcode  | Status\n");
    printf("─────┼──────────────┼────────────┼───────────────┼──────────┼────────\n");
    
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
    
    ir_sensor_init();
    barcode_init();
    
    imu_init(&imu);
    telemetry_init();
}

static void init_controllers(void) {
    printf("\nInitializing controllers...\n");
    
    pid_init(&left_motor_pid, 
             MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, 
             MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);
    
    pid_init(&right_motor_pid, 
             MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, 
             MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);
    
    printf("  ✓ Motor PIDs: Kp=%.2f, Ki=%.3f, Kd=%.3f\n",
           MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD);
    
    heading_control_init(&heading_ctrl,
                        HEADING_PID_KP_DEMO23,
                        HEADING_PID_KI_DEMO23,
                        HEADING_PID_KD_DEMO23,
                        HEADING_CORRECTION_MAX);
    
    printf("  ✓ Heading PID: Kp=%.2f, Ki=%.3f, Kd=%.3f\n",
           HEADING_PID_KP_DEMO23, HEADING_PID_KI_DEMO23, HEADING_PID_KD_DEMO23);
    
    line_following_init();
}

static int apply_deadband(float output) {
    if (output > 0 && output < MOTOR_MIN_POWER) {
        return MOTOR_MIN_POWER;
    } else if (output < 0 && output > -MOTOR_MIN_POWER) {
        return -MOTOR_MIN_POWER;
    }
    return (int)output;
}

static void execute_barcode_command(BarcodeCommand cmd) {
    switch (cmd) {
        case BARCODE_LEFT:
            printf("\n>>> BARCODE: TURN LEFT <<<\n");
            robot_state = ROBOT_STATE_TURNING;
            target_heading -= 90.0f;  // Turn left 90°
            if (target_heading < -180.0f) target_heading += 360.0f;
            heading_control_set_target(&heading_ctrl, target_heading);
            break;
            
        case BARCODE_RIGHT:
            printf("\n>>> BARCODE: TURN RIGHT <<<\n");
            robot_state = ROBOT_STATE_TURNING;
            target_heading += 90.0f;  // Turn right 90°
            if (target_heading > 180.0f) target_heading -= 360.0f;
            heading_control_set_target(&heading_ctrl, target_heading);
            break;
            
        case BARCODE_STOP:
            printf("\n>>> BARCODE: STOP <<<\n");
            robot_state = ROBOT_STATE_STOPPED;
            motor_stop(M1A, M1B);
            motor_stop(M2A, M2B);
            break;
            
        default:
            break;
    }
}

static void state_machine(void) {
    static uint32_t state_start_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    switch (robot_state) {
        case ROBOT_STATE_FOLLOWING:
            line_following_set_state(LINE_STATE_FOLLOWING);
            break;
            
        case ROBOT_STATE_TURNING:
            // Check if turn complete
            if (heading_is_at_target(&heading_ctrl, 5.0f)) {
                printf("Turn complete! Resuming line following...\n");
                robot_state = ROBOT_STATE_FOLLOWING;
                line_following_reset();
            }
            
            // Timeout
            if (state_start_time == 0) {
                state_start_time = current_time;
            } else if (current_time - state_start_time > STATE_TIMEOUT_TURNING_MS) {
                printf("Turn timeout! Resuming line following...\n");
                robot_state = ROBOT_STATE_FOLLOWING;
                state_start_time = 0;
            }
            break;
            
        case ROBOT_STATE_STOPPED:
            // Stay stopped
            break;
            
        default:
            robot_state = ROBOT_STATE_FOLLOWING;
            break;
    }
}

static void control_loop(void) {
    uint32_t last_pid_update = to_ms_since_boot(get_absolute_time());
    uint32_t last_print = last_pid_update;
    
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        // Update sensors
        imu_update(&imu);
        float current_heading = imu_get_heading(&imu);
        
        int32_t left_count = get_left_encoder();
        int32_t right_count = get_right_encoder();
        
        update_motor_speed(&left_metrics, left_count, current_time);
        update_motor_speed(&right_metrics, right_count, current_time);
        
        // Check for barcode
        BarcodeCommand barcode_cmd = barcode_update();
        if (barcode_cmd != BARCODE_NONE && robot_state == ROBOT_STATE_FOLLOWING) {
            execute_barcode_command(barcode_cmd);
            barcode_clear_command();
        }
        
        // Update state machine
        state_machine();
        
        // Control update
        if (current_time - last_pid_update >= PID_UPDATE_INTERVAL_MS) {
            float dt = (current_time - last_pid_update) / 1000.0f;
            
            float base_speed = (robot_state == ROBOT_STATE_TURNING) ? 
                              DEMO2_TURN_SPEED_MM_S : DEMO2_BASE_SPEED_MM_S;
            
            float steering_correction = 0.0f;
            
            if (robot_state == ROBOT_STATE_FOLLOWING) {
                // Line following mode
                steering_correction = line_following_update(dt);
            } else if (robot_state == ROBOT_STATE_TURNING) {
                // Heading control mode
                steering_correction = heading_control_update(&heading_ctrl, current_heading);
            }
            
            if (robot_state != ROBOT_STATE_STOPPED) {
                float left_target_speed = base_speed - steering_correction;
                float right_target_speed = base_speed + steering_correction;
                
                pid_set_target(&left_motor_pid, left_target_speed);
                pid_set_target(&right_motor_pid, right_target_speed);
                
                float left_motor_output = pid_compute(&left_motor_pid, 
                                                      left_metrics.speed_mm_per_sec, dt);
                float right_motor_output = pid_compute(&right_motor_pid, 
                                                       right_metrics.speed_mm_per_sec, dt);
                
                int left_power = apply_deadband(left_motor_output);
                int right_power = apply_deadband(right_motor_output);
                
                motor_drive(M1A, M1B, -left_power);
                motor_drive(M2A, M2B, -right_power);
            }
            
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
        float distance_interval = pulses_to_distance_mm(pulse_diff);
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
    
    right_metrics.last_count = get_right_encoder();
    right_metrics.current_count = right_metrics.last_count;
    right_metrics.distance_mm = 0;
    right_metrics.speed_mm_per_sec = 0;
    right_metrics.last_measurement_time = current_time;
    
    encoder_reset();
}

static float pulses_to_distance_mm(int32_t pulses) {
    return pulses * MM_PER_PULSE;
}

static void print_status_line(void) {
    static uint32_t start_time = 0;
    if (start_time == 0) {
        start_time = to_ms_since_boot(get_absolute_time());
    }
    
    uint32_t elapsed_sec = (to_ms_since_boot(get_absolute_time()) - start_time) / 1000;
    uint16_t ir_reading = ir_read_line_sensor();
    int32_t line_pos = ir_get_line_position();
    LineFollowState line_state = line_following_get_state();
    BarcodeState barcode_state = barcode_get_state();
    
    printf("%3lus | L:%3.0f R:%3.0f | %4d (%+4ld) | %-13s | %-8s | %s\n",
           elapsed_sec,
           left_metrics.speed_mm_per_sec,
           right_metrics.speed_mm_per_sec,
           ir_reading,
           line_pos,
           line_state_to_string(line_state),
           (barcode_state == BARCODE_STATE_DETECTING) ? "Scanning" : "Idle",
           (robot_state == ROBOT_STATE_STOPPED) ? "STOPPED" : 
           (robot_state == ROBOT_STATE_TURNING) ? "TURNING" : "Following");
}
