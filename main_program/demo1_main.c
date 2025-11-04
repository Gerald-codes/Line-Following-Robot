/**
 * demo1_main.c - SIMPLE IMU HEADING CORRECTION WITH OPTIMIZED TELEMETRY
 * Version with reduced telemetry frequency for reliability
 */

#include "pico/stdlib.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "pid.h"
#include "heading_control.h"
#include "config.h"
#include "pin_definitions.h"
#include "calibration.h"
#include <stdio.h>
#include <math.h>

// ADD: telemetry
#include "telemetry.h"

// Simple heading correction settings
#define BASE_SPEED_LEFT 42
#define BASE_SPEED_RIGHT 40
#define DEADBAND_DEGREES 3.0f
#define CORRECTION_PER_DEGREE 1.0f
#define MOTOR_MIN_POWER 30

typedef struct {
    int32_t last_count;
    int32_t current_count;
    float distance_mm;
    float speed_mm_per_sec;
    uint32_t last_measurement_time;
    float mm_per_pulse;
} MotorMetrics;

static PIDController left_motor_pid;
static PIDController right_motor_pid;
static HeadingController heading_ctrl;
static IMU imu;
static MotorMetrics left_metrics = {0};
static MotorMetrics right_metrics = {0};

// Telemetry statistics
static struct {
    uint32_t publish_attempts;
    uint32_t publish_success;
    uint32_t publish_failed_not_connected;
    uint32_t publish_failed_error;
} telemetry_stats = {0};

static void init_hardware(void);
static void init_controllers(void);
static void control_loop(void);

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                           DEMO 1                              ║\n");
    printf("║                (Optimized Telemetry Rate)                     ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");

    // Set conversion factors for each encoder
    left_metrics.mm_per_pulse = LEFT_MM_PER_PULSE;
    right_metrics.mm_per_pulse = RIGHT_MM_PER_PULSE;

    printf("Encoder Configuration:\n");
    printf("  Left:  %.0f PPR -> %.3f mm/pulse\n", LEFT_PULSES_PER_REV, LEFT_MM_PER_PULSE);
    printf("  Right: %.0f PPR -> %.3f mm/pulse\n", RIGHT_PULSES_PER_REV, RIGHT_MM_PER_PULSE);
    printf("\n");

    init_hardware();

    // ADD: bring up Wi-Fi and MQTT
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TELEMETRY INITIALIZATION\n");
    printf("═══════════════════════════════════════════════════════════════\n");
    
    TelemetryStatus status = telemetry_begin();
    
    if (status == TELEMETRY_SUCCESS) {
        printf("[TELEMETRY] ✓ Successfully initialized\n");
    } else {
        printf("[TELEMETRY] ⚠ Initialization failed (status=%d)\n", status);
        printf("[TELEMETRY]   Robot will run but data won't be published\n");
    }
    
    // Wait a moment for MQTT connection to establish
    printf("[TELEMETRY] Waiting 2s for MQTT connection...\n");
    sleep_ms(2000);
    
    if (telemetry_is_connected()) {
        printf("[TELEMETRY] ✓ MQTT connected and ready!\n");
    } else {
        printf("[TELEMETRY] ⚠ MQTT not connected yet\n");
        printf("[TELEMETRY]   Check that broker is reachable\n");
    }
    printf("═══════════════════════════════════════════════════════════════\n\n");

    init_controllers();
    control_loop();
    return 0;
}

static void init_hardware(void) {
    printf("Initializing hardware...\n");

    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    printf("  OK Motors\n");

    encoder_init();
    printf("  OK Encoders\n");

    imu_init(&imu);
    printf("  OK IMU\n");

    calibration_init();
    printf("  OK Button (GP20)\n");
}

static void init_controllers(void) {
    printf("\nInitializing PID controllers...\n");

    pid_init(&left_motor_pid,
             MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD,
             MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);

    pid_init(&right_motor_pid,
             MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD,
             MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);

    printf("  OK Motor PIDs: Kp=%.2f, Ki=%.3f, Kd=%.3f\n",
           MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD);

    heading_control_init(&heading_ctrl,
                        HEADING_PID_KP_DEMO1,
                        HEADING_PID_KI_DEMO1,
                        HEADING_PID_KD_DEMO1,
                        HEADING_CORRECTION_MAX);

    printf("  OK Heading PID: Kp=%.2f, Ki=%.3f, Kd=%.3f\n",
           HEADING_PID_KP_DEMO1, HEADING_PID_KI_DEMO1, HEADING_PID_KD_DEMO1);
}

static void control_loop(void) {
    float target_heading = 0.0f;
    bool heading_set = false;

    uint32_t last_print = to_ms_since_boot(get_absolute_time());
    uint32_t start_time = last_print;

    // Calibrate IMU first
    printf("\nCALIBRATING IMU - Keep robot STILL for 1 second!\n");
    sleep_ms(1000);
    imu_calibrate(&imu);
    printf("OK IMU calibrated\n\n");

    printf("===================================================================\n");
    printf("  Simple IMU Heading Correction\n");
    printf("  Base speeds: Left=%d, Right=%d\n", BASE_SPEED_LEFT, BASE_SPEED_RIGHT);
    printf("  Deadband: +/-%.0f degrees\n", DEADBAND_DEGREES);
    printf("  Correction: %.0f power per degree\n", CORRECTION_PER_DEGREE);
    printf("  Telemetry: 1 Hz (slower for reliability)\n");
    printf("===================================================================\n\n");

    printf("PRESS GP20 BUTTON to set heading and start!\n");
    printf("   (Waiting for button press...)\n\n");

    // Wait for button press (15 second timeout)
    uint32_t wait_start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - wait_start < 15000) {
        imu_update(&imu);
        if (calibration_button_pressed()) {
            target_heading = imu_get_heading(&imu);
            heading_set = true;
            printf("OK Heading set to: %.1f deg (by button)\n", target_heading);
            break;
        }
        sleep_ms(10);
    }

    // If no button press, use current heading
    if (!heading_set) {
        imu_update(&imu);
        target_heading = imu_get_heading(&imu);
        printf("OK Heading set to: %.1f deg (auto)\n", target_heading);
    }

    printf("OK Starting in 2 seconds...\n\n");
    sleep_ms(2000);

    // Reset encoders and initialize metrics
    encoder_reset();
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    left_metrics.last_count = 0;
    left_metrics.current_count = 0;
    left_metrics.distance_mm = 0;
    left_metrics.speed_mm_per_sec = 0;
    left_metrics.last_measurement_time = current_time;

    right_metrics.last_count = 0;
    right_metrics.current_count = 0;
    right_metrics.distance_mm = 0;
    right_metrics.speed_mm_per_sec = 0;
    right_metrics.last_measurement_time = current_time;

    printf("Time | Heading | Error | L Pwr | R Pwr | L Speed | R Speed | Distance | Status\n");
    printf("-----+---------+-------+-------+-------+---------+---------+----------+----------------\n");

    start_time = to_ms_since_boot(get_absolute_time());
    last_print = start_time;
    uint32_t last_speed_update = start_time;

    // Telemetry cadence - REDUCED for reliability
    uint32_t last_telemetry = start_time;
    const uint32_t TELEMETRY_PERIOD_MS = 1000; // 1 Hz (was 200ms/5Hz)
    
    uint32_t last_telemetry_status = start_time;
    const uint32_t TELEMETRY_STATUS_PERIOD_MS = 10000; // Print status every 10s

    while (true) {
        current_time = to_ms_since_boot(get_absolute_time());

        // Read current heading
        imu_update(&imu);
        float current_heading = imu_get_heading(&imu);

        // Read encoders
        int32_t left_count = get_left_encoder();
        int32_t right_count = get_right_encoder();

        // Update speed every 100ms
        if (current_time - last_speed_update >= 100) {
            int32_t left_pulse_diff = left_count - left_metrics.last_count;
            float left_distance_diff = left_pulse_diff * left_metrics.mm_per_pulse;
            float time_diff_sec = (current_time - last_speed_update) / 1000.0f;
            left_metrics.speed_mm_per_sec = left_distance_diff / time_diff_sec;
            left_metrics.distance_mm += left_distance_diff;
            left_metrics.last_count = left_count;

            int32_t right_pulse_diff = right_count - right_metrics.last_count;
            float right_distance_diff = right_pulse_diff * right_metrics.mm_per_pulse;
            right_metrics.speed_mm_per_sec = right_distance_diff / time_diff_sec;
            right_metrics.distance_mm += right_distance_diff;
            right_metrics.last_count = right_count;

            last_speed_update = current_time;
        }

        // Calculate error (target - current)
        float error = normalize_angle(target_heading - current_heading);

        // Calculate motor powers
        int left_power = BASE_SPEED_LEFT;
        int right_power = BASE_SPEED_RIGHT;

        if (fabsf(error) > DEADBAND_DEGREES) {
            int correction = (int)(error * CORRECTION_PER_DEGREE * 0.75);
            left_power  = BASE_SPEED_LEFT  + correction;
            right_power = BASE_SPEED_RIGHT - correction;

            if (left_power  > 100) left_power  = 100;
            if (left_power  <   0) left_power  =   0;
            if (right_power > 100) right_power = 100;
            if (right_power <   0) right_power =   0;
        }

        // Drive motors (negative because positive was backward)
        motor_drive(M1A, M1B, -left_power);
        motor_drive(M2A, M2B, -right_power);

        // Print status every 500ms
        if (current_time - last_print >= 500) {
            uint32_t elapsed_sec = (current_time - start_time) / 1000;

            const char *status;
            if (fabsf(error) <= DEADBAND_DEGREES) {
                status = "OK In deadband";
            } else if (error > 0) {
                status = "-> Turning LEFT";
            } else {
                status = "<- Turning RIGHT";
            }

            float avg_distance = (left_metrics.distance_mm + right_metrics.distance_mm) / 2.0f;

            printf("%3lu s | %+6.1f | %+5.1f | %3d   | %3d   | %5.0f   | %5.0f   | %6.0f   | %s\n",
                   elapsed_sec,
                   current_heading,
                   error,
                   left_power,
                   right_power,
                   left_metrics.speed_mm_per_sec,
                   right_metrics.speed_mm_per_sec,
                   avg_distance,
                   status);

            last_print = current_time;
        }

        // ===== Telemetry publish (every 1000 ms = 1 Hz) =====
        if (current_time - last_telemetry >= TELEMETRY_PERIOD_MS) {
            telemetry_stats.publish_attempts++;
            
            // Check connection before publishing
            bool is_connected = telemetry_is_connected();
            
            if (!is_connected) {
                telemetry_stats.publish_failed_not_connected++;
            } else {
                // Publish heading
                float heading_raw = current_heading;
                float heading_ema = current_heading;
                
                TelemetryStatus status = telemetry_publish_heading(heading_raw, heading_ema);
                if (status == TELEMETRY_SUCCESS) {
                    telemetry_stats.publish_success++;
                } else if (status == TELEMETRY_ERROR_NOT_CONNECTED) {
                    telemetry_stats.publish_failed_not_connected++;
                } else {
                    telemetry_stats.publish_failed_error++;
                }
                
                // Small delay between publishes to avoid overwhelming queue
                sleep_ms(50);
                
                // Publish speed
                status = telemetry_publish_speed(
                    left_metrics.speed_mm_per_sec,
                    right_metrics.speed_mm_per_sec
                );
                if (status == TELEMETRY_SUCCESS) {
                    telemetry_stats.publish_success++;
                } else if (status == TELEMETRY_ERROR_NOT_CONNECTED) {
                    telemetry_stats.publish_failed_not_connected++;
                } else {
                    telemetry_stats.publish_failed_error++;
                }
                
                sleep_ms(50);
                
                // Publish distance
                status = telemetry_publish_distance(
                    left_metrics.distance_mm,
                    right_metrics.distance_mm
                );
                if (status == TELEMETRY_SUCCESS) {
                    telemetry_stats.publish_success++;
                } else if (status == TELEMETRY_ERROR_NOT_CONNECTED) {
                    telemetry_stats.publish_failed_not_connected++;
                } else {
                    telemetry_stats.publish_failed_error++;
                }
            }

            last_telemetry = current_time;
        }
        
        // Print telemetry status every 10 seconds
        if (current_time - last_telemetry_status >= TELEMETRY_STATUS_PERIOD_MS) {
            uint32_t total_expected = telemetry_stats.publish_attempts * 3;
            float success_rate = total_expected > 0 ? 
                (telemetry_stats.publish_success * 100.0f / total_expected) : 0.0f;
            
            printf("\n[TELEMETRY] Connected:%s | Attempts=%lu Success=%lu (%.1f%%) Failed(NC)=%lu Failed(Err)=%lu\n\n",
                   telemetry_is_connected() ? "YES" : "NO",
                   telemetry_stats.publish_attempts,
                   telemetry_stats.publish_success,
                   success_rate,
                   telemetry_stats.publish_failed_not_connected,
                   telemetry_stats.publish_failed_error);
            
            last_telemetry_status = current_time;
        }
        // ===========================================

        // Check for button press to stop
        if (calibration_button_pressed()) {
            printf("\nOK Button pressed - STOPPING\n");
            motor_stop(M1A, M1B);
            motor_stop(M2A, M2B);

            float avg_distance = (left_metrics.distance_mm + right_metrics.distance_mm) / 2.0f;
            printf("\nFinal Statistics:\n");
            printf("  Total distance: %.0f mm (%.2f m)\n", avg_distance, avg_distance / 1000.0f);
            printf("  Left distance:  %.0f mm\n", left_metrics.distance_mm);
            printf("  Right distance: %.0f mm\n", right_metrics.distance_mm);
            printf("  Distance diff:  %.0f mm\n", fabsf(left_metrics.distance_mm - right_metrics.distance_mm));
            
            uint32_t total_expected = telemetry_stats.publish_attempts * 3;
            float success_rate = total_expected > 0 ? 
                (telemetry_stats.publish_success * 100.0f / total_expected) : 0.0f;
            
            printf("\nTelemetry Statistics:\n");
            printf("  Publish cycles:         %lu\n", telemetry_stats.publish_attempts);
            printf("  Total publishes expected: %lu (3 per cycle)\n", total_expected);
            printf("  Successful publishes:   %lu (%.1f%%)\n", 
                   telemetry_stats.publish_success, success_rate);
            printf("  Failed (not connected): %lu\n", telemetry_stats.publish_failed_not_connected);
            printf("  Failed (other errors):  %lu\n", telemetry_stats.publish_failed_error);
            printf("  Connection status:      %s\n", telemetry_is_connected() ? "CONNECTED" : "DISCONNECTED");

            break;
        }

        sleep_ms(10);
    }

    printf("\nProgram ended.\n");
}
