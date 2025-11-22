/**
 * integrated_main.c
 * 
 * INTEGRATED LINE FOLLOWING + OBSTACLE AVOIDANCE
 * 
 * Features:
 * - Line following using single IR sensor
 * - Obstacle detection with ultrasonic sensor
 * - Obstacle avoidance maneuvers
 * - State machine coordination
 * - Ready for barcode integration (future)
 * 
 * States:
 * 1. LINE_FOLLOWING - Normal line tracking
 * 2. OBSTACLE_DETECTED - Obstacle spotted, stop and scan
 * 3. OBSTACLE_AVOIDING - Execute avoidance maneuver
 * 4. RETURNING_TO_LINE - Getting back to line
 * 5. LINE_LOST - Line lost, search mode
 */

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

// Hardware drivers
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "ir_sensor.h"
#include "ultrasonic.h"
#include "servo.h"

// Control algorithms
#include "line_following.h"
#include "obstacle_scanner.h"
#include "avoidance_maneuver.h"

// Utilities
#include "timer_manager.h"
#include "telemetry.h"
#include "calibration.h"
#include "config.h"
#include "pin_definitions.h"

// ============================================================================
// SYSTEM STATE MACHINE
// ============================================================================

typedef enum {
    STATE_IDLE,
    STATE_LINE_FOLLOWING,
    STATE_OBSTACLE_DETECTED,
    STATE_OBSTACLE_SCANNING,
    STATE_OBSTACLE_AVOIDING,
    STATE_RETURNING_TO_LINE,
    STATE_LINE_LOST,
    STATE_STOPPED
} SystemState;

static SystemState current_state = STATE_IDLE;
static SystemState previous_state = STATE_IDLE;

// ============================================================================
// CONFIGURATION
// ============================================================================

// Line following speeds
#define BASE_POWER 40.0f
#define MIN_POWER 25.0f
#define MAX_POWER 60.0f

// Gentle curve detection
#define WHITE_THRESHOLD 300
#define WHITE_DURATION_MS 400
static bool gentle_curve_mode = false;
static uint32_t white_start_time = 0;
static uint32_t gentle_mode_start_time = 0;

// Obstacle detection
#define OBSTACLE_CHECK_DISTANCE_CM 20  // Check for obstacles at 20cm
#define OBSTACLE_CHECK_INTERVAL_MS 500 // Check every 500ms
#define CRITICAL_DISTANCE_CM 15        // Stop if obstacle within 15cm

// Adaptive PID gains
typedef struct {
    float kp;
    float ki;
    float kd;
} PIDGainSet;

static const PIDGainSet SMOOTH_GAINS = {
    .kp = 1.5f,
    .ki = 0.0f,
    .kd = 0.8f
};

static const PIDGainSet AGGRESSIVE_GAINS = {
    .kp = 3.0f,
    .ki = 0.0f,
    .kd = 2.5f
};

#define SMOOTH_ERROR_THRESHOLD 0.2f
#define AGGRESSIVE_ERROR_THRESHOLD 0.6f

// ============================================================================
// GLOBAL STATE
// ============================================================================

static IMU imu;
static float L_power = 0.0f;
static float R_power = 0.0f;
static uint32_t last_obstacle_check = 0;
static uint32_t state_entry_time = 0;

// Line lost detection
static uint32_t line_lost_start = 0;
#define LINE_LOST_TIMEOUT_MS 2000

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

static void init_hardware(void);
static void update_line_following(uint32_t current_time, float dt);
static bool check_for_obstacles(void);
static void handle_obstacle_detected(void);
static void handle_obstacle_avoidance(void);
static void handle_line_lost(void);
static void apply_adaptive_gains(float error_magnitude, bool in_gentle_curve);
static PIDGainSet interpolate_gains(float error_magnitude);
static void update_gentle_curve_mode(uint16_t ir_reading, uint32_t current_time);
static int apply_deadband(float power);
static void change_state(SystemState new_state);
static const char* state_to_string(SystemState state);

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================

static void init_hardware(void) {
    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║     INTEGRATED LINE FOLLOWING + OBSTACLE AVOIDANCE           ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    printf("Initializing hardware...\n");
    
    // Motors
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    printf("  ✓ Motors\n");
    
    // Encoders
    encoder_init();
    printf("  ✓ Encoders\n");
    
    // IMU (struct-based for line following)
    imu_init(&imu);
    imu_calibrate(&imu);
    printf("  ✓ IMU (struct)\n");
    
    // IMU Helper (for avoidance maneuvers)
    imu_helper_init();
    printf("  ✓ IMU Helper\n");
    
    // IR Sensors
    ir_sensor_init();
    printf("  ✓ IR Sensors\n");
    
    // Ultrasonic
    ultrasonic_init(TRIG_PIN, ECHO_PIN);
    printf("  ✓ Ultrasonic\n");
    
    // Servo
    servo_init(SERVO_PIN);
    servo_set_angle(ANGLE_CENTER);
    printf("  ✓ Servo\n");
    
    // Calibration button
    calibration_init();
    printf("  ✓ Calibration Button (GP20)\n");
    
    // Line following controller
    line_following_init();
    printf("  ✓ Line Following Controller\n");
    
    // Obstacle systems
    scanner_init();
    avoidance_init();
    timer_manager_init();
    printf("  ✓ Obstacle Detection Systems\n");
    
    printf("\n✓ All hardware initialized\n\n");
}

// ============================================================================
// STATE MANAGEMENT
// ============================================================================

static void change_state(SystemState new_state) {
    if (new_state != current_state) {
        previous_state = current_state;
        current_state = new_state;
        state_entry_time = to_ms_since_boot(get_absolute_time());
        
        printf("\n>>> STATE CHANGE: %s -> %s\n", 
               state_to_string(previous_state),
               state_to_string(new_state));
        
        // Publish state change to telemetry
        if (telemetry_is_connected()) {
            char msg[64];
            snprintf(msg, sizeof(msg), "State: %s", state_to_string(new_state));
            telemetry_publish_status(msg);
        }
    }
}

static const char* state_to_string(SystemState state) {
    switch (state) {
        case STATE_IDLE: return "IDLE";
        case STATE_LINE_FOLLOWING: return "LINE_FOLLOWING";
        case STATE_OBSTACLE_DETECTED: return "OBSTACLE_DETECTED";
        case STATE_OBSTACLE_SCANNING: return "OBSTACLE_SCANNING";
        case STATE_OBSTACLE_AVOIDING: return "OBSTACLE_AVOIDING";
        case STATE_RETURNING_TO_LINE: return "RETURNING_TO_LINE";
        case STATE_LINE_LOST: return "LINE_LOST";
        case STATE_STOPPED: return "STOPPED";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

static int apply_deadband(float power) {
    int int_power = (int)power;
    
    if (int_power > 0 && int_power < MIN_POWER) {
        return MIN_POWER;
    } else if (int_power < 0 && int_power > -MIN_POWER) {
        return -MIN_POWER;
    }
    
    if (int_power > MAX_POWER) return MAX_POWER;
    if (int_power < -MAX_POWER) return -MAX_POWER;
    
    return int_power;
}

// ============================================================================
// ADAPTIVE GAIN INTERPOLATION
// ============================================================================

static PIDGainSet interpolate_gains(float error_magnitude) {
    PIDGainSet result;
    
    if (error_magnitude < SMOOTH_ERROR_THRESHOLD) {
        result = SMOOTH_GAINS;
    }
    else if (error_magnitude > AGGRESSIVE_ERROR_THRESHOLD) {
        result = AGGRESSIVE_GAINS;
    }
    else {
        // Linear interpolation
        float blend = (error_magnitude - SMOOTH_ERROR_THRESHOLD) / 
                     (AGGRESSIVE_ERROR_THRESHOLD - SMOOTH_ERROR_THRESHOLD);
        
        result.kp = SMOOTH_GAINS.kp + blend * (AGGRESSIVE_GAINS.kp - SMOOTH_GAINS.kp);
        result.ki = SMOOTH_GAINS.ki + blend * (AGGRESSIVE_GAINS.ki - SMOOTH_GAINS.ki);
        result.kd = SMOOTH_GAINS.kd + blend * (AGGRESSIVE_GAINS.kd - SMOOTH_GAINS.kd);
    }
    
    return result;
}

// ============================================================================
// GENTLE CURVE MODE DETECTION
// ============================================================================

static void update_gentle_curve_mode(uint16_t ir_reading, uint32_t current_time) {
    bool on_white = (ir_reading < WHITE_THRESHOLD);
    
    if (on_white) {
        if (white_start_time == 0) {
            white_start_time = current_time;
            printf("⏱️  White surface detected (IR=%d)\n", ir_reading);
        }
        
        uint32_t white_duration = current_time - white_start_time;
        
        if (!gentle_curve_mode && white_duration >= WHITE_DURATION_MS) {
            gentle_curve_mode = true;
            gentle_mode_start_time = current_time;
            printf("🌊 GENTLE CURVE MODE activated\n");
        }
        
        // Timeout after 5 seconds
        if (gentle_curve_mode && (current_time - gentle_mode_start_time > 5000)) {
            gentle_curve_mode = false;
            white_start_time = 0;
            printf("⚠️  GENTLE CURVE MODE timeout\n");
        }
    } else {
        // Back on dark surface
        if (ir_reading > (WHITE_THRESHOLD + 150)) {
            if (gentle_curve_mode) {
                gentle_curve_mode = false;
                printf("✓ GENTLE CURVE MODE exit (IR=%d)\n\n", ir_reading);
            }
            white_start_time = 0;
        }
    }
}

// ============================================================================
// ADAPTIVE GAINS WITH TRANSITION DETECTION
// ============================================================================

static void apply_adaptive_gains(float error_magnitude, bool in_gentle_curve) {
    // Track SIGNED error for better transition detection
    static float prev_error = 0.0f;
    float current_error = line_following_get_error();
    
    // Calculate absolute change in error
    float error_change = fabsf(current_error - prev_error);
    float error_change_rate = error_change / 0.035f;  // Approximate dt
    
    // Only print when actually triggering (not every cycle)
    static bool was_in_transition = false;
    bool in_transition = (error_change_rate > 15.0f);
    
    if (in_transition && !was_in_transition) {
        printf("⚡ RAPID TRANSITION! rate=%.1f/s\n", error_change_rate);
    }
    was_in_transition = in_transition;
    
    if (in_gentle_curve) {
        // Gentle curve gains - smoother response
        line_following_set_kp(2.5f);
        line_following_set_ki(0.0f);
        line_following_set_kd(1.5f);
    } 
    else if (in_transition) {  
        // RAPID TRANSITION: Reduce D, boost P
        PIDGainSet gains = interpolate_gains(error_magnitude);
        line_following_set_kp(gains.kp * 1.4f);
        line_following_set_ki(gains.ki);
        line_following_set_kd(gains.kd * 0.25f);
    }
    else {
        // Normal adaptive gains
        PIDGainSet gains = interpolate_gains(error_magnitude);
        line_following_set_kp(gains.kp);
        line_following_set_ki(gains.ki);
        line_following_set_kd(gains.kd);
    }
    
    prev_error = current_error;
}

// ============================================================================
// LINE FOLLOWING UPDATE
// ============================================================================

static void update_line_following(uint32_t current_time, float dt) {
    // Read IR sensor
    uint16_t ir_reading = ir_read_line_sensor();
    
    // Update gentle curve detection
    update_gentle_curve_mode(ir_reading, current_time);
    
    // Update line following PID
    float steering = line_following_update(dt);
    float error = line_following_get_error();
    float error_magnitude = fabsf(error);
    
    // Apply adaptive gains (with gentle curve awareness)
    apply_adaptive_gains(error_magnitude, gentle_curve_mode);
    
    // Adaptive base power - slow down for large errors
    float base = BASE_POWER - (15.0f * error_magnitude);
    if (base < MIN_POWER) base = MIN_POWER;
    if (base > MAX_POWER) base = MAX_POWER;
    
    // Scale steering based on mode
    float steering_multiplier = 8.0f;
    if (gentle_curve_mode) {
        steering_multiplier = 10.0f;
        base = 35.0f;  // Slower in gentle curve mode
    }
    
    float steering_power = steering * steering_multiplier;
    
    // Adaptive smoothing filter for steering
    static float filtered_steering = 0.0f;
    static LineFollowState prev_lf_state = LINE_FOLLOW_CENTERED;
    
    LineFollowState current_lf_state = line_following_get_state();
    
    // Detect rapid state transition
    bool rapid_transition = false;
    if ((prev_lf_state == LINE_FOLLOW_FAR_LEFT && current_lf_state == LINE_FOLLOW_FAR_RIGHT) ||
        (prev_lf_state == LINE_FOLLOW_FAR_RIGHT && current_lf_state == LINE_FOLLOW_FAR_LEFT)) {
        rapid_transition = true;
        printf("⚡ STATE FLIP! %s→%s\n", 
               line_state_to_string(prev_lf_state),
               line_state_to_string(current_lf_state));
    }
    
    // Adaptive filter coefficient
    float filter_alpha;
    if (rapid_transition) {
        // RESET filter on rapid transition
        filtered_steering = steering_power * 0.4f;
        filter_alpha = 0.2f;
        line_following_reset_integral();
    } else if (error_magnitude > 0.7f) {
        filter_alpha = 0.4f;  // Large error - respond fast
    } else if (gentle_curve_mode) {
        filter_alpha = 0.3f;  // Gentle curve - moderate response
    } else {
        filter_alpha = 0.75f;  // Normal filtering
    }
    
    filtered_steering = filter_alpha * filtered_steering + 
                       (1.0f - filter_alpha) * steering_power;
    steering_power = filtered_steering;
    
    prev_lf_state = current_lf_state;
    
    // Calculate motor powers
    float L_target = base + steering_power;
    float R_target = base - steering_power;
    
    // Smooth ramping
    float max_step = 12.0f;
    float dL = L_target - L_power;
    float dR = R_target - R_power;
    if (dL > max_step) dL = max_step;
    else if (dL < -max_step) dL = -max_step;
    if (dR > max_step) dR = max_step;
    else if (dR < -max_step) dR = -max_step;
    
    L_power += dL;
    R_power += dR;
    
    // Apply deadband and limits
    int left_motor = apply_deadband(L_power);
    int right_motor = apply_deadband(R_power);
    
    // Drive motors
    motor_drive(M1A, M1B, -left_motor);
    motor_drive(M2A, M2B, -right_motor);
    
    // Debug output every 500ms
    static uint32_t last_debug = 0;
    if (current_time - last_debug >= 500) {
        const char* mode_str = gentle_curve_mode ? "GENTLE" : "NORMAL";
        printf("[LINE] %s | Error: %+.2f | Steer: %+.2f | L:%d R:%d | IR:%d | %s\n",
               mode_str, error, steering_power, left_motor, right_motor, ir_reading,
               line_state_to_string(line_following_get_state()));
        last_debug = current_time;
    }
    
    // Check if line is lost
    LineFollowState line_state = line_following_get_state();
    if (line_state == LINE_FOLLOW_LOST) {
        if (line_lost_start == 0) {
            line_lost_start = current_time;
        } else if (current_time - line_lost_start > LINE_LOST_TIMEOUT_MS) {
            change_state(STATE_LINE_LOST);
            line_lost_start = 0;
        }
    } else {
        line_lost_start = 0;  // Reset if line is found
    }
}

// ============================================================================
// OBSTACLE DETECTION
// ============================================================================

static bool check_for_obstacles(void) {
    // Point servo forward
    servo_set_angle(ANGLE_CENTER);
    sleep_ms(100);
    
    // Read distance
    uint64_t distance;
    int status = ultrasonic_get_distance(TRIG_PIN, ECHO_PIN, &distance);
    
    if (status == SUCCESS) {
        printf("[OBSTACLE] Forward distance: %llu cm", distance);
        
        if (distance <= OBSTACLE_CHECK_DISTANCE_CM) {
            printf(" ⚠️  OBSTACLE DETECTED!\n");
            return true;
        } else {
            printf(" ✓\n");
            return false;
        }
    } else {
        printf("[OBSTACLE] Sensor error\n");
        return false;
    }
}

// ============================================================================
// OBSTACLE HANDLING
// ============================================================================

static void handle_obstacle_detected(void) {
    printf("\n[OBSTACLE] Obstacle detected - stopping motors\n");
    
    // Stop motors
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    // Move to scanning state
    change_state(STATE_OBSTACLE_SCANNING);
}

static void handle_obstacle_scanning(void) {
    printf("\n[OBSTACLE] Performing scan...\n");
    
    // Perform scan
    ScanResult result = scanner_perform_scan();
    scanner_print_results(result);
    
    // Determine avoidance direction
    AvoidanceDirection direction = scanner_get_best_avoidance_direction(result);
    
    if (direction == AVOID_NONE) {
        printf("[OBSTACLE] No clear path - stopping\n");
        change_state(STATE_STOPPED);
        return;
    }
    
    // Start avoidance maneuver
    if (avoidance_start(direction)) {
        change_state(STATE_OBSTACLE_AVOIDING);
    } else {
        printf("[OBSTACLE] Failed to start avoidance\n");
        change_state(STATE_STOPPED);
    }
}

static void handle_obstacle_avoidance(void) {
    // Update avoidance maneuver
    AvoidanceState avoid_state = avoidance_update();
    
    // Check if complete
    if (avoidance_is_complete()) {
        if (avoidance_was_successful()) {
            printf("\n[OBSTACLE] Avoidance complete - returning to line following\n");
            avoidance_reset();
            change_state(STATE_RETURNING_TO_LINE);
        } else {
            printf("\n[OBSTACLE] Avoidance failed\n");
            change_state(STATE_STOPPED);
        }
    }
}

static void handle_returning_to_line(void) {
    // After avoidance, we should be near the line
    // Use line following logic but with more tolerance
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    static uint32_t last_update = 0;
    
    float dt = (current_time - last_update) / 1000.0f;
    if (dt > 0.1f) dt = 0.02f;  // Clamp to 20ms
    last_update = current_time;
    
    // Check if we found the line
    if (ir_line_detected()) {
        printf("\n[LINE] Line detected! Resuming normal following\n");
        change_state(STATE_LINE_FOLLOWING);
        return;
    }
    
    // Continue moving forward slowly to find line
    motor_drive(M1A, M1B, -30);
    motor_drive(M2A, M2B, -30);
    
    // Timeout after 3 seconds
    if (current_time - state_entry_time > 3000) {
        printf("\n[LINE] Could not find line - entering search mode\n");
        change_state(STATE_LINE_LOST);
    }
}

// ============================================================================
// LINE LOST HANDLING
// ============================================================================

static void handle_line_lost(void) {
    printf("[LINE] Line lost - searching...\n");
    
    // Simple search: move forward slowly
    motor_drive(M1A, M1B, -25);
    motor_drive(M2A, M2B, -25);
    
    // Check if line is found
    if (ir_line_detected()) {
        printf("[LINE] ✓ Line found!\n");
        change_state(STATE_LINE_FOLLOWING);
        return;
    }
    
    // Timeout after 5 seconds
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - state_entry_time > 5000) {
        printf("[LINE] Search timeout - stopping\n");
        change_state(STATE_STOPPED);
    }
}

// ============================================================================
// MAIN PROGRAM
// ============================================================================

int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(2000);
    
    // Initialize hardware
    init_hardware();
    
    // Optional: Initialize telemetry (comment out if not using)
    // printf("\nInitializing telemetry...\n");
    // if (telemetry_init(MQTT_BROKER_ADDRESS, MQTT_CLIENT_ID) == TELEMETRY_SUCCESS) {
    //     printf("✓ Telemetry connected\n");
    //     scanner_enable_telemetry();
    // }
    
    // Run calibration
    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf("  IR SENSOR CALIBRATION\n");
    printf("  Press GP20 to start calibration\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    while (!calibration_button_pressed()) {
        sleep_ms(10);
    }
    
    calibration_run_sequence();
    
    // Wait for start button
    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf("  Press GP20 to start robot\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    while (!calibration_button_pressed()) {
        sleep_ms(10);
    }
    sleep_ms(200);  // Debounce
    
    // Start in line following mode
    change_state(STATE_LINE_FOLLOWING);
    printf("\n>>> ROBOT STARTED <<<\n\n");
    
    // Main loop
    uint32_t last_update = to_ms_since_boot(get_absolute_time());
    bool running = true;
    
    while (running) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        float dt = (current_time - last_update) / 1000.0f;
        
        // Clamp dt to reasonable range
        if (dt > 0.1f) dt = 0.02f;
        if (dt < 0.001f) dt = 0.001f;
        last_update = current_time;
        
        // Update IMU (both interfaces)
        imu_update(&imu);          // Struct-based for line following
        imu_helper_update();       // Helper for obstacle avoidance
        
        // Process telemetry
        if (telemetry_is_connected()) {
            telemetry_process();
        }
        
        // State machine
        switch (current_state) {
            case STATE_IDLE:
                // Do nothing
                break;
                
            case STATE_LINE_FOLLOWING:
                // Update line following
                update_line_following(current_time, dt);
                
                // Periodically check for obstacles
                if (current_time - last_obstacle_check >= OBSTACLE_CHECK_INTERVAL_MS) {
                    if (check_for_obstacles()) {
                        handle_obstacle_detected();
                    }
                    last_obstacle_check = current_time;
                }
                break;
                
            case STATE_OBSTACLE_DETECTED:
                handle_obstacle_detected();
                break;
                
            case STATE_OBSTACLE_SCANNING:
                handle_obstacle_scanning();
                break;
                
            case STATE_OBSTACLE_AVOIDING:
                handle_obstacle_avoidance();
                break;
                
            case STATE_RETURNING_TO_LINE:
                handle_returning_to_line();
                break;
                
            case STATE_LINE_LOST:
                handle_line_lost();
                break;
                
            case STATE_STOPPED:
                // Stop all motors
                motor_stop(M1A, M1B);
                motor_stop(M2A, M2B);
                printf("[SYSTEM] Stopped - press GP20 to restart\n");
                
                if (calibration_button_pressed()) {
                    change_state(STATE_LINE_FOLLOWING);
                }
                break;
        }
        
        // Emergency stop button
        if (calibration_button_pressed() && current_state != STATE_STOPPED) {
            printf("\n>>> EMERGENCY STOP <<<\n");
            change_state(STATE_STOPPED);
        }
        
        sleep_ms(10);  // 100Hz loop
    }
    
    // Cleanup
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    
    if (telemetry_is_connected()) {
        telemetry_cleanup();
    }
    
    printf("\nProgram ended.\n");
    return 0;
}