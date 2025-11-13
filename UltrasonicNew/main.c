#include "pico/stdlib.h"
#include <stdio.h>
#include "ultrasonic.h"
#include "servo.h"
#include "timer_manager.h"
#include "obstacle_scanner.h"
#include "telemetry.h"
#include "motor.h"

// Navigation parameters
#define CRITICAL_DISTANCE 15  // Stop when obstacle within 15cm
#define NORMAL_SPEED 30       // Forward speed (0-100) - Reduced for better obstacle detection
#define CHECK_INTERVAL_MS 1000 // Check for obstacles every 1 second

// Navigation states
typedef enum {
    STATE_IDLE,
    STATE_MOVING_FORWARD,
    STATE_STOPPED,
    STATE_SCANNING
} NavigationState;

static NavigationState current_state = STATE_IDLE;

void print_system_info(void) {
    printf("\n========================================\n");
    printf("HC-SR04 Ultrasonic + Servo Scanner\n");
    printf("With MQTT Telemetry Support\n");
    printf("With Obstacle Avoidance & Motor Control\n");
    printf("========================================\n");
    printf("Ultrasonic Sensor:\n");
    printf("  TRIG: GPIO %d\n", TRIG_PIN);
    printf("  ECHO: GPIO %d\n", ECHO_PIN);
    printf("Servo Motor:\n");
    printf("  Signal: GPIO %d\n", SERVO_PIN);
    printf("Motor Control:\n");
    printf("  M1A: GPIO %d, M1B: GPIO %d\n", M1A, M1B);
    printf("  M2A: GPIO %d, M2B: GPIO %d\n", M2A, M2B);
    printf("========================================\n");
    printf("Detection Range: %d-%d cm\n", OBSTACLE_THRESHOLD_MIN, OBSTACLE_THRESHOLD_MAX);
    printf("Critical Distance: %d cm (STOP threshold)\n", CRITICAL_DISTANCE);
    printf("Scan Range: %d deg to %d deg (step: %d deg)\n", MIN_ANGLE, MAX_ANGLE, SCAN_STEP);
    printf("Min Obstacle Span: %d deg\n", MIN_OBSTACLE_SPAN);
    printf("========================================\n\n");
    
    printf("MQTT Topics:\n");
    printf("  Obstacle Width: %s\n", TOPIC_OBSTACLE_WIDTH);
    printf("  Obstacle Count: %s\n", TOPIC_OBSTACLE_COUNT);
    printf("  Obstacle Distance: %s\n", TOPIC_OBSTACLE_DISTANCE);
    printf("  Obstacle Angles: %s\n", TOPIC_OBSTACLE_ANGLES);
    printf("  Complete Data: %s\n", TOPIC_OBSTACLE_ALL);
    printf("  Status: %s\n", TOPIC_STATUS);
    printf("  Scan Complete: %s\n", TOPIC_SCAN_COMPLETE);
    printf("========================================\n");
}

/**
 * Quick check forward path (single center point check for speed)
 * Returns true if path is clear, false if obstacle detected
 */
bool check_forward_path(void) {
    // Keep servo at center - no need to move it
    servo_set_angle(ANGLE_CENTER);
    timer_wait_ms(50);  // Short delay for servo to stabilize
    
    uint64_t distance;
    int status = ultrasonic_get_distance(TRIG_PIN, ECHO_PIN, &distance);
    
    if (status == SUCCESS) {
        printf("  Distance: %llu cm", distance);
        
        if (distance <= CRITICAL_DISTANCE) {
            printf(" ⚠️  OBSTACLE!\n");
            
            // Publish to telemetry
            if (telemetry_is_connected()) {
                char msg[64];
                snprintf(msg, sizeof(msg), "OBSTACLE at %llu cm!", distance);
                telemetry_publish_status(msg);
            }
            
            return false;  // Obstacle detected!
        } else {
            printf(" ✓ Clear\n");
            return true;   // Path clear
        }
    } else {
        printf("  Sensor error (code: %d)\n", status);
        return true;  // Assume clear on error (safer to keep moving than false positive stop)
    }
}

/**
 * Stop motors immediately
 */
void stop_robot(void) {
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    current_state = STATE_STOPPED;
    printf("\n!!! MOTORS STOPPED !!!\n");
    
    if (telemetry_is_connected()) {
        telemetry_publish_status("Robot stopped - obstacle detected");
    }
}

/**
 * Start moving forward
 */
void move_forward(void) {
    motor_drive(M1A, M1B, -NORMAL_SPEED);
    motor_drive(M2A, M2B, -NORMAL_SPEED - 4);
    current_state = STATE_MOVING_FORWARD;
    printf("\n>>> Moving forward at speed %d\n", NORMAL_SPEED);
    
    if (telemetry_is_connected()) {
        telemetry_publish_status("Moving forward");
    }
}

/**
 * Demo sequence for Week 10 Demo 3
 * Shows obstacle detection, stopping, and scanning
 */
void demo_obstacle_avoidance(void) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════╗\n");
    printf("║     DEMO 3: OBSTACLE AVOIDANCE DEMONSTRATION       ║\n");
    printf("╚════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("Demo sequence:\n");
    printf("1. Robot moves forward while checking continuously\n");
    printf("2. If obstacle < %d cm, STOP immediately\n", CRITICAL_DISTANCE);
    printf("3. Perform full scan to measure obstacle\n");
    printf("4. Determine clearer path\n");
    printf("\nStarting in 3 seconds...\n\n");
    
    timer_wait_ms(3000);
    
    // Step 1: Start moving forward
    printf("\n[Step 1] Starting forward movement...\n");
    move_forward();
    
    // Step 2: Move forward while continuously checking for obstacles
    printf("\n[Step 2] Moving forward and checking for obstacles...\n");
    bool obstacle_found = false;
    int check_count = 0;
    
    // Keep checking while moving (up to 50 checks = ~5 seconds at 100ms per check)
    while (!obstacle_found && check_count < 50) {
        check_count++;
        printf("Check #%d: ", check_count);
        
        // Quick check forward
        bool path_clear = check_forward_path();
        
        if (!path_clear) {
            // Obstacle detected!
            obstacle_found = true;
            break;
        }
        
        // Path still clear, continue moving for another 100ms
        timer_wait_ms(100);  // Check every 100ms for faster response
    }
    
    if (obstacle_found) {
        // Step 3: Obstacle detected - STOP
        printf("\n[Step 3] !!! OBSTACLE DETECTED !!!\n");
        stop_robot();
        
        // Step 4: Perform full scan
        printf("\n[Step 4] Performing full obstacle scan...\n");
        timer_wait_ms(500);
        
        current_state = STATE_SCANNING;
        ScanResult result = scanner_perform_scan();
        scanner_print_results(result);
        
        // Step 5: Analyze results
        if (result.obstacle_count > 0) {
            printf("\n[Step 5] === OBSTACLE ANALYSIS ===\n");
            Obstacle* obs = &result.obstacles[0];
            printf("Width: %.2f cm\n", obs->smoothed_width);
            printf("Distance: %llu cm\n", obs->min_distance);
            
            // Determine clearer side
            int obstacle_center = (obs->angle_start + obs->angle_end) / 2;
            const char* clearer_side = (obstacle_center > ANGLE_CENTER) ? "LEFT" : "RIGHT";
            printf("Clearer side: %s\n", clearer_side);
            
            if (telemetry_is_connected()) {
                char msg[128];
                snprintf(msg, sizeof(msg), 
                        "Obstacle: %.1fcm wide, %llucm away, go %s", 
                        obs->smoothed_width, obs->min_distance, clearer_side);
                telemetry_publish_status(msg);
            }
        }
        
    } else {
        // No obstacle found after max checks
        printf("\n[Result] Path is clear - no obstacles detected\n");
        printf("Stopping robot (end of demo)\n");
        stop_robot();
    }
    
    printf("\n╔════════════════════════════════════════════════════╗\n");
    printf("║              DEMO 3 COMPLETE                       ║\n");
    printf("╚════════════════════════════════════════════════════╝\n\n");
}

/**
 * Continuous navigation mode
 * Keeps checking and moving until obstacle detected
 */
void continuous_navigation(void) {
    static int check_count = 0;
    check_count++;
    
    printf("\n========== Navigation Check #%d ==========\n", check_count);
    
    switch (current_state) {
        case STATE_IDLE:
            printf("State: IDLE - Starting movement\n");
            move_forward();
            break;
            
        case STATE_MOVING_FORWARD:
            printf("State: MOVING FORWARD\n");
            
            // Check forward path
            bool path_clear = check_forward_path();
            
            if (!path_clear) {
                // Obstacle detected - STOP
                printf("\n!!! OBSTACLE DETECTED !!!\n");
                stop_robot();
                
                // Perform full scan
                printf("\nPerforming full scan...\n");
                timer_wait_ms(500);
                
                current_state = STATE_SCANNING;
                ScanResult result = scanner_perform_scan();
                scanner_print_results(result);
                
                // Stay stopped
                current_state = STATE_STOPPED;
            } else {
                printf("Path clear - continuing\n");
            }
            break;
            
        case STATE_STOPPED:
            printf("State: STOPPED (obstacle ahead)\n");
            printf("Reset required to continue\n");
            break;
            
        case STATE_SCANNING:
            printf("State: SCANNING\n");
            break;
    }
    
    // Process telemetry
    if (telemetry_is_connected()) {
        telemetry_process();
    }
}

int main() {
    stdio_init_all();
    // timer_wait_ms(2000);
    
    print_system_info();
    
    // Initialize all modules
    printf("\nInitializing modules...\n");
    timer_manager_init();
    scanner_init();
    
    // Initialize motors
    printf("Initializing motors...\n");
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    printf("✓ Motors initialized\n");

    // // Initialize telemetry system (optional)
    // printf("\nInitializing telemetry system...\n");
    // TelemetryStatus telem_status = telemetry_init(MQTT_BROKER_ADDRESS, MQTT_CLIENT_ID);
    
    // if (telem_status == TELEMETRY_SUCCESS) {
    //     printf("✓ Telemetry system initialized successfully\n");
    //     printf("✓ Connected to MQTT broker\n");
    //     telemetry_publish_status("System initialized");
        
    //     // Enable telemetry in scanner
    //     scanner_enable_telemetry();
    // } else {
    //     printf("✗ Failed to initialize telemetry: %d\n", telem_status);
    //     printf("  Continuing without telemetry...\n");
    // }
    
    printf("\n========================================\n");
    printf("System ready!\n");
    printf("========================================\n\n");
    
    // Choose operation mode
    printf("Select mode:\n");
    printf("1. Run Demo 3 sequence (recommended for Week 10)\n");
    printf("2. Continuous navigation mode\n");
    printf("3. Scanning only (original behavior)\n");
    printf("\nDefaulting to Demo 3 in 3 seconds...\n\n");
    timer_wait_ms(3000);
    
    // Default: Run Demo 3
    int mode = 1;
    
    if (mode == 1) {
        // Demo 3: One-shot demonstration
        demo_obstacle_avoidance();
        
        // Stay in stopped state
        printf("\nDemo complete. Motors stopped.\n");
        printf("Press RESET to run again.\n\n");
        
        while (1) {
            // Just keep telemetry alive
            if (telemetry_is_connected()) {
                telemetry_process();
            }
            timer_wait_ms(100);
        }
        
    } else if (mode == 2) {
        // Continuous navigation mode
        printf("\n========================================\n");
        printf("Starting continuous navigation mode\n");
        printf("Robot will move forward and stop at obstacles\n");
        printf("========================================\n\n");
        
        timer_wait_ms(2000);
        
        while (1) {
            continuous_navigation();
            timer_wait_ms(CHECK_INTERVAL_MS);
        }
        
    } else {
        // Mode 3: Original scanning-only behavior
        printf("\n========================================\n");
        printf("Starting scanning mode (no motors)\n");
        printf("========================================\n\n");
        
        timer_wait_ms(2000);
        
        int scan_count = 0;
        
        while (1) {
            scan_count++;
            printf("\n========== SCAN #%d ==========\n", scan_count);
            
            // Publish scan start event
            if (telemetry_is_connected()) {
                char msg[64];
                snprintf(msg, sizeof(msg), "Starting scan #%d", scan_count);
                telemetry_publish_status(msg);
            }
            
            // Perform full scan
            ScanResult result = scanner_perform_scan();
            
            // Print and process results
            scanner_print_results(result);

            // Process any pending telemetry messages
            if (telemetry_is_connected()) {
                telemetry_process();
            }
            
            // Wait before next scan using timer
            printf("Waiting 3 seconds before next scan...\n");
            timer_wait_ms(3000);
        }
    }
    
    // Cleanup (never reached in this infinite loop, but good practice)
    telemetry_cleanup();
    
    return 0;
}