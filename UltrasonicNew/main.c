#include "pico/stdlib.h"
#include <stdio.h>
#include "ultrasonic.h"
#include "servo.h"
#include "timer_manager.h"
#include "obstacle_scanner.h"
#include "telemetry.h"


void print_system_info(void) {
    printf("\n========================================\n");
    printf("HC-SR04 Ultrasonic + Servo Scanner\n");

    printf("With MQTT Telemetry Support\n");
    
    printf("========================================\n");
    printf("Ultrasonic Sensor:\n");
    printf("  TRIG: GPIO %d\n", TRIG_PIN);
    printf("  ECHO: GPIO %d\n", ECHO_PIN);
    printf("Servo Motor:\n");
    printf("  Signal: GPIO %d\n", SERVO_PIN);
    printf("========================================\n");
    printf("Detection Range: %d-%d cm\n", OBSTACLE_THRESHOLD_MIN, OBSTACLE_THRESHOLD_MAX);
    printf("Scan Range: %d deg to %d deg (step: %d deg)\n", MIN_ANGLE, MAX_ANGLE, SCAN_STEP);
    printf("Min Obstacle Span: %d deg\n", MIN_OBSTACLE_SPAN);
    printf("Distance Threshold: %d cm\n", DISTANCE_CHANGE_THRESHOLD);
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

int main() {
    stdio_init_all();
    timer_wait_ms(2000);
    
    print_system_info();
    
    // Initialize all modules
    timer_manager_init();
    scanner_init();


    // Initialize telemetry system (optional)
    printf("Initializing telemetry system...\n");
    TelemetryStatus telem_status = telemetry_init(MQTT_BROKER_ADDRESS, MQTT_CLIENT_ID);
    
    if (telem_status == TELEMETRY_SUCCESS) {
        printf("Telemetry system initialized successfully\n");
        printf("Connected to MQTT broker\n");
        telemetry_publish_status("System initialized");
        
        // Enable telemetry in scanner
        scanner_enable_telemetry();
    } else {
        printf("Failed to initialize telemetry: %d\n", telem_status);
        printf("  Continuing without telemetry...\n");
    }
    
    // printf("Telemetry support not enabled (compile with -DENABLE_TELEMETRY to enable)\n");
    
    
    printf("System ready!\n");
    printf("Starting scan in 2 seconds...\n\n");
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

    
    // Cleanup (never reached in this infinite loop, but good practice)
    telemetry_cleanup();
    
    
    return 0;
}