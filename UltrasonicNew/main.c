#include "pico/stdlib.h"
#include <stdio.h>
#include "ultrasonic.h"
#include "servo.h"
#include "timer_manager.h"
#include "obstacle_scanner.h"

void print_system_info(void) {
    printf("\n========================================\n");
    printf("HC-SR04 Ultrasonic + Servo Scanner\n");
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
}

int main() {
    stdio_init_all();
    timer_wait_ms(2000);
    
    print_system_info();
    
    // Initialize all modules
    timer_manager_init();
    scanner_init();
    
    printf("System ready!\n");
    printf("Starting scan in 2 seconds...\n\n");
    timer_wait_ms(2000);
    
    int scan_count = 0;
    
    while (1) {
        scan_count++;
        printf("\n========== SCAN #%d ==========\n", scan_count);
        
        // Perform full scan
        ScanResult result = scanner_perform_scan();
        
        // Print and process results
        scanner_print_results(result);
        
        // Wait before next scan using timer
        printf("Waiting 3 seconds before next scan...\n");
        timer_wait_ms(3000);
    }
    
    return 0;
}