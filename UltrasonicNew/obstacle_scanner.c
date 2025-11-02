#include "obstacle_scanner.h"
#include "ultrasonic.h"
#include "servo.h"
#include "timer_manager.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

// Store last width for exponential moving average smoothing
static float last_width = 0.0f;

// Signal smoothing: keep last 3 distance readings for averaging
#define DISTANCE_HISTORY_SIZE 3
static uint64_t distance_history[DISTANCE_HISTORY_SIZE] = {0};
static int distance_history_count = 0;

// Helper: Get smoothed distance by averaging last readings
static uint64_t get_smoothed_distance(uint64_t new_distance) {
    // Shift history
    for (int i = DISTANCE_HISTORY_SIZE - 1; i > 0; i--) {
        distance_history[i] = distance_history[i-1];
    }
    distance_history[0] = new_distance;
    
    // Increase count until we have all readings
    if (distance_history_count < DISTANCE_HISTORY_SIZE) {
        distance_history_count++;
    }
    
    // Calculate average
    uint64_t sum = 0;
    for (int i = 0; i < distance_history_count; i++) {
        sum += distance_history[i];
    }
    
    return sum / distance_history_count;
}

float scanner_calculate_width(int angle_start, int angle_end, uint64_t min_distance) {
    
    // Convert to relative angles from center
    float start_relative = (angle_start - ANGLE_CENTER) * 3.14159265359f / 180.0f;
    float end_relative = (angle_end - ANGLE_CENTER) * 3.14159265359f / 180.0f;
    
    // Use min_distance for both points (approximation)
    float x1 = (float)min_distance * sinf(start_relative);
    float x2 = (float)min_distance * sinf(end_relative);
    
    float width = fabsf(x2 - x1);
    
    return width;
}

void scanner_init(void) {
    ultrasonic_init(TRIG_PIN, ECHO_PIN);
    servo_init(SERVO_PIN);
    servo_set_angle(ANGLE_CENTER);
    timer_wait_ms(500);
    
    // Reset smoothing
    last_width = 0.0f;
}

ScanResult scanner_perform_scan(void) {
    ScanResult result = {0};
    result.is_scanning = true;
    
    bool in_obstacle = false;
    int obstacle_start_angle = 0;
    int obstacle_count = 0;
    uint64_t obstacle_min_distance = 999;
    int timeout_count = 0;
    int distance_index = 0;
    
    printf("\n=== Starting Scan ===\n");
    
    // Reset distance history for this scan
    distance_history_count = 0;
    for (int i = 0; i < DISTANCE_HISTORY_SIZE; i++) {
        distance_history[i] = 0;
    }
    
    for (int angle = MIN_ANGLE; angle <= MAX_ANGLE; angle += SCAN_STEP) {
        servo_set_angle(angle);
        timer_wait_ms(100);
        
        uint64_t distance;
        int status = ultrasonic_get_distance(TRIG_PIN, ECHO_PIN, &distance);
        
        // Store the raw distance reading (all 41 readings)
        if (distance_index < 41) {
            result.distances[distance_index] = (status == SUCCESS) ? distance : 0;
            distance_index++;
        }
        
        if (status == SUCCESS) {
            timeout_count = 0;
            
            // Apply signal smoothing to filter noise
            uint64_t smoothed_distance = get_smoothed_distance(distance);
            
            bool obstacle_detected = (smoothed_distance >= OBSTACLE_THRESHOLD_MIN && 
                                     smoothed_distance <= OBSTACLE_THRESHOLD_MAX);
            
            printf("Angle: %3d° | Distance: %3llu cm (smoothed: %3llu cm)", angle, distance, smoothed_distance);
            
            if (obstacle_detected && !in_obstacle) {
                // Start of new obstacle
                in_obstacle = true;
                obstacle_start_angle = angle;
                obstacle_min_distance = smoothed_distance;
                obstacle_count++;
                printf(" >>> START\n");
            }
            else if (obstacle_detected && in_obstacle) {
                // Track minimum distance (closest point)
                if (smoothed_distance < obstacle_min_distance) {
                    obstacle_min_distance = smoothed_distance;
                }
                printf(" >>> IN OBSTACLE\n");
            }
            else if (!obstacle_detected && in_obstacle) {
                // End of obstacle
                in_obstacle = false;
                int obstacle_end_angle = angle - SCAN_STEP;
                int angle_span = obstacle_end_angle - obstacle_start_angle;
                
                printf(" >>> END\n");
                
                if (angle_span >= MIN_OBSTACLE_SPAN) {
                    float width = scanner_calculate_width(obstacle_start_angle, 
                                                        obstacle_end_angle, 
                                                        obstacle_min_distance);
                    
                    // Apply exponential moving average (70% new, 30% old)
                    float smoothed_width = (width * 0.7f) + (last_width * 0.3f);
                    last_width = smoothed_width;
                    
                    if (result.obstacle_count < 20) {
                        Obstacle* obs = &result.obstacles[result.obstacle_count];
                        obs->angle_start = obstacle_start_angle;
                        obs->angle_end = obstacle_end_angle;
                        obs->angle_span = angle_span;
                        obs->min_distance = obstacle_min_distance;
                        obs->width = width;
                        obs->smoothed_width = smoothed_width;
                        result.obstacle_count++;
                    }
                } else {
                    obstacle_count--;
                }
            }
            else {
                printf(" [Clear]\n");
            }
        }
        else {
            timeout_count++;
            printf("Angle: %3d° | Timeout (%d)\n", angle, timeout_count);
            
            // End obstacle on FIRST timeout (not after 3+)
            if (timeout_count == 1 && in_obstacle) {
                in_obstacle = false;
                int obstacle_end_angle = angle - SCAN_STEP;
                int angle_span = obstacle_end_angle - obstacle_start_angle;
                
                printf(" >>> OBSTACLE END (timeout)\n");
                
                if (angle_span >= MIN_OBSTACLE_SPAN) {
                    float width = scanner_calculate_width(obstacle_start_angle, 
                                                        obstacle_end_angle, 
                                                        obstacle_min_distance);
                    
                    // Apply exponential moving average
                    float smoothed_width = (width * 0.7f) + (last_width * 0.3f);
                    last_width = smoothed_width;
                    
                    if (result.obstacle_count < 20) {
                        Obstacle* obs = &result.obstacles[result.obstacle_count];
                        obs->angle_start = obstacle_start_angle;
                        obs->angle_end = obstacle_end_angle;
                        obs->angle_span = angle_span;
                        obs->min_distance = obstacle_min_distance;
                        obs->width = width;
                        obs->smoothed_width = smoothed_width;
                        result.obstacle_count++;
                    }
                } else {
                    obstacle_count--;
                }
            }
        }
    }
    
    // Handle obstacle extending to end of scan
    if (in_obstacle) {
        int angle_span = MAX_ANGLE - obstacle_start_angle;
        
        if (angle_span >= MIN_OBSTACLE_SPAN) {
            float width = scanner_calculate_width(obstacle_start_angle, 
                                                MAX_ANGLE, 
                                                obstacle_min_distance);
            
            // Apply exponential moving average
            float smoothed_width = (width * 0.7f) + (last_width * 0.3f);
            last_width = smoothed_width;
            
            if (result.obstacle_count < 20) {
                Obstacle* obs = &result.obstacles[result.obstacle_count];
                obs->angle_start = obstacle_start_angle;
                obs->angle_end = MAX_ANGLE;
                obs->angle_span = angle_span;
                obs->min_distance = obstacle_min_distance;
                obs->width = width;
                obs->smoothed_width = smoothed_width;
                result.obstacle_count++;
            }
        } else {
            obstacle_count--;
        }
    }
    
    printf("=== Scan Complete ===\n");
    printf("Obstacles found: %d\n\n", result.obstacle_count);
    
    // Print all obstacles after scan completes
    for (int i = 0; i < result.obstacle_count; i++) {
        Obstacle* obs = &result.obstacles[i];
        printf("*** OBSTACLE #%d ***\n", i + 1);
        printf("  Angle: %d° to %d° (span: %d°)\n", 
               obs->angle_start, obs->angle_end, obs->angle_span);
        printf("  Min Distance: %llu cm\n", obs->min_distance);
        printf("  Width: %.2f cm (smoothed: %.2f cm)\n\n", 
               obs->width, obs->smoothed_width);
    }
    
    servo_set_angle(ANGLE_CENTER);
    result.is_scanning = false;
    
    return result;
}

void scanner_print_results(ScanResult result) {
    // Summary is already printed during the scan
    // This function can be used for additional detail if needed
    printf("\n=== Scan Summary ===\n");
    printf("Total Obstacles Detected: %d\n", result.obstacle_count);
    
    for (int i = 0; i < result.obstacle_count; i++) {
        Obstacle* obs = &result.obstacles[i];
        printf("\nOBSTACLE #%d:\n", i + 1);
        printf("  Angle Range: %d° - %d° (span: %d°)\n", 
               obs->angle_start, obs->angle_end, obs->angle_span);
        printf("  Closest Distance: %llu cm\n", obs->min_distance);
        printf("  Width: %.2f cm (smoothed: %.2f cm)\n", 
               obs->width, obs->smoothed_width);
    }
}