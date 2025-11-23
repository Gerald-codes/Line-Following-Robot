#include "obstacle_scanner.h"
#include "ultrasonic.h"
#include "servo.h"
#include "timer_manager.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "telemetry.h"
#include "pico/time.h"


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

    if (telemetry_is_ready()) {
        telemetry_publish_obstacle_scan(&result);
    }
    
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
void scanner_get_clear_space_analysis(ScanResult result, int* left_clear, int* right_clear) {
    *left_clear = 0;
    *right_clear = 0;
    
    // Create a boolean array to mark which scan positions are blocked
    // We have (MAX_ANGLE - MIN_ANGLE) / SCAN_STEP + 1 = 21 scan positions
    #define NUM_SCAN_POSITIONS ((MAX_ANGLE - MIN_ANGLE) / SCAN_STEP + 1)
    bool blocked[NUM_SCAN_POSITIONS];
    
    // Initialize all positions as clear
    for (int i = 0; i < NUM_SCAN_POSITIONS; i++) {
        blocked[i] = false;
    }
    
    // Mark all angles that have obstacles
    for (int i = 0; i < result.obstacle_count; i++) {
        Obstacle* obs = &result.obstacles[i];
        for (int angle = obs->angle_start; angle <= obs->angle_end; angle += SCAN_STEP) {
            int index = (angle - MIN_ANGLE) / SCAN_STEP;
            if (index >= 0 && index < NUM_SCAN_POSITIONS) {
                blocked[index] = true;
            }
        }
    }
    
    // Count clear space on RIGHT side (MIN_ANGLE to ANGLE_CENTER) - servo mounted backwards
    // Physical RIGHT = lower angles (50-80°)
    for (int angle = MIN_ANGLE; angle < ANGLE_CENTER; angle += SCAN_STEP) {
        int index = (angle - MIN_ANGLE) / SCAN_STEP;
        if (index >= 0 && index < NUM_SCAN_POSITIONS && !blocked[index]) {
            *right_clear += SCAN_STEP;
        }
    }
    
    // Count clear space on LEFT side (ANGLE_CENTER to MAX_ANGLE) - servo mounted backwards
    // Physical LEFT = higher angles (80-110°)
    for (int angle = ANGLE_CENTER; angle <= MAX_ANGLE; angle += SCAN_STEP) {
        int index = (angle - MIN_ANGLE) / SCAN_STEP;
        if (index >= 0 && index < NUM_SCAN_POSITIONS && !blocked[index]) {
            *left_clear += SCAN_STEP;
        }
    }
}


AvoidanceDirection scanner_get_best_avoidance_direction(ScanResult result) {
    // If no obstacles detected, no avoidance needed
    if (result.obstacle_count == 0) {
        printf("[AVOIDANCE] No obstacles detected - no avoidance needed\n");
        return AVOID_NONE;
    }
    
    int left_clear = 0;
    int right_clear = 0;
    
    // Get clear space analysis
    scanner_get_clear_space_analysis(result, &left_clear, &right_clear);
    
    // Print detailed analysis
    printf("\n=== CLEAR SPACE ANALYSIS ===\n");
    printf("Left side clear space:  %d degrees\n", left_clear);
    printf("Right side clear space: %d degrees\n", right_clear);
    
    // Determine which side is better
    AvoidanceDirection direction;
    if (left_clear > right_clear) {
        direction = AVOID_LEFT;
        printf("RECOMMENDATION: Go LEFT (more clear space)\n");
    } else if (right_clear > left_clear) {
        direction = AVOID_RIGHT;
        printf("RECOMMENDATION: Go RIGHT (more clear space)\n");
    } else {
        // Equal clear space - use distance analysis as tiebreaker
        printf("Equal clear space on both sides - checking distance distribution...\n");
        
        // Find where the obstacle is closest (most dangerous)
        uint64_t min_distance_overall = 999;
        int min_distance_angle = ANGLE_CENTER;
        
        // Analyze distance at multiple points across scan range
        uint64_t left_total_distance = 0;
        uint64_t right_total_distance = 0;
        int left_count = 0;
        int right_count = 0;
        
        for (int i = 0; i < result.obstacle_count; i++) {
            Obstacle* obs = &result.obstacles[i];
            
            // Find closest point in this obstacle
            for (int angle = obs->angle_start; angle <= obs->angle_end; angle += SCAN_STEP) {
                int dist_index = (angle - MIN_ANGLE) / SCAN_STEP;
                if (dist_index >= 0 && dist_index < 41 && result.distances[dist_index] > 0) {
                    uint64_t dist = result.distances[dist_index];
                    
                    // Track closest point
                    if (dist < min_distance_overall) {
                        min_distance_overall = dist;
                        min_distance_angle = angle;
                    }
                    
                    // Accumulate distances for averaging (SWAPPED for servo orientation)
                    // Physical LEFT = higher angles (80-110°)
                    // Physical RIGHT = lower angles (50-80°)
                    if (angle < ANGLE_CENTER) {
                        right_total_distance += dist;  // Changed from left
                        right_count++;
                    } else {
                        left_total_distance += dist;   // Changed from right
                        left_count++;
                    }
                }
            }
        }
        
        // Calculate average distances
        uint64_t avg_left_distance = (left_count > 0) ? (left_total_distance / left_count) : 999;
        uint64_t avg_right_distance = (right_count > 0) ? (right_total_distance / right_count) : 999;
        
        printf("Obstacle closest at angle: %d° (%llu cm)\n", min_distance_angle, min_distance_overall);
        printf("Average left side distance: %llu cm\n", avg_left_distance);
        printf("Average right side distance: %llu cm\n", avg_right_distance);
        
        // Decision: go away from closest point (SWAPPED for servo orientation)
        // Physical LEFT = higher angles (>80°), Physical RIGHT = lower angles (<80°)
        if (min_distance_angle < ANGLE_CENTER) {
            direction = AVOID_LEFT;  // Changed from AVOID_RIGHT
            printf("RECOMMENDATION: Go LEFT (obstacle closest on right at %d°)\n", min_distance_angle);
        } else if (min_distance_angle > ANGLE_CENTER) {
            direction = AVOID_RIGHT;  // Changed from AVOID_LEFT
            printf("RECOMMENDATION: Go RIGHT (obstacle closest on left at %d°)\n", min_distance_angle);
        } else {
            // Closest point is at center, use average distance
            if (avg_left_distance > avg_right_distance) {
                direction = AVOID_LEFT;
                printf("RECOMMENDATION: Go LEFT (better average distance)\n");
            } else {
                direction = AVOID_RIGHT;
                printf("RECOMMENDATION: Go RIGHT (better average distance)\n");
            }
        }
    }
    
    printf("=============================\n\n");
    
    
    
    return direction;
}