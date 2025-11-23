#ifndef OBSTACLE_SCANNER_H
#define OBSTACLE_SCANNER_H

#include <stdint.h>
#include <stdbool.h>
#include "ultrasonic.h"
#include "servo.h"

// Scanning parameters
#define ANGLE_CENTER 80      // Center position of servo
#define MIN_ANGLE (ANGLE_CENTER - 30)  // 60° left from center
#define MAX_ANGLE (ANGLE_CENTER + 30)  // 60° right from center
#define SCAN_STEP 3           // Degrees per step
#define OBSTACLE_THRESHOLD_MIN 2   // cm
#define OBSTACLE_THRESHOLD_MAX 30 // cm
#define MIN_OBSTACLE_SPAN 10   // Minimum angle span to count as obstacle
#define DISTANCE_CHANGE_THRESHOLD 25  // cm

typedef struct {
    int angle_start;
    int angle_end;
    int angle_span;
    uint64_t min_distance;
    float width;
    float smoothed_width;
} Obstacle;

typedef struct {
    Obstacle obstacles[10];
    int obstacle_count;
    bool is_scanning;
    uint64_t distances[21];  // 21 readings from 45-105 deg in 3 deg steps
} ScanResult;

/**
 * Initialize obstacle scanner
 */
void scanner_init(void);

/**
 * Perform a single scan sweep
 * @return ScanResult containing detected obstacles
 */
ScanResult scanner_perform_scan(void);

/**
 * Calculate obstacle width based on angle span and minimum distance
 * @param angle_start Start angle in degrees
 * @param angle_end End angle in degrees
 * @param min_distance Minimum distance in cm
 * @return Obstacle width in cm
 */
float scanner_calculate_width(int angle_start, int angle_end, uint64_t min_distance);

/**
 * Print scan results to console
 * @param result ScanResult to print
 */
void scanner_print_results(ScanResult result);


/**
 * Enable telemetry publishing during scans
 * Call this after scanner_init() and telemetry_init() to enable automatic
 * telemetry publishing during obstacle scanning
 */
void scanner_enable_telemetry(void);

/**
 * Disable telemetry publishing during scans
 */
void scanner_disable_telemetry(void);

/**
 * Check if telemetry is enabled for scanner
 * @return true if telemetry publishing is enabled
 */
bool scanner_is_telemetry_enabled(void);

/**
 * Avoidance direction options
 */
typedef enum {
    AVOID_LEFT,
    AVOID_RIGHT,
    AVOID_NONE
} AvoidanceDirection;

/**
 * Analyze scan result and determine which side has more clear space
 * Uses actual clear space calculation, not just obstacle center
 * @param result ScanResult from scanner_perform_scan()
 * @return AvoidanceDirection indicating which way to go (LEFT, RIGHT, or NONE)
 */
AvoidanceDirection scanner_get_best_avoidance_direction(ScanResult result);

/**
 * Get detailed clear space information for debugging
 * @param result ScanResult from scanner_perform_scan()
 * @param left_clear Output: degrees of clear space on left side
 * @param right_clear Output: degrees of clear space on right side
 */
void scanner_get_clear_space_analysis(ScanResult result, int* left_clear, int* right_clear);


#endif