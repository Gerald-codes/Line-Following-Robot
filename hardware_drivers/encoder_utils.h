/**
 * encoder_utils.h
 * Helper functions to make separate encoder constants easy to use
 */

#ifndef ENCODER_UTILS_H
#define ENCODER_UTILS_H

#include "config.h"
#include "encoder.h"

// Convert pulses to distance for each wheel
static inline float left_pulses_to_mm(int32_t pulses) {
    return pulses * LEFT_MM_PER_PULSE;
}

static inline float right_pulses_to_mm(int32_t pulses) {
    return pulses * RIGHT_MM_PER_PULSE;
}

// Convert distance to pulses for each wheel
static inline int32_t left_mm_to_pulses(float mm) {
    return (int32_t)(mm / LEFT_MM_PER_PULSE);
}

static inline int32_t right_mm_to_pulses(float mm) {
    return (int32_t)(mm / RIGHT_MM_PER_PULSE);
}

// Get average distance traveled
static inline float get_average_distance_mm(void) {
    int32_t left_count = get_left_encoder();
    int32_t right_count = get_right_encoder();
    return (left_pulses_to_mm(left_count) + right_pulses_to_mm(right_count)) / 2.0f;
}

// Get distance difference (for detecting turns/slippage)
static inline float get_distance_difference_mm(void) {
    int32_t left_count = get_left_encoder();
    int32_t right_count = get_right_encoder();
    return left_pulses_to_mm(left_count) - right_pulses_to_mm(right_count);
}

// Calculate robot heading from wheel encoders (dead reckoning)
// Positive = turned left, Negative = turned right
static inline float get_heading_from_encoders(void) {
    float distance_diff = get_distance_difference_mm();
    // angle (radians) = arc_length / radius
    // angle (degrees) = (arc_length / (wheel_base/2)) * (180/π)
    return (distance_diff / WHEEL_BASE_MM) * 57.2958f;  // Convert to degrees
}

#endif // ENCODER_UTILS_H