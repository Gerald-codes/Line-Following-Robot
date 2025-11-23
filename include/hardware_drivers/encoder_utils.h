/**
 * @file    encoder_utils.h
 * @brief   Helper functions for encoder constants
 * @details
 *   Easy-to-use conversions and calculations for wheel encoders.
 */

#ifndef ENCODER_UTILS_H
#define ENCODER_UTILS_H

#include "config.h"
#include "encoder.h"

/**
 * @brief   Convert left encoder pulses to millimeters
 * @param   pulses Number of encoder pulses for left wheel
 * @return  Distance in millimeters
 */
static inline float left_pulses_to_mm(int32_t pulses) {
    return pulses * LEFT_MM_PER_PULSE;
}

/**
 * @brief   Convert right encoder pulses to millimeters
 * @param   pulses Number of encoder pulses for right wheel
 * @return  Distance in millimeters
 */
static inline float right_pulses_to_mm(int32_t pulses) {
    return pulses * RIGHT_MM_PER_PULSE;
}

/**
 * @brief   Convert millimeters to left encoder pulses
 * @param   mm Distance in millimeters for left wheel
 * @return  Number of encoder pulses needed
 */
static inline int32_t left_mm_to_pulses(float mm) {
    return (int32_t)(mm / LEFT_MM_PER_PULSE);
}

/**
 * @brief   Convert millimeters to right encoder pulses
 * @param   mm Distance in millimeters for right wheel
 * @return  Number of encoder pulses needed
 */
static inline int32_t right_mm_to_pulses(float mm) {
    return (int32_t)(mm / RIGHT_MM_PER_PULSE);
}

/**
 * @brief   Get average distance traveled by both wheels
 * @details Uses current encoder counts to compute average
 * @return  Average distance in millimeters
 */
static inline float get_average_distance_mm(void) {
    int32_t left_count = get_left_encoder();
    int32_t right_count = get_right_encoder();
    return (left_pulses_to_mm(left_count) + right_pulses_to_mm(right_count)) / 2.0f;
}

/**
 * @brief   Get distance difference between wheels
 * @details Useful for detecting turns or slippage
 * @return  Difference in millimeters (left - right)
 */
static inline float get_distance_difference_mm(void) {
    int32_t left_count = get_left_encoder();
    int32_t right_count = get_right_encoder();
    return left_pulses_to_mm(left_count) - right_pulses_to_mm(right_count);
}

/**
 * @brief   Calculate robot heading from wheel encoders (dead reckoning)
 * @details Positive value: turned left. Negative value: turned right.
 * @return  Estimated heading in degrees
 */
static inline float get_heading_from_encoders(void) {
    float distance_diff = get_distance_difference_mm();
    return (distance_diff / WHEEL_BASE_MM) * 57.2958f;
}

#endif /* ENCODER_UTILS_H */
