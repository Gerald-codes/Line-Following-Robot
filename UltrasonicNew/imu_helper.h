#ifndef IMU_HELPER_H
#define IMU_HELPER_H

#include <stdbool.h>
#include <stdint.h>

// IMU calibration and configuration
#define IMU_I2C_PORT i2c0
#define IMU_SDA_PIN 0    // GPIO0 (from your teammate's code)
#define IMU_SCL_PIN 1    // GPIO1 (from your teammate's code)
#define IMU_I2C_FREQ 100000  // 100kHz (from your teammate's code)

// Heading tracking
typedef struct {
    float initial_heading;      // Heading when avoidance started
    float current_heading;      // Current heading
    float target_heading;       // Target heading after turn
    bool calibrated;
} IMUContext;

/**
 * Initialize IMU sensor
 * @return true if successful
 */
bool imu_init(void);

/**
 * Read current heading from IMU (yaw angle)
 * @return Current heading in degrees (0-360)
 */
float imu_get_heading(void);

/**
 * Reset heading reference (set current as 0°)
 */
void imu_reset_heading(void);

/**
 * Get relative heading change since reset
 * @return Heading change in degrees (-180 to +180)
 */
float imu_get_relative_heading(void);

/**
 * Check if robot has turned specified angle
 * @param target_angle Desired turn angle in degrees (positive = left, negative = right)
 * @param tolerance Acceptable error in degrees
 * @return true if turn is complete within tolerance
 */
bool imu_has_turned(float target_angle, float tolerance);

/**
 * Get turn progress
 * @param target_angle Target turn angle
 * @return Percentage complete (0.0 to 1.0)
 */
float imu_get_turn_progress(float target_angle);

/**
 * Initialize IMU context for avoidance maneuver
 */
void imu_start_avoidance(void);

/**
 * Update IMU readings (call regularly in main loop)
 */
void imu_update(void);

/**
 * Check if IMU is ready and calibrated
 * @return true if ready to use
 */
bool imu_is_ready(void);

#endif // IMU_HELPER_H