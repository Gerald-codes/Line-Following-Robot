/**
 * imu.h
 * Unified IMU driver for GY-511 (LSM303DLHC)
 * Supports both struct-based and helper-based interfaces
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define IMU_I2C_PORT i2c0
#define IMU_SDA_PIN 16
#define IMU_SCL_PIN 17
#define IMU_I2C_FREQ 100000  // 100kHz

// ============================================================================
// STRUCT-BASED INTERFACE (for line following)
// ============================================================================

typedef struct {
    float ax, ay, az;      // Filtered accelerometer (g-units)
    float mx, my, mz;      // Filtered calibrated magnetometer
    float roll, pitch, yaw;
    float heading_offset;  // Used for heading reference
    bool calibrated;
} IMU;

/**
 * Initialize IMU with struct
 * @param imu Pointer to IMU struct
 */
void imu_init(IMU *imu);

/**
 * Calibrate IMU (zero heading reference)
 * @param imu Pointer to IMU struct
 */
void imu_calibrate(IMU *imu);

/**
 * Update IMU readings (read + filter + compute)
 * @param imu Pointer to IMU struct
 */
void imu_update(IMU *imu);

/**
 * Get heading in degrees
 * @param imu Pointer to IMU struct
 * @return Heading relative to calibration point
 */
float imu_get_heading(IMU *imu);

// ============================================================================
// HELPER INTERFACE (for obstacle avoidance)
// ============================================================================

/**
 * Heading tracking context for avoidance maneuvers
 */
typedef struct {
    float initial_heading;      // Heading when avoidance started
    float current_heading;      // Current heading
    float target_heading;       // Target heading after turn
    bool calibrated;
} IMUContext;

/**
 * Initialize IMU helper (no struct needed)
 * Call this if you're using the helper functions
 * @return true if successful
 */
bool imu_helper_init(void);

/**
 * Update IMU helper (call regularly)
 * Updates internal state for helper functions
 */
void imu_helper_update(void);

/**
 * Get current absolute heading
 * @return Current heading in degrees (0-360)
 */
float imu_helper_get_heading(void);

/**
 * Reset heading reference (set current as 0°)
 */
void imu_helper_reset_heading(void);

/**
 * Get relative heading change since reset
 * @return Heading change in degrees (-180 to +180)
 */
float imu_helper_get_relative_heading(void);

/**
 * Check if robot has turned specified angle
 * @param target_angle Desired turn angle (positive = left, negative = right)
 * @param tolerance Acceptable error in degrees
 * @return true if turn is complete
 */
bool imu_helper_has_turned(float target_angle, float tolerance);

/**
 * Get turn progress
 * @param target_angle Target turn angle
 * @return Percentage complete (0.0 to 1.0)
 */
float imu_helper_get_turn_progress(float target_angle);

/**
 * Initialize IMU context for avoidance maneuver
 */
void imu_helper_start_avoidance(void);

/**
 * Check if IMU is ready and calibrated
 * @return true if ready to use
 */
bool imu_helper_is_ready(void);

#endif // IMU_H