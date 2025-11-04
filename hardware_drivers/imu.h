// hardware_drivers/imu.h

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float ax, ay, az;      // Filtered accelerometer (g-units)
    float mx, my, mz;      // Filtered calibrated magnetometer
    float roll, pitch, yaw;
    float heading_offset;  // Used for heading reference
    bool calibrated;
} IMU;

void imu_init(IMU *imu);
void imu_calibrate(IMU *imu);      // Zero heading reference
void imu_update(IMU *imu);         // Read + filter + compute
float imu_get_heading(IMU *imu);   // Returns yaw in degrees

#endif