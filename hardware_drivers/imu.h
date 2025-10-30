/**
 * imu.h - SIMULATED VERSION
 * IMU sensor interface (simulated - no hardware needed)
 */

#ifndef IMU_H
#define IMU_H

#include "pico/stdlib.h"
#include "filters.h"

#define RAD_TO_DEG   57.2957795f
#define DEG_TO_RAD   0.0174533f

typedef struct {
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float heading;
    float gyro_bias_x, gyro_bias_y, gyro_bias_z;
    ComplementaryFilter filter;
    uint32_t last_update_time;
} IMU;

void imu_init(IMU *imu);
void imu_calibrate(IMU *imu);
void imu_update(IMU *imu);
float imu_get_heading(IMU *imu);
void imu_get_gyro(IMU *imu, float *gx, float *gy, float *gz);
void imu_get_accel(IMU *imu, float *ax, float *ay, float *az);
void imu_reset_heading(IMU *imu);
void imu_set_heading(IMU *imu, float heading);
void imu_print_data(IMU *imu);

#endif // IMU_H