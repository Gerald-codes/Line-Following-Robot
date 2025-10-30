/**
 * imu.c - SIMULATED VERSION (No Hardware Needed)
 * Simulates IMU behavior for testing without actual MPU6050 sensor
 * Estimates heading from wheel encoder difference
 */

#include "imu.h"
#include "filters.h"
#include "config.h"
#include "encoder.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Simulated sensor characteristics
#define SIMULATED_GYRO_NOISE 0.5f
#define SIMULATED_GYRO_DRIFT 0.02f
#define SIMULATED_ACCEL_NOISE 0.02f

// Static variables for simulation
static int32_t last_left_encoder = 0;
static int32_t last_right_encoder = 0;
static float accumulated_drift = 0.0f;

// Helper function to generate random noise
static float random_noise(float magnitude) {
    return ((float)rand() / (float)RAND_MAX - 0.5f) * 2.0f * magnitude;
}

void imu_init(IMU *imu) {
    printf("IMU SIMULATION MODE (No hardware sensor needed)\n");
    printf("Heading estimated from wheel encoders\n\n");
    
    imu->heading = 0.0f;
    imu->gyro_x = 0.0f;
    imu->gyro_y = 0.0f;
    imu->gyro_z = 0.0f;
    imu->accel_x = 0.0f;
    imu->accel_y = 0.0f;
    imu->accel_z = 1.0f;
    imu->gyro_bias_x = 0.0f;
    imu->gyro_bias_y = 0.0f;
    imu->gyro_bias_z = 0.0f;
    imu->last_update_time = to_ms_since_boot(get_absolute_time());
    
    complementary_filter_init(&imu->filter, IMU_FILTER_ALPHA);
    
    last_left_encoder = get_left_encoder();
    last_right_encoder = get_right_encoder();
    
    printf("✓ Simulated IMU initialized\n");
}

void imu_calibrate(IMU *imu) {
    printf("Simulated IMU calibration...\n");
    
    imu->gyro_bias_x = random_noise(0.3f);
    imu->gyro_bias_y = random_noise(0.3f);
    imu->gyro_bias_z = random_noise(0.3f);
    
    printf("Calibration complete!\n");
    printf("Simulated gyro bias: X=%.2f, Y=%.2f, Z=%.2f deg/s\n", 
           imu->gyro_bias_x, imu->gyro_bias_y, imu->gyro_bias_z);
    
    sleep_ms(500);
}

void imu_update(IMU *imu) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    float dt = (current_time - imu->last_update_time) / 1000.0f;
    imu->last_update_time = current_time;
    
    int32_t current_left = get_left_encoder();
    int32_t current_right = get_right_encoder();
    
    int32_t left_diff = current_left - last_left_encoder;
    int32_t right_diff = current_right - last_right_encoder;
    
    float left_distance_mm = left_diff * MM_PER_PULSE;
    float right_distance_mm = right_diff * MM_PER_PULSE;
    
    float distance_diff = right_distance_mm - left_distance_mm;
    float heading_change_from_wheels = (distance_diff / WHEEL_BASE_MM) * RAD_TO_DEG;
    
    if (dt > 0.001f) {
        imu->gyro_z = (heading_change_from_wheels / dt) + random_noise(SIMULATED_GYRO_NOISE);
    } else {
        imu->gyro_z = 0.0f;
    }
    
    accumulated_drift += SIMULATED_GYRO_DRIFT * dt;
    imu->gyro_z += accumulated_drift;
    imu->gyro_z -= imu->gyro_bias_z;
    
    imu->accel_x = random_noise(SIMULATED_ACCEL_NOISE);
    imu->accel_y = random_noise(SIMULATED_ACCEL_NOISE);
    imu->accel_z = 1.0f + random_noise(SIMULATED_ACCEL_NOISE * 0.5f);
    
    imu->gyro_x = random_noise(SIMULATED_GYRO_NOISE * 0.3f);
    imu->gyro_y = random_noise(SIMULATED_GYRO_NOISE * 0.3f);
    
    float gyro_heading = imu->heading + (imu->gyro_z * dt);
    float encoder_heading = imu->heading + heading_change_from_wheels;
    
    imu->heading = complementary_filter_update(&imu->filter, 
                                               gyro_heading, 
                                               encoder_heading);
    
    while (imu->heading > 180.0f) imu->heading -= 360.0f;
    while (imu->heading < -180.0f) imu->heading += 360.0f;
    
    last_left_encoder = current_left;
    last_right_encoder = current_right;
}

float imu_get_heading(IMU *imu) {
    return imu->heading;
}

void imu_get_gyro(IMU *imu, float *gx, float *gy, float *gz) {
    if (gx) *gx = imu->gyro_x;
    if (gy) *gy = imu->gyro_y;
    if (gz) *gz = imu->gyro_z;
}

void imu_get_accel(IMU *imu, float *ax, float *ay, float *az) {
    if (ax) *ax = imu->accel_x;
    if (ay) *ay = imu->accel_y;
    if (az) *az = imu->accel_z;
}

void imu_reset_heading(IMU *imu) {
    imu->heading = 0.0f;
    accumulated_drift = 0.0f;
    complementary_filter_reset(&imu->filter);
    
    last_left_encoder = get_left_encoder();
    last_right_encoder = get_right_encoder();
}

void imu_set_heading(IMU *imu, float heading) {
    imu->heading = heading;
}

void imu_print_data(IMU *imu) {
    printf("[SIM IMU] Heading=%.1f° | Gyro Z=%.1f°/s | Accel: X=%.2fg Y=%.2fg Z=%.2fg\n",
           imu->heading, imu->gyro_z, imu->accel_x, imu->accel_y, imu->accel_z);
}