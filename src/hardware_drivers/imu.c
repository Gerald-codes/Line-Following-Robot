/**
 * @file    imu.c
 * @brief   Unified IMU driver for GY-511 (LSM303DLHC)
 * @details
 *   Combines struct-based and helper-based interfaces for real-time
 *   accelerometer/magnetometer readings, heading calculation, calibration,
 *   and obstacle avoidance. Initializes I2C, handles filtering, and exposes
 *   context methods for both line-following and turn-tracking applications.
 */

#include "imu.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include <stdio.h>

/* GY-511 (LSM303DLHC) I2C addresses and relevant registers */
#define ACC_ADDR 0x19
#define MAG_ADDR 0x1E
#define CTRL_REG1_A 0x20
#define OUT_X_L_A   0x28
#define CRA_REG_M   0x00
#define MR_REG_M    0x02
#define OUT_X_H_M   0x03

/* Filtering constants */
#define FILTER_SIZE 3
#define ALPHA       0.2f
#define RAD_TO_DEG  57.2957795f

/* Magnetometer calibration values */
static float mx_offset = 20;
static float my_offset = 9;
static float mz_offset = -152;
static float mx_scale = 0.975;
static float my_scale = 0.936;
static float mz_scale = 1.105;

/* Shared buffer and state */
static int16_t acc_buf_x[FILTER_SIZE] = {0};
static int16_t acc_buf_y[FILTER_SIZE] = {0};
static int16_t acc_buf_z[FILTER_SIZE] = {0};
static int16_t mag_buf_x[FILTER_SIZE] = {0};
static int16_t mag_buf_y[FILTER_SIZE] = {0};
static int16_t mag_buf_z[FILTER_SIZE] = {0};
static int buf_index = 0;
static float ax_ema = 0, ay_ema = 0, az_ema = 0;
static float mx_ema = 0, my_ema = 0, mz_ema = 0;
static float current_yaw = 0.0f;
static bool hardware_initialized = false;
static IMUContext helper_ctx;
static bool helper_initialized = false;

/**
 * @brief   Compute simple moving average (SMA) for a buffer
 * @param   buf Pointer to buffer array
 * @return  Averaged value
 */
static int16_t compute_sma(int16_t *buf) {
    int32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++)
        sum += buf[i];
    return (int16_t)(sum / FILTER_SIZE);
}

/**
 * @brief   Read raw accelerometer values from IMU hardware
 * @param   ax Pointer to store X axis value
 * @param   ay Pointer to store Y axis value
 * @param   az Pointer to store Z axis value
 */
static void read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t reg = OUT_X_L_A | 0x80;
    uint8_t buf[6];
    i2c_write_blocking(IMU_I2C_PORT, ACC_ADDR, &reg, 1, true);
    i2c_read_blocking(IMU_I2C_PORT, ACC_ADDR, buf, 6, false);

    *ax = (int16_t)(buf[1] << 8 | buf[0]);
    *ay = (int16_t)(buf[3] << 8 | buf[2]);
    *az = (int16_t)(buf[5] << 8 | buf[4]);
}

/**
 * @brief   Read raw magnetometer values from IMU hardware
 * @param   mx Pointer to store X axis value
 * @param   my Pointer to store Y axis value
 * @param   mz Pointer to store Z axis value
 */
static void read_mag(int16_t *mx, int16_t *my, int16_t *mz) {
    uint8_t reg = OUT_X_H_M;
    uint8_t buf[6];
    i2c_write_blocking(IMU_I2C_PORT, MAG_ADDR, &reg, 1, true);
    i2c_read_blocking(IMU_I2C_PORT, MAG_ADDR, buf, 6, false);

    *mx = (int16_t)(buf[0] << 8 | buf[1]);
    *mz = (int16_t)(buf[2] << 8 | buf[3]);
    *my = (int16_t)(buf[4] << 8 | buf[5]);
}

/**
 * @brief   Calculate yaw angle from filtered sensor data
 * @return  Calculated yaw in degrees
 */
static float calculate_yaw(void) {
    float roll  = atan2f(ay_ema, az_ema) * RAD_TO_DEG;
    float pitch = atan2f(-ax_ema, sqrtf(ay_ema*ay_ema + az_ema*az_ema)) * RAD_TO_DEG;

    float radRoll  = roll / RAD_TO_DEG;
    float radPitch = pitch / RAD_TO_DEG;

    float Xh = mx_ema * cosf(radPitch) + mz_ema * sinf(radPitch);
    float Yh = mx_ema * sinf(radRoll) * sinf(radPitch) + 
               my_ema * cosf(radRoll) - 
               mz_ema * sinf(radRoll) * cosf(radPitch);

    float yaw = atan2f(-Yh, Xh) * RAD_TO_DEG;
    return yaw;
}

/**
 * @brief   Internal I2C and sensor hardware initialization
 */
static void init_hardware(void) {
    if (hardware_initialized) return;
    i2c_init(IMU_I2C_PORT, IMU_I2C_FREQ);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);

    sleep_ms(500);
    printf("[IMU] Initializing GY-511...\n");

    uint8_t init_acc[] = {CTRL_REG1_A, 0x27};
    int ret = i2c_write_blocking(IMU_I2C_PORT, ACC_ADDR, init_acc, 2, false);
    if (ret != 2) { printf("[IMU] Failed to initialize accelerometer\n"); return; }

    uint8_t init_mag1[] = {CRA_REG_M, 0x14};
    ret = i2c_write_blocking(IMU_I2C_PORT, MAG_ADDR, init_mag1, 2, false);
    if (ret != 2) { printf("[IMU] Failed to initialize magnetometer config\n"); return; }

    uint8_t init_mag2[] = {MR_REG_M, 0x00};
    ret = i2c_write_blocking(IMU_I2C_PORT, MAG_ADDR, init_mag2, 2, false);
    if (ret != 2) { printf("[IMU] Failed to set magnetometer mode\n"); return; }

    hardware_initialized = true;
    printf("[IMU] ✓ GY-511 initialized on I2C0 (GP%d=SDA, GP%d=SCL)\n", 
           IMU_SDA_PIN, IMU_SCL_PIN);
}

/**
 * @brief   Update filtered sensor state for both interfaces
 */
static void update_sensors(void) {
    if (!hardware_initialized) return;
    int16_t ax, ay, az, mx, my, mz;
    read_accel(&ax, &ay, &az);
    read_mag(&mx, &my, &mz);

    acc_buf_x[buf_index] = ax;
    acc_buf_y[buf_index] = ay;
    acc_buf_z[buf_index] = az;
    mag_buf_x[buf_index] = mx;
    mag_buf_y[buf_index] = my;
    mag_buf_z[buf_index] = mz;

    buf_index = (buf_index + 1) % FILTER_SIZE;

    int16_t ax_sma = compute_sma(acc_buf_x);
    int16_t ay_sma = compute_sma(acc_buf_y);
    int16_t az_sma = compute_sma(acc_buf_z);
    int16_t mx_sma = compute_sma(mag_buf_x);
    int16_t my_sma = compute_sma(mag_buf_y);
    int16_t mz_sma = compute_sma(mag_buf_z);

    ax_ema = ALPHA * ax_sma + (1 - ALPHA) * ax_ema;
    ay_ema = ALPHA * ay_sma + (1 - ALPHA) * ay_ema;
    az_ema = ALPHA * az_sma + (1 - ALPHA) * az_ema;

    mx_ema = ALPHA * ((mx_sma - mx_offset) * mx_scale) + (1 - ALPHA) * mx_ema;
    my_ema = ALPHA * ((my_sma - my_offset) * my_scale) + (1 - ALPHA) * my_ema;
    mz_ema = ALPHA * ((mz_sma - mz_offset) * mz_scale) + (1 - ALPHA) * mz_ema;

    current_yaw = calculate_yaw();
}

/* ====================== Struct-based IMU interface ======================== */

/**
 * @brief   Initialize IMU for structured sensor access
 * @param   imu IMU structure pointer
 */
void imu_init(IMU *imu) {
    init_hardware();
    imu->heading_offset = 0;
    imu->calibrated = false;
    printf("[IMU] Struct interface initialized\n");
}

/**
 * @brief   Calibrate IMU and set heading reference
 * @param   imu IMU structure pointer
 */
void imu_calibrate(IMU *imu) {
    printf("[IMU] Calibrating...\n");
    float heading_sum = 0;
    const int samples = 10;
    for (int i = 0; i < samples; i++) {
        imu_update(imu);
        heading_sum += imu->yaw;
        sleep_ms(100);
    }
    imu->heading_offset = heading_sum / samples;
    imu->calibrated = true;
    printf("[IMU] ✓ Calibrated: offset = %.1f°\n", imu->heading_offset);
}

/**
 * @brief   Update IMU struct values from sensor filter state
 * @param   imu IMU structure pointer
 */
void imu_update(IMU *imu) {
    update_sensors();
    float roll = atan2f(ay_ema, az_ema);
    float pitch = atan2f(-ax_ema, sqrtf(ay_ema*ay_ema + az_ema*az_ema));
    imu->roll = roll * RAD_TO_DEG;
    imu->pitch = pitch * RAD_TO_DEG;
    imu->yaw = current_yaw;
}

/**
 * @brief   Get heading (relative to calibration) from struct
 * @param   imu IMU structure pointer
 * @return  Relative heading in degrees
 */
float imu_get_heading(IMU *imu) {
    float h = imu->yaw - imu->heading_offset;
    while (h > 180.0f) h -= 360.0f;
    while (h < -180.0f) h += 360.0f;
    return h;
}

/* ====================== Helper-based IMU interface ======================== */

/**
 * @brief   Initialize the helper context interface for IMU
 * @return  true if hardware is ready, false otherwise
 */
bool imu_helper_init(void) {
    init_hardware();
    if (!hardware_initialized) return false;
    helper_ctx.initial_heading = 0.0f;
    helper_ctx.current_heading = 0.0f;
    helper_ctx.target_heading = 0.0f;
    helper_ctx.calibrated = false;
    helper_initialized = true;
    printf("[IMU] Helper interface initialized\n");
    printf("[IMU] Warming up filters...\n");
    for (int i = 0; i < 20; i++) {
        imu_helper_update();
        sleep_ms(50);
    }
    printf("[IMU] ✓ Ready\n");
    return true;
}

/**
 * @brief   Update helper context with latest sensor values
 */
void imu_helper_update(void) {
    update_sensors();
    helper_ctx.current_heading = current_yaw;
}

/**
 * @brief   Get current absolute heading (helper context)
 * @return  Heading in degrees
 */
float imu_helper_get_heading(void) {
    return current_yaw;
}

/**
 * @brief   Reset heading reference in helper context
 */
void imu_helper_reset_heading(void) {
    imu_helper_update();
    helper_ctx.initial_heading = current_yaw;
    helper_ctx.calibrated = true;
    printf("[IMU] Heading reset to %.1f°\n", current_yaw);
}

/**
 * @brief   Get relative heading change since reset
 * @return  Heading change in degrees (-180 to +180)
 */
float imu_helper_get_relative_heading(void) {
    if (!helper_ctx.calibrated) return 0.0f;
    float diff = current_yaw - helper_ctx.initial_heading;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

/**
 * @brief   Check if the robot has turned the target angle (with tolerance)
 * @param   target_angle Desired turn angle (degrees)
 * @param   tolerance Acceptable error margin (degrees)
 * @return  true if within tolerance, false otherwise
 */
bool imu_helper_has_turned(float target_angle, float tolerance) {
    float current_turn = imu_helper_get_relative_heading();
    float error = fabsf(current_turn - target_angle);
    return error <= tolerance;
}

/**
 * @brief   Get percent progress toward target turn
 * @param   target_angle Target angle in degrees
 * @return  Progress (0.0 to 1.0)
 */
float imu_helper_get_turn_progress(float target_angle) {
    float current_turn = imu_helper_get_relative_heading();
    float progress = fabsf(current_turn) / fabsf(target_angle);
    if (progress > 1.0f) progress = 1.0f;
    if (progress < 0.0f) progress = 0.0f;
    return progress;
}

/**
 * @brief   Start a new avoidance tracking maneuver in helper context
 */
void imu_helper_start_avoidance(void) {
    imu_helper_reset_heading();
    printf("[IMU] Avoidance started, heading locked at %.1f°\n", helper_ctx.initial_heading);
}

/**
 * @brief   Check if IMU helper is ready and calibrated
 * @return  true if ready, false otherwise
 */
bool imu_helper_is_ready(void) {
    return helper_initialized && helper_ctx.calibrated;
}
