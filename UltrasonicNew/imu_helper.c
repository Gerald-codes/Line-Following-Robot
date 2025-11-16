#include "imu_helper.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include <stdio.h>

// GY-511 (LSM303DLHC) I2C addresses
#define ACC_ADDR 0x19
#define MAG_ADDR 0x1E

// Accelerometer registers
#define CTRL_REG1_A 0x20
#define OUT_X_L_A   0x28

// Magnetometer registers
#define CRA_REG_M   0x00
#define MR_REG_M    0x02
#define OUT_X_H_M   0x03

// Filtering constants
#define FILTER_SIZE 3
#define ALPHA       0.2f
#define RAD_TO_DEG  57.2957795f

// Global IMU context
static IMUContext imu_ctx;
static bool imu_initialized = false;

// SMA buffers
static int16_t acc_buf_x[FILTER_SIZE] = {0};
static int16_t acc_buf_y[FILTER_SIZE] = {0};
static int16_t acc_buf_z[FILTER_SIZE] = {0};
static int16_t mag_buf_x[FILTER_SIZE] = {0};
static int16_t mag_buf_y[FILTER_SIZE] = {0};
static int16_t mag_buf_z[FILTER_SIZE] = {0};
static int buf_index = 0;

// EMA variables
static float ax_ema = 0, ay_ema = 0, az_ema = 0;
static float mx_ema = 0, my_ema = 0, mz_ema = 0;

// Magnetometer calibration (from your teammate's values)
static float mx_offset = 20;
static float my_offset = 9;
static float mz_offset = -152;
static float mx_scale = 0.975;
static float my_scale = 0.936;
static float mz_scale = 1.105;

// Current yaw value
static float current_yaw = 0.0f;

// Helper: Compute SMA
static int16_t compute_sma(int16_t *buf) {
    int32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++)
        sum += buf[i];
    return (int16_t)(sum / FILTER_SIZE);
}

// Read accelerometer
static void read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t reg = OUT_X_L_A | 0x80;
    uint8_t buf[6];
    i2c_write_blocking(IMU_I2C_PORT, ACC_ADDR, &reg, 1, true);
    i2c_read_blocking(IMU_I2C_PORT, ACC_ADDR, buf, 6, false);

    *ax = (int16_t)(buf[1] << 8 | buf[0]);
    *ay = (int16_t)(buf[3] << 8 | buf[2]);
    *az = (int16_t)(buf[5] << 8 | buf[4]);
}

// Read magnetometer
static void read_mag(int16_t *mx, int16_t *my, int16_t *mz) {
    uint8_t reg = OUT_X_H_M;
    uint8_t buf[6];
    i2c_write_blocking(IMU_I2C_PORT, MAG_ADDR, &reg, 1, true);
    i2c_read_blocking(IMU_I2C_PORT, MAG_ADDR, buf, 6, false);

    *mx = (int16_t)(buf[0] << 8 | buf[1]);
    *mz = (int16_t)(buf[2] << 8 | buf[3]);
    *my = (int16_t)(buf[4] << 8 | buf[5]);
}

// Calculate yaw from filtered magnetometer and accelerometer data
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

bool imu_init(void) {
    // Initialize I2C
    i2c_init(IMU_I2C_PORT, IMU_I2C_FREQ);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);
    
    sleep_ms(500);
    printf("[IMU] Initializing GY-511...\n");
    
    // Initialize accelerometer (50Hz, all axes)
    uint8_t init_acc[] = {CTRL_REG1_A, 0x27};
    int ret = i2c_write_blocking(IMU_I2C_PORT, ACC_ADDR, init_acc, 2, false);
    if (ret != 2) {
        printf("[IMU] Failed to initialize accelerometer\n");
        return false;
    }
    
    // Initialize magnetometer (30Hz output)
    uint8_t init_mag1[] = {CRA_REG_M, 0x14};
    ret = i2c_write_blocking(IMU_I2C_PORT, MAG_ADDR, init_mag1, 2, false);
    if (ret != 2) {
        printf("[IMU] Failed to initialize magnetometer config\n");
        return false;
    }
    
    // Set continuous conversion mode
    uint8_t init_mag2[] = {MR_REG_M, 0x00};
    ret = i2c_write_blocking(IMU_I2C_PORT, MAG_ADDR, init_mag2, 2, false);
    if (ret != 2) {
        printf("[IMU] Failed to set magnetometer mode\n");
        return false;
    }
    
    // Initialize context
    imu_ctx.initial_heading = 0.0f;
    imu_ctx.current_heading = 0.0f;
    imu_ctx.target_heading = 0.0f;
    imu_ctx.calibrated = false;
    
    imu_initialized = true;
    printf("[IMU] ✓ GY-511 initialized successfully\n");
    
    // Warm up filters
    printf("[IMU] Warming up filters...\n");
    for (int i = 0; i < 20; i++) {
        imu_update();
        sleep_ms(50);
    }
    
    printf("[IMU] ✓ Ready\n");
    return true;
}

void imu_update(void) {
    if (!imu_initialized) return;
    
    int16_t ax, ay, az, mx, my, mz;
    read_accel(&ax, &ay, &az);
    read_mag(&mx, &my, &mz);
    
    // Update SMA buffers
    acc_buf_x[buf_index] = ax;
    acc_buf_y[buf_index] = ay;
    acc_buf_z[buf_index] = az;
    mag_buf_x[buf_index] = mx;
    mag_buf_y[buf_index] = my;
    mag_buf_z[buf_index] = mz;
    
    buf_index = (buf_index + 1) % FILTER_SIZE;
    
    // Compute SMA
    int16_t ax_sma = compute_sma(acc_buf_x);
    int16_t ay_sma = compute_sma(acc_buf_y);
    int16_t az_sma = compute_sma(acc_buf_z);
    int16_t mx_sma = compute_sma(mag_buf_x);
    int16_t my_sma = compute_sma(mag_buf_y);
    int16_t mz_sma = compute_sma(mag_buf_z);
    
    // Apply EMA + calibration
    ax_ema = ALPHA * ax_sma + (1 - ALPHA) * ax_ema;
    ay_ema = ALPHA * ay_sma + (1 - ALPHA) * ay_ema;
    az_ema = ALPHA * az_sma + (1 - ALPHA) * az_ema;
    
    mx_ema = ALPHA * ((mx_sma - mx_offset) * mx_scale) + (1 - ALPHA) * mx_ema;
    my_ema = ALPHA * ((my_sma - my_offset) * my_scale) + (1 - ALPHA) * my_ema;
    mz_ema = ALPHA * ((mz_sma - mz_offset) * mz_scale) + (1 - ALPHA) * mz_ema;
    
    // Calculate current yaw
    current_yaw = calculate_yaw();
    imu_ctx.current_heading = current_yaw;
}

float imu_get_heading(void) {
    return current_yaw;
}

void imu_reset_heading(void) {
    imu_update();  // Get latest reading
    imu_ctx.initial_heading = current_yaw;
    imu_ctx.calibrated = true;
    printf("[IMU] Heading reset to %.1f°\n", current_yaw);
}

float imu_get_relative_heading(void) {
    if (!imu_ctx.calibrated) {
        return 0.0f;
    }
    
    float diff = current_yaw - imu_ctx.initial_heading;
    
    // Normalize to -180 to +180
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    
    return diff;
}

bool imu_has_turned(float target_angle, float tolerance) {
    float current_turn = imu_get_relative_heading();
    float error = fabsf(current_turn - target_angle);
    
    return error <= tolerance;
}

float imu_get_turn_progress(float target_angle) {
    float current_turn = imu_get_relative_heading();
    float progress = fabsf(current_turn) / fabsf(target_angle);
    
    if (progress > 1.0f) progress = 1.0f;
    if (progress < 0.0f) progress = 0.0f;
    
    return progress;
}

void imu_start_avoidance(void) {
    imu_reset_heading();
    printf("[IMU] Avoidance started, heading locked at %.1f°\n", imu_ctx.initial_heading);
}

bool imu_is_ready(void) {
    return imu_initialized && imu_ctx.calibrated;
}