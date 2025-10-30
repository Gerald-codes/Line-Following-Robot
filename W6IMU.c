#include <stdio.h>
#include <stdint.h>
#include <math.h>        // <<< ADDED
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define SDA_PIN  0
#define SCL_PIN  1

#define ACC_ADDR 0x19
#define MAG_ADDR 0x1E

#define CTRL_REG1_A 0x20
#define OUT_X_L_A   0x28
#define CRA_REG_M   0x00
#define MR_REG_M    0x02
#define OUT_X_H_M   0x03

#define FILTER_SIZE 3
#define ALPHA       0.2f
#define RAD_TO_DEG  57.2957795f   // <<< ADDED

// ===== Simple Moving Average Buffers =====
int16_t acc_buf_x[FILTER_SIZE] = {0};
int16_t acc_buf_y[FILTER_SIZE] = {0};
int16_t acc_buf_z[FILTER_SIZE] = {0};
int16_t mag_buf_x[FILTER_SIZE] = {0};
int16_t mag_buf_y[FILTER_SIZE] = {0};
int16_t mag_buf_z[FILTER_SIZE] = {0};
int buf_index = 0;

// ===== Exponential Moving Average Variables =====
float ax_ema = 0, ay_ema = 0, az_ema = 0;
float mx_ema = 0, my_ema = 0, mz_ema = 0;

// ===== Magnetometer Calibration (from your values) =====
float mx_offset = 20;
float my_offset = 9;
float mz_offset = -152;

float mx_scale = 0.975;
float my_scale = 0.936;
float mz_scale = 1.105;

// ===== Helper: Compute SMA =====
int16_t compute_sma(int16_t *buf) {
    int32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++)
        sum += buf[i];
    return (int16_t)(sum / FILTER_SIZE);
}

// ===== Read Accelerometer =====
void read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t reg = OUT_X_L_A | 0x80;
    uint8_t buf[6];
    i2c_write_blocking(I2C_PORT, ACC_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ACC_ADDR, buf, 6, false);

    *ax = (int16_t)(buf[1] << 8 | buf[0]);
    *ay = (int16_t)(buf[3] << 8 | buf[2]);
    *az = (int16_t)(buf[5] << 8 | buf[4]);
}

// ===== Read Magnetometer =====
void read_mag(int16_t *mx, int16_t *my, int16_t *mz) {
    uint8_t reg = OUT_X_H_M;
    uint8_t buf[6];
    i2c_write_blocking(I2C_PORT, MAG_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MAG_ADDR, buf, 6, false);

    *mx = (int16_t)(buf[0] << 8 | buf[1]);
    *mz = (int16_t)(buf[2] << 8 | buf[3]);
    *my = (int16_t)(buf[4] << 8 | buf[5]);
}

int main() {
    stdio_init_all();

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    sleep_ms(500);
    printf("GY-511 Ready.\n");

    uint8_t init_acc[] = {CTRL_REG1_A, 0x27};
    i2c_write_blocking(I2C_PORT, ACC_ADDR, init_acc, 2, false);

    uint8_t init_mag1[] = {CRA_REG_M, 0x14};
    i2c_write_blocking(I2C_PORT, MAG_ADDR, init_mag1, 2, false);

    uint8_t init_mag2[] = {MR_REG_M, 0x00};
    i2c_write_blocking(I2C_PORT, MAG_ADDR, init_mag2, 2, false);

    while (true) {
        int16_t ax, ay, az, mx, my, mz;
        read_accel(&ax, &ay, &az);
        read_mag(&mx, &my, &mz);

        // Update buffers
        acc_buf_x[buf_index] = ax;
        acc_buf_y[buf_index] = ay;
        acc_buf_z[buf_index] = az;
        mag_buf_x[buf_index] = mx;
        mag_buf_y[buf_index] = my;
        mag_buf_z[buf_index] = mz;

        buf_index = (buf_index + 1) % FILTER_SIZE;

        // SMA
        int16_t ax_sma = compute_sma(acc_buf_x);
        int16_t ay_sma = compute_sma(acc_buf_y);
        int16_t az_sma = compute_sma(acc_buf_z);
        int16_t mx_sma = compute_sma(mag_buf_x);
        int16_t my_sma = compute_sma(mag_buf_y);
        int16_t mz_sma = compute_sma(mag_buf_z);

        // EMA + Calibration
        ax_ema = ALPHA * ax_sma + (1 - ALPHA) * ax_ema;
        ay_ema = ALPHA * ay_sma + (1 - ALPHA) * ay_ema;
        az_ema = ALPHA * az_sma + (1 - ALPHA) * az_ema;

        mx_ema = ALPHA * ((mx_sma - mx_offset) * mx_scale) + (1 - ALPHA) * mx_ema;
        my_ema = ALPHA * ((my_sma - my_offset) * my_scale) + (1 - ALPHA) * my_ema;
        mz_ema = ALPHA * ((mz_sma - mz_offset) * mz_scale) + (1 - ALPHA) * mz_ema;

        // ======== <<< ADDED: ROLL, PITCH, YAW CALCULATION ========

        float roll  = atan2f(ay_ema, az_ema) * RAD_TO_DEG;
        float pitch = atan2f(-ax_ema, sqrtf(ay_ema*ay_ema + az_ema*az_ema)) * RAD_TO_DEG;

        float radRoll  = roll / RAD_TO_DEG;
        float radPitch = pitch / RAD_TO_DEG;

        float Xh = mx_ema * cosf(radPitch) + mz_ema * sinf(radPitch);
        float Yh = mx_ema * sinf(radRoll) * sinf(radPitch) + 
                   my_ema * cosf(radRoll) - 
                   mz_ema * sinf(radRoll) * cosf(radPitch);

        float yaw = atan2f(-Yh, Xh) * RAD_TO_DEG;

        // ======== OUTPUT ========
        // printf("\nRAW:\tACC X=%d Y=%d Z=%d\tMAG X=%d Y=%d Z=%d\n",ax, ay, az, mx, my, mz);

        // printf("FILT:\tACC X=%.1f Y=%.1f Z=%.1f\tMAG X=%.1f Y=%.1f Z=%.1f\n",
        //     ax_ema, ay_ema, az_ema, mx_ema, my_ema, mz_ema);

        // printf("ORI:\tRoll=%.1f  Pitch=%.1f  Yaw=%.1f\n", roll, pitch, yaw);

        printf("%d,%d,%d,%.2f,%.2f,%.2f,%d,%d,%d,%.2f,%.2f,%.2f,%.2f\n",
            ax, ay, az,
            ax_ema, ay_ema, az_ema,
            mx, my, mz,
            mx_ema, my_ema, mz_ema,
            yaw);

        sleep_ms(200);
    }
}
