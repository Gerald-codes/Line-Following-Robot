#include <stdio.h>
#include <stdint.h>
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

#define FILTER_SIZE 3      // SMA window size
#define ALPHA       0.2f   // EMA smoothing factor

// ===== Simple Moving Average (SMA) Buffers =====
int16_t acc_buf_x[FILTER_SIZE] = {0};
int16_t acc_buf_y[FILTER_SIZE] = {0};
int16_t acc_buf_z[FILTER_SIZE] = {0};
int16_t mag_buf_x[FILTER_SIZE] = {0};
int16_t mag_buf_y[FILTER_SIZE] = {0};
int16_t mag_buf_z[FILTER_SIZE] = {0};
int buf_index = 0;

// ===== Exponential Moving Average (EMA) Variables =====
float ax_ema = 0, ay_ema = 0, az_ema = 0;
float mx_ema = 0, my_ema = 0, mz_ema = 0;

// ----- Helper: compute SMA of array -----
int16_t compute_sma(int16_t *buf) {
    int32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++)
        sum += buf[i];
    return (int16_t)(sum / FILTER_SIZE);
}

// ----- I2C Read functions -----
void read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t reg = OUT_X_L_A | 0x80;
    uint8_t buf[6];
    i2c_write_blocking(I2C_PORT, ACC_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ACC_ADDR, buf, 6, false);

    *ax = (int16_t)(buf[1] << 8 | buf[0]);
    *ay = (int16_t)(buf[3] << 8 | buf[2]);
    *az = (int16_t)(buf[5] << 8 | buf[4]);
}

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

    sleep_ms(1000);
    printf("Initializing GY-511 IMU...\n");

    uint8_t init_acc[] = {CTRL_REG1_A, 0x27};
    i2c_write_blocking(I2C_PORT, ACC_ADDR, init_acc, 2, false);

    uint8_t init_mag1[] = {CRA_REG_M, 0x14};
    i2c_write_blocking(I2C_PORT, MAG_ADDR, init_mag1, 2, false);
    uint8_t init_mag2[] = {MR_REG_M, 0x00};
    i2c_write_blocking(I2C_PORT, MAG_ADDR, init_mag2, 2, false);

    printf("GY-511 initialized successfully.\n");

    while (true) {
        int16_t ax, ay, az;
        int16_t mx, my, mz;

        read_accel(&ax, &ay, &az);
        read_mag(&mx, &my, &mz);

        // ---- Update SMA buffers ----
        acc_buf_x[buf_index] = ax;
        acc_buf_y[buf_index] = ay;
        acc_buf_z[buf_index] = az;
        mag_buf_x[buf_index] = mx;
        mag_buf_y[buf_index] = my;
        mag_buf_z[buf_index] = mz;

        buf_index = (buf_index + 1) % FILTER_SIZE;

        // ---- Compute SMA ----
        int16_t ax_sma = compute_sma(acc_buf_x);
        int16_t ay_sma = compute_sma(acc_buf_y);
        int16_t az_sma = compute_sma(acc_buf_z);
        int16_t mx_sma = compute_sma(mag_buf_x);
        int16_t my_sma = compute_sma(mag_buf_y);
        int16_t mz_sma = compute_sma(mag_buf_z);

        // ---- Apply EMA ----
        ax_ema = ALPHA * ax_sma + (1 - ALPHA) * ax_ema;
        ay_ema = ALPHA * ay_sma + (1 - ALPHA) * ay_ema;
        az_ema = ALPHA * az_sma + (1 - ALPHA) * az_ema;
        
        mx_ema = ALPHA * mx_sma + (1 - ALPHA) * mx_ema;
        my_ema = ALPHA * my_sma + (1 - ALPHA) * my_ema;
        mz_ema = ALPHA * mz_sma + (1 - ALPHA) * mz_ema;

        printf("Accel(raw): X=%d Y=%d Z=%d | SMA+EMA: X=%.1f Y=%.1f Z=%.1f | "
               "Mag(raw): X=%d Y=%d Z=%d | SMA+EMA: X=%.1f Y=%.1f Z=%.1f\n",
               ax, ay, az, ax_ema, ay_ema, az_ema,
               mx, my, mz, mx_ema, my_ema, mz_ema);

        sleep_ms(500);
    }
}
