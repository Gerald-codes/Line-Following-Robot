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

// ----------- CONFIGURABLE PARAMETERS -----------
#define ALPHA 0.2f     // Low-pass filter factor (0.1 = very smooth, 0.5 = responsive)
#define ACC_SCALE 16384.0f   // LSB/g for ±2g
#define MAG_SCALE 0.00014f   // Gauss/LSB typical
// ------------------------------------------------

// Global offsets (to be determined by calibration)
float acc_offset_x = 0, acc_offset_y = 0, acc_offset_z = 0;
float mag_offset_x = 0, mag_offset_y = 0, mag_offset_z = 0;

// Filtered outputs
float acc_fx = 0, acc_fy = 0, acc_fz = 0;
float mag_fx = 0, mag_fy = 0, mag_fz = 0;

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

// ------------ CALIBRATION ROUTINE ------------
void calibrate_sensors() {
    printf("Calibrating... keep IMU still!\n");
    const int samples = 100;
    long ax_sum = 0, ay_sum = 0, az_sum = 0;
    long mx_sum = 0, my_sum = 0, mz_sum = 0;

    for (int i = 0; i < samples; i++) {
        int16_t ax, ay, az, mx, my, mz;
        read_accel(&ax, &ay, &az);
        read_mag(&mx, &my, &mz);
        ax_sum += ax; ay_sum += ay; az_sum += az;
        mx_sum += mx; my_sum += my; mz_sum += mz;
        sleep_ms(10);
    }

    acc_offset_x = (float)ax_sum / samples;
    acc_offset_y = (float)ay_sum / samples;
    acc_offset_z = ((float)az_sum / samples) - ACC_SCALE; // subtract gravity (1g)
    mag_offset_x = (float)mx_sum / samples;
    mag_offset_y = (float)my_sum / samples;
    mag_offset_z = (float)mz_sum / samples;

    printf("Calibration done.\n");
}
// --------------------------------------------

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

    calibrate_sensors();  // <--- Run once at startup

    while (true) {
        int16_t ax_raw, ay_raw, az_raw, mx_raw, my_raw, mz_raw;
        read_accel(&ax_raw, &ay_raw, &az_raw);
        read_mag(&mx_raw, &my_raw, &mz_raw);

        // Apply calibration (subtract offset)
        float ax = (ax_raw - acc_offset_x) / ACC_SCALE;
        float ay = (ay_raw - acc_offset_y) / ACC_SCALE;
        float az = (az_raw - acc_offset_z) / ACC_SCALE;
        float mx = (mx_raw - mag_offset_x) * MAG_SCALE;
        float my = (my_raw - mag_offset_y) * MAG_SCALE;
        float mz = (mz_raw - mag_offset_z) * MAG_SCALE;

        // Apply simple low-pass filter
        acc_fx = ALPHA * ax + (1 - ALPHA) * acc_fx;
        acc_fy = ALPHA * ay + (1 - ALPHA) * acc_fy;
        acc_fz = ALPHA * az + (1 - ALPHA) * acc_fz;
        mag_fx = ALPHA * mx + (1 - ALPHA) * mag_fx;
        mag_fy = ALPHA * my + (1 - ALPHA) * mag_fy;
        mag_fz = ALPHA * mz + (1 - ALPHA) * mag_fz;

        printf("Accel[g]: X=%.2f Y=%.2f Z=%.2f | Mag[Gs]: X=%.2f Y=%.2f Z=%.2f\n",
               acc_fx, acc_fy, acc_fz, mag_fx, mag_fy, mag_fz);

        sleep_ms(200);
    }
}
