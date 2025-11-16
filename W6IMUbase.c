#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define SDA_PIN 0   // GPIO0
#define SCL_PIN 1   // GPIO1

// I2C addresses
#define ACC_ADDR 0x19  // Accelerometer
#define MAG_ADDR 0x1E  // Magnetometer

// Accelerometer registers
#define OUT_X_L_A 0x28
// Magnetometer registers
#define OUT_X_H_M 0x03

// Read two bytes from a register
int16_t read16(i2c_inst_t *i2c, uint8_t addr, uint8_t reg) {
    uint8_t buf[2];
    buf[0] = reg | 0x80; // Auto-increment
    i2c_write_blocking(i2c, addr, buf, 1, true);
    i2c_read_blocking(i2c, addr, buf, 2, false);
    return (int16_t)((buf[1] << 8) | buf[0]);
}

int main() {
    stdio_init_all();

    // Initialize I2C
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // --- Add initialization here ---
    uint8_t init_acc[] = {0x20, 0x27};
    i2c_write_blocking(I2C_PORT, ACC_ADDR, init_acc, 2, false);

    uint8_t init_mag[] = {0x02, 0x00};
    i2c_write_blocking(I2C_PORT, MAG_ADDR, init_mag, 2, false);

    printf("Starting GY-511 read loop...\n");

    while (1) {
        // Read accelerometer
        int16_t ax = read16(I2C_PORT, ACC_ADDR, OUT_X_L_A);
        int16_t ay = read16(I2C_PORT, ACC_ADDR, OUT_X_L_A + 2);
        int16_t az = read16(I2C_PORT, ACC_ADDR, OUT_X_L_A + 4);

        // Read magnetometer
        int16_t mx = read16(I2C_PORT, MAG_ADDR, OUT_X_H_M);
        int16_t my = read16(I2C_PORT, MAG_ADDR, OUT_X_H_M + 2);
        int16_t mz = read16(I2C_PORT, MAG_ADDR, OUT_X_H_M + 4);

        printf("Accel: X=%d Y=%d Z=%d | Mag: X=%d Y=%d Z=%d\n",
               ax, ay, az, mx, my, mz);

        sleep_ms(500);
    }
}


