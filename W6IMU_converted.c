#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define SDA_PIN  0   // GPIO0 → SDA (data)
#define SCL_PIN  1   // GPIO1 → SCL (clock

// I2C device addresses for GY-511 (LSM303DLHC)
//   - Accelerometer @ 0x19
//   - Magnetometer @ 0x1E
#define ACC_ADDR 0x19
#define MAG_ADDR 0x1E

// Accelerometer registers
//   Bit7-4: Data rate (0100 = 50Hz)
//   Bit3: Low-power enable
//   Bit2-0: Axis enable (XYZ all on = 111)
#define CTRL_REG1_A 0x20
// OUT_X_L_A (0x28): First data register for accelerometer
//   Contains X low byte, followed by X high, Y low, Y high, etc.
#define OUT_X_L_A   0x28

// Magnetometer registers
#define CRA_REG_M   0x00 // Config A: data rate, averaging
#define MR_REG_M    0x02 // Mode register: single or continuous conversion
#define OUT_X_H_M   0x03 // First data register (X high byte)

// Helper function to read 16-bit value (low + high byte)
int16_t read16_acc(uint8_t addr, uint8_t reg) {
    uint8_t buf[6]; 
    reg |= 0x80; // Enable auto-increment
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);  
    i2c_read_blocking(I2C_PORT, addr, buf, 6, false);   

    int16_t x = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t y = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t z = (int16_t)(buf[5] << 8 | buf[4]);
    return x; // placeholder (for illustration only)
}

// Read all accelerometer axes
void read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t reg = OUT_X_L_A | 0x80; // Set auto-increment bit (bit7=1)
    uint8_t buf[6]; // X, Y, Z data → each axis = 2 bytes (low + high)
    i2c_write_blocking(I2C_PORT, ACC_ADDR, &reg, 1, true); // Write register address we want to start reading from
    i2c_read_blocking(I2C_PORT, ACC_ADDR, buf, 6, false); // Read 6 bytes (X_L, X_H, Y_L, Y_H, Z_L, Z_H)

    // Combine high + low bytes into signed 16-bit values
    *ax = (int16_t)(buf[1] << 8 | buf[0]);
    *ay = (int16_t)(buf[3] << 8 | buf[2]);
    *az = (int16_t)(buf[5] << 8 | buf[4]);
}

// Read all magnetometer axes (note: X/Z/Y order in chip)
void read_mag(int16_t *mx, int16_t *my, int16_t *mz) {
    uint8_t reg = OUT_X_H_M;
    uint8_t buf[6]; // 6 bytes total for X, Z, Y axes
    i2c_write_blocking(I2C_PORT, MAG_ADDR, &reg, 1, true); // Write register address to start reading from
    i2c_read_blocking(I2C_PORT, MAG_ADDR, buf, 6, false); // Read XH, XL, ZH, ZL, YH, YL

    *mx = (int16_t)(buf[0] << 8 | buf[1]);
    *mz = (int16_t)(buf[2] << 8 | buf[3]);
    *my = (int16_t)(buf[4] << 8 | buf[5]);
}

int main() {
    stdio_init_all();

    i2c_init(I2C_PORT, 100 * 1000); // Initialize I²C at 100kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    sleep_ms(1000);
    printf("Initializing GY-511 IMU...\n");

    // Initialize accelerometer (normal mode, 50Hz, all axes on)
    // CTRL_REG1_A = 0x27 → 0b00100111
    //  - ODR = 50Hz (0100)
    //  - All axes enabled (XYZ)
    uint8_t init_acc[] = {CTRL_REG1_A, 0x27};
    i2c_write_blocking(I2C_PORT, ACC_ADDR, init_acc, 2, false);

    // Initialize magnetometer (continuous conversion mode)
    // CRA_REG_M = 0x14 → 0b00010100
    //  - Data output rate = 30Hz
    uint8_t init_mag1[] = {CRA_REG_M, 0x14}; // 30Hz output rate
    i2c_write_blocking(I2C_PORT, MAG_ADDR, init_mag1, 2, false);
    
    // MR_REG_M = 0x00
    //  - Continuous-conversion mode (device keeps updating readings)
    uint8_t init_mag2[] = {MR_REG_M, 0x00};
    i2c_write_blocking(I2C_PORT, MAG_ADDR, init_mag2, 2, false);

    printf("GY-511 initialized successfully.\n");

    while (true) {
        int16_t ax, ay, az;
        int16_t mx, my, mz;

        read_accel(&ax, &ay, &az);
        read_mag(&mx, &my, &mz);

        // Convert raw values to g (±2g range = 16384 LSB/g)
        float ax_g = ax / 16384.0f;
        float ay_g = ay / 16384.0f;
        float az_g = az / 16384.0f;

        // Convert magnetometer to Gauss (0.00014 Gauss/LSB typical)
        float mx_gs = mx * 0.00014f;
        float my_gs = my * 0.00014f;
        float mz_gs = mz * 0.00014f;

        printf("Accel[g]: X=%.2f Y=%.2f Z=%.2f | Mag[Gs]: X=%.2f Y=%.2f Z=%.2f\n",
               ax_g, ay_g, az_g, mx_gs, my_gs, mz_gs);

        sleep_ms(500);
    }
}
