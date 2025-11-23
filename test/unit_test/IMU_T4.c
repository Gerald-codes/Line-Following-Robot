/**
 * IMU-T4.c
 *
 * Test Case ID: IMU-T4
 *
 * Description:
 *   Verify that roll, pitch, and yaw calculations provide stable
 *   and accurate orientation angles from the IMU.
 *
 * Test Method:
 *   1. Place IMU on a flat surface → check roll/pitch stability (~0°).
 *   2. Tilt IMU to 15° pitch → compare measured pitch to target angle.
 *   3. Tilt IMU to 45° roll → compare measured roll to target angle.
 *   4. Rotate IMU to magnetic north → yaw reading must be within ±5°.
 *
 * Success Criteria:
 *   - Flat surface: roll and pitch must remain stable within ±2°.
 *   - Pitch test: measured pitch ≈ 15° ± 3°.
 *   - Roll test: measured roll ≈ 45° ± 3°.
 *   - Yaw test: within ±5° of magnetic north.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* -------------------------------------------------------------
 * HARDWARE CONFIGURATION (same as previous tests)
 * -------------------------------------------------------------*/
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

#define RAD_TO_DEG 57.2957795f

/* -------------------------------------------------------------
 * FUNCTION PROTOTYPES
 * -------------------------------------------------------------*/
static void imu_init(void);
static void read_accel(int16_t *ax, int16_t *ay, int16_t *az);
static void read_mag(int16_t *mx, int16_t *my, int16_t *mz);

static float calc_roll(float ax, float ay, float az);
static float calc_pitch(float ax, float ay, float az);
static float calc_yaw(float mx, float my, float mz, float roll, float pitch);

/* -------------------------------------------------------------
 * INITIALISE IMU
 * -------------------------------------------------------------*/
static void imu_init(void)
{
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    sleep_ms(200);

    uint8_t acc_cfg[2] = {CTRL_REG1_A, 0x27};
    i2c_write_blocking(I2C_PORT, ACC_ADDR, acc_cfg, 2, false);

    uint8_t mag_cfg1[2] = {CRA_REG_M, 0x14};
    uint8_t mag_cfg2[2] = {MR_REG_M, 0x00};

    i2c_write_blocking(I2C_PORT, MAG_ADDR, mag_cfg1, 2, false);
    i2c_write_blocking(I2C_PORT, MAG_ADDR, mag_cfg2, 2, false);
}

/* -------------------------------------------------------------
 * RAW SENSOR READS
 * -------------------------------------------------------------*/
static void read_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t reg = OUT_X_L_A | 0x80;
    uint8_t buf[6];

    i2c_write_blocking(I2C_PORT, ACC_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ACC_ADDR, buf, 6, false);

    *ax = (int16_t)((buf[1] << 8) | buf[0]);
    *ay = (int16_t)((buf[3] << 8) | buf[2]);
    *az = (int16_t)((buf[5] << 8) | buf[4]);
}

static void read_mag(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t reg = OUT_X_H_M;
    uint8_t buf[6];

    i2c_write_blocking(I2C_PORT, MAG_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MAG_ADDR, buf, 6, false);

    *mx = (int16_t)((buf[0] << 8) | buf[1]);
    *mz = (int16_t)((buf[2] << 8) | buf[3]);
    *my = (int16_t)((buf[4] << 8) | buf[5]);
}

/* -------------------------------------------------------------
 * ORIENTATION ANGLE CALCULATIONS
 * -------------------------------------------------------------*/
static float calc_roll(float ax, float ay, float az)
{
    return atan2f(ay, az) * RAD_TO_DEG;
}

static float calc_pitch(float ax, float ay, float az)
{
    return atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;
}

static float calc_yaw(float mx, float my, float mz, float roll_deg, float pitch_deg)
{
    float roll  = roll_deg / RAD_TO_DEG;
    float pitch = pitch_deg / RAD_TO_DEG;

    /* Tilt compensation */
    float Xh = mx * cosf(pitch) + mz * sinf(pitch);
    float Yh = mx * sinf(roll)*sinf(pitch) + my*cosf(roll) - mz*sinf(roll)*cosf(pitch);

    return atan2f(-Yh, Xh) * RAD_TO_DEG;
}

/* -------------------------------------------------------------
 * CAPTURE ORIENTATION AVERAGE OVER 3 SECONDS
 * -------------------------------------------------------------*/
static void capture_mean_angles(float *roll_out, float *pitch_out, float *yaw_out)
{
    float r_sum = 0, p_sum = 0, y_sum = 0;

    for (int i = 0; i < 30; i++)   // 30 samples = 3 seconds @ 10Hz
    {
        int16_t ax, ay, az;
        int16_t mx, my, mz;

        read_accel(&ax, &ay, &az);
        read_mag(&mx, &my, &mz);

        float roll  = calc_roll(ax, ay, az);
        float pitch = calc_pitch(ax, ay, az);
        float yaw   = calc_yaw(mx, my, mz, roll, pitch);

        r_sum += roll;
        p_sum += pitch;
        y_sum += yaw;

        sleep_ms(100);
    }

    *roll_out  = r_sum / 30.0f;
    *pitch_out = p_sum / 30.0f;
    *yaw_out   = y_sum / 30.0f;
}

/* -------------------------------------------------------------
 * MAIN
 * -------------------------------------------------------------*/
int main()
{
    stdio_init_all();
    sleep_ms(1500);

    printf("\n=============================================\n");
    printf("             IMU-T4 ORIENTATION TEST\n");
    printf("=============================================\n\n");

    imu_init();

    float roll, pitch, yaw;

    /* ---------------------------------------------------------
     * TEST 1 — Flat surface (should be ~0° roll/pitch)
     * ---------------------------------------------------------*/
    printf("Place IMU on a flat surface.\nPress Enter when ready.\n");
    getchar();

    capture_mean_angles(&roll, &pitch, &yaw);

    printf("\nMeasured (flat): Roll=%.2f  Pitch=%.2f\n", roll, pitch);

    bool flat_ok = (fabsf(roll) < 2.0f) && (fabsf(pitch) < 2.0f);

    /* ---------------------------------------------------------
     * TEST 2 — Pitch test (15° target)
     * ---------------------------------------------------------*/
    printf("\nTilt IMU to approx 15° pitch.\nPress Enter when ready.\n");
    getchar();

    capture_mean_angles(&roll, &pitch, &yaw);
    printf("Measured Pitch: %.2f° (target = 15°)\n", pitch);

    bool pitch_ok = fabsf(pitch - 15.0f) < 3.0f;

    /* ---------------------------------------------------------
     * TEST 3 — Roll test (45° target)
     * ---------------------------------------------------------*/
    printf("\nTilt IMU to approx 45° roll.\nPress Enter when ready.\n");
    getchar();

    capture_mean_angles(&roll, &pitch, &yaw);
    printf("Measured Roll: %.2f° (target = 45°)\n", roll);

    bool roll_ok = fabsf(roll - 45.0f) < 3.0f;

    /* ---------------------------------------------------------
     * TEST 4 — Yaw test (Magnetic North)
     * ---------------------------------------------------------*/
    printf("\nRotate IMU to point toward MAGNETIC NORTH.\nPress Enter when ready.\n");
    getchar();

    capture_mean_angles(&roll, &pitch, &yaw);
    printf("Measured Yaw: %.2f° (target = 0° north)\n", yaw);

    bool yaw_ok = fabsf(yaw) < 5.0f;

    /* ---------------------------------------------------------
     * FINAL RESULT
     * ---------------------------------------------------------*/
    printf("\n=============================================\n");
    printf("                IMU-T4 SUMMARY\n");
    printf("=============================================\n");

    printf("Flat check:   %s\n", flat_ok  ? "PASS" : "FAIL");
    printf("Pitch (15°):  %s\n", pitch_ok ? "PASS" : "FAIL");
    printf("Roll  (45°):  %s\n", roll_ok  ? "PASS" : "FAIL");
    printf("Yaw   (N):    %s\n", yaw_ok   ? "PASS" : "FAIL");

    bool overall = flat_ok && pitch_ok && roll_ok && yaw_ok;

    printf("---------------------------------------------\n");
    printf("RESULT: %s\n", overall ? "PASS" : "FAIL");
    printf("=============================================\n");

    return overall ? 0 : 1;
}
