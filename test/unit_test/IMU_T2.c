/**
 * IMU-T2.c
 *
 * Test Case ID: IMU-T2
 * Description:
 *   Verify that the Exponential Moving Average (EMA) filter smooths accelerometer
 *   and magnetometer readings during motion, without introducing excessive latency.
 *
 * Test Method:
 *   - Collect IMU data for 60 seconds, while physically shaking the IMU board.
 *   - Apply EMA filter (α = 0.2) to raw accelerometer/magnetometer values.
 *   - Display raw vs EMA-filtered values in real-time.
 *   - Check that EMA output:
 *        (1) Removes noise spikes
 *        (2) Does NOT react with sudden large jumps
 *        (3) Still follows the general movement smoothly
 *
 * Success Criteria:
 *   - EMA values must not jump more than 30% as much as the raw values do.
 *   - EMA must remain visually smooth.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* ============================================================================
 * HARDWARE CONFIGURATION
 * These match the actual wiring used on your Pico W.
 * ==========================================================================*/
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

/* ============================================================================
 * FILTER & TEST CONFIGURATION
 * ==========================================================================*/
#define EMA_ALPHA 0.2f               // Smoothing factor (0–1). Lower = smoother.
#define SAMPLE_RATE_HZ 10            // 10 Hz sampling rate
#define TEST_DURATION_SEC 60         // 60-second test
#define MAX_EMA_JUMP_RATIO 0.30f     // EMA should not jump more than 30% of raw movement

/* ============================================================================
 * IMU INITIALISATION
 * Sets up I2C on the Pico W and configures the accelerometer/magnetometer.
 * ==========================================================================*/
static void imu_init(void)
{
    /* Initialise I2C bus at 100kHz */
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    sleep_ms(100); // Allow bus to stabilise

    /* Enable accelerometer: normal mode, all axes enabled */
    uint8_t acc_init[2] = {CTRL_REG1_A, 0x27};
    i2c_write_blocking(I2C_PORT, ACC_ADDR, acc_init, 2, false);

    /* Configure magnetometer */
    uint8_t mag_init1[2] = {CRA_REG_M, 0x14};
    uint8_t mag_init2[2] = {MR_REG_M, 0x00}; // Continuous-conversion mode

    i2c_write_blocking(I2C_PORT, MAG_ADDR, mag_init1, 2, false);
    i2c_write_blocking(I2C_PORT, MAG_ADDR, mag_init2, 2, false);
}

/* ============================================================================
 * READ RAW ACCELEROMETER DATA
 * Reads X/Y/Z from the LSM303 accelerometer.
 * ==========================================================================*/
static void read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t reg = OUT_X_L_A | 0x80u; // auto-increment
    uint8_t buf[6];

    i2c_write_blocking(I2C_PORT, ACC_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ACC_ADDR, buf, 6, false);

    *ax = (int16_t)((buf[1] << 8) | buf[0]);
    *ay = (int16_t)((buf[3] << 8) | buf[2]);
    *az = (int16_t)((buf[5] << 8) | buf[4]);
}

/* ============================================================================
 * READ RAW MAGNETOMETER DATA
 * ==========================================================================*/
static void read_mag_raw(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t reg = OUT_X_H_M;
    uint8_t buf[6];

    i2c_write_blocking(I2C_PORT, MAG_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MAG_ADDR, buf, 6, false);

    *mx = (int16_t)((buf[0] << 8) | buf[1]);
    *mz = (int16_t)((buf[2] << 8) | buf[3]);
    *my = (int16_t)((buf[4] << 8) | buf[5]);
}

/* ============================================================================
 * MAIN TEST PROGRAM
 * ==========================================================================*/
int main(void)
{
    stdio_init_all();
    sleep_ms(1500);

    printf("\n=============================================\n");
    printf("   IMU-T2: Exponential Moving Average Test\n");
    printf("=============================================\n\n");

    printf("This test checks EMA behaviour while the sensor is MOVING.\n");
    printf("Shake the IMU during the 60-second duration.\n\n");
    printf("Press Enter to begin.\n");
    getchar();

    imu_init();
    printf("[INIT] IMU ready.\n\n");

    /* EMA filter state for each axis (initial values = 0) */
    float ax_ema = 0, ay_ema = 0, az_ema = 0;
    float mx_ema = 0, my_ema = 0, mz_ema = 0;

    /* Previous raw readings (used to detect jumps) */
    int16_t ax_prev = 0, ay_prev = 0, az_prev = 0;
    int16_t mx_prev = 0, my_prev = 0, mz_prev = 0;

    bool overall_pass = true;

    const int total_iterations = SAMPLE_RATE_HZ * TEST_DURATION_SEC;

    /* =============================================================
     * MAIN SAMPLING LOOP (60 seconds @ 10 Hz)
     * - Reads raw IMU values
     * - Updates EMA filter
     * - Checks for abnormal “jumps”
     * - Prints results to the screen
     * =============================================================*/
    for (int i = 0; i < total_iterations; i++)
    {
        int16_t ax, ay, az;
        int16_t mx, my, mz;

        /* Read fresh IMU values */
        read_accel_raw(&ax, &ay, &az);
        read_mag_raw(&mx, &my, &mz);

        /* -------------------------------
         * APPLY EMA FILTER:
         * EMA = α·new_value + (1-α)·prev_EMA
         * ------------------------------*/
        ax_ema = EMA_ALPHA * ax + (1 - EMA_ALPHA) * ax_ema;
        ay_ema = EMA_ALPHA * ay + (1 - EMA_ALPHA) * ay_ema;
        az_ema = EMA_ALPHA * az + (1 - EMA_ALPHA) * az_ema;

        mx_ema = EMA_ALPHA * mx + (1 - EMA_ALPHA) * mx_ema;
        my_ema = EMA_ALPHA * my + (1 - EMA_ALPHA) * my_ema;
        mz_ema = EMA_ALPHA * mz + (1 - EMA_ALPHA) * mz_ema;

        /* ===============================================================
         * JUMP DETECTION:
         * If the EMA moves almost as much as the RAW in one step,
         * it means the EMA failed to smooth the data.
         *
         * Condition:
         *    |EMA_jump| > 0.30 * |RAW_jump|
         * ===============================================================*/
        bool jump_fail = false;

        if (i > 0)
        {
            float ax_jump_raw = fabsf(ax - ax_prev);
            float ax_jump_ema = fabsf(ax_ema - ax_prev);
            if (ax_jump_raw > 0 && ax_jump_ema > MAX_EMA_JUMP_RATIO * ax_jump_raw)
                jump_fail = true;

            float ay_jump_raw = fabsf(ay - ay_prev);
            float ay_jump_ema = fabsf(ay_ema - ay_prev);
            if (ay_jump_raw > 0 && ay_jump_ema > MAX_EMA_JUMP_RATIO * ay_jump_raw)
                jump_fail = true;

            float az_jump_raw = fabsf(az - az_prev);
            float az_jump_ema = fabsf(az_ema - az_prev);
            if (az_jump_raw > 0 && az_jump_ema > MAX_EMA_JUMP_RATIO * az_jump_raw)
                jump_fail = true;
        }

        overall_pass = overall_pass && !jump_fail;

        /* ===============================================================
         * REAL-TIME PRINTING
         * Shows exactly how EMA behaves compared to raw readings.
         * ===============================================================*/
        printf("RAW ACC: %6d %6d %6d | EMA: %8.1f %8.1f %8.1f%s\n",
               ax, ay, az,
               ax_ema, ay_ema, az_ema,
               jump_fail ? "  <-- EMA JUMP DETECTED" : "");

        /* Store previous samples for jump comparison */
        ax_prev = ax; ay_prev = ay; az_prev = az;
        mx_prev = mx; my_prev = my; mz_prev = mz;

        /* Wait for next sample (10 Hz) */
        sleep_ms(100);
    }

    /* ============================================================================
     * FINAL RESULT SUMMARY
     * ==========================================================================*/
    printf("\n=============================================\n");
    printf("              IMU-T2 FINAL RESULT\n");
    printf("=============================================\n");

    if (overall_pass)
    {
        printf("PASS: EMA successfully smoothed IMU readings with no sudden jumps.\n");
    }
    else
    {
        printf("FAIL: EMA produced jump behaviour exceeding allowed limits.\n");
    }

    printf("=============================================\n\n");

    return overall_pass ? 0 : 1;
}
