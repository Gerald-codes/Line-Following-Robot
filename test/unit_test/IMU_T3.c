/**
 * IMU-T3.c
 *
 * Test Case ID: IMU-T3
 *
 * Description:
 *   Verify that magnetometer calibration removes hard-iron and soft-iron
 *   distortions by centering magnetic readings and making magnitudes consistent.
 *
 * Test Method:
 *   1. Collect raw magnetometer readings for 60 seconds.
 *   2. User rotates the IMU slowly in all directions.
 *   3. Record min/max of X, Y, Z axes.
 *   4. Compute calibration:
 *         offset = (min + max) / 2     <-- hard-iron
 *         scale  = (max - min) ratio   <-- soft-iron
 *   5. Apply calibration to live readings.
 *   6. Re-collect data for 60 seconds with calibration active.
 *   7. Compare magnetic vector magnitude stability before vs after.
 *
 * Success Criteria:
 *   - Magnetic magnitudes after calibration must show:
 *         * Lower standard deviation
 *         * Mean magnitude centered (consistent field strength)
 *         * Reduced axis bias
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* --------------------------------------------------------------------------
 * HARDWARE CONFIGURATION
 * -------------------------------------------------------------------------- */
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

/* --------------------------------------------------------------------------
 * TEST CONFIGURATION
 * -------------------------------------------------------------------------- */
#define SAMPLE_RATE_HZ 10
#define TEST_DURATION_SEC 60
#define NUM_SAMPLES (SAMPLE_RATE_HZ * TEST_DURATION_SEC)

/* --------------------------------------------------------------------------
 * FUNCTION PROTOTYPES
 * -------------------------------------------------------------------------- */
static void imu_init(void);
static void read_mag(int16_t *mx, int16_t *my, int16_t *mz);

static float calc_std(const float *data, int len);
static float calc_mean(const float *data, int len);

/* --------------------------------------------------------------------------
 * IMU INITIALISATION
 * -------------------------------------------------------------------------- */
static void imu_init(void)
{
    i2c_init(I2C_PORT, 100000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    sleep_ms(200);

    uint8_t mag_cfg1[2] = {CRA_REG_M, 0x14};
    uint8_t mag_cfg2[2] = {MR_REG_M, 0x00};

    i2c_write_blocking(I2C_PORT, MAG_ADDR, mag_cfg1, 2, false);
    i2c_write_blocking(I2C_PORT, MAG_ADDR, mag_cfg2, 2, false);
}

/* --------------------------------------------------------------------------
 * READ RAW MAGNETOMETER VALUES
 * -------------------------------------------------------------------------- */
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

/* --------------------------------------------------------------------------
 * SIMPLE STATISTICS: MEAN + STD DEV
 * -------------------------------------------------------------------------- */
static float calc_mean(const float *data, int len)
{
    float sum = 0;
    for (int i = 0; i < len; i++)
        sum += data[i];
    return sum / len;
}

static float calc_std(const float *data, int len)
{
    float mean = calc_mean(data, len);
    float acc = 0;
    for (int i = 0; i < len; i++)
    {
        float d = data[i] - mean;
        acc += d * d;
    }
    return sqrtf(acc / len);
}

/* --------------------------------------------------------------------------
 * MAIN TEST PROGRAM
 * -------------------------------------------------------------------------- */
int main()
{
    stdio_init_all();
    sleep_ms(1500);

    printf("\n===============================================\n");
    printf("        IMU-T3: MAGNETOMETER CALIBRATION TEST\n");
    printf("===============================================\n\n");

    printf("Instructions:\n");
    printf(" • Rotate the IMU board slowly in ALL directions.\n");
    printf(" • Keep rotating for the full 60-second duration.\n");
    printf(" • The test records min/max magnetic values.\n\n");
    printf("Press Enter to start RAW data capture...\n");
    getchar();

    imu_init();

    /* Buffers for magnitude values */
    float mag_raw[NUM_SAMPLES];
    float mag_cal[NUM_SAMPLES];

    /* Track min/max per axis */
    int16_t min_x = 32767, min_y = 32767, min_z = 32767;
    int16_t max_x = -32768, max_y = -32768, max_z = -32768;

    /* ================================================================
     * STEP 1 — RAW MAGNETOMETER DATA COLLECTION (60 seconds)
     * ================================================================*/
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        int16_t mx, my, mz;
        read_mag(&mx, &my, &mz);

        /* Track hard-iron distortions (offset center shift) */
        if (mx < min_x) min_x = mx;
        if (my < min_y) min_y = my;
        if (mz < min_z) min_z = mz;

        if (mx > max_x) max_x = mx;
        if (my > max_y) max_y = my;
        if (mz > max_z) max_z = mz;

        /* Compute raw magnitude */
        mag_raw[i] = sqrtf(mx*mx + my*my + mz*mz);

        sleep_ms(100);
    }

    printf("\nRAW collection complete!\n");
    printf("-----------------------------------------------\n");
    printf("RAW MIN/MAX VALUES\n");
    printf("X: min=%d  max=%d\n", min_x, max_x);
    printf("Y: min=%d  max=%d\n", min_y, max_y);
    printf("Z: min=%d  max=%d\n", min_z, max_z);
    printf("-----------------------------------------------\n\n");

    /* ================================================================
     * STEP 2 — COMPUTE CALIBRATION PARAMETERS
     * ================================================================*/

    float offset_x = (min_x + max_x) / 2.0f;
    float offset_y = (min_y + max_y) / 2.0f;
    float offset_z = (min_z + max_z) / 2.0f;

    float scale_x = (max_x - min_x) / 2.0f;
    float scale_y = (max_y - min_y) / 2.0f;
    float scale_z = (max_z - min_z) / 2.0f;

    float avg_scale = (scale_x + scale_y + scale_z) / 3.0f;

    float corr_x = avg_scale / scale_x;
    float corr_y = avg_scale / scale_y;
    float corr_z = avg_scale / scale_z;

    printf("Calculated Calibration:\n");
    printf("Offsets:  X=%.1f  Y=%.1f  Z=%.1f\n", offset_x, offset_y, offset_z);
    printf("Scales:   X=%.3f  Y=%.3f  Z=%.3f\n\n", corr_x, corr_y, corr_z);

    printf("Press Enter to apply calibration and re-measure...\n");
    getchar();

    /* ================================================================
     * STEP 3 — APPLY CALIBRATION & RECAPTURE MAG VALUES
     * ================================================================*/
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        int16_t mx, my, mz;
        read_mag(&mx, &my, &mz);

        /* Hard-iron correction */
        float mx_off = mx - offset_x;
        float my_off = my - offset_y;
        float mz_off = mz - offset_z;

        /* Soft-iron correction (scale equalization) */
        float mx_cal = mx_off * corr_x;
        float my_cal = my_off * corr_y;
        float mz_cal = mz_off * corr_z;

        /* Magnitude after calibration */
        mag_cal[i] = sqrtf(mx_cal*mx_cal + my_cal*my_cal + mz_cal*mz_cal);

        sleep_ms(100);
    }

    /* ================================================================
     * STEP 4 — RESULTS & PASS/FAIL
     * ================================================================*/
    float raw_std = calc_std(mag_raw, NUM_SAMPLES);
    float cal_std = calc_std(mag_cal, NUM_SAMPLES);

    float raw_mean = calc_mean(mag_raw, NUM_SAMPLES);
    float cal_mean = calc_mean(mag_cal, NUM_SAMPLES);

    printf("\n===============================================\n");
    printf("                RESULTS SUMMARY\n");
    printf("===============================================\n");

    printf("MAG VECTOR MAGNITUDE (RAW):\n");
    printf("  Mean = %.2f   StdDev = %.2f\n\n", raw_mean, raw_std);

    printf("MAG VECTOR MAGNITUDE (CALIBRATED):\n");
    printf("  Mean = %.2f   StdDev = %.2f\n\n", cal_mean, cal_std);

    bool passed = (cal_std < raw_std * 0.7f);  // At least 30% improvement

    if (passed)
    {
        printf("PASS: Calibration successfully centered and normalized magnetic readings.\n");
    }
    else
    {
        printf("FAIL: Magnetic distortions persist after calibration.\n");
    }

    printf("===============================================\n\n");

    return passed ? 0 : 1;
}
