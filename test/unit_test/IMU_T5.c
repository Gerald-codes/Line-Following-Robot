/**
 * IMU-T5.c
 *
 * Test Case ID: IMU-T5
 *
 * Description:
 *   Verify that filtered IMU signals (SMA + EMA combined) produce smoother and
 *   more stable data than raw readings, while still preserving true motion.
 *
 * Test Method:
 *   1. Place IMU stationary on a flat stable surface for 10–15 seconds.
 *   2. Capture raw accelerometer + magnetometer data.
 *   3. Apply SMA (size = 3) followed by EMA (α = 0.2).
 *   4. Print raw and filtered values side-by-side to serial monitor.
 *   5. Export values into a plotting software or spreadsheet.
 *   6. Compare:
 *        - Noise amplitude (variance)
 *        - Spike reduction
 *        - Smoothness and delay
 *
 * Expected Result:
 *   Filtered IMU data should contain:
 *       ✓ Reduced jitter
 *       ✓ Fewer spikes
 *       ✓ Noticeably smoother curves
 *       ✓ Minimal delay (due to small SMA window and low EMA α)
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* ---------------------------------------------------------------
 * HARDWARE CONFIGURATION
 * ---------------------------------------------------------------*/
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

/* ---------------------------------------------------------------
 * FILTER CONFIGURATION (same as your final IMU design)
 * ---------------------------------------------------------------*/
#define SMA_SIZE 3
#define EMA_ALPHA 0.2f
#define SAMPLE_RATE_HZ 20     // faster sampling helps visualization
#define TEST_DURATION_SEC 15
#define NUM_SAMPLES (SAMPLE_RATE_HZ * TEST_DURATION_SEC)

/* ---------------------------------------------------------------
 * SMA BUFFERS
 * ---------------------------------------------------------------*/
static int16_t acc_x_buf[SMA_SIZE];
static int16_t acc_y_buf[SMA_SIZE];
static int16_t acc_z_buf[SMA_SIZE];

static int16_t mag_x_buf[SMA_SIZE];
static int16_t mag_y_buf[SMA_SIZE];
static int16_t mag_z_buf[SMA_SIZE];

static int sma_index = 0;

/* ---------------------------------------------------------------
 * IMU INITIALISATION
 * ---------------------------------------------------------------*/
static void imu_init(void)
{
    i2c_init(I2C_PORT, 100000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    sleep_ms(200);

    uint8_t acc_cfg[2] = {CTRL_REG1_A, 0x27};
    uint8_t mag_cfg1[2] = {CRA_REG_M, 0x14};
    uint8_t mag_cfg2[2] = {MR_REG_M, 0x00};

    i2c_write_blocking(I2C_PORT, ACC_ADDR, acc_cfg, 2, false);
    i2c_write_blocking(I2C_PORT, MAG_ADDR, mag_cfg1, 2, false);
    i2c_write_blocking(I2C_PORT, MAG_ADDR, mag_cfg2, 2, false);
}

/* ---------------------------------------------------------------
 * RAW SENSOR READS
 * ---------------------------------------------------------------*/
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

/* ---------------------------------------------------------------
 * SIMPLE MOVING AVERAGE
 * ---------------------------------------------------------------*/
static int16_t sma(const int16_t *buf)
{
    int32_t sum = 0;
    for (int i = 0; i < SMA_SIZE; i++)
        sum += buf[i];
    return (int16_t)(sum / SMA_SIZE);
}

/* ---------------------------------------------------------------
 * MAIN TEST ROUTINE
 * ---------------------------------------------------------------*/
int main(void)
{
    stdio_init_all();
    sleep_ms(1500);

    printf("\n===============================================\n");
    printf("            IMU-T5 FILTERED VS RAW TEST\n");
    printf("===============================================\n\n");

    printf("Place IMU on a STABLE flat surface.\n");
    printf("Sampling will run for %d seconds.\n", TEST_DURATION_SEC);
    printf("Press Enter to begin...\n");
    getchar();

    imu_init();

    /* EMA filter state */
    float ax_ema = 0, ay_ema = 0, az_ema = 0;
    float mx_ema = 0, my_ema = 0, mz_ema = 0;

    /* Initialize SMA buffers with the first reading */
    {
        int16_t ax, ay, az, mx, my, mz;
        read_accel(&ax, &ay, &az);
        read_mag(&mx, &my, &mz);

        for (int i = 0; i < SMA_SIZE; i++)
        {
            acc_x_buf[i] = ax;
            acc_y_buf[i] = ay;
            acc_z_buf[i] = az;

            mag_x_buf[i] = mx;
            mag_y_buf[i] = my;
            mag_z_buf[i] = mz;
        }
    }

    printf("\nStreaming raw and filtered IMU data...\n");
    printf("Format:\n");
    printf("RAW_AX, RAW_AY, RAW_AZ, SMA_AX, SMA_AY, SMA_AZ, EMA_AX, EMA_AY, EMA_AZ, "
           "RAW_MX, RAW_MY, RAW_MZ, SMA_MX, SMA_MY, SMA_MZ, EMA_MX, EMA_MY, EMA_MZ\n");
    printf("===============================================================\n");

    /* ============================================================
     * MAIN SAMPLING LOOP
     * ============================================================*/
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        int16_t ax, ay, az;
        int16_t mx, my, mz;

        read_accel(&ax, &ay, &az);
        read_mag(&mx, &my, &mz);

        /* Update SMA buffer */
        acc_x_buf[sma_index] = ax;
        acc_y_buf[sma_index] = ay;
        acc_z_buf[sma_index] = az;

        mag_x_buf[sma_index] = mx;
        mag_y_buf[sma_index] = my;
        mag_z_buf[sma_index] = mz;

        sma_index = (sma_index + 1) % SMA_SIZE;

        /* Compute SMA */
        int16_t ax_sma = sma(acc_x_buf);
        int16_t ay_sma = sma(acc_y_buf);
        int16_t az_sma = sma(acc_z_buf);

        int16_t mx_sma = sma(mag_x_buf);
        int16_t my_sma = sma(mag_y_buf);
        int16_t mz_sma = sma(mag_z_buf);

        /* EMA on top of SMA output */
        ax_ema = EMA_ALPHA * ax_sma + (1 - EMA_ALPHA) * ax_ema;
        ay_ema = EMA_ALPHA * ay_sma + (1 - EMA_ALPHA) * ay_ema;
        az_ema = EMA_ALPHA * az_sma + (1 - EMA_ALPHA) * az_ema;

        mx_ema = EMA_ALPHA * mx_sma + (1 - EMA_ALPHA) * mx_ema;
        my_ema = EMA_ALPHA * my_sma + (1 - EMA_ALPHA) * my_ema;
        mz_ema = EMA_ALPHA * mz_sma + (1 - EMA_ALPHA) * mz_ema;

        /* -------------------------------------------------------
         * OUTPUT ONE ROW (easy to import into Excel/Matlab/Python)
         * -------------------------------------------------------*/
        printf("%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,"
               "%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f\n",
               ax, ay, az,
               ax_sma, ay_sma, az_sma,
               ax_ema, ay_ema, az_ema,
               mx, my, mz,
               mx_sma, my_sma, mz_sma,
               mx_ema, my_ema, mz_ema
        );

        sleep_ms(1000 / SAMPLE_RATE_HZ);
    }

    printf("\n===============================================\n");
    printf("       IMU-T5 DATA COLLECTION COMPLETE\n");
    printf("===============================================\n");
    printf("Export the above data to your plotting tool.\n");
    printf("You can now analyse:\n");
    printf(" • Raw noise amplitude\n");
    printf(" • SMA smoothing effect\n");
    printf(" • EMA smoothing effect\n");
    printf(" • Delay between raw → SMA → EMA\n");
    printf("===============================================\n\n");

    return 0;
}
