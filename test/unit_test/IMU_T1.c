/**
 * IMU-T1.c
 *
 * Test Case ID: IMU-T1
 * Description: Verify that the Simple Moving Average (SMA) filter smooths
 *              accelerometer and magnetometer readings by reducing short-term
 *              noise fluctuations.
 *
 * Tests: SMA smoothing behaviour for accelerometer and magnetometer
 *
 * Test Method:
 *  - Run the system to collect raw accelerometer and magnetometer data
 *    for 60 seconds at 10 Hz (600 samples).
 *  - Apply SMA filter using filter size of 3.
 *  - Compare the standard deviation of the raw data and the filtered data
 *    for each axis (X, Y, Z).
 *
 * Success Criteria:
 *  - For each axis (ACC X/Y/Z, MAG X/Y/Z), the standard deviation of the
 *    SMA-filtered signal must be lower than the raw signal by at least 5%.
 *  - This demonstrates reduction of short-term noise with minimal delay due
 *    to small filter size (3 samples).
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* --------------------------------------------------------------------------
 * HARDWARE & IMU DEFINES
 * -------------------------------------------------------------------------- */

#define I2C_PORT        i2c0
#define SDA_PIN         0
#define SCL_PIN         1

#define ACC_ADDR        0x19
#define MAG_ADDR        0x1E

#define CTRL_REG1_A     0x20
#define OUT_X_L_A       0x28

#define CRA_REG_M       0x00
#define MR_REG_M        0x02
#define OUT_X_H_M       0x03

/* --------------------------------------------------------------------------
 * FILTER & TEST CONFIGURATION
 * -------------------------------------------------------------------------- */

#define FILTER_SIZE                 3

#define SAMPLE_RATE_HZ              10
#define TEST_DURATION_SEC           60
#define NUM_SAMPLES                 (SAMPLE_RATE_HZ * TEST_DURATION_SEC)

#define MIN_IMPROVEMENT_FRACTION    0.05f   /* 5% reduction required */

/* --------------------------------------------------------------------------
 * TERMINAL COLORS
 * -------------------------------------------------------------------------- */

#define COLOR_GREEN   "\033[32m"
#define COLOR_RED     "\033[31m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_RESET   "\033[0m"

/* --------------------------------------------------------------------------
 * DATA STRUCTURES
 * -------------------------------------------------------------------------- */

typedef struct
{
    float raw_std;
    float filtered_std;
    float improvement;   /* (raw_std - filtered_std) / raw_std */
    bool  passed;
} AxisResult;

typedef struct
{
    int total_axes;
    int passed_axes;
    int failed_axes;
} TestSummary;

typedef struct
{
    int16_t ax_raw[NUM_SAMPLES];
    int16_t ay_raw[NUM_SAMPLES];
    int16_t az_raw[NUM_SAMPLES];
    int16_t mx_raw[NUM_SAMPLES];
    int16_t my_raw[NUM_SAMPLES];
    int16_t mz_raw[NUM_SAMPLES];

    int16_t ax_sma[NUM_SAMPLES];
    int16_t ay_sma[NUM_SAMPLES];
    int16_t az_sma[NUM_SAMPLES];
    int16_t mx_sma[NUM_SAMPLES];
    int16_t my_sma[NUM_SAMPLES];
    int16_t mz_sma[NUM_SAMPLES];
} ImuDataSet;

/* --------------------------------------------------------------------------
 * GLOBALS FOR SMA BUFFERS
 * -------------------------------------------------------------------------- */

static int16_t acc_buf_x[FILTER_SIZE];
static int16_t acc_buf_y[FILTER_SIZE];
static int16_t acc_buf_z[FILTER_SIZE];
static int16_t mag_buf_x[FILTER_SIZE];
static int16_t mag_buf_y[FILTER_SIZE];
static int16_t mag_buf_z[FILTER_SIZE];

static int sma_index = 0;

/* --------------------------------------------------------------------------
 * HELPER: PRINT TEST HEADER
 * -------------------------------------------------------------------------- */

static void print_test_header(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                       IMU-T1: SMA FILTER                      ║\n");
    printf("║                                                               ║\n");
    printf("║  Test: SMA smoothing of accelerometer and magnetometer noise  ║\n");
    printf("║  Requirement: IMU noise reduction via SMA (FILTER_SIZE = 3)   ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

/* --------------------------------------------------------------------------
 * I2C / IMU INITIALISATION
 * -------------------------------------------------------------------------- */

static void imu_init(void)
{
    /* Initialise I2C peripheral */
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    /* Small delay for bus stabilisation */
    sleep_ms(100);

    /* Accelerometer: enable X/Y/Z, 10 Hz, normal mode (0x27) */
    uint8_t init_acc[2];
    init_acc[0] = CTRL_REG1_A;
    init_acc[1] = 0x27;
    (void)i2c_write_blocking(I2C_PORT, ACC_ADDR, init_acc, 2, false);

    /* Magnetometer: data rate and mode setup (similar to your main code) */
    uint8_t init_mag1[2];
    init_mag1[0] = CRA_REG_M;
    init_mag1[1] = 0x14; /* Example: 30 Hz, gain etc. */

    (void)i2c_write_blocking(I2C_PORT, MAG_ADDR, init_mag1, 2, false);

    uint8_t init_mag2[2];
    init_mag2[0] = MR_REG_M;
    init_mag2[1] = 0x00; /* Continuous-conversion mode */

    (void)i2c_write_blocking(I2C_PORT, MAG_ADDR, init_mag2, 2, false);
}

/* --------------------------------------------------------------------------
 * IMU RAW READ FUNCTIONS
 * -------------------------------------------------------------------------- */

static void read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t reg = (uint8_t)(OUT_X_L_A | 0x80u); /* auto-increment */
    uint8_t buf[6];

    (void)i2c_write_blocking(I2C_PORT, ACC_ADDR, &reg, 1, true);
    (void)i2c_read_blocking(I2C_PORT, ACC_ADDR, buf, 6, false);

    *ax = (int16_t)((buf[1] << 8) | buf[0]);
    *ay = (int16_t)((buf[3] << 8) | buf[2]);
    *az = (int16_t)((buf[5] << 8) | buf[4]);
}

static void read_mag_raw(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t reg = OUT_X_H_M;
    uint8_t buf[6];

    (void)i2c_write_blocking(I2C_PORT, MAG_ADDR, &reg, 1, true);
    (void)i2c_read_blocking(I2C_PORT, MAG_ADDR, buf, 6, false);

    *mx = (int16_t)((buf[0] << 8) | buf[1]);
    *mz = (int16_t)((buf[2] << 8) | buf[3]);
    *my = (int16_t)((buf[4] << 8) | buf[5]);
}

/* --------------------------------------------------------------------------
 * SMA COMPUTATION
 * -------------------------------------------------------------------------- */

static int16_t compute_sma(const int16_t *buffer)
{
    int32_t sum = 0;
    int i;

    for (i = 0; i < FILTER_SIZE; i++)
    {
        sum += buffer[i];
    }

    return (int16_t)(sum / FILTER_SIZE);
}

/* --------------------------------------------------------------------------
 * STANDARD DEVIATION
 * -------------------------------------------------------------------------- */

static float calculate_stddev(const int16_t *data, int length)
{
    if (length <= 1)
    {
        return 0.0f;
    }

    int i;
    int64_t sum = 0;
    for (i = 0; i < length; i++)
    {
        sum += data[i];
    }

    float mean = (float)sum / (float)length;

    float var_accum = 0.0f;
    for (i = 0; i < length; i++)
    {
        float diff = (float)data[i] - mean;
        var_accum += diff * diff;
    }

    /* Use population standard deviation (divide by N) */
    float variance = var_accum / (float)length;
    return sqrtf(variance);
}

/* --------------------------------------------------------------------------
 * SAMPLE COLLECTION (60 s @ 10 Hz)
 * -------------------------------------------------------------------------- */

static void collect_samples(ImuDataSet *dataset)
{
    int16_t ax, ay, az;
    int16_t mx, my, mz;
    int i, k;

    printf("%s[INFO]%s Collecting IMU data for %d seconds at %d Hz...\n",
           COLOR_BLUE, COLOR_RESET, TEST_DURATION_SEC, SAMPLE_RATE_HZ);
    printf("Please keep the robot car / IMU board %sSTILL%s during this test.\n\n",
           COLOR_YELLOW, COLOR_RESET);

    /* Initialise SMA buffers with first reading to avoid start-up bias */
    read_accel_raw(&ax, &ay, &az);
    read_mag_raw(&mx, &my, &mz);

    for (k = 0; k < FILTER_SIZE; k++)
    {
        acc_buf_x[k] = ax;
        acc_buf_y[k] = ay;
        acc_buf_z[k] = az;
        mag_buf_x[k] = mx;
        mag_buf_y[k] = my;
        mag_buf_z[k] = mz;
    }

    sma_index = 0;

    /* Store first sample */
    dataset->ax_raw[0] = ax;
    dataset->ay_raw[0] = ay;
    dataset->az_raw[0] = az;
    dataset->mx_raw[0] = mx;
    dataset->my_raw[0] = my;
    dataset->mz_raw[0] = mz;

    dataset->ax_sma[0] = compute_sma(acc_buf_x);
    dataset->ay_sma[0] = compute_sma(acc_buf_y);
    dataset->az_sma[0] = compute_sma(acc_buf_z);
    dataset->mx_sma[0] = compute_sma(mag_buf_x);
    dataset->my_sma[0] = compute_sma(mag_buf_y);
    dataset->mz_sma[0] = compute_sma(mag_buf_z);

    sleep_ms(100); /* 1 / 10 Hz */

    for (i = 1; i < NUM_SAMPLES; i++)
    {
        read_accel_raw(&ax, &ay, &az);
        read_mag_raw(&mx, &my, &mz);

        dataset->ax_raw[i] = ax;
        dataset->ay_raw[i] = ay;
        dataset->az_raw[i] = az;
        dataset->mx_raw[i] = mx;
        dataset->my_raw[i] = my;
        dataset->mz_raw[i] = mz;

        /* Update SMA buffers (circular) */
        acc_buf_x[sma_index] = ax;
        acc_buf_y[sma_index] = ay;
        acc_buf_z[sma_index] = az;
        mag_buf_x[sma_index] = mx;
        mag_buf_y[sma_index] = my;
        mag_buf_z[sma_index] = mz;

        sma_index++;
        if (sma_index >= FILTER_SIZE)
        {
            sma_index = 0;
        }

        dataset->ax_sma[i] = compute_sma(acc_buf_x);
        dataset->ay_sma[i] = compute_sma(acc_buf_y);
        dataset->az_sma[i] = compute_sma(acc_buf_z);
        dataset->mx_sma[i] = compute_sma(mag_buf_x);
        dataset->my_sma[i] = compute_sma(mag_buf_y);
        dataset->mz_sma[i] = compute_sma(mag_buf_z);

        sleep_ms(100);
    }

    printf("%s[INFO]%s Data collection complete. %d samples recorded.\n\n",
           COLOR_BLUE, COLOR_RESET, NUM_SAMPLES);
}

/* --------------------------------------------------------------------------
 * AXIS EVALUATION & PRINTING
 * -------------------------------------------------------------------------- */

static AxisResult evaluate_axis(const int16_t *raw,
                                const int16_t *filtered,
                                int length)
{
    AxisResult result;
    result.raw_std = calculate_stddev(raw, length);
    result.filtered_std = calculate_stddev(filtered, length);
    result.improvement = 0.0f;
    result.passed = false;

    if (result.raw_std > 0.0f)
    {
        result.improvement =
            (result.raw_std - result.filtered_std) / result.raw_std;

        if ((result.filtered_std < result.raw_std) &&
            (result.improvement >= MIN_IMPROVEMENT_FRACTION))
        {
            result.passed = true;
        }
    }
    else
    {
        /* If raw noise is ~zero, there is nothing to smooth; mark as pass. */
        result.improvement = 0.0f;
        result.passed = true;
    }

    return result;
}

static void print_axis_result(const char *label, const AxisResult *res)
{
    const char *status_color = res->passed ? COLOR_GREEN : COLOR_RED;
    const char *status_text  = res->passed ? "PASS" : "FAIL";

    printf("┌─────────────────────────────────────────────────────────────┐\n");
    printf("│ Axis: %-7s                                              │\n", label);
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Raw Std Dev:        %8.4f                               │\n", res->raw_std);
    printf("│ Filtered Std Dev:   %8.4f                               │\n", res->filtered_std);
    printf("│ Improvement:        %8.2f %%                             │\n",
           res->improvement * 100.0f);
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│ Result: %s%-4s%s                                               │\n",
           status_color, status_text, COLOR_RESET);
    printf("└─────────────────────────────────────────────────────────────┘\n\n");
}

/* --------------------------------------------------------------------------
 * FINAL SUMMARY
 * -------------------------------------------------------------------------- */

static void print_final_summary(const TestSummary *summary)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║                    FINAL IMU-T1 SUMMARY                       ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("  Total Axes Tested : %d\n", summary->total_axes);
    printf("  Passed Axes       : %s%d%s\n",
           COLOR_GREEN, summary->passed_axes, COLOR_RESET);
    printf("  Failed Axes       : %s%d%s\n",
           COLOR_RED, summary->failed_axes, COLOR_RESET);
    printf("  Success Rate      : %.1f%%\n",
           (summary->passed_axes * 100.0f) /
           (float)summary->total_axes);
    printf("\n");

    if (summary->failed_axes == 0)
    {
        printf("  %s✓ IMU-T1: ALL AXES PASSED (SMA smoothing effective)%s\n",
               COLOR_GREEN, COLOR_RESET);
    }
    else
    {
        printf("  %s✗ IMU-T1: SOME AXES FAILED (insufficient smoothing)%s\n",
               COLOR_RED, COLOR_RESET);
    }
    printf("\n");
}

/* --------------------------------------------------------------------------
 * MAIN
 * -------------------------------------------------------------------------- */

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    print_test_header();

    printf("[INIT] Initialising I2C and IMU (GY-511)...\n");
    imu_init();
    printf("[INIT] %s✓ IMU initialised%s\n\n", COLOR_GREEN, COLOR_RESET);

    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  TEST INSTRUCTIONS:\n");
    printf("  1. Place the robot car / IMU board on a stable, flat surface.\n");
    printf("  2. Ensure there is no movement or vibration during the test.\n");
    printf("  3. Press Enter to start 60 seconds of IMU sampling.\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");

    (void)getchar();

    ImuDataSet dataset;
    collect_samples(&dataset);

    TestSummary summary;
    summary.total_axes  = 6;
    summary.passed_axes = 0;
    summary.failed_axes = 0;

    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  AXIS-BY-AXIS SMA EFFECTIVENESS RESULTS\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");

    AxisResult ax_res = evaluate_axis(dataset.ax_raw, dataset.ax_sma, NUM_SAMPLES);
    print_axis_result("ACC X", &ax_res);
    summary.passed_axes += ax_res.passed ? 1 : 0;
    summary.failed_axes += ax_res.passed ? 0 : 1;

    AxisResult ay_res = evaluate_axis(dataset.ay_raw, dataset.ay_sma, NUM_SAMPLES);
    print_axis_result("ACC Y", &ay_res);
    summary.passed_axes += ay_res.passed ? 1 : 0;
    summary.failed_axes += ay_res.passed ? 0 : 1;

    AxisResult az_res = evaluate_axis(dataset.az_raw, dataset.az_sma, NUM_SAMPLES);
    print_axis_result("ACC Z", &az_res);
    summary.passed_axes += az_res.passed ? 1 : 0;
    summary.failed_axes += az_res.passed ? 0 : 1;

    AxisResult mx_res = evaluate_axis(dataset.mx_raw, dataset.mx_sma, NUM_SAMPLES);
    print_axis_result("MAG X", &mx_res);
    summary.passed_axes += mx_res.passed ? 1 : 0;
    summary.failed_axes += mx_res.passed ? 0 : 1;

    AxisResult my_res = evaluate_axis(dataset.my_raw, dataset.my_sma, NUM_SAMPLES);
    print_axis_result("MAG Y", &my_res);
    summary.passed_axes += my_res.passed ? 1 : 0;
    summary.failed_axes += my_res.passed ? 0 : 1;

    AxisResult mz_res = evaluate_axis(dataset.mz_raw, dataset.mz_sma, NUM_SAMPLES);
    print_axis_result("MAG Z", &mz_res);
    summary.passed_axes += mz_res.passed ? 1 : 0;
    summary.failed_axes += mz_res.passed ? 0 : 1;

    print_final_summary(&summary);

    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  IMU-T1 TEST COMPLETE\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");

    return (summary.failed_axes == 0) ? 0 : 1;
}
