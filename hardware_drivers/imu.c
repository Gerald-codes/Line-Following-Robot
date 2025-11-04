// #include "imu.h"
// #include "pico/stdlib.h"
// #include "hardware/i2c.h"
// #include <math.h>
// #include <stdio.h>

// #define I2C_PORT i2c0
// #define SDA_PIN  16
// #define SCL_PIN  17

// #define ACC_ADDR 0x19
// #define MAG_ADDR 0x1E
// #define CTRL_REG1_A 0x20
// #define OUT_X_L_A   0x28
// #define CRA_REG_M   0x00
// #define MR_REG_M    0x02
// #define OUT_X_H_M   0x03

// #define FILTER_SIZE 3
// #define ALPHA       0.2f
// #define RAD_TO_DEG  57.2957795f

// // Calibration values
// static float mx_offset = 20, my_offset = 9, mz_offset = -152;
// static float mx_scale = 0.975, my_scale = 0.936, mz_scale = 1.105;

// static int16_t acc_buf_x[FILTER_SIZE], acc_buf_y[FILTER_SIZE], acc_buf_z[FILTER_SIZE];
// static int16_t mag_buf_x[FILTER_SIZE], mag_buf_y[FILTER_SIZE], mag_buf_z[FILTER_SIZE];
// static int buf_index = 0;

// static float ax_ema=0, ay_ema=0, az_ema=0;
// static float mx_ema=0, my_ema=0, mz_ema=0;

// static int16_t sma(int16_t *buf) {
//     int32_t sum=0;
//     for(int i=0;i<FILTER_SIZE;i++) sum+=buf[i];
//     return sum / FILTER_SIZE;
// }

// static void read_accel(int16_t *ax,int16_t *ay,int16_t *az){
//     uint8_t reg=OUT_X_L_A|0x80, buf[6];
//     i2c_write_blocking(I2C_PORT,ACC_ADDR,&reg,1,true);
//     i2c_read_blocking(I2C_PORT,ACC_ADDR,buf,6,false);
//     *ax = (buf[1]<<8|buf[0]);
//     *ay = (buf[3]<<8|buf[2]);
//     *az = (buf[5]<<8|buf[4]);
// }

// static void read_mag(int16_t *mx,int16_t *my,int16_t *mz){
//     uint8_t reg=OUT_X_H_M, buf[6];
//     i2c_write_blocking(I2C_PORT,MAG_ADDR,&reg,1,true);
//     i2c_read_blocking(I2C_PORT,MAG_ADDR,buf,6,false);
//     *mx = (buf[0]<<8|buf[1]);
//     *mz = (buf[2]<<8|buf[3]);
//     *my = (buf[4]<<8|buf[5]);
// }

// void imu_init(IMU *imu) {
//     i2c_init(I2C_PORT,100000);
//     gpio_set_function(SDA_PIN,GPIO_FUNC_I2C);
//     gpio_set_function(SCL_PIN,GPIO_FUNC_I2C);
//     gpio_pull_up(SDA_PIN);
//     gpio_pull_up(SCL_PIN);

//     uint8_t init_acc[]={CTRL_REG1_A,0x27};
//     i2c_write_blocking(I2C_PORT,ACC_ADDR,init_acc,2,false);
//     uint8_t init_mag1[]={CRA_REG_M,0x14};
//     i2c_write_blocking(I2C_PORT,MAG_ADDR,init_mag1,2,false);
//     uint8_t init_mag2[]={MR_REG_M,0x00};
//     i2c_write_blocking(I2C_PORT,MAG_ADDR,init_mag2,2,false);

//     imu->heading_offset=0;
//     imu->calibrated=false;
// }

// void imu_calibrate(IMU *imu){
//     imu_update(imu);
//     imu->heading_offset = imu->yaw;
//     imu->calibrated = true;
// }

// void imu_update(IMU *imu){
//     int16_t ax,ay,az,mx,my,mz;
//     read_accel(&ax,&ay,&az);
//     read_mag(&mx,&my,&mz);

//     acc_buf_x[buf_index]=ax; acc_buf_y[buf_index]=ay; acc_buf_z[buf_index]=az;
//     mag_buf_x[buf_index]=mx; mag_buf_y[buf_index]=my; mag_buf_z[buf_index]=mz;
//     buf_index=(buf_index+1)%FILTER_SIZE;

//     ax_ema = ALPHA*sma(acc_buf_x) + (1-ALPHA)*ax_ema;
//     ay_ema = ALPHA*sma(acc_buf_y) + (1-ALPHA)*ay_ema;
//     az_ema = ALPHA*sma(acc_buf_z) + (1-ALPHA)*az_ema;

//     mx_ema = ALPHA*((sma(mag_buf_x)-mx_offset)*mx_scale)+(1-ALPHA)*mx_ema;
//     my_ema = ALPHA*((sma(mag_buf_y)-my_offset)*my_scale)+(1-ALPHA)*my_ema;
//     mz_ema = ALPHA*((sma(mag_buf_z)-mz_offset)*mz_scale)+(1-ALPHA)*mz_ema;

//     float roll = atan2f(ay_ema,az_ema);
//     float pitch = atan2f(-ax_ema,sqrtf(ay_ema*ay_ema+az_ema*az_ema));
//     float Xh = mx_ema*cosf(pitch)+mz_ema*sinf(pitch);
//     float Yh = mx_ema*sinf(roll)*sinf(pitch)+my_ema*cosf(roll)-mz_ema*sinf(roll)*cosf(pitch);

//     imu->roll = roll*RAD_TO_DEG;
//     imu->pitch= pitch*RAD_TO_DEG;
//     imu->yaw = atan2f(-Yh,Xh)*RAD_TO_DEG;
// }

// float imu_get_heading(IMU *imu){
//     float h = imu->yaw - imu->heading_offset;
//     if(h>180)h-=360;
//     if(h<-180)h+=360;
//     printf("HEADING: %.1f",h);
//     return h;
// }

// hardware_drivers/imu.c
// FIXED: Added missing newline in printf

#include "imu.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include <stdio.h>

#define I2C_PORT i2c0
#define SDA_PIN  16
#define SCL_PIN  17

#define ACC_ADDR 0x19
#define MAG_ADDR 0x1E
#define CTRL_REG1_A 0x20
#define OUT_X_L_A   0x28
#define CRA_REG_M   0x00
#define MR_REG_M    0x02
#define OUT_X_H_M   0x03

#define FILTER_SIZE 3
#define ALPHA       0.2f
#define RAD_TO_DEG  57.2957795f

// Calibration values
static float mx_offset = 20, my_offset = 9, mz_offset = -152;
static float mx_scale = 0.975, my_scale = 0.936, mz_scale = 1.105;

static int16_t acc_buf_x[FILTER_SIZE], acc_buf_y[FILTER_SIZE], acc_buf_z[FILTER_SIZE];
static int16_t mag_buf_x[FILTER_SIZE], mag_buf_y[FILTER_SIZE], mag_buf_z[FILTER_SIZE];
static int buf_index = 0;

static float ax_ema=0, ay_ema=0, az_ema=0;
static float mx_ema=0, my_ema=0, mz_ema=0;

static int16_t sma(int16_t *buf) {
    int32_t sum=0;
    for(int i=0;i<FILTER_SIZE;i++) sum+=buf[i];
    return sum / FILTER_SIZE;
}

static void read_accel(int16_t *ax,int16_t *ay,int16_t *az){
    uint8_t reg=OUT_X_L_A|0x80, buf[6];
    i2c_write_blocking(I2C_PORT,ACC_ADDR,&reg,1,true);
    i2c_read_blocking(I2C_PORT,ACC_ADDR,buf,6,false);
    *ax = (buf[1]<<8|buf[0]);
    *ay = (buf[3]<<8|buf[2]);
    *az = (buf[5]<<8|buf[4]);
}

static void read_mag(int16_t *mx,int16_t *my,int16_t *mz){
    uint8_t reg=OUT_X_H_M, buf[6];
    i2c_write_blocking(I2C_PORT,MAG_ADDR,&reg,1,true);
    i2c_read_blocking(I2C_PORT,MAG_ADDR,buf,6,false);
    *mx = (buf[0]<<8|buf[1]);
    *mz = (buf[2]<<8|buf[3]);
    *my = (buf[4]<<8|buf[5]);
}

void imu_init(IMU *imu) {
    i2c_init(I2C_PORT,100000);
    gpio_set_function(SDA_PIN,GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN,GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    uint8_t init_acc[]={CTRL_REG1_A,0x27};
    i2c_write_blocking(I2C_PORT,ACC_ADDR,init_acc,2,false);
    uint8_t init_mag1[]={CRA_REG_M,0x14};
    i2c_write_blocking(I2C_PORT,MAG_ADDR,init_mag1,2,false);
    uint8_t init_mag2[]={MR_REG_M,0x00};
    i2c_write_blocking(I2C_PORT,MAG_ADDR,init_mag2,2,false);

    imu->heading_offset=0;
    imu->calibrated=false;
    
    printf("IMU initialized on I2C0 (GP%d=SDA, GP%d=SCL)\n", SDA_PIN, SCL_PIN);
}

void imu_calibrate(IMU *imu){
    printf("Calibrating IMU...\n");
    
    // Take multiple samples for better calibration
    float heading_sum = 0;
    const int samples = 10;
    
    for (int i = 0; i < samples; i++) {
        imu_update(imu);
        heading_sum += imu->yaw;
        sleep_ms(100);
    }
    
    imu->heading_offset = heading_sum / samples;
    imu->calibrated = true;
    
    printf("IMU calibrated: offset = %.1f°\n", imu->heading_offset);
}

void imu_update(IMU *imu){
    int16_t ax,ay,az,mx,my,mz;
    read_accel(&ax,&ay,&az);
    read_mag(&mx,&my,&mz);

    acc_buf_x[buf_index]=ax; acc_buf_y[buf_index]=ay; acc_buf_z[buf_index]=az;
    mag_buf_x[buf_index]=mx; mag_buf_y[buf_index]=my; mag_buf_z[buf_index]=mz;
    buf_index=(buf_index+1)%FILTER_SIZE;

    ax_ema = ALPHA*sma(acc_buf_x) + (1-ALPHA)*ax_ema;
    ay_ema = ALPHA*sma(acc_buf_y) + (1-ALPHA)*ay_ema;
    az_ema = ALPHA*sma(acc_buf_z) + (1-ALPHA)*az_ema;

    mx_ema = ALPHA*((sma(mag_buf_x)-mx_offset)*mx_scale)+(1-ALPHA)*mx_ema;
    my_ema = ALPHA*((sma(mag_buf_y)-my_offset)*my_scale)+(1-ALPHA)*my_ema;
    mz_ema = ALPHA*((sma(mag_buf_z)-mz_offset)*mz_scale)+(1-ALPHA)*mz_ema;

    float roll = atan2f(ay_ema,az_ema);
    float pitch = atan2f(-ax_ema,sqrtf(ay_ema*ay_ema+az_ema*az_ema));
    float Xh = mx_ema*cosf(pitch)+mz_ema*sinf(pitch);
    float Yh = mx_ema*sinf(roll)*sinf(pitch)+my_ema*cosf(roll)-mz_ema*sinf(roll)*cosf(pitch);

    imu->roll = roll*RAD_TO_DEG;
    imu->pitch= pitch*RAD_TO_DEG;
    imu->yaw = atan2f(-Yh,Xh)*RAD_TO_DEG;
}

float imu_get_heading(IMU *imu){
    float h = imu->yaw - imu->heading_offset;
    if(h>180)h-=360;
    if(h<-180)h+=360;
    // FIXED: Added newline to prevent buffer issues
    // printf("HEADING: %.1f\n", h);  // Uncomment for debugging
    return h;
}