/**
 * motor_controller.c
 */

#include "motor_controller.h"
#include "motor.h"
#include "line_following.h"
#include "pid_controller.h"
#include "config.h"
#include "pin_definitions.h"
#include <stdio.h>
#include <math.h>

static float L_power = 0.0f;
static float R_power = 0.0f;

void motor_controller_init(void) {
    L_power = 0.0f;
    R_power = 0.0f;
}

static int apply_deadband(float power) {
    int int_power = (int)power;
    
    if (int_power > 0 && int_power < MIN_POWER) {
        return MIN_POWER;
    } else if (int_power < 0 && int_power > -MIN_POWER) {
        return -MIN_POWER;
    }
    
    if (int_power > MAX_POWER) return MAX_POWER;
    if (int_power < -MAX_POWER) return -MAX_POWER;
    
    return int_power;
}

void motor_controller_update_line_following(float dt) {
    // Get PID output
    float steering = line_following_update(dt);
    float error = line_following_get_error();
    
    // Apply adaptive gains
    pid_controller_apply_adaptive_gains(fabsf(error));
    
    // Calculate motor powers
    L_power = BASE_POWER - steering;
    R_power = BASE_POWER + steering;
    
    // Apply deadband
    int left_motor = apply_deadband(L_power);
    int right_motor = apply_deadband(R_power);
    
    // Drive motors
    motor_drive(M1A, M1B, -left_motor);
    motor_drive(M2A, M2B, -right_motor);
    
    // Debug output
    static uint32_t last_debug = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_debug >= 500) {
        printf("[LINE] Error: %+.2f | Steering: %+.2f | L:%d R:%d\n",
               error, steering, left_motor, right_motor);
        last_debug = now;
    }
}

void motor_controller_drive_direct(int left_power, int right_power) {
    int left = apply_deadband((float)left_power);
    int right = apply_deadband((float)right_power);
    
    motor_drive(M1A, M1B, -left);
    motor_drive(M2A, M2B, -right);
    
    L_power = (float)left;
    R_power = (float)right;
}

void motor_controller_stop_all(void) {
    motor_stop(M1A, M1B);
    motor_stop(M2A, M2B);
    L_power = 0.0f;
    R_power = 0.0f;
}

void motor_controller_get_powers(float *left, float *right) {
    if (left) *left = L_power;
    if (right) *right = R_power;
}
