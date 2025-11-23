/**
 * @file    pid.h
 * @brief   Generic PID controller implementation
 * @details Provides proportional-integral-derivative controller with
 *          anti-windup and configurable output limits
 */

#ifndef PID_H
#define PID_H

#include "pico/stdlib.h"

typedef struct
{
    /* PID gains */
    float Kp;
    float Ki;
    float Kd;

    /* PID state */
    float setpoint;         /* Target value */
    float integral;         /* Accumulated error */
    float previous_error;   /* Last error for derivative */

    /* Output limits */
    float output_min;
    float output_max;

    /* Anti-windup */
    float integral_max;
} PIDController;

/**
 * @brief Initialize PID controller
 * @param pid Pointer to PIDController structure
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param out_min Minimum output value
 * @param out_max Maximum output value
 */
void pid_init(PIDController *pid, float Kp, float Ki, float Kd,
              float out_min, float out_max);

/**
 * @brief Update PID controller
 * @details Call this regularly in control loop
 * @param pid Pointer to PIDController
 * @param measured_value Current process value
 * @param dt Time delta in seconds since last update
 * @return Control output value
 */
float pid_compute(PIDController *pid, float measured_value, float dt);

/**
 * @brief Set new target setpoint
 * @param pid Pointer to PIDController
 * @param setpoint New target value
 */
void pid_set_target(PIDController *pid, float setpoint);

/**
 * @brief Reset PID state
 * @details Clears integral and previous error
 * @param pid Pointer to PIDController
 */
void pid_reset(PIDController *pid);

#endif /* PID_H */
