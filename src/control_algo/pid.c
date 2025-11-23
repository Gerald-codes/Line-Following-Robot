/**
 * @file    pid.c
 * @brief   Generic PID controller implementation
 * @details Implements proportional-integral-derivative controller with
 *          anti-windup and configurable output limits
 */

/* Includes */
#include "pid.h"

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
              float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0;
    pid->integral = 0;
    pid->previous_error = 0;
    pid->output_min = out_min;
    pid->output_max = out_max;
    
    /* Anti-windup limit (prevent integral from getting too large) */
    pid->integral_max = (out_max - out_min) / 2.0f;
}

/**
 * @brief Compute PID output
 * @param pid Pointer to PIDController
 * @param measured_value Current process value
 * @param dt Time delta in seconds
 * @return Control output value
 */
float pid_compute(PIDController *pid, float measured_value, float dt)
{
    /* Calculate error */
    float error = pid->setpoint - measured_value;
    
    /* Proportional term */
    float P = pid->Kp * error;
    
    /* Integral term (with anti-windup) */
    pid->integral += error * dt;
    
    /* Clamp integral to prevent windup */
    if (pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    else if (pid->integral < -pid->integral_max)
    {
        pid->integral = -pid->integral_max;
    }
    
    float I = pid->Ki * pid->integral;
    
    /* Derivative term */
    float derivative = (error - pid->previous_error) / dt;
    float D = pid->Kd * derivative;
    
    /* Calculate total output */
    float output = P + I + D;
    
    /* Clamp output to limits */
    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }
    
    /* Save error for next iteration */
    pid->previous_error = error;
    
    return output;
}

/**
 * @brief Set new target setpoint
 * @param pid Pointer to PIDController
 * @param setpoint New target value
 */
void pid_set_target(PIDController *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

/**
 * @brief Reset PID state
 * @param pid Pointer to PIDController
 */
void pid_reset(PIDController *pid)
{
    pid->integral = 0;
    pid->previous_error = 0;
}
