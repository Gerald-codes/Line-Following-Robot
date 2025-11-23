/**
 * @file    motor.c
 * @brief   Motor PWM initialization and drive routines
 * @details
 *   Provides functions for initializing motor PWM channels, driving
 *   motors at variable speed and direction, and performing stop/brake
 *   actions. Supports Pico SDK hardware PWM.
 */

#include "motor.h"
#include "hardware/pwm.h"
#include <stdlib.h>

/**
 * @brief   Initialize a motor channel for PWM control
 * @param   pinA First motor pin
 * @param   pinB Second motor pin
 */
void motor_init(uint pinA, uint pinB) {
    gpio_set_function(pinA, GPIO_FUNC_PWM);
    gpio_set_function(pinB, GPIO_FUNC_PWM);

    uint sliceA = pwm_gpio_to_slice_num(pinA);
    uint sliceB = pwm_gpio_to_slice_num(pinB);

    pwm_set_clkdiv(sliceA, 125.0f);
    pwm_set_wrap(sliceA, 999);
    pwm_set_enabled(sliceA, true);

    pwm_set_clkdiv(sliceB, 125.0f);
    pwm_set_wrap(sliceB, 999);
    pwm_set_enabled(sliceB, true);
}

/**
 * @brief   Drive motor with speed (range -100 to +100)
 * @param   pinA First motor pin
 * @param   pinB Second motor pin
 * @param   speed Signed speed, -100 (reverse) to +100 (forward)
 */
void motor_drive(uint pinA, uint pinB, int speed) {
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    uint duty = abs(speed) * 10;

    if (speed > 0) {
        pwm_set_gpio_level(pinA, duty);
        pwm_set_gpio_level(pinB, 0);
    } else if (speed < 0) {
        pwm_set_gpio_level(pinA, 0);
        pwm_set_gpio_level(pinB, duty);
    } else {
        pwm_set_gpio_level(pinA, 0);
        pwm_set_gpio_level(pinB, 0);
    }
}

/**
 * @brief   Stop motor by setting both pins low
 * @param   pinA First motor pin
 * @param   pinB Second motor pin
 */
void motor_stop(uint pinA, uint pinB) {
    pwm_set_gpio_level(pinA, 0);
    pwm_set_gpio_level(pinB, 0);
}

/**
 * @brief   Brake motor by setting both pins high
 * @param   pinA First motor pin
 * @param   pinB Second motor pin
 */
void motor_brake(uint pinA, uint pinB) {
    pwm_set_gpio_level(pinA, 1000);
    pwm_set_gpio_level(pinB, 1000);
}
