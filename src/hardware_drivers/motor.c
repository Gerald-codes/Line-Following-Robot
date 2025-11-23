#include "motor.h"
#include "hardware/pwm.h"
#include <stdlib.h>

// Initialize a motor channel
void motor_init(uint pinA, uint pinB) {
    // Set both pins as PWM
    gpio_set_function(pinA, GPIO_FUNC_PWM);
    gpio_set_function(pinB, GPIO_FUNC_PWM);
    
    uint sliceA = pwm_gpio_to_slice_num(pinA);
    uint sliceB = pwm_gpio_to_slice_num(pinB);
    
    // Configure PWM
    pwm_set_clkdiv(sliceA, 125.0f);
    pwm_set_wrap(sliceA, 999);
    pwm_set_enabled(sliceA, true);
    
    pwm_set_clkdiv(sliceB, 125.0f);
    pwm_set_wrap(sliceB, 999);
    pwm_set_enabled(sliceB, true);
}

// Drive motor with speed from -100 (full reverse) to 100 (full forward)
void motor_drive(uint pinA, uint pinB, int speed) {
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    
    uint duty = abs(speed) * 10;
    
    if (speed > 0) {
        // Forward
        pwm_set_gpio_level(pinA, duty);
        pwm_set_gpio_level(pinB, 0);
    } else if (speed < 0) {
        // Reverse
        pwm_set_gpio_level(pinA, 0);
        pwm_set_gpio_level(pinB, duty);
    } else {
        // Stop
        pwm_set_gpio_level(pinA, 0);
        pwm_set_gpio_level(pinB, 0);
    }
}

// Stop motor
void motor_stop(uint pinA, uint pinB) {
    pwm_set_gpio_level(pinA, 0);
    pwm_set_gpio_level(pinB, 0);
}

// Brake motor (both pins high)
void motor_brake(uint pinA, uint pinB) {
    pwm_set_gpio_level(pinA, 1000);
    pwm_set_gpio_level(pinB, 1000);
}