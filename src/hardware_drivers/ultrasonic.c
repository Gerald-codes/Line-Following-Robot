/**
 * @file    ultrasonic.c
 * @brief   Ultrasonic distance sensor implementation
 * @details 
 *   Provides functions for initializing and reading HC-SR04 ultrasonic sensor
 */

#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"


/**
 * @brief   Initialize ultrasonic sensor GPIO pins
 * @param   trigPin  Trigger pin number for sensor
 * @param   echoPin  Echo pin number for sensor
 * @return  None
 */
void ultrasonic_init(unsigned int trigPin, unsigned int echoPin) {
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
}


/**
 * @brief   Send trigger pulse and measure echo time
 * @param   trigPin      Trigger pin number
 * @param   echoPin      Echo pin number
 * @param   pulse_width  Pointer to store measured pulse width (microseconds)
 * @return  SUCCESS on successful measurement, ERROR_TIMEOUT on timeout
 */
int ultrasonic_get_pulse(unsigned int trigPin, unsigned int echoPin, uint64_t *pulse_width) {
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);
    
    uint64_t width = 0;
    while (gpio_get(echoPin) == 0) {
        width++;
        sleep_us(1);
        if (width > TIMEOUT_US) {
            return ERROR_TIMEOUT;
        }
    }
    
    absolute_time_t startTime = get_absolute_time();
    width = 0;
    while (gpio_get(echoPin) == 1) {
        width++;
        sleep_us(1);
        if (width > TIMEOUT_US) {
            return ERROR_TIMEOUT;
        }
    }
    
    absolute_time_t endTime = get_absolute_time();
    *pulse_width = absolute_time_diff_us(startTime, endTime);
    
    return SUCCESS;
}


/**
 * @brief   Get ultrasonic sensor distance measurement
 * @param   trigPin   Trigger pin number
 * @param   echoPin   Echo pin number
 * @param   distance  Pointer to store measured distance (cm)
 * @return  SUCCESS on successful measurement
 *          ERROR_INVALID_PARAM if pin numbers are invalid
 *          ERROR_TIMEOUT if echo times out
 *          ERROR_OUT_OF_RANGE if distance is outside valid range
 */
int ultrasonic_get_distance(unsigned int trigPin, unsigned int echoPin, uint64_t *distance) {
    if (trigPin > 29 || echoPin > 29) {
        return ERROR_INVALID_PARAM;
    }
    
    if (trigPin == echoPin) {
        return ERROR_INVALID_PARAM;
    }
    
    uint64_t pulseLength = 0;
    int status = ultrasonic_get_pulse(trigPin, echoPin, &pulseLength);
    
    if (status != SUCCESS) {
        return status;
    }
    
    /* Calculate distance: Speed of sound = 343 m/s = 29 us/cm (round trip) */
    *distance = pulseLength / 29 / 2;
    
    if (*distance < MIN_DISTANCE || *distance > MAX_DISTANCE) {
        return ERROR_OUT_OF_RANGE;
    }
    
    return SUCCESS;
}
