#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

void ultrasonic_init(unsigned int trigPin, unsigned int echoPin) {
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
}

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

int ultrasonic_get_distance(unsigned int trigPin, unsigned int echoPin, uint64_t *distance) {
    if (trigPin > 29 || echoPin > 29) {
        return ERROR_INVALID_PARAM;
    }
    
    if (trigPin == echoPin) {
        return ERROR_INVALID_PARAM;
    }
    
    uint64_t pulseLength;
    int status = ultrasonic_get_pulse(trigPin, echoPin, &pulseLength);
    
    if (status != SUCCESS) {
        return status;
    }
    
    *distance = pulseLength / 29 / 2;
    
    if (*distance < MIN_DISTANCE || *distance > MAX_DISTANCE) {
        return ERROR_OUT_OF_RANGE;
    }
    
    return SUCCESS;
}