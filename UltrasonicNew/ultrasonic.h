#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <sys/types.h>

// Ultrasonic sensor pins
#define TRIG_PIN 1
#define ECHO_PIN 0
#define TIMEOUT_US 30000
#define MIN_DISTANCE 2
#define MAX_DISTANCE 400

// Error codes
#define SUCCESS 0
#define ERROR_TIMEOUT 1
#define ERROR_INVALID_PARAM 2
#define ERROR_OUT_OF_RANGE 3

/**
 * Initialize ultrasonic sensor GPIO pins
 * @param trigPin Trigger pin number
 * @param echoPin Echo pin number
 */
void ultrasonic_init(unsigned int trigPin, unsigned int echoPin);

/**
 * Send trigger pulse and measure echo time
 * @param trigPin Trigger pin number
 * @param echoPin Echo pin number
 * @param pulse_width Output: measured pulse width in microseconds
 * @return SUCCESS or error code
 */
int ultrasonic_get_pulse(unsigned int trigPin, unsigned int echoPin, uint64_t *pulse_width);

/**
 * Get distance measurement in centimeters
 * @param trigPin Trigger pin number
 * @param echoPin Echo pin number
 * @param distance Output: distance in centimeters
 * @return SUCCESS or error code
 */
int ultrasonic_get_distance(unsigned int trigPin, unsigned int echoPin, uint64_t *distance);

#endif