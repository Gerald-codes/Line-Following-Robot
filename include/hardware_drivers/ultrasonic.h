/**
 * @file    ultrasonic.h
 * @brief   Ultrasonic distance sensor interface for obstacle detection
 * @details
 *   Pin definitions, timeout and error codes, and functions for
 *   initializing the ultrasonic sensor and obtaining pulse and distance measurements.
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <sys/types.h>

/**
 * @brief   Ultrasonic sensor pin assignments
 */
#define TRIG_PIN        28
#define ECHO_PIN         7
#define TIMEOUT_US   30000          /**< Timeout for echo wait (microseconds) */
#define MIN_DISTANCE      2         /**< Minimum valid distance (cm) */
#define MAX_DISTANCE    400         /**< Maximum valid distance (cm) */

/**
 * @brief   Ultrasonic sensor error codes
 */
#define SUCCESS             0       /**< Operation successful */
#define ERROR_TIMEOUT       1       /**< Timeout waiting for echo */
#define ERROR_INVALID_PARAM 2       /**< Invalid argument provided */
#define ERROR_OUT_OF_RANGE  3       /**< Measurement outside expected range */

/**
 * @brief   Initialize ultrasonic sensor GPIO pins
 * @param   trigPin Trigger pin number for sensor
 * @param   echoPin Echo pin number for sensor
 */
void ultrasonic_init(unsigned int trigPin, unsigned int echoPin);

/**
 * @brief   Send trigger pulse and measure echo time
 * @param   trigPin Trigger pin number
 * @param   echoPin Echo pin number
 * @param   pulse_width Output: measured pulse width (microseconds)
 * @return  SUCCESS or error code
 */
int ultrasonic_get_pulse(unsigned int trigPin, unsigned int echoPin, uint64_t *pulse_width);

/**
 * @brief   Get ultrasonic sensor distance measurement (centimeters)
 * @param   trigPin Trigger pin number
 * @param   echoPin Echo pin number
 * @param   distance Output: measured distance (cm)
 * @return  SUCCESS or error code
 */
int ultrasonic_get_distance(unsigned int trigPin, unsigned int echoPin, uint64_t *distance);

#endif /* ULTRASONIC_H */
