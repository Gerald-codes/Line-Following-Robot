/**
 * @file    servo.h
 * @brief   SG90 Servo motor interface and PWM configuration
 * @details
 *   GPIO and timing definitions, plus core functions for initializing,
 *   setting, and reading servo angle using PWM.
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <sys/types.h>

/**
 * @brief   Servo pin assignment
 */
#define SERVO_PIN        15

/**
 * @brief   SG90 standard servo parameters
 */
#define SERVO_MIN_PULSE  500    /**< 0.5ms pulse width (0 degrees) */
#define SERVO_MAX_PULSE 2500    /**< 2.5ms pulse width (180 degrees) */
#define SERVO_FREQ        50    /**< 50Hz (20ms PWM period) */
#define SERVO_OFFSET       0    /**< Pulse width offset for physical center */

/**
 * @brief   Initialize servo motor PWM
 * @param   servoPin GPIO pin to use for servo signal
 */
void servo_init(unsigned int servoPin);

/**
 * @brief   Set servo to specified angle
 * @param   angle Angle in degrees (0-180)
 */
void servo_set_angle(int angle);

/**
 * @brief   Get current servo angle
 * @return  Current angle in degrees (0-180)
 */
int servo_get_angle(void);

#endif /* SERVO_H */
