#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <sys/types.h>

// Servo pin
#define SERVO_PIN 17

// Servo parameters (SG90 standard servo)
#define SERVO_MIN_PULSE 500   // 0.5ms = 0 degrees
#define SERVO_MAX_PULSE 2500  // 2.5ms = 180 degrees
#define SERVO_FREQ 50         // 50Hz = 20ms period
#define SERVO_OFFSET 0        // Adjust this to set your physical center position

/**
 * Initialize servo motor PWM
 * @param servoPin GPIO pin for servo control
 */
void servo_init(unsigned int servoPin);

/**
 * Set servo to specified angle
 * @param angle Angle in degrees (0-180)
 */
void servo_set_angle(int angle);

/**
 * Get current servo angle
 * @return Current angle in degrees (0-180)
 */
int servo_get_angle(void);

#endif