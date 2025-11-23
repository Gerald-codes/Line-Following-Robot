/**
 * @file    motor.h
 * @brief   Motor interface and driver routines for robot control
 * @details
 *   Defines pin assignments and provides initialization,
 *   drive, stop, and brake functions for the motor subsystem.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"

/**
 * @brief   Motor pin definitions
 */
#define M1A  8
#define M1B  9
#define M2A 10
#define M2B 11

/**
 * @brief   Initialize motor channel pins
 * @param   pinA First motor pin (e.g., M1A)
 * @param   pinB Second motor pin (e.g., M1B)
 */
void motor_init(uint pinA, uint pinB);

/**
 * @brief   Drive motor with specified speed
 * @param   pinA First motor pin
 * @param   pinB Second motor pin
 * @param   speed Signed speed; positive for forward, negative for reverse
 */
void motor_drive(uint pinA, uint pinB, int speed);

/**
 * @brief   Stop motor (disable both pins)
 * @param   pinA First motor pin
 * @param   pinB Second motor pin
 */
void motor_stop(uint pinA, uint pinB);

/**
 * @brief   Brake motor (active braking routine)
 * @param   pinA First motor pin
 * @param   pinB Second motor pin
 */
void motor_brake(uint pinA, uint pinB);

#endif /* MOTOR_H */
