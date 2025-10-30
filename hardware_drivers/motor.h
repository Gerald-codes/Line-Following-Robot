#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"

// Motor pin definitions
#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11

// Function declarations
void motor_init(uint pinA, uint pinB);
void motor_drive(uint pinA, uint pinB, int speed);
void motor_stop(uint pinA, uint pinB);
void motor_brake(uint pinA, uint pinB);

#endif // MOTOR_H