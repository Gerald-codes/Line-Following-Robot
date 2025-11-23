/**
 * @file    pin_definitions.h
 * @brief   GPIO pin assignments for robot hardware
 * @details Defines pin mappings for motors, encoders, and sensors
 *          on the Raspberry Pi Pico platform
 */

#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

/* Motor pins */
#define M1A                 8
#define M1B                 9
#define M2A                 10
#define M2B                 11

/* Encoder pins */
#define LEFT_ENCODER        3
#define RIGHT_ENCODER       5

/* Sensor pins */
#define IR_BARCODE_PIN      6

#endif /* PIN_DEFINITIONS_H */
