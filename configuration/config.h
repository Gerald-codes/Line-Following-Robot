/**
 * config.h - SIMPLIFIED LINE FOLLOWING
 * Single PID configuration - no mode switching
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "pico/stdlib.h"
#include <stdbool.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

#define WHEEL_DIAMETER_MM 65.0f
#define WHEEL_BASE_MM 130.0f
#define PULSES_PER_REVOLUTION 43
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * 3.14159f)
#define MM_PER_PULSE (WHEEL_CIRCUMFERENCE_MM / PULSES_PER_REVOLUTION)

#define PID_UPDATE_INTERVAL_MS 100
#define TELEMETRY_INTERVAL_MS 500
#define SENSOR_READ_INTERVAL_MS 10

// ============================================================================
// MOTOR SPEED PID - Controls how motors reach target speeds
// ============================================================================

#define MOTOR_PID_KP 0.5f
#define MOTOR_PID_KI 0.01f
#define MOTOR_PID_KD 0.0f
#define MOTOR_PID_OUTPUT_MIN -100
#define MOTOR_PID_OUTPUT_MAX 100

// ============================================================================
// LINE FOLLOWING PID - Single controller for all situations
// 🎯 TUNE THESE FOR YOUR ROBOT
// ============================================================================

#define LINE_PID_KP 0.01f            // 🔧 Main tuning knob
                                      // Position +1000 → Steering = 5 mm/s
                                      // 
                                      // Too jerky? → DECREASE to 0.003
                                      // Too slow to correct? → INCREASE to 0.008
                                      //
                                      // Range: 0.003 - 0.010

#define LINE_PID_KI 0.00f            // 🔧 Leave at 0 unless steady drift
                                      // Only increase if robot consistently pulls one direction

#define LINE_PID_KD 0.03f            // 🔧 Damping - prevents overshoot
                                      // Kd/Kp ratio: 0.015/0.005 = 3.0 (good damping)
                                      //
                                      // Still oscillates? → INCREASE to 0.020
                                      // Too sluggish? → DECREASE to 0.010
                                      //
                                      // Range: 0.010 - 0.025

#define LINE_STEERING_MAX 40         // 🔧 Maximum steering correction (mm/s)
                                      // With base speed 55: motors range 35-75 mm/s
                                      // Ratio: 75/35 = 2.1:1 (moderate turning)
                                      //
                                      // Too violent? → DECREASE to 15
                                      // Can't make corners? → INCREASE to 25
                                      //
                                      // Range: 15 - 30

#define LINE_POSITION_MIN -2000
#define LINE_POSITION_CENTER 0
#define LINE_POSITION_MAX 2000

// ============================================================================
// SPEED SETTINGS
// ============================================================================

extern float demo2_base_speed_mm_s;   // Runtime adjustable base speed
                                      // Default: 55 mm/s

// ============================================================================
// MOTOR CALIBRATION
// ============================================================================

#define LEFT_MOTOR_OFFSET 2           // Power offset if motors unbalanced
                                      // Robot veers right? → INCREASE
                                      // Robot veers left? → DECREASE

// ============================================================================
// IMU CONFIGURATION
// ============================================================================

#define IMU_FILTER_ALPHA 0.96f
#define IMU_CALIBRATION_SAMPLES 100
#define IMU_GYRO_BIAS_MAX 5.0f
#define IMU_HEADING_CHANGE_THRESHOLD 1.0f
#define HEADING_TOLERANCE_DEGREES 5.0f
// ============================================================================
// TUNING GUIDE
// ============================================================================
//
// PROBLEM: Robot oscillates (swings left-right repeatedly)
//   → DECREASE LINE_PID_KP to 0.003
//   → INCREASE LINE_PID_KD to 0.020
//
// PROBLEM: Robot drifts off line slowly
//   → INCREASE LINE_PID_KP to 0.008
//   → Check LEFT_MOTOR_OFFSET
//
// PROBLEM: Steering always at max (±20 in debug)
//   → DECREASE LINE_PID_KP (it's too high!)
//   → Or INCREASE LINE_STEERING_MAX
//
// PROBLEM: Can't follow line at all
//   → Check sensor calibration first!
//   → Verify position values are reasonable (±500 to ±1500)
//   → Try LINE_PID_KP = 0.010 and LINE_STEERING_MAX = 30
//
// ============================================================================

#endif // CONFIG_H