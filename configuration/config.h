#ifndef CONFIG_H
#define CONFIG_H

#include "pico/stdlib.h"
#include <stdbool.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Wheel specifications - SEPARATE VALUES FOR EACH ENCODER!
#define WHEEL_DIAMETER_MM 65.0f
#define WHEEL_BASE_MM 130.0f
#define LEFT_PULSES_PER_REV 41           // Calibrated left encoder value
#define RIGHT_PULSES_PER_REV 45          // Calibrated right encoder value
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * 3.14159f)
#define LEFT_MM_PER_PULSE (WHEEL_CIRCUMFERENCE_MM / LEFT_PULSES_PER_REV)
#define RIGHT_MM_PER_PULSE (WHEEL_CIRCUMFERENCE_MM / RIGHT_PULSES_PER_REV)

#define PID_UPDATE_INTERVAL_MS 100
#define TELEMETRY_INTERVAL_MS 500
#define SENSOR_READ_INTERVAL_MS 10

// ============================================================================
// MOTOR SPEED PID PARAMETERS - TUNED FOR LOW RESOLUTION ENCODERS
// ============================================================================

#define MOTOR_PID_KP 0.08f                // Very low - prevents overreaction
#define MOTOR_PID_KI 0.01f                // Slow integral buildup
#define MOTOR_PID_KD 0.00f                // No derivative - too noisy with low PPR
#define MOTOR_PID_OUTPUT_MIN -100
#define MOTOR_PID_OUTPUT_MAX 100

// ============================================================================
// LINE FOLLOWING PID - Single controller for all situations
// 🎯 TUNE THESE FOR YOUR ROBOT
// ============================================================================

#define LINE_PID_KP 0.05f            // 🔧 Main tuning knob
                                      // Position +1000 → Steering = 5 mm/s
                                      // 
                                      // Too jerky? → DECREASE to 0.003
                                      // Too slow to correct? → INCREASE to 0.008
                                      //
                                      // Range: 0.003 - 0.010

#define LINE_PID_KI 0.01f            // 🔧 Leave at 0 unless steady drift
                                      // Only increase if robot consistently pulls one direction

#define LINE_PID_KD 0.01f            // 🔧 Damping - prevents overshoot
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

// Start moderate - low resolution encoders need lower speeds
#define DEMO1_BASE_SPEED_MM_S 200.0f      // Moderate speed

#define DEMO2_BASE_SPEED_MM_S 100.0f
#define DEMO2_TURN_SPEED_MM_S 80.0f

#define DEMO3_BASE_SPEED_MM_S 100.0f
#define DEMO3_SCAN_SPEED_MM_S 60.0f
#define DEMO3_AVOID_SPEED_MM_S 80.0f

#define MIN_SPEED_MM_S 30.0f
#define MAX_SPEED_MM_S 300.0f

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
// STATE MACHINE (Demo 2, 3)
// ============================================================================

typedef enum {
    ROBOT_STATE_IDLE = 0,
    ROBOT_STATE_FOLLOWING,
    ROBOT_STATE_TURNING,
    ROBOT_STATE_STOPPED,
    ROBOT_STATE_SCANNING,
    ROBOT_STATE_AVOIDING,
    ROBOT_STATE_RETURNING,
    ROBOT_STATE_LOST,
    ROBOT_STATE_ERROR
} RobotState;

#define STATE_TIMEOUT_TURNING_MS 5000
#define STATE_TIMEOUT_AVOIDING_MS 10000
#define STATE_TIMEOUT_LOST_MS 3000

// ============================================================================
// TELEMETRY (Demo 1, 2, 3)
// ============================================================================

#define MQTT_TOPIC_SPEED "robot/speed"
#define MQTT_TOPIC_HEADING "robot/heading"
#define MQTT_TOPIC_DISTANCE "robot/distance"
#define MQTT_TOPIC_LINE "robot/line"
#define MQTT_TOPIC_BARCODE "robot/barcode"
#define MQTT_TOPIC_OBSTACLE "robot/obstacle"
#define MQTT_TOPIC_STATE "robot/state"
#define MQTT_TOPIC_ERROR "robot/error"

// ============================================================================
// SAFETY LIMITS
// ============================================================================

#define SAFETY_MAX_SPEED_MM_S 300
#define SAFETY_LINE_LOST_TIMEOUT_MS 2000
#define SAFETY_STUCK_TIMEOUT_MS 3000
#define SAFETY_STUCK_SPEED_THRESHOLD 10

// ============================================================================
// DEBUG OPTIONS
// ============================================================================

#define DEBUG_PRINT_INTERVAL_MS 500
#define DEBUG_ENABLE_MOTOR_PID 1
#define DEBUG_ENABLE_LINE_PID 1
#define DEBUG_ENABLE_HEADING 1
#define DEBUG_ENABLE_SENSORS 0

// ============================================================================
// MOTOR CALIBRATION - NO LONGER NEEDED WITH CORRECT ENCODER VALUES
// ============================================================================

#define LEFT_MOTOR_CORRECTION 1.00f       
#define RIGHT_MOTOR_CORRECTION 1.00f      

#endif // CONFIG_H