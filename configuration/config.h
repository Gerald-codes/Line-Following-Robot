/**
 * config.h - FIXED WITH SEPARATE ENCODER VALUES
 * Uses correct encoder values for left (41 PPR) and right (45 PPR)
 */

#ifndef CONFIG_H
#define CONFIG_H

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

// Control loop timing - MUCH SLOWER for low resolution encoders
#define PID_UPDATE_INTERVAL_MS 100        // 10Hz - gives time for multiple pulses
#define TELEMETRY_INTERVAL_MS 500         // 2Hz telemetry
#define SENSOR_READ_INTERVAL_MS 10        // 100Hz sensor reading

// ============================================================================
// MOTOR SPEED PID PARAMETERS - TUNED FOR LOW RESOLUTION ENCODERS
// ============================================================================

#define MOTOR_PID_KP 0.08f                // Very low - prevents overreaction
#define MOTOR_PID_KI 0.01f                // Slow integral buildup
#define MOTOR_PID_KD 0.00f                // No derivative - too noisy with low PPR
#define MOTOR_PID_OUTPUT_MIN -100
#define MOTOR_PID_OUTPUT_MAX 100

// ============================================================================
// LINE FOLLOWING PID PARAMETERS (Demo 2)
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
// HEADING PID PARAMETERS (Demo 1, 2, 3)
// ============================================================================

// Demo 1: Straight driving - VERY GENTLE
#define HEADING_PID_KP_DEMO1 0.15f        // Low correction
#define HEADING_PID_KI_DEMO1 0.00f        // No integral for now
#define HEADING_PID_KD_DEMO1 0.00f        // No derivative - too noisy

// Demo 2 & 3: Accurate turning
#define HEADING_PID_KP_DEMO23 0.20f
#define HEADING_PID_KI_DEMO23 0.00f
#define HEADING_PID_KD_DEMO23 0.00f

#define HEADING_CORRECTION_MAX 20         // Very limited correction
#define HEADING_TOLERANCE_DEGREES 5.0f    // More tolerance

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
// IR SENSOR CONFIGURATION (Demo 2, 3)
// ============================================================================

#define NUM_IR_SENSORS 5
#define IR_THRESHOLD_WHITE 2000
#define IR_THRESHOLD_BLACK 500
#define IR_LINE_LOST_THRESHOLD 100

#define IR_WEIGHT_0 -2000
#define IR_WEIGHT_1 -1000
#define IR_WEIGHT_2 0
#define IR_WEIGHT_3 1000
#define IR_WEIGHT_4 2000

// ============================================================================
// IMU CONFIGURATION (Demo 1, 2, 3)
// ============================================================================

#define IMU_FILTER_ALPHA 0.96f
#define IMU_CALIBRATION_SAMPLES 100
#define IMU_GYRO_BIAS_MAX 5.0f
#define IMU_HEADING_CHANGE_THRESHOLD 1.0f

// ============================================================================
// ULTRASONIC SENSOR (Demo 3)
// ============================================================================

#define ULTRASONIC_TIMEOUT_US 30000
#define ULTRASONIC_MIN_DISTANCE_MM 20
#define ULTRASONIC_MAX_DISTANCE_MM 4000
#define OBSTACLE_DETECTION_DISTANCE_MM 200
#define OBSTACLE_CLEARANCE_MIN_MM 300

// ============================================================================
// SERVO CONFIGURATION (Demo 3)
// ============================================================================

#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define SERVO_FREQUENCY_HZ 50
#define SERVO_ANGLE_CENTER 90
#define SERVO_ANGLE_LEFT 20
#define SERVO_ANGLE_RIGHT 160
#define SERVO_SCAN_DELAY_MS 300

// ============================================================================
// OBSTACLE AVOIDANCE (Demo 3)
// ============================================================================

#define AVOID_TURN_ANGLE_DEG 90.0f
#define AVOID_FORWARD_DISTANCE_MM 300
#define AVOID_RETURN_TIMEOUT_MS 5000

typedef enum {
    OBSTACLE_STATE_NONE = 0,
    OBSTACLE_STATE_DETECTED,
    OBSTACLE_STATE_SCANNING,
    OBSTACLE_STATE_AVOIDING,
    OBSTACLE_STATE_RETURNING
} ObstacleState;

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