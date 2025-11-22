/**
 * config.h - Configuration for Single IR Line Following
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Wheel specifications
#define WHEEL_DIAMETER_MM 65.0f
#define WHEEL_BASE_MM 130.0f
#define LEFT_PULSES_PER_REV 41
#define RIGHT_PULSES_PER_REV 45
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * 3.14159f)
#define LEFT_MM_PER_PULSE (WHEEL_CIRCUMFERENCE_MM / LEFT_PULSES_PER_REV)
#define RIGHT_MM_PER_PULSE (WHEEL_CIRCUMFERENCE_MM / RIGHT_PULSES_PER_REV)

// Control loop timing
#define PID_UPDATE_INTERVAL_MS 100
#define TELEMETRY_INTERVAL_MS 500
#define SENSOR_READ_INTERVAL_MS 10

// ============================================================================
// MOTOR SPEED PID PARAMETERS
// ============================================================================

#define MOTOR_PID_KP 0.08f
#define MOTOR_PID_KI 0.01f
#define MOTOR_PID_KD 0.00f
#define MOTOR_PID_OUTPUT_MIN -100
#define MOTOR_PID_OUTPUT_MAX 100

// ============================================================================
// SINGLE IR LINE FOLLOWING PID
// ============================================================================
// These are BASE values - adaptive gains will scale them

#define LINE_PID_KP 1.5f            // Base proportional gain
                                     // Error of 1.0 → Steering ~1.5
                                     // 
                                     // Too jerky? → DECREASE to 1.0
                                     // Too slow? → INCREASE to 2.5
                                     // Range: 1.0 - 3.0

#define LINE_PID_KI 0.0f             // Keep at zero for simple control
                                     // Only add if steady-state error

#define LINE_PID_KD 0.8f             // Damping to prevent oscillation
                                     // Good Kd/Kp ratio: ~0.5
                                     // 
                                     // Still oscillates? → INCREASE to 1.5
                                     // Too sluggish? → DECREASE to 0.5
                                     // Range: 0.5 - 2.0

// ============================================================================
// IR SENSOR CONFIGURATION
// ============================================================================
// Single sensor positioned on line edge
// Lower reading = white, higher reading = black

#define IR_WHITE_DEFAULT 200         // Typical white surface reading
#define IR_BLACK_DEFAULT 3000        // Typical black line reading
#define IR_THRESHOLD_OFFSET 100      // Added to average for threshold

// ============================================================================
// HEADING PID PARAMETERS
// ============================================================================

#define HEADING_PID_KP_DEMO1 0.15f
#define HEADING_PID_KI_DEMO1 0.00f
#define HEADING_PID_KD_DEMO1 0.00f

#define HEADING_PID_KP_DEMO23 0.20f
#define HEADING_PID_KI_DEMO23 0.00f
#define HEADING_PID_KD_DEMO23 0.00f

#define HEADING_CORRECTION_MAX 20
#define HEADING_TOLERANCE_DEGREES 5.0f

// ============================================================================
// SPEED SETTINGS
// ============================================================================

#define DEMO1_BASE_SPEED_MM_S 200.0f
#define DEMO2_BASE_SPEED_MM_S 60.0f
#define DEMO2_TURN_SPEED_MM_S 80.0f
#define DEMO3_BASE_SPEED_MM_S 100.0f
#define DEMO3_SCAN_SPEED_MM_S 60.0f
#define DEMO3_AVOID_SPEED_MM_S 80.0f

#define MIN_SPEED_MM_S 30.0f
#define MAX_SPEED_MM_S 300.0f

// ============================================================================
// IMU CONFIGURATION
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
// STATE MACHINE
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
// TELEMETRY
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
// MOTOR CALIBRATION
// ============================================================================
#define LEFT_MOTOR_OFFSET 2
#define LEFT_MOTOR_CORRECTION 1.00f
#define RIGHT_MOTOR_CORRECTION 1.00f

#endif // CONFIG_H