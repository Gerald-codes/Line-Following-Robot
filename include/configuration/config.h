/**
 * @file    config.h
 * @brief   System configuration parameters for robot control
 * @details Comprehensive configuration file containing PID tuning parameters,
 *          hardware specifications, speed settings, sensor configurations,
 *          state machine definitions, and MQTT telemetry topics
 */

#ifndef CONFIG_H
#define CONFIG_H


/* HARDWARE CONFIGURATION */

/* Wheel specifications */
#define WHEEL_DIAMETER_MM           65.0f
#define WHEEL_BASE_MM               130.0f
#define LEFT_PULSES_PER_REV         41
#define RIGHT_PULSES_PER_REV        45
#define WHEEL_CIRCUMFERENCE_MM      (WHEEL_DIAMETER_MM * 3.14159f)
#define LEFT_MM_PER_PULSE           (WHEEL_CIRCUMFERENCE_MM / LEFT_PULSES_PER_REV)
#define RIGHT_MM_PER_PULSE          (WHEEL_CIRCUMFERENCE_MM / RIGHT_PULSES_PER_REV)

/* Control loop timing */
#define PID_UPDATE_INTERVAL_MS      100
#define TELEMETRY_INTERVAL_MS       1000
#define SENSOR_READ_INTERVAL_MS     10

/* MOTOR SPEED PID PARAMETERS */
#define MOTOR_PID_KP                0.08f
#define MOTOR_PID_KI                0.01f
#define MOTOR_PID_KD                0.00f
#define MOTOR_PID_OUTPUT_MIN        -100
#define MOTOR_PID_OUTPUT_MAX        100

/* SINGLE IR LINE FOLLOWING PID */

/* These are BASE values - adaptive gains will scale them */
#define LINE_PID_KP                 1.5f    /* Base proportional gain (1.0-3.0) */
#define LINE_PID_KI                 0.0f    /* Integral gain */
#define LINE_PID_KD                 0.8f    /* Damping gain (0.5-2.0) */


typedef enum
{
    OBSTACLE_STATE_NONE = 0,
    OBSTACLE_STATE_DETECTED,
    OBSTACLE_STATE_SCANNING,
    OBSTACLE_STATE_AVOIDING,
    OBSTACLE_STATE_RETURNING
} ObstacleState;


/* STATE MACHINE */
typedef enum
{
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

/* TELEMETRY - MQTT TOPICS */

/* WiFi credentials */
#define WIFI_SSID                   "Javiersphone"
#define WIFI_PASSWORD               "imcool123"

/* MQTT broker settings */
#define MQTT_BROKER_IP              "10.170.105.160"
#define MQTT_BROKER_PORT            1883
#define MQTT_CLIENT_ID              "pico_robot_car"

/* Core telemetry topics */
#define MQTT_TOPIC_MOTOR            "robot/motor"
#define MQTT_TOPIC_LINE             "robot/line"
#define MQTT_TOPIC_IMU              "robot/imu"
#define MQTT_TOPIC_STATE            "robot/state"
#define MQTT_TOPIC_CALIBRATION      "robot/calibration"
#define MQTT_TOPIC_STATUS           "robot/status"

/* Encoder-based telemetry topics */
#define MQTT_TOPIC_SPEED            "robot/speed"
#define MQTT_TOPIC_DISTANCE         "robot/distance"
#define MQTT_TOPIC_ENCODER          "robot/encoder"

/* Barcode telemetry topic */
#define MQTT_TOPIC_BARCODE          "robot/barcode"

/* Obstacle detection telemetry topics */
#define MQTT_TOPIC_OBSTACLE         "robot/obstacle"
#define MQTT_TOPIC_SCAN             "robot/scan"
#define MQTT_TOPIC_AVOIDANCE        "robot/avoidance"

/* Error and diagnostics topics */
#define MQTT_TOPIC_ERROR            "robot/error"
#define MQTT_TOPIC_DIAGNOSTICS      "robot/diagnostics"

#endif /* CONFIG_H */
