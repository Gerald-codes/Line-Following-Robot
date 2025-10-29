#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"

// Ultrasonic sensor pins
#define TRIG_PIN 5
#define ECHO_PIN 4
#define TIMEOUT_US 30000
#define MIN_DISTANCE 2
#define MAX_DISTANCE 400

// Servo pin
#define SERVO_PIN 17

// Servo parameters (SG90 standard servo)
#define SERVO_MIN_PULSE 500   // 0.5ms = 0 degrees
#define SERVO_MAX_PULSE 2500  // 2.5ms = 180 degrees
#define SERVO_FREQ 50         // 50Hz = 20ms period
#define SERVO_OFFSET 0        // Adjust this to set your physical center position
                              // If mounted at different angle, adjust here
                              // Try: -10, -5, 0, 5, 10 degrees

// Scanning parameters
#define ANGLE_CENTER 75      // Center position of servo
#define MIN_ANGLE ANGLE_CENTER - 30          // 30° left from center
#define MAX_ANGLE ANGLE_CENTER + 30         // 30° right from center
#define SCAN_STEP 3           // Degrees per step
#define OBSTACLE_THRESHOLD_MIN 5   // cm
#define OBSTACLE_THRESHOLD_MAX 100 // cm
#define MIN_OBSTACLE_SPAN 10   // Minimum angle span to count as obstacle
#define DISTANCE_CHANGE_THRESHOLD 25  // cm - separate objects if distance changes by this much

// Error codes
#define SUCCESS 0
#define ERROR_TIMEOUT 1
#define ERROR_INVALID_PARAM 2
#define ERROR_OUT_OF_RANGE 3

// Servo control variables
uint servo_slice;
uint servo_channel;
uint32_t servo_wrap;  // Store wrap value for later use

void setupUltrasonicPins(uint trigPin, uint echoPin) {
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
}

void setupServo(uint servoPin) {
    gpio_set_function(servoPin, GPIO_FUNC_PWM);
    servo_slice = pwm_gpio_to_slice_num(servoPin);
    servo_channel = pwm_gpio_to_channel(servoPin);
    
    // Calculate PWM parameters for 50Hz
    // Clock = 125MHz, need 50Hz (20ms period)
    uint32_t clock_freq = 125000000;
    uint32_t divider = 64;
    servo_wrap = (clock_freq / divider / SERVO_FREQ) - 1;
    
    pwm_set_clkdiv(servo_slice, divider);
    pwm_set_wrap(servo_slice, servo_wrap);
    pwm_set_enabled(servo_slice, true);
}

void setServoAngle(int angle) {
    // Apply offset to all angles
    int adjusted_angle = angle + SERVO_OFFSET;
    
    if (adjusted_angle < 0) adjusted_angle = 0;
    if (adjusted_angle > 180) adjusted_angle = 180;
    
    // Calculate pulse width for angle
    uint32_t pulse_width = SERVO_MIN_PULSE + 
                          ((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * adjusted_angle / 180);
    
    // Convert to PWM level
    // Period is 20ms = 20000us
    uint32_t level = (servo_wrap * pulse_width) / 20000;
    
    pwm_set_chan_level(servo_slice, servo_channel, level);
}

int getPulse(uint trigPin, uint echoPin, uint64_t *pulse_width) {
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

    uint64_t width = 0;

    // Wait for echo
    while (gpio_get(echoPin) == 0) {
        width++;
        sleep_us(1);
        if (width > TIMEOUT_US) {
            return ERROR_TIMEOUT;
        }
    }
    
    absolute_time_t startTime = get_absolute_time();
    width = 0;
    
    // Measure echo pulse
    while (gpio_get(echoPin) == 1) {
        width++;
        sleep_us(1);
        if (width > TIMEOUT_US) {
            return ERROR_TIMEOUT;
        }
    }
    
    absolute_time_t endTime = get_absolute_time();
    *pulse_width = absolute_time_diff_us(startTime, endTime);
    
    return SUCCESS;
}

int getCm(uint trigPin, uint echoPin, uint64_t *distance) {
    if (trigPin > 29 || echoPin > 29) {
        return ERROR_INVALID_PARAM;
    }
    
    if (trigPin == echoPin) {
        return ERROR_INVALID_PARAM;
    }
    
    uint64_t pulseLength;
    int status = getPulse(trigPin, echoPin, &pulseLength);
    
    if (status != SUCCESS) {
        return status;
    }
    
    *distance = pulseLength / 29 / 2;
    
    if (*distance < MIN_DISTANCE || *distance > MAX_DISTANCE) {
        return ERROR_OUT_OF_RANGE;
    }
    
    return SUCCESS;
}

float calculateObstacleWidth(int angle_start, int angle_end, uint64_t distance) {
    int angle_diff = angle_end - angle_start;
    
    if (angle_diff <= 0) {
        return 0.0f;
    }
    
    // Convert angle difference to radians
    float angle_radians = (angle_diff * 3.14159265359f) / 180.0f;
    float sin_value = sinf(angle_radians / 2.0f);
    
    // Chord length formula: width = 2 * r * sin(θ/2)
    float width = 2.0f * (float)distance * sin_value;
    
    return width;
}

void scanForObstacles() {
    printf("\n=== Starting Scan ===\n");
    
    bool in_obstacle = false;
    int obstacle_start_angle = 0;
    int obstacle_count = 0;
    uint64_t obstacle_distance = 0;
    uint64_t last_valid_distance = 0;
    int timeout_count = 0;
    
    // Scan from left to right
    for (int angle = MIN_ANGLE; angle <= MAX_ANGLE; angle += SCAN_STEP) {
        setServoAngle(angle);
        sleep_ms(100); // Wait for servo to move and stabilize
        
        uint64_t distance;
        int status = getCm(TRIG_PIN, ECHO_PIN, &distance);
        
        if (status == SUCCESS) {
            timeout_count = 0;  // Reset timeout counter
            
            bool obstacle_detected = (distance >= OBSTACLE_THRESHOLD_MIN && 
                                     distance <= OBSTACLE_THRESHOLD_MAX);
            
            printf("Angle: %3d° | Distance: %3llu cm", angle, distance);
            
            if (obstacle_detected && !in_obstacle) {
                // Start of new obstacle
                in_obstacle = true;
                obstacle_start_angle = angle;
                obstacle_distance = distance;
                last_valid_distance = distance;
                obstacle_count++;
                printf(" >>> START\n");
            }
            else if (obstacle_detected && in_obstacle) {
                // Check if distance changed significantly (new object)
                uint64_t distance_diff = (obstacle_distance > distance) ? 
                                        (obstacle_distance - distance) : 
                                        (distance - obstacle_distance);
                
                if (distance_diff > DISTANCE_CHANGE_THRESHOLD) {
                    // Large distance change - this is a different object
                    printf(" >>> DISTANCE JUMP (end of previous, start of new)\n");
                    
                    // End previous obstacle
                    int obstacle_end_angle = angle - SCAN_STEP;
                    int angle_span = obstacle_end_angle - obstacle_start_angle;
                    
                    if (angle_span >= MIN_OBSTACLE_SPAN) {
                        float width = calculateObstacleWidth(obstacle_start_angle, 
                                                            obstacle_end_angle, 
                                                            obstacle_distance);
                        printf("\n*** OBSTACLE #%d ***\n", obstacle_count);
                        printf("  Angle: %d° to %d° (span: %d°)\n", 
                               obstacle_start_angle, obstacle_end_angle, angle_span);
                        printf("  Distance: %llu cm\n", obstacle_distance);
                        printf("  Width: %.2f cm\n\n", width);
                    } else {
                        obstacle_count--;
                    }
                    
                    // Start new obstacle
                    obstacle_start_angle = angle;
                    obstacle_distance = distance;
                    obstacle_count++;
                } else {
                    // Continue tracking same obstacle
                    printf(" >>> IN OBSTACLE\n");
                    // Average the distance for more stable readings
                    obstacle_distance = (obstacle_distance * 3 + distance) / 4;
                }
                last_valid_distance = distance;
            }
            else if (!obstacle_detected && in_obstacle) {
                // End of obstacle
                in_obstacle = false;
                int obstacle_end_angle = angle - SCAN_STEP;
                int angle_span = obstacle_end_angle - obstacle_start_angle;
                
                printf(" >>> END\n");
                
                if (angle_span >= MIN_OBSTACLE_SPAN) {
                    float width = calculateObstacleWidth(obstacle_start_angle, 
                                                        obstacle_end_angle, 
                                                        obstacle_distance);
                    printf("\n*** OBSTACLE #%d ***\n", obstacle_count);
                    printf("  Angle: %d° to %d° (span: %d°)\n", 
                           obstacle_start_angle, obstacle_end_angle, angle_span);
                    printf("  Distance: %llu cm\n", obstacle_distance);
                    printf("  Width: %.2f cm\n\n", width);
                } else {
                    obstacle_count--;
                }
            }
            else {
                printf(" [Clear]\n");
            }
        }
        else {
            // Timeout - skip but don't end obstacle immediately
            timeout_count++;
            printf("Angle: %3d° | Timeout (%d)\n", angle, timeout_count);
            
            // If too many timeouts in a row, end the obstacle
            if (timeout_count > 2 && in_obstacle) {
                in_obstacle = false;
                int obstacle_end_angle = angle - (SCAN_STEP * 2);
                int angle_span = obstacle_end_angle - obstacle_start_angle;
                
                printf(" >>> OBSTACLE END (timeout)\n");
                
                if (angle_span >= MIN_OBSTACLE_SPAN) {
                    float width = calculateObstacleWidth(obstacle_start_angle, 
                                                        obstacle_end_angle, 
                                                        obstacle_distance);
                    printf("\n*** OBSTACLE #%d ***\n", obstacle_count);
                    printf("  Angle: %d° to %d° (span: %d°)\n", 
                           obstacle_start_angle, obstacle_end_angle, angle_span);
                    printf("  Distance: %llu cm\n", obstacle_distance);
                    printf("  Width: %.2f cm\n\n", width);
                } else {
                    obstacle_count--;
                }
            }
        }
    }
    
    // Handle case where obstacle extends to end of scan
    if (in_obstacle) {
        int angle_span = MAX_ANGLE - obstacle_start_angle;
        
        if (angle_span >= MIN_OBSTACLE_SPAN) {
            float width = calculateObstacleWidth(obstacle_start_angle, 
                                                MAX_ANGLE, 
                                                obstacle_distance);
            printf("\n*** OBSTACLE #%d ***\n", obstacle_count);
            printf("  Angle: %d° to %d° (span: %d°)\n", 
                   obstacle_start_angle, MAX_ANGLE, angle_span);
            printf("  Distance: %llu cm\n", obstacle_distance);
            printf("  Width: %.2f cm\n\n", width);
        } else {
            obstacle_count--;
        }
    }
    
    printf("=== Scan Complete ===\n");
    printf("Obstacles found: %d\n\n", obstacle_count);
    
    // Return to center position
    setServoAngle(ANGLE_CENTER);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n========================================\n");
    printf("HC-SR04 Ultrasonic + Servo Scanner\n");
    printf("========================================\n");
    printf("Ultrasonic Sensor:\n");
    printf("  TRIG: GPIO %d\n", TRIG_PIN);
    printf("  ECHO: GPIO %d\n", ECHO_PIN);
    printf("Servo Motor:\n");
    printf("  Signal: GPIO %d\n", SERVO_PIN);
    printf("========================================\n");
    printf("Detection Range: %d-%d cm\n", OBSTACLE_THRESHOLD_MIN, OBSTACLE_THRESHOLD_MAX);
    printf("Scan Range: %d° to %d° (step: %d°)\n", MIN_ANGLE, MAX_ANGLE, SCAN_STEP);
    printf("Min Obstacle Span: %d°\n", MIN_OBSTACLE_SPAN);
    printf("Distance Threshold: %d cm\n", DISTANCE_CHANGE_THRESHOLD);
    printf("========================================\n\n");
    
    setupUltrasonicPins(TRIG_PIN, ECHO_PIN);
    setupServo(SERVO_PIN);
    
    // Center the servo initially
    setServoAngle(90);
    sleep_ms(500);
    
    printf("System ready!\n");
    printf("Starting scan in 2 seconds...\n\n");
    sleep_ms(2000);
    
    int scan_count = 0;
    
    while (1) {
        scan_count++;
        printf("\n========== SCAN #%d ==========\n", scan_count);
        
        // Perform full scan
        scanForObstacles();
        
        // Wait before next scan
        printf("Waiting 3 seconds before next scan...\n");
        sleep_ms(3000);
    }
    
    return 0;
}