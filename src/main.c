/**
 * main.c
 * 
 * Application entry point
 * Minimal initialization and main loop
 */

#include "pico/stdlib.h"
#include <stdio.h>

// Hardware
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "ir_sensor.h"
#include "ultrasonic.h"
#include "servo.h"

// Control
#include "line_following.h"
#include "obstacle_scanner.h"
#include "avoidance_maneuver.h"

// Application
#include "state_machine.h"
#include "robot_controller.h"
#include "motor_controller.h"
#include "pid_controller.h"

// Utilities
#include "timer_manager.h"
#include "telemetry.h"
#include "calibration.h"
#include "config.h"
#include "pin_definitions.h"

// Global IMU instance
static IMU imu;

static void init_hardware(void) {
    printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║     INTEGRATED LINE FOLLOWING + OBSTACLE AVOIDANCE           ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    printf("Initializing hardware...\n");
    
    motor_init(M1A, M1B);
    motor_init(M2A, M2B);
    printf("  ✓ Motors\n");
    
    encoder_init();
    printf("  ✓ Encoders\n");
    
    imu_init(&imu);
    imu_calibrate(&imu);
    printf("  ✓ IMU (struct)\n");
    
    imu_helper_init();
    printf("  ✓ IMU Helper\n");
    
    ir_sensor_init();
    printf("  ✓ IR Sensors\n");
    
    ultrasonic_init(TRIG_PIN, ECHO_PIN);
    printf("  ✓ Ultrasonic\n");
    
    servo_init(SERVO_PIN);
    servo_set_angle(ANGLE_CENTER);
    printf("  ✓ Servo\n");
    
    calibration_init();
    printf("  ✓ Calibration Button\n");
    
    line_following_init();
    printf("  ✓ Line Following\n");
    
    scanner_init();
    avoidance_init();
    timer_manager_init();
    printf("  ✓ Obstacle Systems\n");
    
    printf("\n✓ All hardware initialized\n\n");
}

static void init_subsystems(void) {
    state_machine_init();
    robot_controller_init();
    motor_controller_init();
    pid_controller_init();
}

static void run_calibration(void) {
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("  IR SENSOR CALIBRATION\n");
    printf("  Press GP20 to start calibration\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    while (!calibration_button_pressed()) {
        sleep_ms(10);
    }
    
    calibration_run_sequence();
}

static void wait_for_start(void) {
    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf("  Press GP20 to start robot\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    
    while (!calibration_button_pressed()) {
        sleep_ms(10);
    }
    sleep_ms(200);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    // Initialize everything
    init_hardware();
    init_subsystems();
    
    // Optional telemetry
    // telemetry_init(MQTT_BROKER_ADDRESS, MQTT_CLIENT_ID);
    
    // Calibration sequence
    run_calibration();
    wait_for_start();
    
    // Start robot
    state_machine_transition(STATE_LINE_FOLLOWING);
    printf("\n>>> ROBOT STARTED <<<\n\n");
    
    // Main loop
    uint32_t last_update = to_ms_since_boot(get_absolute_time());
    
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        float dt = (current_time - last_update) / 1000.0f;
        
        // Clamp dt
        if (dt > 0.1f) dt = 0.02f;
        if (dt < 0.001f) dt = 0.001f;
        last_update = current_time;
        
        // Update sensors
        imu_update(&imu);
        imu_helper_update();
        
        // Update telemetry
        if (telemetry_is_connected()) {
            telemetry_process();
        }
        
        // Main robot control
        robot_controller_update(dt);
        
        sleep_ms(10);  // 100Hz loop
    }
    
    return 0;
}
