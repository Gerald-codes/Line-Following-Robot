# ============================================================================
# Test Suite CMakeLists.txt - Integrated Robot System
# ============================================================================

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ----------------------------------------------------------------------------
# Pico SDK Setup
# ----------------------------------------------------------------------------
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)
project(robo_pico C CXX ASM)
pico_sdk_init()

# ----------------------------------------------------------------------------
# Include Directories
# ----------------------------------------------------------------------------
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/configuration
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/hardware_drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/control_algo
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/utilities
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/utilities
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/control_algo
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/hardware_drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/configuration
)

# ----------------------------------------------------------------------------
# Build Robot System Library (ALL full system modules here)
# ----------------------------------------------------------------------------
add_library(robot_lib
    ../src/control_algo/obstacle_scanner.c
    ../src/control_algo/obstacle_control.c      
    ../src/control_algo/avoidance_maneuver.c    
    ../src/control_algo/pid.c
    
    ../src/hardware_drivers/ultrasonic.c
    ../src/hardware_drivers/servo.c
    ../src/hardware_drivers/motor.c
    ../src/hardware_drivers/encoder.c
    ../src/hardware_drivers/imu.c

    ../src/utilities/timer_manager.c
    ../src/utilities/telemetry.c
    ../src/utilities/mqtt_client.c
    ../src/utilities/wifi.c
)

target_link_libraries(robot_lib
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_timer
    hardware_irq
    hardware_i2c
    pico_cyw43_arch_lwip_threadsafe_background
    cyw43_driver
)

# ============================================================================
# OBSTACLE DETECTION TESTS (OBS-T1 through OBS-T9)
# ============================================================================

# Helper function to create obstacle tests
function(add_obs_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name}
        pico_stdlib
        robot_lib
        hardware_gpio
        hardware_pwm
        hardware_timer
    )
    pico_add_extra_outputs(${name})
    pico_enable_stdio_usb(${name} 1)
    pico_enable_stdio_uart(${name} 0)
endfunction()

add_obs_test(OBS_T1 unit_test/OBS_T1.c)
add_obs_test(OBS_T2 unit_test/OBS_T2.c)
add_obs_test(OBS_T3 unit_test/OBS_T3.c)
add_obs_test(OBS_T4 unit_test/OBS_T4.c)
add_obs_test(OBS_T5 unit_test/OBS_T5.c)
add_obs_test(OBS_T6 unit_test/OBS_T6.c)
add_obs_test(OBS_T7 unit_test/OBS_T7.c)
add_obs_test(OBS_T8 unit_test/OBS_T8.c)
add_obs_test(OBS_T9 unit_test/OBS_T9.c)


# MOTOR-T1: GPIO Initialization
add_executable(MOTOR_T1
    unit_test/MOTOR_T1.c
    ../src/hardware_drivers/motor.c
)
target_link_libraries(MOTOR_T1 pico_stdlib hardware_pwm hardware_gpio)
pico_add_extra_outputs(MOTOR_T1)
pico_enable_stdio_usb(MOTOR_T1 1)
pico_enable_stdio_uart(MOTOR_T1 0)

# MOTOR-T2: PWM Speed Levels
add_executable(MOTOR_T2
    unit_test/MOTOR_T2.c
    ../src/hardware_drivers/motor.c
)
target_link_libraries(MOTOR_T2 pico_stdlib hardware_pwm hardware_gpio)
pico_add_extra_outputs(MOTOR_T2)
pico_enable_stdio_usb(MOTOR_T2 1)
pico_enable_stdio_uart(MOTOR_T2 0)

# MOTOR-T3: Encoder Synchronization
add_executable(MOTOR_T3
    unit_test/MOTOR_T3.c
    ../src/hardware_drivers/motor.c
    ../src/hardware_drivers/encoder.c
)
target_link_libraries(MOTOR_T3 pico_stdlib hardware_pwm hardware_gpio hardware_irq)
pico_add_extra_outputs(MOTOR_T3)
pico_enable_stdio_usb(MOTOR_T3 1)
pico_enable_stdio_uart(MOTOR_T3 0)

# MOTOR-T4: PID Calculation
add_executable(MOTOR_T4
    unit_test/MOTOR_T4.c
    ../src/control_algo/pid.c
)
target_link_libraries(MOTOR_T4 pico_stdlib)
pico_add_extra_outputs(MOTOR_T4)
pico_enable_stdio_usb(MOTOR_T4 1)
pico_enable_stdio_uart(MOTOR_T4 0)

# MOTOR-T5: PID Output Limits
add_executable(MOTOR_T5
    unit_test/MOTOR_T5.c
    ../src/control_algo/pid.c
)
target_link_libraries(MOTOR_T5 pico_stdlib)
pico_add_extra_outputs(MOTOR_T5)
pico_enable_stdio_usb(MOTOR_T5 1)
pico_enable_stdio_uart(MOTOR_T5 0)

# MOTOR-T6: RPM Stability
add_executable(MOTOR_T6
    unit_test/MOTOR_T6.c
    ../src/hardware_drivers/motor.c
    ../src/hardware_drivers/encoder.c
    ../src/control_algo/pid.c
)
target_link_libraries(MOTOR_T6 pico_stdlib hardware_pwm hardware_gpio hardware_irq)
pico_add_extra_outputs(MOTOR_T6)
pico_enable_stdio_usb(MOTOR_T6 1)
pico_enable_stdio_uart(MOTOR_T6 0)

# MOTOR-T7: Distance Calibration
add_executable(MOTOR_T7
    unit_test/MOTOR_T7.c
    ../src/hardware_drivers/motor.c
    ../src/hardware_drivers/encoder.c
)
target_link_libraries(MOTOR_T7 pico_stdlib hardware_pwm hardware_gpio hardware_irq)
pico_add_extra_outputs(MOTOR_T7)
pico_enable_stdio_usb(MOTOR_T7 1)
pico_enable_stdio_uart(MOTOR_T7 0)


add_executable(integration_test_heading_keeping
    integration_test/INTEG_T4.c
    ../src/hardware_drivers/motor.c
    ../src/hardware_drivers/encoder.c
    ../src/hardware_drivers/imu.c
    ../src/control_algo/pid.c
    ../src/control_algo/heading_control.c
    ../src/utilities/calibration.c
)

target_link_libraries(integration_test_heading_keeping
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_irq
    hardware_i2c
    robot_lib
)

pico_add_extra_outputs(integration_test_heading_keeping)
pico_enable_stdio_usb(integration_test_heading_keeping 1)
pico_enable_stdio_uart(integration_test_heading_keeping 0)


add_executable(integration_test_obstacle
    integration_test/telemetry_obstacle.c
)

target_link_libraries(integration_test_obstacle
    pico_stdlib
    robot_lib
    hardware_gpio
    hardware_pwm
    hardware_timer
    hardware_irq
)

pico_add_extra_outputs(integration_test_obstacle)
pico_enable_stdio_usb(integration_test_obstacle 1)
pico_enable_stdio_uart(integration_test_obstacle 0)

# INTEG-T3: Integration Test for Obstacle Avoidance
add_executable(INTEG_T3
    integration_test/INTEG_T3.c
)

target_link_libraries(INTEG_T3
    pico_stdlib
    robot_lib
    hardware_gpio
    hardware_pwm
    hardware_timer
    hardware_i2c        # IMU I2C communication

)

pico_enable_stdio_usb(INTEG_T3 1)
pico_enable_stdio_uart(INTEG_T3 0)

pico_add_extra_outputs(INTEG_T3)


# ============================================================================
# INTEGRATION TEST: OBSTACLE DETECTION + TELEMETRY (INTEG-T4)
# ============================================================================

add_executable(integration_test_obstacle
    integration_test/telemetry_obstacle.c
)

target_link_libraries(integration_test_obstacle
    pico_stdlib
    robot_lib
    hardware_gpio
    hardware_pwm
    hardware_timer
    hardware_irq
)

pico_add_extra_outputs(integration_test_obstacle)
pico_enable_stdio_usb(integration_test_obstacle 1)
pico_enable_stdio_uart(integration_test_obstacle 0)
