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
# MOTOR TESTS (MOTOR-T1 through MOTOR-T7)
# ============================================================================

# TEST: Motor Comprehensive (T1-T5)
# # ----------------------------------------------------------------------------
# add_executable(test_motor_comprehensive
#     unit_test/MOTOR_T1-5.c
#     ../src/hardware_drivers/motor.c
#     ../src/hardware_drivers/encoder.c
#     ../src/control_algo/pid.c
# )

# target_link_libraries(test_motor_comprehensive
#     pico_stdlib
#     hardware_pwm
#     hardware_gpio
#     hardware_irq
# )

# pico_add_extra_outputs(test_motor_comprehensive)
# pico_enable_stdio_usb(test_motor_comprehensive 1)
# pico_enable_stdio_uart(test_motor_comprehensive 0)

# # TEST: Motor Long Duration (T6-T7)
# # ----------------------------------------------------------------------------
# add_executable(test_motor_long
#     unit_test/MOTOR_T6-7.c
#     ../src/hardware_drivers/motor.c
#     ../src/hardware_drivers/encoder.c
#     ../src/control_algo/pid.c
# )

# target_link_libraries(test_motor_long
#     pico_stdlib
#     hardware_pwm
#     hardware_gpio
#     hardware_irq
# )

# pico_add_extra_outputs(test_motor_long)
# pico_enable_stdio_usb(test_motor_long 1)
# pico_enable_stdio_uart(test_motor_long 0)

# ============================================================================
# LEGACY MOTOR TESTS (Keep for compatibility)
# ============================================================================

# add_executable(test_motor
#     unit_test/test_motor.c
#     ../src/hardware_drivers/motor.c
# )

# target_link_libraries(test_motor
#     pico_stdlib
#     hardware_pwm
#     hardware_gpio
#     robot_lib
# )

# pico_add_extra_outputs(test_motor)
# pico_enable_stdio_usb(test_motor 1)
# pico_enable_stdio_uart(test_motor 0)

# ============================================================================
# ENCODER TESTS
# ============================================================================

# add_executable(test_encoder
#     unit_test/test_encoder.c
#     ../src/hardware_drivers/encoder.c
#     ../src/control_algo/pid.c
# )

# target_link_libraries(test_encoder
#     pico_stdlib
#     hardware_pwm
#     hardware_gpio
#     hardware_irq
#     robot_lib
# )

# pico_add_extra_outputs(test_encoder)
# pico_enable_stdio_usb(test_encoder 1)
# pico_enable_stdio_uart(test_encoder 0)

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

# ============================================================================
# INTEGRATION TESTS
# ============================================================================

# Integration Test: Heading Keeping Mode
# ----------------------------------------------------------------------------
# add_executable(integration_test_heading_keeping
#     integration_test/integration_test_heading_keeping.c
#     ../src/hardware_drivers/motor.c
#     ../src/hardware_drivers/encoder.c
#     ../src/hardware_drivers/imu.c
#     ../src/control_algo/pid.c
#     ../src/control_algo/heading_control.c
#     ../src/utilities/calibration.c
# )

# target_link_libraries(integration_test_heading_keeping
#     pico_stdlib
#     hardware_pwm
#     hardware_gpio
#     hardware_irq
#     hardware_i2c
#     robot_lib
# )

# pico_add_extra_outputs(integration_test_heading_keeping)
# pico_enable_stdio_usb(integration_test_heading_keeping 1)
# pico_enable_stdio_uart(integration_test_heading_keeping 0)



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
# Print Configuration
# ============================================================================
message(STATUS "")
message(STATUS "=============================================")
message(STATUS "Robot Test Suite Build Configuration")
message(STATUS "=============================================")
message(STATUS "Board Type:       ${PICO_BOARD}")
message(STATUS "C Standard:       C${CMAKE_C_STANDARD}")
message(STATUS "Build System:     FULLY CONFIGURED âœ“")
message(STATUS "Linked Modules:   robot_lib âœ“")
message(STATUS "---------------------------------------------")
message(STATUS "Motor Tests:      MOTOR-T1 to T7 âœ“")
message(STATUS "Obstacle Tests:   OBS-T1 to OBS-T9 âœ“")
message(STATUS "Integration Test: Heading Keeping âœ“")
message(STATUS "=============================================")
message(STATUS "")