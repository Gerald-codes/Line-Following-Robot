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
    ../src/control_algo/pid.c
    
    ../src/hardware_drivers/ultrasonic.c
    ../src/hardware_drivers/servo.c
    ../src/hardware_drivers/motor.c
    ../src/hardware_drivers/encoder.c
    
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
    pico_cyw43_arch_lwip_threadsafe_background
    cyw43_driver
)

# ============================================================================
# MOTOR TESTS (MOTOR-T1 through MOTOR-T7)
# ============================================================================

# TEST: Motor Comprehensive (T1-T5)
# ----------------------------------------------------------------------------
add_executable(test_motor_comprehensive
    unit_test/MOTOR_T1-5.c
    ../src/hardware_drivers/motor.c
    ../src/hardware_drivers/encoder.c
    ../src/control_algo/pid.c
)

target_link_libraries(test_motor_comprehensive
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_irq
)

pico_add_extra_outputs(test_motor_comprehensive)
pico_enable_stdio_usb(test_motor_comprehensive 1)
pico_enable_stdio_uart(test_motor_comprehensive 0)

# TEST: Motor Long Duration (T6-T7)
# ----------------------------------------------------------------------------
add_executable(test_motor_long
    unit_test/MOTOR_T6-7.c
    ../src/hardware_drivers/motor.c
    ../src/hardware_drivers/encoder.c
    ../src/control_algo/pid.c
)

target_link_libraries(test_motor_long
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_irq
)

pico_add_extra_outputs(test_motor_long)
pico_enable_stdio_usb(test_motor_long 1)
pico_enable_stdio_uart(test_motor_long 0)

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
# Tests the complete IMU-Motor closed-loop control system
# Validates heading maintenance using PID control with IMU feedback
#
# Test Scenarios:
#   INT-T1: IMU initialization and calibration
#   INT-T2: Heading error calculation
#   INT-T3: Motor correction response
#   INT-T4: Heading stability over time (10 seconds)
#   INT-T5: Recovery from disturbances (3 cycles)
#
# Hardware Requirements:
#   - Motors: M1A, M1B (left), M2A, M2B (right)
#   - Encoders: GP2/GP3 (left), GP4/GP5 (right)
#   - IMU: GY-511 LSM303DLHC on I2C (GP16=SDA, GP17=SCL)
#   - Button: GP20 (calibration trigger)
# ----------------------------------------------------------------------------

add_executable(integration_test_heading_keeping
    # Main test file
    integration_test/heading_keeping.c
    
    # Hardware drivers (core dependencies)
    ../src/hardware_drivers/motor.c
    ../src/hardware_drivers/encoder.c
    ../src/hardware_drivers/imu.c
     ../src/hardware_drivers/calibration.c
     
    # Control algorithms
    ../src/control_algo/pid.c
    ../src/control_algo/heading_control.c
    
)

target_link_libraries(integration_test_heading_keeping
    # Pico SDK core
    pico_stdlib
    
    # Hardware interfaces
    hardware_pwm        # Motor PWM control
    hardware_gpio       # GPIO pins
    hardware_irq        # Encoder interrupts
    hardware_i2c        # IMU I2C communication
)

# Enable USB serial for test output
pico_enable_stdio_usb(integration_test_heading_keeping 1)
pico_enable_stdio_uart(integration_test_heading_keeping 0)

# Generate UF2 file
pico_add_extra_outputs(integration_test_heading_keeping)

# ============================================================================
# Print Configuration
# ============================================================================
message(STATUS "")
message(STATUS "╔════════════════════════════════════════════════════════════╗")
message(STATUS "║          Robot Test Suite Build Configuration             ║")
message(STATUS "╚════════════════════════════════════════════════════════════╝")
message(STATUS "Board Type:         ${PICO_BOARD}")
message(STATUS "C Standard:         C${CMAKE_C_STANDARD}")
message(STATUS "Build System:       FULLY CONFIGURED ✓")
message(STATUS "Shared Library:     robot_lib ✓")
message(STATUS "────────────────────────────────────────────────────────────")
message(STATUS "Motor Tests:        MOTOR-T1 to T7 (2 executables) ✓")
message(STATUS "Obstacle Tests:     OBS-T1 to OBS-T9 (9 executables) ✓")
message(STATUS "Integration Tests:  Heading Keeping (1 executable) ✓")
message(STATUS "────────────────────────────────────────────────────────────")
message(STATUS "Test Coverage:")
message(STATUS "  • Unit Tests:         11 test executables")
message(STATUS "  • Integration Tests:  1 test executable")
message(STATUS "  • Total:              12 test executables")
message(STATUS "╚════════════════════════════════════════════════════════════╝")
message(STATUS "")