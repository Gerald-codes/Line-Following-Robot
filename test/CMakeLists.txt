# Test Suite CMakeLists.txt
# CMakeLists.txt - Integrated Robot System

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
# Pico SDK Setup
# ============================================================================
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
# ============================================================================
# Include Directories
# ============================================================================
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

# TEST: Motor Comprehensive (T1-T5)
# ============================================================================
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

# ============================================================================
# TEST: Motor Long Duration (T6-T7)
# ============================================================================
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