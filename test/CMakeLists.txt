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

# ============================================================================
# TEST: Motor Driver
# ============================================================================
add_executable(test_motor
    unit_test/test_motor.c
    ../src/hardware_drivers/motor.c
)

target_link_libraries(test_motor
    pico_stdlib
    hardware_pwm
    hardware_gpio
)

pico_add_extra_outputs(test_motor)
pico_enable_stdio_usb(test_motor 1)
pico_enable_stdio_uart(test_motor 0)

# ============================================================================
# TEST: Encoder Driver
# ============================================================================
add_executable(test_encoder
    unit_test/test_encoder.c
    ../src/hardware_drivers/encoder.c
    ../src/hardware_drivers/motor.c
)

target_link_libraries(test_encoder
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_irq
)

pico_add_extra_outputs(test_encoder)
pico_enable_stdio_usb(test_encoder 1)
pico_enable_stdio_uart(test_encoder 0)

# ============================================================================
# Print build information
# ============================================================================
message(STATUS "")
message(STATUS "===================================")
message(STATUS "Robot Test Suite Build Configuration")
message(STATUS "===================================")
message(STATUS "Tests to build:")
message(STATUS "  - test_motor")
message(STATUS "  - test_encoder")
message(STATUS "===================================")
message(STATUS "")