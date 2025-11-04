# CMakeLists.txt with Demo 1 and Demo 2

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(robo_pico C CXX ASM)

pico_sdk_init()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/configuration
    ${CMAKE_CURRENT_SOURCE_DIR}/control_algo
    ${CMAKE_CURRENT_SOURCE_DIR}/hardware_drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/utilities
)

# ============================================================================
# DEMO 1: Basic Motion & Sensing Integration
# ============================================================================

add_executable(demo1
    main_program/demo1_main.c
    hardware_drivers/motor.c
    hardware_drivers/encoder.c
    hardware_drivers/imu.c
    control_algo/pid.c
    control_algo/heading_control.c
    control_algo/filters.c
    utilities/telemetry.c
)

target_link_libraries(demo1
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_i2c
)

pico_enable_stdio_usb(demo1 1)
pico_enable_stdio_uart(demo1 0)
pico_add_extra_outputs(demo1)

# ============================================================================
# DEMO 2: Line Following with Barcode Detection
# ============================================================================

add_executable(demo2
    main_program/demo2_main.c
    hardware_drivers/motor.c
    hardware_drivers/encoder.c
    hardware_drivers/imu.c
    hardware_drivers/ir_sensor.c
    hardware_drivers/barcode.c
    hardware_drivers/calibration.c
    control_algo/pid.c
    control_algo/heading_control.c
    control_algo/line_following.c
    control_algo/filters.c
    utilities/telemetry.c
)

target_link_libraries(demo2
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_adc
)

pico_enable_stdio_usb(demo2 1)
pico_enable_stdio_uart(demo2 0)
pico_add_extra_outputs(demo2)

# ============================================================================
# TEST PROGRAMS
# ============================================================================

add_executable(encoder_test
    main_program/encoder_test_main.c
    hardware_drivers/motor.c
    hardware_drivers/encoder.c
)

target_link_libraries(encoder_test
    pico_stdlib
    hardware_pwm
    hardware_gpio
)

pico_enable_stdio_usb(encoder_test 1)
pico_enable_stdio_uart(encoder_test 0)
pico_add_extra_outputs(encoder_test)

add_executable(motor_test
    main_program/motor_test.c
    hardware_drivers/motor.c
    hardware_drivers/encoder.c
)

target_link_libraries(motor_test
    pico_stdlib
    hardware_pwm
    hardware_gpio
)

pico_enable_stdio_usb(motor_test 1)
pico_enable_stdio_uart(motor_test 0)
pico_add_extra_outputs(motor_test)
