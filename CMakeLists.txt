# CMakeLists.txt for Robotic Car Project with MQTT Telemetry
# Pico W version with WiFi and MQTT support

cmake_minimum_required(VERSION 3.13)

# Include Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(robotic_car C CXX ASM)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# ============================================================================
# DEMO 1: Basic Motion & Sensing Integration with MQTT Telemetry
# ============================================================================
add_executable(demo1_main
    demo1_main.c
    telemetry.c
    motor.c
    encoder.c
    imu.c
    pid.c
    heading_control.c
    filters.c
)

target_link_libraries(demo1_main
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background  # WiFi with background polling
    pico_lwip_mqtt                               # MQTT support via lwIP
    hardware_pwm
    hardware_gpio
    hardware_i2c                                 # For IMU (if using real hardware)
)

# Enable USB output, disable UART
pico_enable_stdio_usb(demo1_main 1)
pico_enable_stdio_uart(demo1_main 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(demo1_main)

# ============================================================================
# ENCODER TEST
# ============================================================================
add_executable(encoder_test
    encoder_test_main.c
    encoder.c
    motor.c
)

target_link_libraries(encoder_test
    pico_stdlib
    hardware_pwm
    hardware_gpio
)

pico_enable_stdio_usb(encoder_test 1)
pico_enable_stdio_uart(encoder_test 0)
pico_add_extra_outputs(encoder_test)

# ============================================================================
# MOTOR TEST
# ============================================================================
add_executable(motor_test
    motor_test.c
    motor.c
    encoder.c
)

target_link_libraries(motor_test
    pico_stdlib
    hardware_pwm
    hardware_gpio
)

pico_enable_stdio_usb(motor_test 1)
pico_enable_stdio_uart(motor_test 0)
pico_add_extra_outputs(motor_test)

