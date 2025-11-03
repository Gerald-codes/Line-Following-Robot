cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ObstacleAvoidance C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable with all source files
add_executable(Ultrasonic
    main.c
    ultrasonic.c
    servo.c
    timer_manager.c
    obstacle_scanner.c
)

# Add include directories
target_include_directories(Ultrasonic PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Pull in common dependencies - Added hardware_pwm for servo control and m for math
target_link_libraries(Ultrasonic 
    pico_stdlib
    hardware_gpio
    hardware_timer
    hardware_pwm
    m
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(Ultrasonic 1)
pico_enable_stdio_uart(Ultrasonic 0)

# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(Ultrasonic)
