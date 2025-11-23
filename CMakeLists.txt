# CMakeLists.txt - Integrated Robot System

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

/* Pico SDK Setup */
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

/* Include Directories */
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control_algo
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utilities
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utilities
    ${CMAKE_CURRENT_SOURCE_DIR}/include/control_algo
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hardware_drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/configuration
)

/* Main Executable - Integrated Robot System */
add_executable(main
    src/main.c
    
    /* Hardware drivers */
    src/hardware_drivers/motor.c
    src/hardware_drivers/encoder.c
    src/hardware_drivers/imu.c
    src/hardware_drivers/ir_sensor.c
    src/hardware_drivers/ultrasonic.c
    src/hardware_drivers/servo.c
    src/hardware_drivers/calibration.c
    src/hardware_drivers/barcode_scanner.c
    
    /* Control algorithms */
    src/control_algo/pid.c
    src/control_algo/heading_control.c
    src/control_algo/line_following.c
    src/control_algo/obstacle_scanner.c
    src/control_algo/avoidance_maneuver.c
    src/control_algo/obstacle_control.c
    src/control_algo/barcode_control.c
    
    /* Utilities */
    src/utilities/timer_manager.c
    src/utilities/wifi.c
    src/utilities/mqtt_client.c
    src/utilities/telemetry.c
)

target_link_libraries(main
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_adc
    hardware_i2c
    hardware_irq
    pico_cyw43_arch_lwip_threadsafe_background
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
pico_add_extra_outputs(main)

/* Print Build Information */
message(STATUS "")
message(STATUS "Robot Project Build Configuration")
message(STATUS "-----------------------------------------------------------")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Board: ${PICO_BOARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "")
message(STATUS "Executables:")
message(STATUS "  • main.uf2 - Integrated robot system")
message(STATUS "")
message(STATUS "-----------------------------------------------------------")
message(STATUS "")
