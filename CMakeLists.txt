# CMakeLists.txt — LINE-FOLLOWING-ROBOT (Raspberry Pi Pico W)

cmake_minimum_required(VERSION 3.13)

# Import Pico SDK via env var (keep this if your current project uses it)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(line_following_robot C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# We’re using the Pico W
set(PICO_BOARD pico_w)

# Init the SDK
pico_sdk_init()

# --------- Folder helpers (match your tree) ----------
set(SRC_MAIN ${CMAKE_CURRENT_SOURCE_DIR}/main_program)
set(SRC_DRV  ${CMAKE_CURRENT_SOURCE_DIR}/hardware_drivers)
set(SRC_UTIL ${CMAKE_CURRENT_SOURCE_DIR}/utilities)
set(SRC_CTRL ${CMAKE_CURRENT_SOURCE_DIR}/control_algo)
set(SRC_CFG  ${CMAKE_CURRENT_SOURCE_DIR}/configuration)

# Drivers (hardware-layer) used across apps
set(DRIVER_SRCS
    ${SRC_DRV}/motor.c
    ${SRC_DRV}/encoder.c
    ${SRC_DRV}/imu.c
)

# Control/algorithm layer (PID, filters, heading control)
set(CONTROL_SRCS
    ${SRC_CTRL}/pid.c
    ${SRC_CTRL}/filters.c
    ${SRC_CTRL}/heading_control.c
)

# Make headers visible to all targets
include_directories(${SRC_MAIN} ${SRC_DRV} ${SRC_UTIL} ${SRC_CTRL} ${SRC_CFG})

# =====================================================
# demo1_main — full app with telemetry (Wi-Fi + MQTT)
# =====================================================
add_executable(demo1_main
    ${SRC_MAIN}/demo1_main.c
    ${SRC_UTIL}/telemetry.c
    ${DRIVER_SRCS}
    ${CONTROL_SRCS}
)

target_link_libraries(demo1_main
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_i2c
    pico_cyw43_arch_lwip_threadsafe_background  # Wi-Fi
    pico_lwip_mqtt                              # MQTT (remove if unused)
)

pico_enable_stdio_usb(demo1_main 1)
pico_enable_stdio_uart(demo1_main 0)
pico_add_extra_outputs(demo1_main)

# ==========================================
# encoder_test — test encoders & basic motor
# ==========================================
add_executable(encoder_test
    ${SRC_MAIN}/encoder_test_main.c
    ${SRC_DRV}/encoder.c
    ${SRC_DRV}/motor.c
    # add ${SRC_CTRL}/pid.c here if this test needs PID
)

target_link_libraries(encoder_test
    pico_stdlib
    hardware_gpio
    hardware_pwm
)

pico_enable_stdio_usb(encoder_test 1)
pico_enable_stdio_uart(encoder_test 0)
pico_add_extra_outputs(encoder_test)

# =========================
# motor_test — spin motors
# =========================
add_executable(motor_test
    ${SRC_MAIN}/motor_test.c
    ${SRC_DRV}/motor.c
    ${SRC_DRV}/encoder.c   # keep if you read speed; remove if not needed
)

target_link_libraries(motor_test
    pico_stdlib
    hardware_gpio
    hardware_pwm
)

pico_enable_stdio_usb(motor_test 1)
pico_enable_stdio_uart(motor_test 0)
pico_add_extra_outputs(motor_test)
