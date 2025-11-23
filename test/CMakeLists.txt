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
# EXISTING TESTS: Motor and Encoder
# ============================================================================

add_executable(test_motor
    unit_test/test_motor.c
    ../src/hardware_drivers/motor.c
)
target_link_libraries(test_motor pico_stdlib hardware_pwm hardware_gpio robot_lib)
pico_add_extra_outputs(test_motor)

add_executable(test_encoder
    unit_test/test_encoder.c
    ../src/hardware_drivers/encoder.c
    ../src/hardware_drivers/motor.c
)
target_link_libraries(test_encoder pico_stdlib hardware_pwm hardware_gpio hardware_irq robot_lib)
pico_add_extra_outputs(test_encoder)

# ============================================================================
# OBSTACLE DETECTION TESTS (OBS-T1 through OBS-T9)
# ============================================================================

# Macro to avoid copy-pasting
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
# Print Configuration
# ============================================================================
message(STATUS "")
message(STATUS "=============================================")
message(STATUS "Robot Test Suite Build Configuration")
message(STATUS "=============================================")
message(STATUS "Build System: FULLY FIXED ✓")
message(STATUS "Linked Modules: robot_lib ✓")
message(STATUS "Tests Enabled: OBS-T1 → OBS-T9 ✓")
message(STATUS "=============================================")
message(STATUS "")
