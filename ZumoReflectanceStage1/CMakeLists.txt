cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico CACHE STRING "RP2040 board")
if(NOT PICO_SDK_PATH)
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()
if(NOT PICO_SDK_PATH)
    message(FATAL_ERROR "Set PICO_SDK_PATH to your installed Pico SDK")
endif()
include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
project(ZumoReflectanceStage1 C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()
add_executable(zumo_reflectance_stage1 main.c)
target_link_libraries(zumo_reflectance_stage1
    pico_stdlib
    hardware_gpio
    hardware_sync
)
pico_enable_stdio_usb(zumo_reflectance_stage1 1)
pico_enable_stdio_uart(zumo_reflectance_stage1 0)
pico_add_extra_outputs(zumo_reflectance_stage1)
