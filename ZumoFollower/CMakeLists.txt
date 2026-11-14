cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pololu_zumo_2040_robot CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(ZumoFollower C CXX ASM)

pico_sdk_init()

# Libreria de Pololu (motores, botones). Misma ruta relativa que usa EncoderCount.
add_subdirectory(../zumo-2040-robot/c/pololu_zumo_2040_robot pololu_zumo_2040_robot)

set(WAYPOINTS_DIR ${CMAKE_CURRENT_LIST_DIR}/../Waypoints)

add_executable(ZumoFollower
    main.c
    mcp2515.c
    ${WAYPOINTS_DIR}/waypoints.c
    ${WAYPOINTS_DIR}/path.c
    ${WAYPOINTS_DIR}/camera_can.c
)

pico_set_program_name(ZumoFollower "ZumoFollower")
pico_set_program_version(ZumoFollower "0.1")

pico_enable_stdio_uart(ZumoFollower 0)
pico_enable_stdio_usb(ZumoFollower 1)

target_include_directories(ZumoFollower PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${WAYPOINTS_DIR}
)

# Prueba con las ruedas en el aire (no mueve motores): cmake -DDRY_RUN=ON
option(DRY_RUN "No mover motores, solo imprimir" OFF)
if(DRY_RUN)
    target_compile_definitions(ZumoFollower PRIVATE DRY_RUN=1)
endif()

target_link_libraries(ZumoFollower
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_pwm
    pololu_zumo_2040_robot
)

pico_add_extra_outputs(ZumoFollower)
