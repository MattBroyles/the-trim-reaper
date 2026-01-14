cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(ibus_pwm_pico2 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(ibus_pwm_pico2
    main.cpp
)

target_link_libraries(ibus_pwm_pico2
    pico_stdlib
    hardware_uart
    hardware_pwm
    hardware_clocks
)

pico_enable_stdio_usb(ibus_pwm_pico2 1)
pico_enable_stdio_uart(ibus_pwm_pico2 0)

pico_add_extra_outputs(ibus_pwm_pico2)

