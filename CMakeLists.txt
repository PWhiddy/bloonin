cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from GIT
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_FETCH_FROM_GIT on)

# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(bloon
    bloon.c
    ov5640.c
    ov5640_demo.c
    )

# pull in common dependencies
target_link_libraries(bloon
    pico_stdlib
    hardware_i2c
    hardware_pwm
    hardware_pio
    hardware_dma
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(bloon pico_cyw43_arch_none)
endif()

pico_enable_stdio_usb(bloon 1)
pico_enable_stdio_uart(bloon 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(bloon)
