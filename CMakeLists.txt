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

option(RUN_OV5640_DEMO "Run the OV5640 ASCII camera demo from blink.c" OFF)

add_executable(blink
    blink.c
    ov5640.c
    ov5640_demo.c
    )

# pull in common dependencies
target_link_libraries(blink
    pico_stdlib
    hardware_i2c
    hardware_pwm
    hardware_pio
    hardware_dma
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(blink pico_cyw43_arch_none)
endif()

if (RUN_OV5640_DEMO)
    target_compile_definitions(blink PRIVATE RUN_OV5640_DEMO=1)
endif()

pico_enable_stdio_usb(blink 1)
pico_enable_stdio_uart(blink 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(blink)
