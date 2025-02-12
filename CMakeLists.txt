cmake_minimum_required(VERSION 3.13...3.27)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
set(PICO_SDK_PATH "~/pico-sdk")
include(pico_sdk_import.cmake)SSS

project(serial_example)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(serial_example
    main.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(serial_example pico_stdlib hardware_adc)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(serial_example)

# enable usb output, disable uart output
pico_enable_stdio_usb(serial_example 1)
pico_enable_stdio_uart(serial_example 0)
