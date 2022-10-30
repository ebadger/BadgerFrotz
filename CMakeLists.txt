cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(badgerfrotz)

pico_sdk_init()

add_subdirectory(./exe)

pico_enable_stdio_usb(badgerfrotz 1)
#pico_enable_stdio_uart(badgerfrotz 0)
