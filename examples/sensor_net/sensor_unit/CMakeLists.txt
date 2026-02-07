# Sensor unit for ESP-Motion (mm-iot-esp32): HaLow STA + ESP-NOW + optional camera (Xiao ESP32-S3-Sense)
# Build: idf.py set-target esp32s3 && idf.py build
# Flash to Xiao ESP32-S3-Sense. Use same HaLow SSID/password as gateway.

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ENV{MMIOT_ROOT})
    message(FATAL_ERROR "MMIOT_ROOT not set. Set it to the path of mm-iot-esp32 (e.g. export MMIOT_ROOT=~/mm-iot-esp32)")
endif()

message(STATUS "MMIOT_ROOT: $ENV{MMIOT_ROOT}")

add_compile_definitions(LWIP_NETIF_LINK_CALLBACK=1)
add_compile_options(-Wno-error=unused-variable -Wno-unused-variable)

if(DEFINED COUNTRY_CODE)
    add_compile_definitions(COUNTRY_CODE="${COUNTRY_CODE}")
endif()

set(EXTRA_COMPONENT_DIRS "../components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sensor_unit)
