# Sensor unit for ESP-Motion (mm-iot-esp32): ESP-NOW only (Xiao ESP32-S3-Sense)
# Build: idf.py set-target esp32s3 && idf.py build
# No HaLow, no HTTP; sensor data via ESP-NOW to gateway.

cmake_minimum_required(VERSION 3.16)

add_compile_options(-Wno-error=unused-variable -Wno-unused-variable)

if(DEFINED COUNTRY_CODE)
    add_compile_definitions(COUNTRY_CODE="${COUNTRY_CODE}")
endif()

# Only bme68x and ld2410 (no halow_mesh / morselib / mmipal)
set(EXTRA_COMPONENT_DIRS "../components/bme68x" "../components/ld2410" "../components/ds18b20")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sensor_unit)
