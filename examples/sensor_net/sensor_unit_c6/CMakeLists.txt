# ESP32-C6 sensor unit: ESP-NOW only (no HaLow). Uses local bme68x only.
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "components" "../components/ds18b20")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sensor_unit_c6)
