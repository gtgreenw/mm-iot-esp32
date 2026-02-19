# Copyright 2023 Morse Micro
# SPDX-License-Identifier: Apache-2.0
#
# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ENV{MMIOT_ROOT})
    message(FATAL_ERROR "MMIOT_ROOT environment variable not defined. Please set as the path to the framework directory in the MM-IoT-SDK")
endif()

message(STATUS "MMIOT_ROOT: $ENV{MMIOT_ROOT}")

# Workaround to allow us to use the link status callback in LWIP. The ESP-IDF does not current (as
# of v5.1.1) exposed this option from the lwip (esp-lwip) component.
add_compile_definitions(LWIP_NETIF_LINK_CALLBACK=1)
# Increase DNS retry attempts for more resilient lookups.
add_compile_definitions(DNS_MAX_RETRIES=8)
# NAPT is enabled via sdkconfig (CONFIG_LWIP_IP_FORWARD + CONFIG_LWIP_IPV4_NAPT) so lwipopts.h
# sets IP_NAPT; do not add_compile_definitions(IP_NAPT=1) here or it redefines and breaks the build.

if(DEFINED COUNTRY_CODE)
    message (Using country code: ${COUNTRY_CODE})
    add_compile_definitions(COUNTRY_CODE="${COUNTRY_CODE}")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(iperf)

# Override WiFi task stack size (libpp default too small for AP+STA+NAPT; avoid "stack overflow in task wifi")
# --wrap redirects libpp's call to our __wrap_*; -u forces pulling in the object that defines it from main.
target_link_libraries(${CMAKE_PROJECT_NAME}.elf PRIVATE
    "-Wl,--wrap=config_get_wifi_task_stack_size"
    "-Wl,-u,__wrap_config_get_wifi_task_stack_size"
)
