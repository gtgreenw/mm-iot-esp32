# Sensor unit: HaLow STA + ESP-NOW sender + optional camera stream (Xiao ESP32-S3-Sense)
# camera_stream.c always built (stubs when CONFIG_SENSOR_CAMERA_ENABLE=n)
set(src
    "src/main.c"
    "src/esp_now_send.c"
    "src/camera_stream.c"
    "src/halow_config.c"
    "src/halow_start.c"
    "src/mm_app_common.c"
    "src/mm_app_loadconfig.c"
)

set(inc "include" "src"
    "$ENV{IDF_PATH}/components/lwip/lwip/src/include"
    "$ENV{IDF_PATH}/components/esp_netif/include")

set(priv_reqs driver morselib mm_shims mmipal lwip esp_timer esp_http_server nvs_flash esp_wifi esp_netif esp_event bme68x)
if(CONFIG_SENSOR_CAMERA_ENABLE)
    list(APPEND priv_reqs esp32-camera)
endif()
# When camera disabled, camera_stream.c compiles stubs only (no esp32-camera)

idf_component_register(SRCS ${src}
                       INCLUDE_DIRS ${inc}
                       PRIV_REQUIRES ${priv_reqs})
