# Camera sensor unit: XIAO ESP32-S3-Sense — HaLow STA + MJPEG /stream + minimal ESP-NOW (no BME/motion)
set(src
    "src/main.c"
    "src/camera_unit_settings.c"
    "src/esp_now_send_camera.c"
    "src/camera_stream.c"
    "src/mic_stream.c"
    "src/halow_config.c"
    "src/halow_start.c"
    "src/mm_app_common.c"
    "src/mm_app_loadconfig.c"
)

set(inc "include" "src"
    "$ENV{IDF_PATH}/components/lwip/include"
    "$ENV{IDF_PATH}/components/lwip/lwip/src/include"
    "$ENV{IDF_PATH}/components/esp_netif/include")

set(priv_reqs driver halow_mesh morselib mm_shims mmipal lwip esp_timer esp_http_server nvs_flash esp_wifi esp_netif esp_event esp32-camera)

idf_component_register(SRCS ${src}
                      INCLUDE_DIRS ${inc}
                      PRIV_REQUIRES ${priv_reqs})
