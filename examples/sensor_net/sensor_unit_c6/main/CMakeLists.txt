set(src
    "src/main.c"
    "src/esp_now_send.c"
    "src/ble_logger.c"
)

set(inc "include" "src")

set(priv_reqs driver esp_timer nvs_flash esp_wifi esp_netif esp_event bme68x esp_adc ds18b20)
if(CONFIG_SENSOR_BLE_LOG_ENABLE)
    list(APPEND priv_reqs bt)
endif()

idf_component_register(SRCS ${src}
                       INCLUDE_DIRS ${inc}
                       PRIV_REQUIRES ${priv_reqs})
