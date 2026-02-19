# Sensor unit: ESP-NOW only (no HaLow, no HTTP).
# BLE: NimBLE only (like C6); use ble_logger_bt.c when enabled, stub ble_logger.c when disabled.
set(src
    "src/main.c"
    "src/esp_now_send.c"
    "src/wifi_logger.c"
)
if(CONFIG_SENSOR_BLE_LOG_ENABLE)
    list(APPEND src "src/ble_logger_bt.c")
else()
    list(APPEND src "src/ble_logger.c")
endif()

set(inc "include" "src")
set(priv_incs "")

# ld2410 required when CONFIG_SENSOR_MOTION_MMWAVE (header needed for #if blocks in esp_now_send.c)
# esp_adc required when CONFIG_SENSOR_MOISTURE_ENABLE (adc_oneshot.h); always add so include path exists when option is y
set(priv_reqs driver esp_timer nvs_flash esp_wifi esp_netif esp_event bme68x ld2410 esp_adc ds18b20)
if(CONFIG_SENSOR_BLE_LOG_ENABLE)
    list(APPEND priv_reqs bt)
endif()

idf_component_register(SRCS ${src}
                       INCLUDE_DIRS ${inc}
                       PRIV_INCLUDE_DIRS ${priv_incs}
                       PRIV_REQUIRES ${priv_reqs})

# NimBLE include paths (like C6); PRIV_REQUIRES bt often does not propagate them to main.
if(CONFIG_SENSOR_BLE_LOG_ENABLE)
    idf_build_get_property(idf_path IDF_PATH)
    set(bt_inc "${idf_path}/components/bt")
    target_include_directories(${COMPONENT_LIB} PRIVATE
        "${bt_inc}/host/nimble/port/include"
        "${bt_inc}/porting/nimble/include"
        "${bt_inc}/porting/npl/freertos/include"
        "${bt_inc}/host/nimble/nimble/porting/nimble/include"
        "${bt_inc}/host/nimble/nimble/porting/npl/freertos/include"
        "${bt_inc}/host/nimble/nimble/nimble/include"
        "${bt_inc}/host/nimble/nimble/nimble/host/include"
        "${bt_inc}/host/nimble/nimble/nimble/host/util/include"
        "${bt_inc}/host/nimble/nimble/nimble/host/services/gap/include"
        "${bt_inc}/host/nimble/nimble/nimble/transport/include"
    )
endif()
