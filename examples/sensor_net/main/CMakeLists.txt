# main/CMakeLists.txt

set(src
    "src/wifi_task_stack_override.c"
    "src/mm_app_loadconfig.c"
    "src/mm_app_common.c"
    "src/iperf.c"
    "src/nat_router.c"
    "src/settings.c"
    "src/web_config.c"
    "src/dns_forwarder.c"
    "src/esp_now_rcv.c"
    "src/sensor_gateway_http.c"
    "src/dashboard_embedded.c"
    "src/time_sync.c"
)

# We must explicitly add these paths to find internal ESP-IDF netif headers
# Add these specific paths to the INCLUDE_DIRS section
set(inc
    "src"
    "$ENV{IDF_PATH}/components/lwip/lwip/src/include"
    "$ENV{IDF_PATH}/components/esp_netif/include"
)

idf_component_register(SRCS ${src}
                       INCLUDE_DIRS ${inc}
                       PRIV_REQUIRES 
                            driver 
                            morselib 
                            mm_shims 
                            mmipal 
                            mmiperf 
                            lwip 
                            esp_event 
                            wpa_supplicant 
                            esp_wifi 
                            esp_timer
                            nvs_flash 
                            esp_http_server
                            )
