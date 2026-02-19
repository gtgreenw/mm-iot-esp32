/*
 * ESP32-S3 gateway: 2.4 GHz WiFi STA only, no HaLow.
 * Connects to a 2.4 GHz network; dashboard is served on the IP from the router.
 * Dashboard URL is printed to the console when STA gets an IP.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nat_router.h"
#include "settings.h"
#include "time_sync.h"
#include "esp_now_rcv.h"
#if CONFIG_SENSOR_NET_HOMEKIT
#include "sensor_homekit.h"
#endif

static const char *TAG = "gateway_2g";

void app_main(void)
{
    /* Printf so something appears even when CONFIG_LOG_DEFAULT_LEVEL_NONE */
    printf("\nGateway 2G starting...\n");
    ESP_LOGI(TAG, "Gateway 2G-only starting (no HaLow, STA only)");
    esp_log_level_set("httpd", ESP_LOG_WARN);
    esp_log_level_set("httpd_uri", ESP_LOG_WARN);
    esp_log_level_set("httpd_txrx", ESP_LOG_WARN);

    settings_init();
    time_sync_init();

    /* STA only: connect to 2.4 GHz WiFi; HTTP server and dashboard on STA IP. */
    start_2ghz_sta_only();

    /* ESP-NOW gateway on 2.4 GHz ch6 (sensor_net compatible). */
    esp_now_rcv_init();
    /* Start ESP-NOW from main task after 500 ms so esp_now_init() runs in app_main context. */
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_now_rcv_start_deferred();

#if CONFIG_SENSOR_NET_HOMEKIT
    sensor_homekit_start();
#endif

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
