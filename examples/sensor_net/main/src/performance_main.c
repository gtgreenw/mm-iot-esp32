/*
 * Copyright 2023 Morse Micro
 * SPDX-License-Identifier: Apache-2.0
 *
 * Performance mode: app_main that only starts settings page and network
 * (HaLow, WiFi STA, 2.4 GHz AP). No dashboard, ESP-NOW, iperf, sensor gateway.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mm_app_common.h"
#include "nat_router.h"
#include "settings.h"
#include "sdkconfig.h"

static const char *TAG = "perf_main";

static bool setup_halow_or_fallback(void)
{
    if (app_wlan_start_with_timeout(30000)) {
        printf(">> Link up: HaLow connected.\n");
        return true;
    }
    printf(">> Link failed: timeout. Starting 2.4 GHz AP for setup...\n");
    start_2ghz_ap();
    printf(">> Settings: http://192.168.4.1/settings (scan/select HaLow, save & reboot).\n\n");
    return false;
}

void app_main(void)
{
    printf("\nSENSORnet Gateway (performance mode)\n");
    printf("Settings + network only: HaLow, WiFi STA, 2.4 GHz AP.\n\n");
    esp_log_level_set("httpd", ESP_LOG_WARN);
    esp_log_level_set("httpd_uri", ESP_LOG_WARN);
    esp_log_level_set("httpd_txrx", ESP_LOG_WARN);

    settings_init();
    bridge_settings_t s;
    settings_load(&s);
    bool use_wifi_backhaul = (s.backhaul_mode == BACKHAUL_MODE_WIFI_2G);

    if (!use_wifi_backhaul) {
        if (!app_wlan_init()) {
            printf(">> No HaLow module or init failed. Using 2.4 GHz Wi-Fi backhaul.\n");
            start_2ghz_apsta_backhaul();
            printf(">> Settings: http://192.168.4.1/settings\n\n");
        } else {
            if (!setup_halow_or_fallback()) {
                while (true) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
            printf(">> Stabilizing HaLow stack (5 s)...\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
            printf(">> Launching 2.4 GHz AP\n");
            start_2ghz_ap();
            printf(">> Settings: http://192.168.4.1/settings\n\n");
        }
    } else {
        printf(">> Backhaul: 2.4 GHz Wi-Fi (HaLow disabled).\n");
        start_2ghz_apsta_backhaul();
        printf(">> Settings: http://192.168.4.1/settings\n\n");
    }

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
