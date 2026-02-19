/*
 * WiFi scan logger: periodic scan and fill wifi_scan_packet_t for gateway.
 */
#include "wifi_logger.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "wifi_logger";

#define WIFI_SCAN_INTERVAL_MS  (60 * 1000)  /* 1 minute */

static bool s_enabled = false;
static uint64_t s_last_scan_ms = 0;

static int cmp_rssi(const void *a, const void *b)
{
    const wifi_ap_record_t *ap_a = (const wifi_ap_record_t *)a;
    const wifi_ap_record_t *ap_b = (const wifi_ap_record_t *)b;
    return (int)ap_b->rssi - (int)ap_a->rssi;  /* stronger first */
}

void wifi_logger_start(void)
{
    s_enabled = true;
    ESP_LOGI(TAG, "WiFi logging enabled");
}

void wifi_logger_stop(void)
{
    s_enabled = false;
    ESP_LOGI(TAG, "WiFi logging disabled");
}

bool wifi_logger_is_enabled(void)
{
    return s_enabled;
}

int wifi_logger_try_scan_and_fill(wifi_scan_packet_t *pkt)
{
    if (!pkt || !s_enabled) return 0;

    uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
    if (s_last_scan_ms != 0 && (now_ms - s_last_scan_ms) < WIFI_SCAN_INTERVAL_MS) {
        return 0;
    }
    s_last_scan_ms = now_ms;

    wifi_scan_config_t scan_cfg = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = { .active = { .min = 100, .max = 300 } },
    };
    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "scan_start failed: %s", esp_err_to_name(err));
        return 0;
    }

    uint16_t num_ap = 0;
    if (esp_wifi_scan_get_ap_num(&num_ap) != ESP_OK || num_ap == 0) {
        return 0;
    }
    if (num_ap > 16) num_ap = 16;
    wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc((size_t)num_ap * sizeof(wifi_ap_record_t));
    if (!ap_list) return 0;
    if (esp_wifi_scan_get_ap_records(&num_ap, ap_list) != ESP_OK) {
        free(ap_list);
        return 0;
    }
    qsort(ap_list, (size_t)num_ap, sizeof(wifi_ap_record_t), cmp_rssi);

    memset(pkt, 0, sizeof(*pkt));
    pkt->magic = WIFI_SCAN_PACKET_MAGIC;
    pkt->version = WIFI_SCAN_PACKET_VERSION;
    if (esp_wifi_get_mac(WIFI_IF_STA, pkt->src_mac) != ESP_OK) {
        memset(pkt->src_mac, 0, 6);
    }
    pkt->scan_ts_ms = (uint32_t)now_ms;
    uint8_t n = (num_ap > WIFI_SCAN_ENTRIES_MAX) ? (uint8_t)WIFI_SCAN_ENTRIES_MAX : (uint8_t)num_ap;
    pkt->num_entries = n;
    for (uint8_t i = 0; i < n; i++) {
        size_t ssid_len = strlen((const char *)ap_list[i].ssid);
        if (ssid_len >= WIFI_SCAN_SSID_MAX) ssid_len = WIFI_SCAN_SSID_MAX - 1;
        memcpy(pkt->entries[i].ssid, ap_list[i].ssid, ssid_len);
        pkt->entries[i].ssid[ssid_len] = '\0';
        memcpy(pkt->entries[i].bssid, ap_list[i].bssid, 6);
        pkt->entries[i].rssi_dbm = (int8_t)ap_list[i].rssi;
        pkt->entries[i].channel = (uint8_t)ap_list[i].primary;
    }
    free(ap_list);
    return (int)n;
}
