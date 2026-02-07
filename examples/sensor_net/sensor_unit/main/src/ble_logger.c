/*
 * BLE scan logger for sensor unit.
 */
#include "ble_logger.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

#if CONFIG_SENSOR_BLE_LOG_ENABLE

static const char *TAG = "ble_logger";

#define BLE_LOG_CACHE_MAX 32
#define BLE_LOG_THROTTLE_MS 5000

typedef struct {
    uint8_t addr[6];
    uint32_t last_log_ms;
    bool used;
} ble_seen_t;

static ble_seen_t s_seen[BLE_LOG_CACHE_MAX];
static ble_logger_stats_t s_stats;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static bool record_seen(const uint8_t addr[6], uint32_t now, bool *is_new)
{
    int free_idx = -1;
    int oldest_idx = 0;
    uint32_t oldest_ms = UINT32_MAX;
    for (int i = 0; i < BLE_LOG_CACHE_MAX; i++) {
        if (!s_seen[i].used) {
            free_idx = i;
            continue;
        }
        if (memcmp(s_seen[i].addr, addr, 6) == 0) {
            *is_new = false;
            if (now - s_seen[i].last_log_ms < BLE_LOG_THROTTLE_MS) {
                return false;
            }
            s_seen[i].last_log_ms = now;
            return true;
        }
        if (s_seen[i].last_log_ms < oldest_ms) {
            oldest_ms = s_seen[i].last_log_ms;
            oldest_idx = i;
        }
    }
    int idx = (free_idx >= 0) ? free_idx : oldest_idx;
    memcpy(s_seen[idx].addr, addr, 6);
    s_seen[idx].last_log_ms = now;
    s_seen[idx].used = true;
    *is_new = true;
    return true;
}

static void log_addr(const uint8_t addr[6], int rssi)
{
    ESP_LOGI(TAG, "BLE device %02X:%02X:%02X:%02X:%02X:%02X RSSI %d dBm",
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], rssi);
}

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(0);
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan = param;
        if (scan->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            uint32_t now = now_ms();
            bool is_new = false;
            bool log_ok = record_seen(scan->scan_rst.bda, now, &is_new);
            memcpy(s_stats.addr, scan->scan_rst.bda, 6);
            s_stats.rssi_dbm = (int8_t)scan->scan_rst.rssi;
            s_stats.has_addr = true;
            if (is_new) {
                s_stats.seen_count++;
            }
            if (log_ok) {
                log_addr(scan->scan_rst.bda, scan->scan_rst.rssi);
            }
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        esp_ble_gap_start_scanning(0);
        break;
    default:
        break;
    }
}

bool ble_logger_start(void)
{
    memset(s_seen, 0, sizeof(s_seen));
    memset(&s_stats, 0, sizeof(s_stats));
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bt_controller_init failed: %s", esp_err_to_name(err));
        return false;
    }
    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bt_controller_enable failed: %s", esp_err_to_name(err));
        return false;
    }
    err = esp_bluedroid_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid_init failed: %s", esp_err_to_name(err));
        return false;
    }
    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid_enable failed: %s", esp_err_to_name(err));
        return false;
    }
    esp_ble_gap_register_callback(gap_cb);

    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_PASSIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,
        .scan_window = 0x30,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
    };
    err = esp_ble_gap_set_scan_params(&scan_params);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gap_set_scan_params failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "BLE scan logger started");
    return true;
}

void ble_logger_get_stats(ble_logger_stats_t *out)
{
    if (!out) return;
    *out = s_stats;
}

#else

bool ble_logger_start(void)
{
    return false;
}

void ble_logger_get_stats(ble_logger_stats_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
}

#endif
