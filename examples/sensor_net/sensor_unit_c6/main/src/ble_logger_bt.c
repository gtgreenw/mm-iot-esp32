/*
 * BLE scan logger for sensor unit (ESP32-C6) using NimBLE.
 * Scans for CONFIG_SENSOR_BLE_SCAN_DURATION_SEC (default 2 s), then waits until
 * CONFIG_SENSOR_BLE_SCAN_PERIOD_SEC (default 20 s) before starting the next scan.
 * Only compiled when CONFIG_SENSOR_BLE_LOG_ENABLE is y; component has PRIV_REQUIRES bt.
 */
#include "ble_logger.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

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
static bool s_enabled = false;
/* True only after nimble_port_init() + host task started; guards stop from touching uninitialized stack. */
static bool s_started = false;

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

static void start_scan(void);

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        const struct ble_gap_disc_desc *d = &event->disc;
        uint32_t now = now_ms();
        bool is_new = false;
        bool log_ok = record_seen(d->addr.val, now, &is_new);
        memcpy(s_stats.addr, d->addr.val, 6);
        s_stats.rssi_dbm = d->rssi;
        s_stats.has_addr = true;
        if (is_new) {
            s_stats.seen_count++;
        }
        if (log_ok) {
            log_addr(d->addr.val, d->rssi);
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE: {
        /* Scan ran for CONFIG_SENSOR_BLE_SCAN_DURATION_SEC; wait remainder of period before next scan. */
        int period_sec = CONFIG_SENSOR_BLE_SCAN_PERIOD_SEC;
        int duration_sec = CONFIG_SENSOR_BLE_SCAN_DURATION_SEC;
        int delay_sec = period_sec - duration_sec;
        if (delay_sec > 0) {
            vTaskDelay(pdMS_TO_TICKS((uint32_t)delay_sec * 1000));
        }
        start_scan();
        return 0;
    }
    default:
        return 0;
    }
}

static void start_scan(void)
{
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }
    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.passive = 1;
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    /* Duration in 10 ms units; scan for CONFIG_SENSOR_BLE_SCAN_DURATION_SEC then stop. */
    uint32_t duration_ticks = (uint32_t)CONFIG_SENSOR_BLE_SCAN_DURATION_SEC * 100;
    rc = ble_gap_disc(own_addr_type, duration_ticks, &disc_params, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    }
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: %d", rc);
        return;
    }
    start_scan();
}

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

bool ble_logger_start(void)
{
    s_enabled = true;
    memset(s_seen, 0, sizeof(s_seen));
    memset(&s_stats, 0, sizeof(s_stats));

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return false;
    }

    ble_hs_cfg.sync_cb = on_sync;
    ble_svc_gap_init();

    nimble_port_freertos_init(host_task);
    s_started = true;
    ESP_LOGI(TAG, "BLE scan started: %d s every %d s (NimBLE)",
             (int)CONFIG_SENSOR_BLE_SCAN_DURATION_SEC, (int)CONFIG_SENSOR_BLE_SCAN_PERIOD_SEC);
    return true;
}

void ble_logger_stop(void)
{
    s_enabled = false;
    if (!s_started) {
        return;  /* BLE stack was never started (e.g. NVS had BLE log off); avoid touching NimBLE. */
    }
    s_started = false;
    (void)ble_gap_disc_cancel();
    if (nimble_port_stop() == 0) {
        nimble_port_deinit();
        ESP_LOGI(TAG, "BLE scan stopped, stack deinit (for deep sleep)");
    }
}

bool ble_logger_is_enabled(void)
{
    return s_enabled;
}

void ble_logger_get_stats(ble_logger_stats_t *out)
{
    if (!out) return;
    *out = s_stats;
}
