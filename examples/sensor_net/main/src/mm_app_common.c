/*
 * Copyright 2023 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "mmosal.h"
#include "mmhal.h"
#include "mmwlan.h"
#include "mmipal.h"
#include "mm_app_common.h"
#include "mm_app_loadconfig.h"
#include "nat_router.h"
#include "time_sync.h"
#include "esp_err.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"

/** Maximum number of DNS servers to attempt to retrieve from config store. */
#ifndef DNS_MAX_SERVERS
#define DNS_MAX_SERVERS                 2
#endif

/** Delay before attempting HaLow reconnection after link down (ms). */
#define HALOW_RECONNECT_DELAY_MS        5000
#define HALOW_RECONNECT_TASK_STACK      (4096)
#define HALOW_RECONNECT_TASK_PRIO       (tskIDLE_PRIORITY + 2)

/** Binary semaphore used to start user_main() once the link comes up. */
static struct mmosal_semb *link_established = NULL;

/** True after first link-up (so we only block app_wlan_start once). */
static volatile bool initial_connect_done = false;

/** True when link is up. */
static volatile bool link_up = false;

/** True while we are in the process of reconnecting (sta_disable + sta_enable). */
static volatile bool reconnecting = false;

/** One-shot timer to attempt HaLow reconnection after link down. */
static TimerHandle_t reconnect_timer = NULL;
static TaskHandle_t reconnect_task = NULL;
static TaskHandle_t bw_scan_task = NULL;
static struct mmosal_semb *bw_scan_done = NULL;
static uint8_t halow_op_bw_mhz = 0;
static uint8_t halow_bssid[6] = {0};
static bool halow_bssid_valid = false;
static int16_t halow_bw_best_rssi = -127;

static void halow_scan_rx_cb(const struct mmwlan_scan_result *result, void *arg)
{
    (void)arg;
    if (!result || !result->bssid)
        return;
    bool match = false;
    if (halow_bssid_valid) {
        match = (memcmp(halow_bssid, result->bssid, sizeof(halow_bssid)) == 0);
    }
    if (!match) {
        return;
    }
    if (result->op_bw_mhz && result->rssi >= halow_bw_best_rssi) {
        halow_bw_best_rssi = result->rssi;
        halow_op_bw_mhz = result->op_bw_mhz;
    }
}

static void halow_scan_complete_cb(enum mmwlan_scan_state state, void *arg)
{
    (void)state;
    (void)arg;
    if (bw_scan_done) {
        mmosal_semb_give(bw_scan_done);
    }
}

static void halow_bw_scan_task(void *arg)
{
    (void)arg;
    if (bw_scan_done == NULL) {
        bw_scan_done = mmosal_semb_create("bw_scan_done");
    }
    halow_bw_best_rssi = -127;
    halow_op_bw_mhz = 0;
    halow_bssid_valid = (mmwlan_get_bssid(halow_bssid) == MMWLAN_SUCCESS);

    struct mmwlan_scan_req scan_req = MMWLAN_SCAN_REQ_INIT;
    scan_req.scan_rx_cb = halow_scan_rx_cb;
    scan_req.scan_complete_cb = halow_scan_complete_cb;
    if (mmwlan_scan_request(&scan_req) == MMWLAN_SUCCESS) {
        mmosal_semb_wait(bw_scan_done, 4000);
    }
    bw_scan_task = NULL;
    vTaskDelete(NULL);
}

static void app_wlan_request_bw_scan(void)
{
    if (bw_scan_task != NULL) {
        return;
    }
    xTaskCreate(halow_bw_scan_task, "halow_bw", 3072, NULL, tskIDLE_PRIORITY + 1, &bw_scan_task);
}

/**
 * WLAN station status callback, invoked when WLAN STA state changes.
 *
 * @param sta_state  The new STA state.
 */
static void sta_status_callback(enum mmwlan_sta_state sta_state)
{
    switch (sta_state)
    {
    case MMWLAN_STA_DISABLED:
        printf("WLAN STA disabled\n");
        break;

    case MMWLAN_STA_CONNECTING:
        printf("WLAN STA connecting\n");
        break;

    case MMWLAN_STA_CONNECTED:
        printf("WLAN STA connected\n");
        break;
    }
}

/**
 * Attempt HaLow reconnection: disable STA then re-enable with current config.
 * Called from the reconnect timer (do not call from link_status_callback).
 */
static void do_halow_reconnect(void)
{
    enum mmwlan_status status;
    struct mmwlan_sta_args sta_args = MMWLAN_STA_ARGS_INIT;

    reconnecting = true;

    status = mmwlan_sta_disable();
    if (status != MMWLAN_SUCCESS)
    {
        printf("HaLow reconnect: sta_disable failed (%d), will retry later\n", (int)status);
        reconnecting = false;
        return;
    }

    load_mmwlan_sta_args(&sta_args);
    load_mmwlan_settings();

    printf("HaLow reconnect: attempting to reconnect to %s ...\n", sta_args.ssid);
    status = mmwlan_sta_enable(&sta_args, sta_status_callback);
    if (status != MMWLAN_SUCCESS)
    {
        printf("HaLow reconnect: sta_enable failed (%d), will retry later\n", (int)status);
        reconnecting = false;
        return;
    }

    /* reconnecting is cleared when link_status_callback gets MMIPAL_LINK_UP */
}

static void reconnect_timer_cb(TimerHandle_t t)
{
    (void)t;
    if (reconnect_task != NULL) {
        xTaskNotifyGive(reconnect_task);
    } else {
        printf("HaLow reconnect: task missing; reconnect skipped\n");
    }
}

static void reconnect_task_fn(void *arg)
{
    (void)arg;
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        do_halow_reconnect();
    }
}

/**
 * Link status callback
 *
 * @param link_status   Current link status info.
 */
static void link_status_callback(const struct mmipal_link_status *link_status)
{
    uint32_t time_ms = mmosal_get_time_ms();
    if (link_status->link_state == MMIPAL_LINK_UP)
    {
        link_up = true;
        time_sync_start();
        if (reconnecting)
        {
            printf("HaLow reconnected. Time: %lu ms, ", time_ms);
            reconnecting = false;
            /* Restore default netif and gateway so NAPT/DNS use HaLow for internet again. */
            nat_router_refresh_halow_default_route();
        }
        else
        {
            printf("Link is up. Time: %lu ms, ", time_ms);
        }
        printf("IP: %s, ", link_status->ip_addr);
        printf("Netmask: %s, ", link_status->netmask);
        printf("Gateway: %s\n", link_status->gateway);

        if (!initial_connect_done)
        {
            initial_connect_done = true;
            mmosal_semb_give(link_established);
        }
        app_wlan_request_bw_scan();
    }
    else
    {
        link_up = false;
        printf("HaLow link down. Time: %lu ms", time_ms);
        if (initial_connect_done && !reconnecting && reconnect_timer != NULL)
        {
            if (xTimerStart(reconnect_timer, 0) == pdPASS)
            {
                printf(" (reconnect in %u s)", (unsigned)(HALOW_RECONNECT_DELAY_MS / 1000));
            }
            printf("\n");
        }
        else
        {
            printf("\n");
        }
    }
}

void app_wlan_init(void)
{
    enum mmwlan_status status;
    struct mmwlan_version version;

    /* Ensure we don't call twice */
    MMOSAL_ASSERT(link_established == NULL);
    link_established = mmosal_semb_create("link_established");

    /* Initialize Morse subsystems, note that they must be called in this order. */
    mmhal_init();
    mmwlan_init();

    status = mmwlan_set_sgi_enabled(true);
    if (status != MMWLAN_SUCCESS) {
        printf("Warning: failed to enable HaLow SGI (%d)\n", (int)status);
    }
    status = mmwlan_set_rts_threshold(2347);
    if (status != MMWLAN_SUCCESS) {
        printf("Warning: failed to set HaLow RTS threshold (%d)\n", (int)status);
    }

    mmwlan_set_channel_list(load_channel_list());
    status = mmwlan_set_power_save_mode(MMWLAN_PS_DISABLED);
    if (status != MMWLAN_SUCCESS) {
        printf("Warning: failed to disable HaLow power save (%d)\n", (int)status);
    }
#if CONFIG_PM_ENABLE
    {
        /* Keep CPU at full speed and disallow light sleep. */
        esp_pm_config_t pm_cfg = {
            .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
            .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
            .light_sleep_enable = false,
        };
        esp_err_t pm_err = esp_pm_configure(&pm_cfg);
        if (pm_err != ESP_OK) {
            printf("Warning: esp_pm_configure failed: %s\n", esp_err_to_name(pm_err));
        }
    }
#endif

    /* Load IP stack settings from config store, or use defaults if no entry found in
     * config store. */
    struct mmipal_init_args mmipal_init_args = MMIPAL_INIT_ARGS_DEFAULT;
    load_mmipal_init_args(&mmipal_init_args);

    /* Initialize IP stack. */
    if (mmipal_init(&mmipal_init_args) != MMIPAL_SUCCESS)
    {
        printf("Error initializing network interface.\n");
        MMOSAL_ASSERT(false);
    }

    mmipal_set_link_status_callback(link_status_callback);

    if (xTaskCreate(reconnect_task_fn, "halow_reconn", HALOW_RECONNECT_TASK_STACK,
                    NULL, HALOW_RECONNECT_TASK_PRIO, &reconnect_task) != pdPASS) {
        reconnect_task = NULL;
        printf("Warning: could not create HaLow reconnect task\n");
    }
    reconnect_timer = xTimerCreate("halow_reconn", pdMS_TO_TICKS(HALOW_RECONNECT_DELAY_MS),
                                   pdFALSE, NULL, reconnect_timer_cb);
    if (reconnect_timer == NULL)
    {
        printf("Warning: could not create HaLow reconnect timer\n");
    }

    status = mmwlan_get_version(&version);
    MMOSAL_ASSERT(status == MMWLAN_SUCCESS);
    printf("Morse firmware version %s, morselib version %s, Morse chip ID 0x%lx\n\n",
           version.morse_fw_version, version.morselib_version, version.morse_chip_id);
}

void app_wlan_start(void)
{
    (void)app_wlan_start_with_timeout(UINT32_MAX);
}

bool app_wlan_start_with_timeout(uint32_t timeout_ms)
{
    enum mmwlan_status status;

    /* Load Wi-Fi settings from config store */
    struct mmwlan_sta_args sta_args = MMWLAN_STA_ARGS_INIT;
    load_mmwlan_sta_args(&sta_args);
    load_mmwlan_settings();

    printf("Attempting to connect to %s ", sta_args.ssid);
    if (sta_args.security_type == MMWLAN_SAE)
    {
        printf("with passphrase %s", sta_args.passphrase);
    }
    printf("\n");
    if (timeout_ms != UINT32_MAX)
    {
        printf("Timeout %lu ms\n", (unsigned long)timeout_ms);
    }
    else
    {
        printf("This may take some time (~30 seconds)\n");
    }

    status = mmwlan_sta_enable(&sta_args, sta_status_callback);
    MMOSAL_ASSERT(status == MMWLAN_SUCCESS);

    mmosal_semb_wait(link_established, timeout_ms);
    return link_up;
}

void app_wlan_stop(void)
{
    /* Shutdown wlan interface */
    mmwlan_shutdown();
}

bool app_wlan_request_reconnect(void)
{
    if (reconnect_task == NULL) {
        return false;
    }
    xTaskNotifyGive(reconnect_task);
    app_wlan_request_bw_scan();
    return true;
}

uint8_t app_wlan_get_op_bw_mhz(void)
{
    return halow_op_bw_mhz;
}
