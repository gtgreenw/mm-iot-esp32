/*
 * Copyright 2023 Morse Micro
 * SPDX-License-Identifier: Apache-2.0
 *
 * NVS-backed settings for HaLow STA and 2.4GHz AP. Used by loadconfig, nat_router, and web UI.
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

#define SETTINGS_NS           "bridge"
#define SETTINGS_MAX_SSID     32
#define SETTINGS_MAX_PASS     64
#define SETTINGS_MAX_COUNTRY  4
#define BACKHAUL_MODE_HALOW   0
#define BACKHAUL_MODE_WIFI_2G 1

typedef struct {
    char halow_ssid[SETTINGS_MAX_SSID];
    char halow_pass[SETTINGS_MAX_PASS];
    char wifi_backhaul_ssid[SETTINGS_MAX_SSID];
    char wifi_backhaul_pass[SETTINGS_MAX_PASS];
    char ap_ssid[SETTINGS_MAX_SSID];
    char ap_pass[SETTINGS_MAX_PASS];
    char country[SETTINGS_MAX_COUNTRY];
    int8_t ap_tx_power_dbm;  /* 2.4 GHz AP TX power (2–20 dBm); default 12 */
    bool iperf_server_enabled; /* Enable iperf server modes */
    uint8_t backhaul_mode; /* BACKHAUL_MODE_* */
} bridge_settings_t;

/**
 * Load settings from NVS. If a key is missing, the corresponding field is filled with the default.
 * Defaults: HaLow backhaul (halow / letmein111), 2.4 GHz AP (XIAO_S3_HALOW / letmein111),
 * 2.4 GHz backhaul SSID/pass empty, US, 2.4 GHz TX power 6 dBm, iperf server disabled.
 */
void settings_load(bridge_settings_t *out);

/**
 * Save settings to NVS. Returns true on success.
 */
bool settings_save(const bridge_settings_t *s);

/**
 * Initialize NVS partition for settings (call once at boot before any load/save).
 */
void settings_init(void);

#endif /* SETTINGS_H */
