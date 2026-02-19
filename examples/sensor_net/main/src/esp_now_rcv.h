#ifndef ESP_NOW_RCV_H
#define ESP_NOW_RCV_H

#include <stddef.h>
#include <stdbool.h>
#include "packet.h"

#define MAX_NODES 16
#define NODE_MAC_LEN 18

typedef struct {
    char mac[NODE_MAC_LEN];
    sensor_packet_t pkt;
    uint32_t last_ms;
    uint32_t last_motion_uptime_ms;
    uint32_t last_motion_seen_ms;  /* gateway uptime ms when we last received motion==1 */
    int8_t rssi_dbm;
    uint8_t last_motion;
    uint32_t trigger_count;
    uint32_t last_env_log_ms;
} node_entry_t;

void esp_now_rcv_init(void);
/** Optional: call after WiFi is connected (e.g. STA got IP) to re-sync channel if needed. */
void esp_now_rcv_start_deferred(void);
int esp_now_rcv_get_channel(void);  /* Current ESP-NOW/WiFi channel (1–14), 0 if unknown */
int esp_now_rcv_node_count(void);
const node_entry_t *esp_now_rcv_get_node(int i);
const char *esp_now_rcv_get_label(const char *mac);
void esp_now_rcv_set_label(const char *mac, const char *label);
const char *esp_now_rcv_get_location(const char *mac);
void esp_now_rcv_set_location(const char *mac, const char *location);
bool esp_now_rcv_send_reset(const char *mac);
bool esp_now_rcv_send_plant_label(const char *mac, int channel, const char *label);

/* UI skin/theme persistence (NVS) */
void esp_now_rcv_get_ui_skin(char *skin, size_t skin_sz, char *font_size, size_t fs_sz);
void esp_now_rcv_set_ui_skin(const char *skin, const char *font_size);

/* Persistent sensor data log (ring buffer in NVS) */
#define SENSOR_LOG_MAX 32
typedef struct {
    char mac[NODE_MAC_LEN];
    int64_t ts_ms;
    sensor_packet_t pkt;
} sensor_log_entry_t;

void sensor_log_append(const uint8_t *mac_addr, const sensor_packet_t *p);
int sensor_log_count(void);
const sensor_log_entry_t *sensor_log_get(int i);
void sensor_log_clear(void);

#endif
