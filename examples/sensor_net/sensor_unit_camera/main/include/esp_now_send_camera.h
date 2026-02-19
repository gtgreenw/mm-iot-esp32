#ifndef ESP_NOW_SEND_CAMERA_H
#define ESP_NOW_SEND_CAMERA_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize ESP-NOW (same capabilities as sensor_unit: send sensor_packet_t, handle blink/reset, relay as mesh hop). */
void esp_now_send_camera_init(void);

/** Return true if ESP-NOW is ready. */
bool esp_now_send_camera_ready(void);

/** Current runtime state: ESP-NOW is active. */
bool esp_now_send_camera_is_enabled(void);

/** Enable or disable ESP-NOW; persists to NVS and applies immediately. */
void esp_now_send_camera_set_enabled(bool enabled);

/** Number of other ESP-NOW sensors seen recently (sensor packets received in last 5 min). */
int esp_now_send_camera_peers_seen_count(void);

/** Send one sensor packet (camera: zeros + uptime, same format as sensor_unit). Call periodically for gateway discovery. */
void esp_now_send_camera_packet(void);

/** Restore WiFi TX power to normal (4 dBm) after boot settle. Call 5 s after boot complete. */
void esp_now_send_camera_restore_wifi_tx_power(void);

#ifdef __cplusplus
}
#endif

#endif
