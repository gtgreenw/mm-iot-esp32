#ifndef ESP_NOW_SEND_H
#define ESP_NOW_SEND_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void esp_now_send_init(void);
bool esp_now_send_ready(void);
/** Current runtime state: ESP-NOW send is active. */
bool esp_now_send_is_enabled(void);
/** Enable or disable ESP-NOW send; persists to NVS and applies immediately. */
void esp_now_send_set_enabled(bool enabled);
/** Number of other ESP-NOW sensors seen recently (sensor packets received in last 5 min). */
int esp_now_send_peers_seen_count(void);
void esp_now_send_packet(void);
/** Send immediately when motion 0→1 (instant alert); env data still sent periodically. */
void esp_now_send_packet_on_motion_trigger(void);
/** Send when motion 1→0 so dashboard clears quickly. */
void esp_now_send_packet_on_motion_cleared(void);
/** If WiFi logging is enabled and interval elapsed, run scan and send WiFi scan packet to gateway. */
void esp_now_send_wifi_scan_if_due(void);

#ifdef __cplusplus
}
#endif

#endif
