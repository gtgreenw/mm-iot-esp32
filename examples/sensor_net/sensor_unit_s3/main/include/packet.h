#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

#define SENSOR_PACKET_MAGIC  0x53
#define SENSOR_PACKET_VERSION 7
#define SENSOR_LABEL_MAX 32
#define SENSOR_STREAM_HOST_MAX 16
#define SENSOR_MOISTURE_CHANNELS 4
#define SENSOR_PLANT_LABEL_LEN 16

/* Sentinel for "no water/probe temp" (valid range is well above this). */
#define SENSOR_TEMP_WATER_INVALID (-1000.0f)
/* Sentinel for "no TDS" (valid ppm is 0–1000+). */
#define SENSOR_TDS_INVALID (-1.0f)

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  motion;
    float    temperature;
    float    humidity;
    float    pressure;
    float    gas;
    uint32_t last_motion_ms;
    /** Total motion triggers; maintained and persisted in NVS on sensor unit, sent to gateway each packet. */
    uint32_t trigger_count;
    uint16_t ble_seen_count;
    int8_t   ble_last_rssi_dbm;
    uint8_t  ble_last_addr[6];
    uint32_t uptime_ms;
    char     label[SENSOR_LABEL_MAX];
    char     stream_host[SENSOR_STREAM_HOST_MAX];
    /* LD2410 / 24GHz mmWave: only set when motion source is mmWave. */
    uint8_t  mmwave_state;             /* 0=none, 1=moving, 2=stationary, 3=both */
    uint16_t mmwave_moving_cm;         /* moving target distance cm */
    uint16_t mmwave_stationary_cm;     /* stationary target distance cm */
    uint8_t  mmwave_moving_energy;     /* moving target energy 0–100 */
    uint8_t  mmwave_stationary_energy; /* stationary target energy 0–100 */
    uint16_t mmwave_detection_dist_cm; /* detection distance cm */
    /* Grove capacitive moisture (D0/D1 on XIAO S3); 0–100% or <0 if disabled */
    float    moisture[SENSOR_MOISTURE_CHANNELS];
    char     plant_label[SENSOR_MOISTURE_CHANNELS][SENSOR_PLANT_LABEL_LEN];
    uint8_t  is_outdoor;               /* 0=indoor, 1=outdoor; persisted on sensor unit */
    float    temperature_water;       /* DS18B20/probe °C, or SENSOR_TEMP_WATER_INVALID */
    float    tds_ppm;                  /* TDS (ppm), or SENSOR_TDS_INVALID */
} sensor_packet_t;

#define SENSOR_PACKET_SIZE sizeof(sensor_packet_t)
/* v7: 186 bytes (v6 ext 178 + temperature_water + tds_ppm). Gateway accepts v6 (178) and v7 (186). */
_Static_assert(SENSOR_PACKET_SIZE == 186, "S3 v7 packet must be 186 bytes");

/* Gateway beacon (gateway → broadcast); sensor uses this to lock channel during scan. */
#define GATEWAY_PACKET_MAGIC 0x47

/* Gateway → node command packet (ESP-NOW) */
#define CMD_PACKET_MAGIC 0x43
#define CMD_TYPE_BLINK   1
#define CMD_TYPE_RESET   2
#define CMD_TYPE_SET_BLE_LOG 3
#define CMD_TYPE_SET_LABEL   4
#define CMD_TYPE_SET_DEEP_SLEEP 5
#define CMD_TYPE_SET_WIFI_LOG 6
#define CMD_TYPE_SET_LOCATION 7

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t cmd_type;
} cmd_packet_t;

#define CMD_PACKET_SIZE sizeof(cmd_packet_t)

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t cmd_type;
    uint8_t enabled;
} cmd_ble_log_packet_t;

#define CMD_BLE_LOG_PACKET_SIZE sizeof(cmd_ble_log_packet_t)

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t cmd_type;
    uint8_t enabled;
} cmd_deep_sleep_packet_t;

#define CMD_DEEP_SLEEP_PACKET_SIZE sizeof(cmd_deep_sleep_packet_t)

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t cmd_type;
    char    label[SENSOR_LABEL_MAX];
} cmd_label_packet_t;

#define CMD_LABEL_PACKET_SIZE sizeof(cmd_label_packet_t)

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t cmd_type;
    uint8_t enabled;
} cmd_wifi_log_packet_t;

#define CMD_WIFI_LOG_PACKET_SIZE sizeof(cmd_wifi_log_packet_t)

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t cmd_type;
    uint8_t is_outdoor;  /* 0=indoor, 1=outdoor */
} cmd_location_packet_t;

#define CMD_LOCATION_PACKET_SIZE sizeof(cmd_location_packet_t)

/* WiFi scan result packet (sensor → gateway via ESP-NOW) */
#define WIFI_SCAN_PACKET_MAGIC   0x57
#define WIFI_SCAN_PACKET_VERSION 1
#define WIFI_SCAN_SSID_MAX       32
#define WIFI_SCAN_ENTRIES_MAX    5

typedef struct __attribute__((packed)) {
    char     ssid[WIFI_SCAN_SSID_MAX];
    uint8_t  bssid[6];
    int8_t   rssi_dbm;
    uint8_t  channel;
} wifi_scan_entry_t;

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  src_mac[6];
    uint32_t scan_ts_ms;
    uint8_t  num_entries;
    wifi_scan_entry_t entries[WIFI_SCAN_ENTRIES_MAX];
} wifi_scan_packet_t;

#define WIFI_SCAN_PACKET_SIZE sizeof(wifi_scan_packet_t)

/* ESP-NOW sensor packet forwarded over HaLow mesh */
#define ESPNOW_FWD_MAGIC   0x45
#define ESPNOW_FWD_VERSION 1

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t version;
    uint8_t src_mac[6];
    sensor_packet_t pkt;
} espnow_fwd_sensor_packet_t;

#define ESPNOW_FWD_SENSOR_PACKET_SIZE sizeof(espnow_fwd_sensor_packet_t)

#endif
