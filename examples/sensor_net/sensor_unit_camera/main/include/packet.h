#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

#define SENSOR_PACKET_MAGIC  0x53
#define SENSOR_PACKET_VERSION 6
#define SENSOR_LABEL_MAX 32
#define SENSOR_STREAM_HOST_MAX 16

typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint8_t  version;
  uint8_t  motion;
  float    temperature;
  float    humidity;
  float    pressure;
  float    gas;
  uint32_t last_motion_ms;
  uint32_t trigger_count;
  uint16_t ble_seen_count;
  int8_t   ble_last_rssi_dbm;
  uint8_t  ble_last_addr[6];
  uint32_t uptime_ms;
  char     label[SENSOR_LABEL_MAX];
  char     stream_host[SENSOR_STREAM_HOST_MAX];
  /* LD2410 / 24GHz mmWave: only set when motion source is mmWave (sensor sends v4+5 or v5+5). */
  uint8_t  mmwave_state;           /* 0=none, 1=moving, 2=stationary, 3=both */
  uint16_t mmwave_moving_cm;       /* movement target distance cm */
  uint16_t mmwave_stationary_cm;   /* stationary target distance cm */
  uint8_t  is_outdoor;             /* 0=indoor, 1=outdoor; persisted on sensor unit */
} sensor_packet_t;

#define SENSOR_PACKET_SIZE sizeof(sensor_packet_t)

#define CMD_PACKET_MAGIC 0x43
#define CMD_TYPE_BLINK   1
#define CMD_TYPE_RESET   2
#define CMD_TYPE_SET_LABEL   4
#define CMD_TYPE_SET_LOCATION 7

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t cmd_type;
} cmd_packet_t;

#define CMD_PACKET_SIZE sizeof(cmd_packet_t)

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t cmd_type;
  char    label[SENSOR_LABEL_MAX];
} cmd_label_packet_t;

#define CMD_LABEL_PACKET_SIZE sizeof(cmd_label_packet_t)

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t cmd_type;
  uint8_t is_outdoor;
} cmd_location_packet_t;

#define CMD_LOCATION_PACKET_SIZE sizeof(cmd_location_packet_t)

/* ESP-NOW sensor packet forwarded over HaLow mesh (gateway expects this format) */
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
