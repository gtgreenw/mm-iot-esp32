#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

#define SENSOR_PACKET_MAGIC  0x53
#define SENSOR_PACKET_VERSION 4

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
} sensor_packet_t;

#define SENSOR_PACKET_SIZE sizeof(sensor_packet_t)

/* Gateway → node command packet (ESP-NOW) */
#define CMD_PACKET_MAGIC 0x43
#define CMD_TYPE_BLINK   1
#define CMD_TYPE_RESET   2

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t cmd_type;
} cmd_packet_t;

#define CMD_PACKET_SIZE sizeof(cmd_packet_t)

#endif
