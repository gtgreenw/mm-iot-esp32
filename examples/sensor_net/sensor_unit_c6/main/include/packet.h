#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

#define SENSOR_PACKET_MAGIC  0x53
#define SENSOR_PACKET_VERSION 9
#define SENSOR_PACKET_VERSION_V8 8
#define SENSOR_PACKET_VERSION_V7 7
#define SENSOR_MOISTURE_CHANNELS 4
#define SENSOR_PLANT_LABEL_LEN 16

/* Sentinel for "no water/probe temp" (valid range is well above this). */
#define SENSOR_TEMP_WATER_INVALID (-1000.0f)
#define SENSOR_TDS_INVALID (-1.0f)

typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint8_t  version;
  uint8_t  motion;
  float    temperature;      /* BME/air temp °C */
  float    temperature_water; /* DS18B20/probe °C, or SENSOR_TEMP_WATER_INVALID */
  float    humidity;
  float    pressure;
  float    gas;
  float    tds_ppm;         /* TDS (ppm), or SENSOR_TDS_INVALID */
  float    moisture[SENSOR_MOISTURE_CHANNELS]; /* 0–100% per channel, or <0 if disabled */
  uint32_t last_motion_ms;
  uint32_t trigger_count;
  uint16_t ble_seen_count;
  int8_t   ble_last_rssi_dbm;
  uint8_t  ble_last_addr[6];
  uint32_t uptime_ms;
  char     plant_label[SENSOR_MOISTURE_CHANNELS][SENSOR_PLANT_LABEL_LEN];
  /* S3 sensor unit (LD2410 / Seeed mmWave): C6 sends zeros */
  uint8_t  mmwave_state;
  uint16_t mmwave_moving_cm;
  uint16_t mmwave_stationary_cm;
  uint8_t  mmwave_moving_energy;
  uint8_t  mmwave_stationary_energy;
  uint16_t mmwave_detection_dist_cm;
} sensor_packet_t;

#define SENSOR_PACKET_SIZE sizeof(sensor_packet_t)

/* Gateway beacon (gateway → broadcast); sensor uses this to lock channel during scan. */
#define GATEWAY_PACKET_MAGIC 0x47

/* Gateway → node command packet (ESP-NOW) */
#define CMD_PACKET_MAGIC 0x43
#define CMD_TYPE_BLINK   1
#define CMD_TYPE_RESET   2
#define CMD_TYPE_SET_PLANT_LABEL 3

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t cmd_type;
} cmd_packet_t;

#define CMD_PACKET_SIZE sizeof(cmd_packet_t)

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t cmd_type;
  uint8_t channel;   /* moisture channel index 0-3 */
  char    label[SENSOR_PLANT_LABEL_LEN];
} cmd_plant_label_packet_t;

#define CMD_PLANT_LABEL_PACKET_SIZE sizeof(cmd_plant_label_packet_t)

#endif
