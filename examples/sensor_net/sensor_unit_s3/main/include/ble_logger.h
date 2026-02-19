/*
 * BLE scan logger for sensor unit.
 */
#ifndef BLE_LOGGER_H
#define BLE_LOGGER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t addr[6];
    int8_t rssi_dbm;
    uint16_t seen_count;
    bool has_addr;
} ble_logger_stats_t;

bool ble_logger_start(void);
/** Stop BLE scan and deinit stack. */
void ble_logger_stop(void);
/** Return true if BLE logging is enabled. */
bool ble_logger_is_enabled(void);
void ble_logger_get_stats(ble_logger_stats_t *out);

#endif
