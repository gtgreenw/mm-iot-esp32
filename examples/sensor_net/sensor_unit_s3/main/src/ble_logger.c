/*
 * BLE scan logger stub when CONFIG_SENSOR_BLE_LOG_ENABLE is n.
 * No BLE includes; this file is only compiled when BLE is disabled.
 */
#include "ble_logger.h"
#include <string.h>

bool ble_logger_start(void)
{
    (void)0;
    return false;
}

void ble_logger_stop(void)
{
}

bool ble_logger_is_enabled(void)
{
    return false;
}

void ble_logger_get_stats(ble_logger_stats_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
}
