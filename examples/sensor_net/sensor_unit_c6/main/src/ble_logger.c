/*
 * BLE scan logger for sensor unit.
 */
#include "ble_logger.h"

#include <string.h>

bool ble_logger_start(void)
{
    return false;
}

void ble_logger_get_stats(ble_logger_stats_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
}
