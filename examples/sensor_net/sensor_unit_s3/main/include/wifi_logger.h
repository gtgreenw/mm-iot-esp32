/*
 * WiFi scan logger for sensor unit.
 * Periodically scans for WiFi networks and reports them to the gateway.
 */
#ifndef WIFI_LOGGER_H
#define WIFI_LOGGER_H

#include <stdbool.h>
#include "packet.h"

/** Start WiFi logging (enabled flag; scanning runs when polled). */
void wifi_logger_start(void);
/** Stop WiFi logging. */
void wifi_logger_stop(void);
/** Return true if WiFi logging is enabled. */
bool wifi_logger_is_enabled(void);

/**
 * Run a scan if interval has elapsed and fill pkt. Call from main loop.
 * Returns number of entries (0 = skip send or error).
 */
int wifi_logger_try_scan_and_fill(wifi_scan_packet_t *pkt);

#endif
