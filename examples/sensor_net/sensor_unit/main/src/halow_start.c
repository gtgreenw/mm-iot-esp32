/**
 * HaLow connection: calls mm-iot-esp32 app_wlan_init + app_wlan_start.
 * start_halow_connection_with_timeout(timeout_ms) returns true if connected, false on timeout.
 */
#include "mm_app_common.h"

void start_halow_connection(void)
{
	app_wlan_init();
	app_wlan_start();
}

bool start_halow_connection_with_timeout(uint32_t timeout_ms)
{
	app_wlan_init();
	return app_wlan_start_with_timeout(timeout_ms);
}
