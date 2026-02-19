/*
 * Sensor Net Gateway – Apple HomeKit bridge
 * Exposes ESP-NOW sensor nodes as HomeKit Temperature, Humidity, and Motion accessories.
 */
#ifndef SENSOR_HOMEKIT_H
#define SENSOR_HOMEKIT_H

#ifdef __cplusplus
extern "C" {
#endif

/** Start the HomeKit bridge in a dedicated task.
 * Call after WiFi and ESP-NOW are up (e.g. after esp_now_rcv_start_deferred() or esp_now_rcv_init()).
 * No-op if CONFIG_SENSOR_NET_HOMEKIT is not set.
 */
void sensor_homekit_start(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_HOMEKIT_H */
