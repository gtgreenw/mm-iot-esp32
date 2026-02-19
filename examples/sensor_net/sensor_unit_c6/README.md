# Sensor Unit (ESP32-C6, ESP-NOW only)

Minimal sensor unit firmware for **Xiao ESP32-C6**. Sends `sensor_packet_t` via ESP-NOW
to the gateway. No HaLow and no camera. Uses a large NVS partition for local logging.

## Build and flash

From the **sensor_unit_c6** directory (no MMIOT_ROOT needed; ESP-NOW only, local bme68x):

```bash
cd examples/sensor_net/sensor_unit_c6
idf.py set-target esp32c6
idf.py build
idf.py -p /dev/cu.usbmodem101 flash monitor
```

## Notes

- **Partition table**: `partitions.csv` allocates most flash to NVS for local logging.
- **BLE logging**: Controlled by `CONFIG_SENSOR_BLE_LOG_ENABLE` in menuconfig.
- **Grove moisture**: Disabled by default (`CONFIG_SENSOR_MOISTURE_ENABLE=n` in `sdkconfig.defaults`). Enable in menuconfig to send soil moisture. If you disable it in menuconfig but the unit still sends moisture data, run `idf.py fullclean` then `idf.py build` so the new config is applied; at boot you should see log "Grove moisture: DISABLED".
