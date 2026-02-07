# Sensor Unit (ESP32-C6, ESP-NOW only)

Minimal sensor unit firmware for **Xiao ESP32-C6**. Sends `sensor_packet_t` via ESP-NOW
to the gateway. No HaLow and no camera. Uses a large NVS partition for local logging.

## Build and flash

From the **mm-iot-esp32** repo:

```bash
cd /Users/Gavin/esp/mm-iot-esp32/examples/halow_good/sensor_unit_c6
idf.py set-target esp32c6
idf.py build
idf.py -p /dev/cu.usbmodem101 flash monitor
```

## Notes

- **Partition table**: `partitions.csv` allocates most flash to NVS for local logging.
- **BLE logging**: Controlled by `CONFIG_SENSOR_BLE_LOG_ENABLE` in menuconfig.
