# HaLow Sensor Gateway and Stations (sensor_net)

This example includes a HaLow gateway with an ESP-NOW sensor mesh and optional
sensor unit firmware. Use this README to build and flash each component.

## Prerequisites

- ESP-IDF installed and exported (`. $IDF_PATH/export.sh`)
- Python 3 and `pip` for ESP-IDF tools
- Serial access to the target device(s)

## Project layout

- `main/` - Gateway application (HaLow + ESP-NOW + web dashboard)
- `sensor_unit/` - Sensor unit firmware (ESP-NOW sensor node)
- `sensor_unit_c6/` - Alternate sensor unit firmware (ESP32-C6)

## Build and flash the gateway

1) From the repo root:
```
cd examples/sensor_net
```

2) Set the target and build (use the correct target for your board):
```
idf.py set-target esp32s3
idf.py build
```

3) Flash and monitor (replace the port):
```
idf.py -p /dev/ttyUSB0 flash monitor
```

## Build and flash a sensor unit

### `sensor_unit/`
```
cd examples/sensor_net/sensor_unit
idf.py set-target <your_target>
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### `sensor_unit_c6/`
```
cd examples/sensor_net/sensor_unit_c6
idf.py set-target esp32c6
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Using the dashboard

Once the gateway is running, connect to its AP and open the dashboard:

- Default dashboard: `http://192.168.4.1/`
- Settings page: `http://192.168.4.1/settings`

## Notes

- Make sure your gateway and sensor units are on the correct channels and
  configured for ESP-NOW compatibility.
- If you use different boards, update the `idf.py set-target` values accordingly.
