# Sensor Unit (HaLow + ESP-NOW + optional camera)

Sensor node firmware for the ESP-Motion gateway. Runs on **Xiao ESP32-S3-Sense** (or any ESP32-S3 with HaLow + 2.4 GHz). Connects to the same HaLow network as the gateway, sends sensor packets via ESP-NOW (broadcast, channel 6), and optionally serves an MJPEG camera stream for the gateway dashboard.

## Features

- **Setup mode**: On first boot (no HaLow config in NVS), runs 2.4 GHz AP **ESP-Sensor-EN** (WPA2, password `sensor123`). Join the AP, open **http://192.168.4.1**, enter the same HaLow SSID/password as your gateway, then **Save and reboot**.
- **Running mode**: Connects to HaLow (30 s timeout). Sends `sensor_packet_t` via ESP-NOW to broadcast (gateway receives on its 2.4 GHz AP). Gets DHCP on HaLow.
- **Optional camera**: When **Enable camera stream** is set in menuconfig (`idf.py menuconfig` → Sensor unit), initializes the OV2640 camera on Xiao ESP32-S3-Sense and serves MJPEG at **http://\<sensor-ip\>/stream**. Add this URL in the gateway dashboard **Cameras** tab to view the stream.

## Build and flash

From the **mm-iot-esp32** repo (not from this folder; copy or use from examples):

```bash
export MMIOT_ROOT=~/mm-iot-esp32   # or your mm-iot-esp32 path
cd $MMIOT_ROOT/examples/sensor_net/sensor_unit
idf.py set-target esp32s3
idf.py menuconfig   # optional: enable/disable "Enable camera stream (/stream for gateway dashboard)"
idf.py build
idf.py -p /dev/cu.usbmodem101 flash monitor
```

## Camera (Xiao ESP32-S3-Sense)

- **Board**: Seeed Studio XIAO ESP32S3 Sense (OV2640 on expansion).
- **Pins**: Uses the standard Sense camera slot (DVP Y9=48, Y8=11, … XCLK=10, SDA=40, SCL=39; PWDN/RESET=-1). See [Seeed wiki](https://wiki.seeedstudio.com/xiao_esp32s3_camera_usage).
- **PSRAM**: Required; enabled in `sdkconfig.defaults`.
- **Stream**: MJPEG at `/stream` (same format as the `web_camera_serve` example). Gateway dashboard **Cameras** tab: add `http://<sensor-ip>/stream` and Save.

## Gateway dashboard

On the gateway device, open the dashboard on HaLow (e.g. `http://<gateway-ip>`). Go to the **Cameras** tab, enter the sensor unit stream URL (e.g. `http://192.168.x.x/stream`), click **Save**. The stream appears in the grid.
