# Camera sensor unit (XIAO ESP32-S3-Sense)

Dedicated firmware for the **XIAO ESP32-S3-Sense** board as a camera-only sensor node in the sensor_net gateway setup. No BME680 or motion sensor; just HaLow STA, MJPEG stream, and minimal ESP-NOW so the gateway discovers the node.

## Hardware

- **Board:** [Seeed XIAO ESP32-S3-Sense](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) (ESP32-S3, 8MB PSRAM, OV2640 camera)
- **Camera:** OV2640 (built-in on Sense board)

## Features

- **Setup mode:** If HaLow is not configured, the unit starts a 2.4 GHz AP (`ESP-Sensor-EN` / `sensor123`), same as other sensor units. Join it, open http://192.168.4.1, enter the same HaLow SSID/password as your gateway, save → device reboots and connects to HaLow.
- **Running mode:** Connects to HaLow via the mm-iot-esp32 stack, serves **video at http://\<sensor-ip\>/live** (MJPEG, max 20 fps), and sends minimal ESP-NOW packets (sensor_packet_t with zeros + uptime) so the gateway discovers the node and can show the camera URL.
- **ESP-NOW → HaLow forwarding:** The camera receives ESP-NOW sensor packets from other nodes (e.g. BME680/motion units on channel 6) and **forwards them over the HaLow mesh** to the gateway. The gateway sees each sensor by its original MAC. Enable **HaLow mesh overlay** in the gateway Settings (on by default) so it receives these forwarded packets.
- **Stream quality:** Set in Settings on the camera; for raw MJPEG use `http://\<sensor-ip\>/live/stream?quality=low` (or `medium`/`high`/`auto`).
- **HaLow link LED:** GPIO **2**. Boot = solid on, connecting = fast flash (~5 Hz), connected = one blink per second (500 ms on / 500 ms off). Gateway “Blink” command triggers 3 blinks via ESP-NOW.
- **Snapshots:** **GET /snapshot** — captures one JPEG frame and returns it as `image/jpeg`.
- **Microphone:** Built-in PDM mic (DATA=41, CLK=42). **GET /audio** — streams raw PCM 16-bit LE, 16 kHz mono (`audio/raw`). Use alongside the video stream for audio on the dashboard.
- **Combined video + audio:** **GET /live** — one page that shows the MJPEG stream and plays the audio stream together (synced on load). Use **http://\<sensor-ip\>/live** for a single “one video with sound” view in the browser.
- **RTSP:** Not built-in. For VLC, open `http://\<sensor-ip\>/live/stream` as a network stream. To add a real RTSP server (video only, port 8554), integrate the [espp/rtsp](https://components.espressif.com/components/espp/rtsp) component and feed frames via `camera_stream_get_one_jpeg()`.

## Troubleshooting

- **Interrupt WDT timeout on CPU0** during "Connecting to HaLow": The project disables the interrupt watchdog in `sdkconfig.defaults` (`CONFIG_ESP_INT_WDT=n`) because the HaLow SPI/GPIO ISR path can block long enough to trigger it. If you still see this panic, your `sdkconfig` may have the option enabled (e.g. from a previous menuconfig). Remove it and rebuild so defaults apply:
  ```bash
  rm -f sdkconfig sdkconfig.old
  idf.py set-target esp32s3
  idf.py build
  ```

## Build and flash

From the **sensor_net** example root, set `MMIOT_ROOT` to the mm-iot-esp32 framework directory, then:

```bash
cd examples/sensor_net/sensor_unit_camera
export MMIOT_ROOT=/path/to/mm-iot-esp32
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

Or from the repo root:

```bash
export MMIOT_ROOT=$(pwd)   # if you're in mm-iot-esp32
cd examples/sensor_net/sensor_unit_camera
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## Gateway integration

1. Flash this firmware to the XIAO ESP32-S3-Sense.
2. On first boot (no HaLow config), join AP `ESP-Sensor-EN` (password `sensor123`), open http://192.168.4.1, set HaLow SSID/password to match the gateway, save. Device reboots and joins HaLow.
3. On the gateway dashboard, the node appears via ESP-NOW. Add the camera URL: `http://<sensor-ip>/live` (the sensor prints its HaLow IP in the log; the gateway may also show it from ESP-NOW discovery).
4. **Sensor forwarding:** Any ESP-NOW sensor (e.g. sensor_unit) that the camera can hear on channel 6 is forwarded to the gateway over HaLow. Ensure the gateway has **HaLow mesh overlay** enabled in Settings (default: on) so it receives and displays those sensors.

## Configuration

- **Kconfig:** `idf.py menuconfig`  
  - `SENSOR_CAMERA_ENABLE` – always on for this firmware.  
  - `SENSOR_LED_GPIO` – LED for gateway blink (default **2** on this board so SD CS can use 21; -1 to disable).

## Comparison with sensor_unit

| Feature            | sensor_unit           | sensor_unit_camera   |
|--------------------|-----------------------|----------------------|
| Board              | XIAO ESP32-S3-Sense   | XIAO ESP32-S3-Sense  |
| Camera (MJPEG)     | Optional (Kconfig)   | Always on            |
| SD snapshots/clips | No                    | No                       |
| Mic (/audio)       | No                    | Yes (16 kHz PCM)     |
| BME680             | Optional              | No                   |
| Motion (RCWL-0516) | Optional              | No                   |
| ESP-NOW            | Full sensor_packet_t  | Minimal (uptime only)|
| Use case           | Full sensor node      | Camera + mic          |
