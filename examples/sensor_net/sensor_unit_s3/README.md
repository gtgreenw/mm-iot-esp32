# Sensor Unit S3 (ESP-NOW only)

Sensor node firmware for the ESP-Motion gateway. Runs on **Xiao ESP32-S3-Sense** (or any ESP32-S3). Sends sensor packets (motion, BME680, optional BLE log) via **ESP-NOW** to the gateway on 2.4 GHz. No HaLow, no HTTP portal.

## Build and flash

From the **mm-iot-esp32** repo:

```bash
cd $MMIOT_ROOT/examples/sensor_net/sensor_unit_s3
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem101 flash monitor
```

## Motion: mmWave (LD2410 / Seeed 24GHz for XIAO)

This build defaults to **UART mmWave** (LD2410 / Seeed 24GHz mmWave for XIAO). Motion and distance (moving/stationary cm) come from the radar over UART.

- **Wiring**: Sensor **TX** → MCU **RX**. Sensor **RX** → MCU **TX** (always cross TX/RX).
- **Baud**: This build defaults to **9600**, matching the [Seeed mmWave for XIAO to Home Assistant](https://wiki.seeedstudio.com/mmwave_for_xiao_to_ha_bt/) and [Arduino](https://wiki.seeedstudio.com/mmwave_for_xiao_arduino) examples. If you haven’t changed the sensor’s baud in the HLKRadarTool app, set it to 9600 (More → settings), then reboot the sensor. Use 256000 in menuconfig only if the sensor is still at factory 256000.

**Which pins to use**

- **Stackable Seeed “24GHz mmWave for XIAO”** on a **Seeed Studio XIAO ESP32-S3** (non-Sense; silkscreen D#): the expansion uses **D2 (sensor TX)** and **D3 (sensor RX)**. On the **XIAO ESP32-S3** (non-Sense), **D2 = GPIO 3** and **D3 = GPIO 4**. Set **MCU RX = 3**, **MCU TX = 4**. Defaults in `sdkconfig.defaults` are **TX=4, RX=3**. **Check the boot log**: it must show `LD2410 UART1 TX=4 RX=3`. If you see **TX=3 RX=4**, run `idf.py fullclean` then `idf.py build` so defaults apply (or set in menuconfig: mmWave UART TX GPIO = 4, RX GPIO = 3).
- **XIAO ESP32-S3 Sense** (camera/mic): some pinouts use D2=GPIO2, D3=GPIO3 — try **RX=2, TX=3** in menuconfig if RX=3/TX=4 doesn’t work.
- **External wiring** to other pins (e.g. 17/18): use menuconfig **mmWave UART TX/RX GPIO** to match your wiring.

To use **GPIO motion** (e.g. RCWL-0516) instead: in menuconfig set **Motion sensor type** to **GPIO (RCWL-0516)**.

## Debugging mmWave (no motion / no distance)

If the dashboard shows no motion and no distance for the S3 unit with mmWave attached:

1. **Confirm mmWave is enabled**  
   In `idf.py menuconfig` → **Sensor unit** → **Motion sensor type** must be **UART mmWave (LD2410 / 24GHz Seeed XIAO)**. If it’s GPIO, the LD2410 is never used.

2. **Check serial log at boot**  
   - **"Motion: mmWave (LD2410) UART1 TX=17 RX=18 256000 baud - waiting for first frame"** → Init OK; waiting for first UART frame from the sensor.  
   - **"Motion: mmWave init failed - check UART pins TX=… RX=… and baud …"** → UART init failed (wrong port/pins or in use).  
   - **"LD2410 first frame: state=… move=… cm stat=… cm"** (from `ld2410` component) → Sensor is sending; motion/distance will appear on the gateway.

3. **"LD2410 no frame yet"** (every ~30 s)  
   Init succeeded but no valid frame has been received. Check:
   - **Pins**: On **XIAO ESP32-S3 (non-Sense)** D2=GPIO3, D3=GPIO4 → **MCU RX=3, TX=4**. Rebuild after `idf.py fullclean` so defaults apply.
   - **Wiring**: Sensor **TX** → MCU **RX**, Sensor **RX** → MCU **TX**. If wired externally, confirm which GPIOs you used and set them in menuconfig.
   - **Baud**: Default is **9600** (Seeed HA/Arduino examples). Set the sensor to 9600 in **HLKRadarTool** (More → settings) if needed, then rebuild with default baud. Try **256000** in menuconfig only if the sensor was never changed from factory.
   - **Power**: Sensor powered; antenna facing out. Some modules have a PC config tool that can change baud or protocol—leave at 256000 and default protocol.

4. **Rebuild after changing defaults**  
   If you changed `sdkconfig.defaults` (e.g. to enable mmWave), run `idf.py fullclean` then `idf.py build` so defaults are applied.

5. **Both pin orders tried, still no data**  
   The LD2410 driver logs **"rx first data: N bytes, hex …"** the first time any UART bytes are received.  
   - **If you never see that line**: no bytes are reaching the MCU. On **non-Sense XIAO ESP32-S3** the correct pair is RX=3, TX=4; try **swapping** to RX=4, TX=3 in case the expansion’s D2/D3 are reversed. On **Sense** boards, try **RX=2, TX=3**.  
   - **If you see "rx first data"** but still no "first frame": the hex should start with **F1 F2 F3 F4** for a valid LD2410 frame. If it doesn’t, baud is likely wrong (try 256000 if the sensor was never changed, or 115200).  

After the first valid frame, motion and distance (moving/stationary cm) are sent in every sensor packet and shown on the gateway dashboard and in the Sensors/Motion views.
