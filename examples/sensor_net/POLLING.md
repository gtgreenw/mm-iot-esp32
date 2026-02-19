# Polling and periodic intervals

Reference for all polling / interval settings in the sensor_net example.

---

## Dashboard (browser → gateway HTTP)

Defined in **`main/web/dashboard.js`** (embedded via `dashboard_embedded.c`).  
These run only when the dashboard is open in a browser.

| What | Interval | Location (approx) |
|------|----------|-------------------|
| **fetchSensors** | 1 s | `setInterval(fetchSensors, 1000)` (fast refresh for motion/sensors) |
| **fetchEnvHistory** | 60 s | `setInterval(fetchEnvHistory, 60000)` |
| **fetchHalow** | 10 s | `setInterval(fetchHalow, 10000)` |
| **fetchWifi2g** | 10 s | `setInterval(fetchWifi2g, 10000)` |
| **fetchDebug** (Data log / system tab) | 10 s | `setInterval(fetchDebug, 10000)` |
| Initial fetch delays | 400 ms, 700 ms, 1100 ms | `setTimeout(fetchHalow, 400)` etc. |
| BLE whitelist / BLE log status clear | 2.5 s | `setTimeout(..., 2500)` |
| Halow reconnect status clear | 2 s | `setTimeout(..., 2000)` |

---

## Gateway (main app) – ESP-NOW / logs

Defined in **`main/src/esp_now_rcv.c`** unless noted.

| Symbol | Value | Meaning |
|--------|--------|---------|
| **GATEWAY_BEACON_MS** | 10000 (10 s) | ESP-NOW gateway beacon send interval |
| **GATEWAY_STALE_MS** | 30000 (30 s) | Gateways not seen within this are considered stale |
| **LOG_PERSIST debounce** | 1000 ms | Delay after wake before NVS persist (max 1 persist/s under load) |
| **ESPNOW_DEFERRED_START_MS** | 500 ms | Delay before starting ESP-NOW after boot (one-shot) |
| **Mesh fwd log throttle** | 5000 ms | Min interval between "Mesh fwd sensor" log lines (in code) |
| **ESP-NOW set_channel delay** | 200 ms | Delay after `esp_wifi_set_channel` before `esp_now_init` |

BLE whitelist capture (used by `/api/ble_whitelist/capture`):

| Symbol | Value | Defined in |
|--------|--------|------------|
| **BLE_WHITELIST_CAPTURE_DEFAULT_MS** | 15000 (15 s) | `main/src/esp_now_rcv.h` |

---

## Gateway – HaLow / mesh / NAT

| Symbol | Value | Meaning | File |
|--------|--------|----------|------|
| **HALOW_RECONNECT_DELAY_MS** | 5000 (5 s) | HaLow reconnect timer period (after link down) | `main/src/mm_app_common.c` |
| **HALOW_MESH_DV_INTERVAL_MS** | 2000 (2 s) | Mesh task: interval between DV send + tick | `main/src/iperf.c` |
| **route_fix timer** | 3000 ms | One-shot: set default netif after AP start | `main/src/nat_router.c` |
| **HALOW_MESH_ROUTE_TIMEOUT_MS** | 120000 (2 min) | Route entry timeout in mesh component | `components/halow_mesh/halow_mesh.h` |

Other one-off delays in **`main/src/iperf.c`**: 1000 ms (main loop), 5000 ms (stabilize), 500 ms (web_config restart).

---

## Gateway – web config

| What | Value | File |
|------|--------|------|
| Restart task delay before reboot | 500 ms | `main/src/web_config.c` |
| HaLow scan wait | 8000 ms | `main/src/web_config.c` (mmosal_semb_wait) |

---

## Sensor units (sensor_unit / sensor_unit_c6)

| Symbol | Value | Meaning | File |
|--------|--------|----------|------|
| **SENSOR_MOTION_POLL_MS** | 200 ms | Main loop: check motion for instant send on 0→1 | `main.c` |
| **SENSOR_PERIODIC_MS** | 60000 (60 s) | Main loop: env data (temp/humidity/pressure/gas) send interval | `main.c` |
| **esp_now_send_packet_on_motion_trigger()** | on 0→1 | Send one packet immediately when motion triggers | `esp_now_send.c` |
| **BLINK_MS** | 120 ms | LED blink / backoff in ESP-NOW send | `esp_now_send.c` |
| ESP-NOW send backoff after failure | 200 ms | `esp_now_send.c` |
| I2C timeouts | 100 ms | BME / I2C read/write timeouts | `esp_now_send.c` |

---

## Summary (gateway at idle)

- **Every 2 s**: HaLow mesh DV + tick (if mesh overlay enabled).
- **Every 5 s**: ESP-NOW gateway beacon; HaLow reconnect timer (when link down).
- **Every 30 s**: Gateway stale check (log/API only).
- **Dashboard**: When open, 1 s for Sensors (motion), 10 s for Halow/Wifi2g/Debug, 60 s for EnvHistory.

To reduce load: increase dashboard intervals (e.g. 5 s → 10 s or 30 s → 60 s), or increase **GATEWAY_BEACON_MS** / **HALOW_MESH_DV_INTERVAL_MS** / **LOG_PERSIST** debounce.
