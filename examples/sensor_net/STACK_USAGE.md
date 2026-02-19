# Stack usage in sensor_net

Summary of where task stacks are allocated and how to find what is using them.

## Main app (gateway) – largest consumers

| Task / config | Stack (bytes) | Source |
|---------------|---------------|--------|
| **main** | **20,480** | `sdkconfig.defaults`: `CONFIG_ESP_MAIN_TASK_STACK_SIZE` |
| **wifi** | **24,576** | `wifi_task_stack_override.c` (override); `CONFIG_ESP_WIFI_TASK_STACK_SIZE` in sdkconfig. AP+STA+NAPT needs this to avoid "stack overflow in task wifi". |
| **httpd** (web config + dashboard) | **8,192** | `web_config.c`: `WEB_CONFIG_STACK_SIZE` |
| halow_reconn | 4,096 | `mm_app_common.c`: `HALOW_RECONNECT_TASK_STACK` |
| dns_fwd | 4,096 | `dns_forwarder.c`: `DNS_FORWARDER_STACK_WORDS` |
| halow_bw | 3,072 | `mm_app_common.c`: `xTaskCreate(..., 3072, ...)` |

## Optional app tasks (if built)

| Task | Stack (bytes) | Source |
|------|---------------|--------|
| weather | 8,192 | `weather_fetch.c` (not in default main CMakeLists) |
| hap_bridge (HomeKit) | 4,096 | `sensor_homekit.c` (not in default main CMakeLists) |
| ld2410 | 2,048 | `components/ld2410/ld2410.c` (if component used) |

## ESP-IDF system tasks (from sdkconfig)

| Task | Typical stack (bytes) |
|------|------------------------|
| System event | 2,304 |
| Timer | 3,584 |
| IPC | 1,280 |
| Idle (×2 on dual-core) | 1,536 each |
| FreeRTOS timer | 2,048 (depth) |
| LWIP TCP/IP | 3,072 |
| pthread default | 3,072 |

## sensor_unit_c6

| Task | Stack (bytes) | Source |
|------|---------------|--------|
| main | 3,584 | `sensor_unit_c6/sdkconfig`: `CONFIG_ESP_MAIN_TASK_STACK_SIZE` |
| blink | 1,536 | `esp_now_send.c` |

## What uses the most stack (gateway)

1. **Main task (20 KB)** – `app_main` and init; kept large for init + packet structs (see `TASKS_AND_PSRAM.md`).
2. **WiFi task (16 KB)** – AP+STA+NAPT (beacon, power save, etc.); 8 KB was insufficient on ESP32-S3.
3. **HTTP server (8 KB)** – Each HTTP request runs on the server task stack; dashboard + API + long headers.

## How to see actual usage at runtime

Use FreeRTOS high-water mark to see minimum free stack per task (bytes left at worst point):

- **API**: `uxTaskGetStackHighWaterMark(TaskHandle_t)` – returns minimum remaining stack in **words** (4 bytes on ESP32).
- Multiply by 4 to get bytes, or use `uxTaskGetStackHighWaterMark(...) * sizeof(StackType_t)`.

**Runtime report:** Call `stack_usage_report()` (from `main/src/stack_usage.h`) to print every task’s stack high-water mark to serial. Add once in `app_main` after init (e.g. after a 10–30 s delay) or trigger from a debug path. Requires `CONFIG_FREERTOS_USE_TRACE_FACILITY=y` (default in ESP-IDF).
