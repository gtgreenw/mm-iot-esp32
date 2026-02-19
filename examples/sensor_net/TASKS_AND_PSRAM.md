# Task / core affinity and PSRAM (ESP32-S3)

## Dual-core layout

ESP32-S3 has two cores: **Core 0 (PRO_CPU)** and **Core 1 (APP_CPU)**. Tasks are pinned so that HaLow/SPI and driver work stay on Core 0, and application/network work runs on Core 1 to avoid contention and balance load.

| Core | Role | Tasks |
|------|------|--------|
| **0 (CORE_HALOW)** | HaLow/SPI, driver callbacks | `halow_reconn`, `halow_bw` (bandwidth scan) |
| **1 (CORE_APP_NET)** | App, network, HTTP, iperf3 | `halow_mesh`, `dns_fwd`, `gw_beacon`, `log_persist`, `restart` |

- **app_main** runs on Core 1 (IDF default). After it returns, the main task idles; iperf3 and HTTP are callback-driven from lwIP.
- Core affinity is applied only when `portNUM_PROCESSORS > 1`; single-core builds use `xTaskCreate` without pinning.

Definitions: `mm_app_common.h` → `CORE_HALOW` (0), `CORE_APP_NET` (1).

## Task list (stack sizes)

| Task | Stack | Core | Source |
|------|--------|------|--------|
| halow_reconn | 4096 | 0 | mm_app_common.c |
| halow_bw | 3072 | 0 | mm_app_common.c |
| halow_mesh | 4096 | 1 | iperf.c |
| dns_fwd | 4096 | 1 | dns_forwarder.c |
| gw_beacon | 4096 | 1 | esp_now_rcv.c |
| log_persist | 4096 | 1 | esp_now_rcv.c |
| restart | 2048 | 1 | web_config.c |

Main task stack is set in `sdkconfig.defaults`: `CONFIG_ESP_MAIN_TASK_STACK_SIZE=20480`.

## 8MB PSRAM utilization

- **CONFIG_SPIRAM=y** and **CONFIG_SPIRAM_USE_MALLOC=y**: `malloc()` can use PSRAM.
- **CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=512**: Allocations ≤ 512 B stay in internal RAM; larger (e.g. task stacks, LWIP/HaLow buffers) prefer PSRAM.
- **CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y**: Task stacks are allowed in PSRAM (allocated via heap, so they follow the 512 B rule above).
- **CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y**: WiFi/LWIP buffers try PSRAM when possible.
- **CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y**: Some BSS can be placed in PSRAM.
- **CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=81920**: Keeps ~80 KB internal free for DMA/driver needs.

So the 8MB PSRAM is used for: large heap allocations (including task stacks), LWIP/WiFi buffers where possible, and optional BSS. Internal RAM is reserved for small allocs, critical DMA, and the 80 KB reserve. PSRAM size is auto-detected on ESP32-S3; ensure the board has 8MB for full use.
