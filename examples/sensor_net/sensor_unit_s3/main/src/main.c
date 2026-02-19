/**
 * Sensor unit for ESP-Motion (mm-iot-esp32) — ESP-NOW only.
 *
 * Sends sensor packets (motion, BME680, BLE/WiFi scan when enabled) via ESP-NOW
 * to the gateway on 2.4 GHz. No HaLow, no HTTP portal.
 *
 * Build: idf.py set-target esp32s3 && idf.py build
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "packet.h"
#include "esp_now_send.h"
#include "ble_logger.h"

#ifndef CONFIG_FREERTOS_UNICORE
#define SENSOR_APP_CORE_ID  1
#endif

static const char *TAG = "sensor_unit";
#define FW_VERSION "2.0.0"

#define SENSOR_MOTION_POLL_MS     200
/* Send full packet (incl. mmWave) this often so dashboard stays current. Was 60s. */
#define SENSOR_PERIODIC_MS         (2 * 1000)
#define SENSOR_HEARTBEAT_MS        (10 * 1000)

static void log_boot_banner(void)
{
	ESP_LOGI(TAG,
		"\n"
		" ███████╗███████╗███╗   ██╗███████╗ ██████╗ ██████╗\n"
		" ██╔════╝██╔════╝████╗  ██║██╔════╝██╔═══██╗██╔══██╗\n"
		" ███████╗█████╗  ██╔██╗ ██║███████╗██║   ██║██████╔╝\n"
		" ╚════██║██╔══╝  ██║╚██╗██║╚════██║██║   ██║██╔══██╗\n"
		" ███████║███████╗██║ ╚████║███████║╚██████╔╝██║  ██║\n"
		" ╚══════╝╚══════╝╚═╝  ╚═══╝╚══════╝ ╚═════╝ ╚═╝  ╚═╝\n"
		"      N E T   ::   E S P - N O W   S E N S O R   N O D E\n"
		"      version " FW_VERSION "\n");
}

static void sensor_loop_task(void *arg)
{
	(void)arg;
	uint64_t last_periodic_ms = (uint64_t)(esp_timer_get_time() / 1000);
	uint64_t last_heartbeat_ms = last_periodic_ms;
	for (;;) {
		uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
		esp_now_send_packet_on_motion_trigger();
		esp_now_send_packet_on_motion_cleared();
		esp_now_send_wifi_scan_if_due();
		if (now_ms - last_periodic_ms >= SENSOR_PERIODIC_MS) {
			last_periodic_ms = now_ms;
			esp_now_send_packet();
		}
		if (now_ms - last_heartbeat_ms >= SENSOR_HEARTBEAT_MS) {
			last_heartbeat_ms = now_ms;
			ESP_LOGI(TAG, "alive: uptime=%lus free_heap=%lu",
				(unsigned long)(now_ms / 1000ULL),
				(unsigned long)esp_get_free_heap_size());
		}
		vTaskDelay(pdMS_TO_TICKS(SENSOR_MOTION_POLL_MS));
	}
}

void app_main(void)
{
	log_boot_banner();

	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ESP_ERROR_CHECK(nvs_flash_init());
	}
	esp_event_loop_create_default();
	esp_netif_init();

	esp_now_send_init();
	vTaskDelay(pdMS_TO_TICKS(500));

	ESP_LOGI(TAG, "ESP-NOW sensor unit ready [Xiao ESP32-S3-Sense]");

	esp_now_send_packet();
	uint64_t last_periodic_ms = (uint64_t)(esp_timer_get_time() / 1000);
	uint64_t last_heartbeat_ms = last_periodic_ms;

#if (portNUM_PROCESSORS > 1)
	TaskHandle_t sensor_task = NULL;
	if (xTaskCreatePinnedToCore(sensor_loop_task, "sensor_loop",
			4096, NULL, tskIDLE_PRIORITY + 2, &sensor_task, SENSOR_APP_CORE_ID) == pdPASS) {
		ESP_LOGI(TAG, "Sensor loop on core %d", SENSOR_APP_CORE_ID);
		vTaskDelay(portMAX_DELAY);
	}
#endif
	for (;;) {
		uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
		esp_now_send_packet_on_motion_trigger();
		esp_now_send_packet_on_motion_cleared();
		esp_now_send_wifi_scan_if_due();
		if (now_ms - last_periodic_ms >= SENSOR_PERIODIC_MS) {
			last_periodic_ms = now_ms;
			esp_now_send_packet();
		}
		if (now_ms - last_heartbeat_ms >= SENSOR_HEARTBEAT_MS) {
			last_heartbeat_ms = now_ms;
			ESP_LOGI(TAG, "alive: uptime=%lus free_heap=%lu",
				(unsigned long)(now_ms / 1000ULL),
				(unsigned long)esp_get_free_heap_size());
		}
		vTaskDelay(pdMS_TO_TICKS(SENSOR_MOTION_POLL_MS));
	}
}
