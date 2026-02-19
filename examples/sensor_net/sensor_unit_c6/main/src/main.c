/**
 * Sensor unit for ESP-Motion (ESP32-C6).
 *
 * Minimal ESP-NOW sender (no HaLow). Periodically sends sensor packets on channel 6.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_now_send.h"
#include "ble_logger.h"

static const char *TAG = "sensor_unit_c6";
#define FW_VERSION "1.0.1"

#define SENSOR_SEND_INTERVAL_MS   2000

/* Seeed Studio XIAO ESP32-C6: FM8625H RF switch. GPIO3 = power (LOW = on), GPIO14 = select (HIGH = external). */
#define XIAO_C6_ANTENNA_GPIO_POWER  3
#define XIAO_C6_ANTENNA_GPIO_SELECT 14

static const char *CYBERPUNK_BANNER =
	"\n"
	" ███████╗███████╗███╗   ██╗███████╗ ██████╗ ██████╗\n"
	" ██╔════╝██╔════╝████╗  ██║██╔════╝██╔═══██╗██╔══██╗\n"
	" ███████╗█████╗  ██╔██╗ ██║███████╗██║   ██║██████╔╝\n"
	" ╚════██║██╔══╝  ██║╚██╗██║╚════██║██║   ██║██╔══██╗\n"
	" ███████║███████╗██║ ╚████║███████║╚██████╔╝██║  ██║\n"
	" ╚══════╝╚══════╝╚═╝  ╚═══╝╚══════╝ ╚═════╝ ╚═╝  ╚═╝\n"
	"      N E T   ::   E S P - N O W   S E N S O R   N O D E\n"
	"      channel 6 uplink | live telemetry | cyberpunk mode\n";

void app_main(void)
{
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ESP_ERROR_CHECK(nvs_flash_init());
	}

#if CONFIG_SENSOR_C6_EXTERNAL_ANTENNA
	/* Seeed Studio XIAO ESP32-C6: power RF switch (GPIO3 LOW) and select external antenna (GPIO14 HIGH). */
	gpio_config_t io = {
		.pin_bit_mask = (1ULL << XIAO_C6_ANTENNA_GPIO_POWER) | (1ULL << XIAO_C6_ANTENNA_GPIO_SELECT),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&io);
	gpio_set_level(XIAO_C6_ANTENNA_GPIO_POWER, 0);   /* LOW = RF switch powered */
	gpio_set_level(XIAO_C6_ANTENNA_GPIO_SELECT, 1);  /* HIGH = external antenna */
	ESP_LOGI(TAG, "External antenna enabled (GPIO%d=LOW, GPIO%d=HIGH)", XIAO_C6_ANTENNA_GPIO_POWER, XIAO_C6_ANTENNA_GPIO_SELECT);
#endif

	esp_event_loop_create_default();
	if (esp_netif_init() != ESP_OK) {
		ESP_LOGE(TAG, "esp_netif_init failed");
		return;
	}

#if CONFIG_SENSOR_BLE_LOG_ENABLE
	ble_logger_start();
#endif

	esp_now_send_init();
	ESP_LOGI(TAG, "%s", CYBERPUNK_BANNER);
	ESP_LOGI(TAG, "Version %s", FW_VERSION);
	ESP_LOGI(TAG, "SENSOR NET sensor unit (ESP-NOW) [Xiao ESP32-C6]");
	while (1) {
		esp_now_send_packet();
		vTaskDelay(pdMS_TO_TICKS(SENSOR_SEND_INTERVAL_MS));
	}
}
