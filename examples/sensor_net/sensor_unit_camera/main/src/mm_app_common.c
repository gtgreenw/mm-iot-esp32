/*
 * Copyright 2023 Morse Micro
 * SPDX-License-Identifier: Apache-2.0
 *
 * From mm-iot-esp32 examples; WLAN init/start and IP stack for HaLow STA.
 * HaLow link LED (same as gateway): GPIO 21, boot=solid, connecting=fast flash, connected=1 blink/s.
 */
#include <stdio.h>
#include <stdint.h>
#include "mmosal.h"
#include "mmhal.h"
#include "mmwlan.h"
#include "mmipal.h"
#include "mm_app_common.h"
#include "mm_app_loadconfig.h"
#include "camera_unit_settings.h"
#include "lwip/inet.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "nvs.h"

static const char *TAG = "halow";

/** HaLow link LED: GPIO 21 (Xiao ESP32-S3-Sense built-in). Boot=solid, connecting=fast flash, connected=1 blink/s. Active-low. */
#define HALOW_LED_GPIO         21
#define HALOW_LED_TIMER_MS     100
#define HALOW_LED_BLINK_MS     120
#define HALOW_LED_BLINK_COUNT  3
#define HALOW_LED_ON           0   /* active-low: low = on */
#define HALOW_LED_OFF          1

typedef enum {
	HALOW_LED_BOOT,       /* solid on */
	HALOW_LED_CONNECTING, /* fast flash */
	HALOW_LED_CONNECTED,  /* one blink per second */
} halow_led_state_t;

static volatile halow_led_state_t halow_led_state = HALOW_LED_BOOT;
static TimerHandle_t halow_led_timer = NULL;
static int halow_led_tick = 0;

/** When HaLow firmware fails to boot (e.g. init failure), we continue without HaLow. */
static bool s_halow_available = true;

static void halow_led_timer_cb(TimerHandle_t t)
{
	(void)t;
	switch (halow_led_state) {
	case HALOW_LED_BOOT:
		gpio_set_level(HALOW_LED_GPIO, HALOW_LED_ON);
		break;
	case HALOW_LED_CONNECTING:
		gpio_set_level(HALOW_LED_GPIO, gpio_get_level(HALOW_LED_GPIO) == HALOW_LED_ON ? HALOW_LED_OFF : HALOW_LED_ON);
		break;
	case HALOW_LED_CONNECTED:
		halow_led_tick = (halow_led_tick + 1) % 10;
		gpio_set_level(HALOW_LED_GPIO, halow_led_tick < 5 ? HALOW_LED_ON : HALOW_LED_OFF);
		break;
	}
}

static void halow_led_set_state(halow_led_state_t state)
{
	halow_led_state = state;
	halow_led_tick = 0;
}

static void halow_led_init(void)
{
	bool led_enabled = true;
	camera_unit_settings_get_led_enabled(&led_enabled);

	gpio_config_t io = {
		.pin_bit_mask = (1ULL << HALOW_LED_GPIO),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&io);
	gpio_set_level(HALOW_LED_GPIO, HALOW_LED_OFF);

	if (!led_enabled)
		return;

	halow_led_timer = xTimerCreate("halow_led", pdMS_TO_TICKS(HALOW_LED_TIMER_MS),
	                              pdTRUE, NULL, halow_led_timer_cb);
	if (halow_led_timer != NULL)
		xTimerStart(halow_led_timer, 0);
}

/** Called from blink task when gateway sends blink command; does 3 blinks then resumes link state (active-low). */
void halow_led_request_blink(void)
{
	if (halow_led_timer == NULL) return;
	xTimerStop(halow_led_timer, 0);
	for (int i = 0; i < HALOW_LED_BLINK_COUNT; i++) {
		gpio_set_level(HALOW_LED_GPIO, HALOW_LED_ON);
		vTaskDelay(pdMS_TO_TICKS(HALOW_LED_BLINK_MS));
		gpio_set_level(HALOW_LED_GPIO, HALOW_LED_OFF);
		vTaskDelay(pdMS_TO_TICKS(HALOW_LED_BLINK_MS));
	}
	if (halow_led_timer != NULL)
		xTimerStart(halow_led_timer, 0);
}

static struct mmosal_semb *link_established = NULL;
static bool link_up = false;
static uint32_t ip_addr_u32 = 0;
static uint32_t gw_addr_u32 = 0;
uint8_t mac_addr[MMWLAN_MAC_ADDR_LEN];

static void sta_status_callback(enum mmwlan_sta_state sta_state)
{
	switch (sta_state) {
	case MMWLAN_STA_DISABLED:
		ESP_LOGW(TAG, "STA disabled");
		break;
	case MMWLAN_STA_CONNECTING:
		ESP_LOGW(TAG, "STA connecting...");
		break;
	case MMWLAN_STA_CONNECTED:
		ESP_LOGW(TAG, "STA connected (associating)");
		break;
	}
}

static void link_status_callback(const struct mmipal_link_status *link_status)
{
	if (link_status->link_state == MMIPAL_LINK_UP) {
		link_up = true;
		ip_addr_u32 = ipaddr_addr(link_status->ip_addr);
		gw_addr_u32 = ipaddr_addr(link_status->gateway);
		halow_led_set_state(HALOW_LED_CONNECTED);
		if (link_established != NULL) {
			mmosal_semb_give(link_established);
		}
		app_wlan_arp_send();
		ESP_LOGI(TAG, "Link UP  IP=%s  gateway=%s", link_status->ip_addr, link_status->gateway);
	} else {
		link_up = false;
		halow_led_set_state(HALOW_LED_CONNECTING);
		ESP_LOGW(TAG, "Link DOWN");
	}
}

void app_wlan_init(void)
{
	enum mmwlan_status status;
	struct mmwlan_version version;

	halow_led_init();
	MMOSAL_ASSERT(link_established == NULL);
	link_established = mmosal_semb_create("link_established");
	if (link_established == NULL) {
		ESP_LOGE(TAG, "link_established semaphore create failed; HaLow unavailable.");
		s_halow_available = false;
		return;
	}

	/* Same HaLow init order as gateway (main): RTS, channel list, power save, TX power. */
	mmhal_init();
	/* Let HaLow chip settle after first SPI (reduces finicky boot crashes). */
	vTaskDelay(pdMS_TO_TICKS(600));
	mmwlan_init();
	vTaskDelay(pdMS_TO_TICKS(300));

	/* Disable periodic health checks to reduce SPI traffic and avoid health task stack overflow. */
	(void)mmwlan_set_health_check_interval(0, 0);

	status = mmwlan_set_rts_threshold(2347);
	if (status != MMWLAN_SUCCESS)
		ESP_LOGW(TAG, "mmwlan_set_rts_threshold(2347) failed (%d)", (int)status);
	mmwlan_set_channel_list(load_channel_list());
	status = mmwlan_set_power_save_mode(MMWLAN_PS_DISABLED);
	if (status != MMWLAN_SUCCESS)
		ESP_LOGW(TAG, "mmwlan_set_power_save_mode failed (%d)", (int)status);
	/* HaLow TX power override from NVS (camera unit settings). 0 = regulatory default. */
	{
		nvs_handle_t h;
		if (nvs_open("sensor", NVS_READONLY, &h) == ESP_OK) {
			uint16_t halow_txp = 0;
			if (nvs_get_u16(h, "halow_txp", &halow_txp) == ESP_OK && halow_txp != 0) {
				status = mmwlan_override_max_tx_power(halow_txp);
				if (status != MMWLAN_SUCCESS)
					ESP_LOGW(TAG, "HaLow TX power override failed (%d)", (int)status);
				else
					ESP_LOGI(TAG, "HaLow TX power set to %u dBm", (unsigned)halow_txp);
			}
			nvs_close(h);
		}
	}

	struct mmipal_init_args mmipal_init_args = MMIPAL_INIT_ARGS_DEFAULT;
	load_mmipal_init_args(&mmipal_init_args);
	if (mmipal_init(&mmipal_init_args) != MMIPAL_SUCCESS) {
		ESP_LOGE(TAG, "mmipal_init failed; HaLow unavailable.");
		s_halow_available = false;
		return;
	}
	mmipal_set_link_status_callback(link_status_callback);

	status = mmwlan_get_version(&version);
	if (status != MMWLAN_SUCCESS) {
		ESP_LOGE(TAG, "get_version failed (%d); HaLow unavailable (e.g. firmware did not boot).", (int)status);
		s_halow_available = false;
		return;
	}
	status = mmwlan_get_mac_addr(mac_addr);
	if (status != MMWLAN_SUCCESS) {
		ESP_LOGE(TAG, "get_mac_addr failed (%d); HaLow unavailable.", (int)status);
		s_halow_available = false;
		return;
	}
}

void app_wlan_start(void)
{
	(void)app_wlan_start_with_timeout(UINT32_MAX);
}

bool app_wlan_start_with_timeout(uint32_t timeout_ms)
{
	enum mmwlan_status status;
	struct mmwlan_sta_args sta_args = MMWLAN_STA_ARGS_INIT;

	if (!s_halow_available) {
		ESP_LOGW(TAG, "Skipping connect (HaLow unavailable).");
		return false;
	}

	load_mmwlan_sta_args(&sta_args);
	load_mmwlan_settings();

	ESP_LOGI(TAG, "Connecting to %s %s", sta_args.ssid, sta_args.security_type == MMWLAN_SAE ? "(SAE)" : "");
	if (timeout_ms != UINT32_MAX)
		ESP_LOGI(TAG, "Timeout %lu ms", (unsigned long)timeout_ms);

	halow_led_set_state(HALOW_LED_CONNECTING);
	status = mmwlan_sta_enable(&sta_args, sta_status_callback);
	if (status != MMWLAN_SUCCESS) {
		ESP_LOGE(TAG, "sta_enable failed (%d); HaLow unavailable.", (int)status);
		s_halow_available = false;
		return false;
	}

	if (link_established == NULL) {
		ESP_LOGE(TAG, "link_established NULL; cannot wait for link.");
		return false;
	}
	/* Wait in short chunks so we yield and feed the task watchdog during connect */
#define HALOW_WAIT_CHUNK_MS  500
	uint32_t waited_ms = 0;
	while (waited_ms < timeout_ms) {
		uint32_t chunk_ms = (timeout_ms - waited_ms) < HALOW_WAIT_CHUNK_MS
				    ? (timeout_ms - waited_ms) : HALOW_WAIT_CHUNK_MS;
		if (mmosal_semb_wait(link_established, chunk_ms)) {
			return link_up;
		}
		waited_ms += chunk_ms;
		vTaskDelay(pdMS_TO_TICKS(1));  /* yield to idle / watchdog */
	}
	return link_up;
}

bool app_wlan_halow_available(void)
{
	return s_halow_available;
}

void app_wlan_stop(void)
{
	mmwlan_shutdown();
}

void app_wlan_arp_send(void)
{
	if (!link_up) return;
	uint8_t arp_packet[] = {
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
		0x08, 0x06,
		0x00, 0x01, 0x08, 0x00, 0x06, 0x04, 0x00, 0x01,
		mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
		((uint8_t *)&ip_addr_u32)[0], ((uint8_t *)&ip_addr_u32)[1], ((uint8_t *)&ip_addr_u32)[2], ((uint8_t *)&ip_addr_u32)[3],
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		((uint8_t *)&gw_addr_u32)[0], ((uint8_t *)&gw_addr_u32)[1], ((uint8_t *)&gw_addr_u32)[2], ((uint8_t *)&gw_addr_u32)[3],
	};
	mmwlan_tx(arp_packet, sizeof(arp_packet));
}
