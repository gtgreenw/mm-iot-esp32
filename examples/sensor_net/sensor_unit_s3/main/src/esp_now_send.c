/**
 * ESP-NOW sender for sensor unit: sends sensor_packet_t to broadcast (gateway receives on 2.4 GHz).
 * Also receives gateway→node command packets (e.g. blink LED). Uses same channel (6) as gateway AP.
 */
#include "packet.h"
#include "ble_logger.h"
#include "wifi_logger.h"
#include "ld2410.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_oneshot.h"
#include "bme68x.h"
#include "ds18b20.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "esp_now_send";

#define ESPNOW_CHANNEL_DEFAULT CONFIG_ESPNOW_CHANNEL
#define ESPNOW_SCAN_CHANNEL_MIN 1
#define ESPNOW_SCAN_CHANNEL_MAX 14
#define ESPNOW_SCAN_WAIT_MS    400
#define BLINK_MS 120
#define BLINK_COUNT 3
#define BME_POLL_INTERVAL_MS 5000
#define DS18B20_POLL_INTERVAL_MS 5000
#ifndef CONFIG_SENSOR_MOTION_CONFIRM_MS
#define CONFIG_SENSOR_MOTION_CONFIRM_MS 500
#endif
#define MAX_PEERS_SEEN  32
#define PEER_STALE_MS   (5 * 60 * 1000)  /* 5 min */

static uint8_t s_broadcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t s_self_mac[6] = { 0 };
static bool s_esp_now_ok = false;
/** Current ESP-NOW channel (fixed or from scan). */
static uint8_t s_espnow_channel = ESPNOW_CHANNEL_DEFAULT;
#if CONFIG_SENSOR_ESPNOW_CHANNEL_SCAN
static SemaphoreHandle_t s_scan_ack_sem = NULL;
static volatile bool s_scanning = false;
#endif

typedef struct {
	uint8_t mac[6];
	uint32_t last_seen_ms;
} peer_seen_t;
static peer_seen_t s_peers_seen[MAX_PEERS_SEEN];
static SemaphoreHandle_t s_peers_mux = NULL;
static nvs_handle_t s_nvs = 0;
static uint32_t s_last_motion_ms = 0;
static uint32_t s_trigger_count = 0;
static uint8_t s_prev_motion = 0;
static uint32_t s_send_count = 0;
/** When motion went high (ms); 0 = not in confirm window. Used for debounce. */
static uint32_t s_motion_high_since_ms = 0;
static uint64_t s_last_bme_poll_ms = 0;
static bool s_has_bme_cache = false;
static float s_cached_temperature = 0.0f;
static float s_cached_humidity = 0.0f;
static float s_cached_pressure = 0.0f;
static float s_cached_gas = 0.0f;
static bool s_motion_ready = false;
static bool s_bme_ready = false;
#if CONFIG_SENSOR_MOTION_MMWAVE
static uint64_t s_last_mmwave_nodata_log_ms = 0;
#endif
#if CONFIG_SENSOR_MOISTURE_ENABLE
static adc_oneshot_unit_handle_t s_moisture_adc_handle = NULL;
static int s_moisture_num_channels = 0;
static adc_channel_t s_moisture_channels[SENSOR_MOISTURE_CHANNELS];
static bool s_moisture_ready[SENSOR_MOISTURE_CHANNELS];
#endif
#if (CONFIG_SENSOR_DS18B20_GPIO >= 0)
static uint64_t s_last_ds18b20_poll_ms = 0;
static float s_cached_ds18b20_temp = 0.0f;
static bool s_has_ds18b20_cache = false;
#endif
#if CONFIG_SENSOR_TDS_ENABLE
static adc_oneshot_unit_handle_t s_tds_adc_handle = NULL;
static adc_channel_t s_tds_channel = 0;
static bool s_tds_ready = false;
#endif

static uint8_t read_motion_level(void);

typedef struct {
	i2c_port_t port;
	uint8_t addr;
} bme68x_i2c_ctx_t;

static bme68x_i2c_ctx_t s_bme_ctx;
static struct bme68x_dev s_bme_dev;
static struct bme68x_conf s_bme_conf;
static struct bme68x_heatr_conf s_bme_heatr;

#define NVS_NAMESPACE "sensor"
#define NVS_LAST_MOTION_KEY "last_motion"
/** Trigger count persisted here and sent in every sensor_packet_t to gateway. */
#define NVS_TRIGGER_COUNT_KEY "trigger_count"
#define NVS_KEY_ESPNOW_EN   "espnow_en"
#define NVS_BLE_LOG_KEY     "ble_log"
#define NVS_WIFI_LOG_KEY    "wifi_log"
#define NVS_ESPNOW_CHANNEL_KEY "espnow_ch"
#define NVS_LABEL_KEY "label"
#define NVS_IS_OUTDOOR_KEY "outdoor"
#define DEFAULT_ESPNOW_ENABLED  1

static QueueHandle_t s_blink_queue = NULL;
/** Label and is_outdoor persisted in NVS; loaded at boot and on SET_LABEL/SET_LOCATION. */
static char s_label[SENSOR_LABEL_MAX];
static uint8_t s_is_outdoor = 0;

/* Xiao ESP32-S3-Sense built-in LED is active-low: low = on, high = off */
#if CONFIG_SENSOR_LED_GPIO >= 0
#define LED_ON  0
#define LED_OFF 1

static void blink_task(void *arg)
{
	(void)arg;
	int level = 0;
	for (;;) {
		if (xQueueReceive(s_blink_queue, &level, portMAX_DELAY) != pdTRUE)
			continue;
		for (int i = 0; i < BLINK_COUNT; i++) {
			gpio_set_level(CONFIG_SENSOR_LED_GPIO, LED_ON);
			vTaskDelay(pdMS_TO_TICKS(BLINK_MS));
			gpio_set_level(CONFIG_SENSOR_LED_GPIO, LED_OFF);
			vTaskDelay(pdMS_TO_TICKS(BLINK_MS));
		}
	}
}

#endif

static void persist_motion_state(void)
{
	if (!s_nvs)
		return;
	nvs_set_u32(s_nvs, NVS_LAST_MOTION_KEY, s_last_motion_ms);
	nvs_set_u32(s_nvs, NVS_TRIGGER_COUNT_KEY, s_trigger_count);
	nvs_commit(s_nvs);
}

static void record_peer_seen(const uint8_t *mac, uint32_t now_ms)
{
	if (!s_peers_mux || !mac)
		return;
	if (xSemaphoreTake(s_peers_mux, pdMS_TO_TICKS(50)) != pdTRUE)
		return;
	int slot = -1;
	int oldest = -1;
	uint32_t oldest_ms = 0;
	for (int i = 0; i < MAX_PEERS_SEEN; i++) {
		if (memcmp(s_peers_seen[i].mac, mac, 6) == 0) {
			slot = i;
			break;
		}
		if (s_peers_seen[i].last_seen_ms == 0) {
			if (slot < 0)
				slot = i;
			break;
		}
		if (oldest < 0 || s_peers_seen[i].last_seen_ms < oldest_ms) {
			oldest = i;
			oldest_ms = s_peers_seen[i].last_seen_ms;
		}
	}
	if (slot < 0 && oldest >= 0)
		slot = oldest;
	if (slot >= 0) {
		memcpy(s_peers_seen[slot].mac, mac, 6);
		s_peers_seen[slot].last_seen_ms = now_ms;
	}
	xSemaphoreGive(s_peers_mux);
}

#if CONFIG_SENSOR_ESPNOW_CHANNEL_SCAN
/* Scan ACK = receive gateway beacon (in recv_cb), not send success (send success is unreliable). */
static void esp_now_scan_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	(void)mac_addr;
	(void)status;
}
#endif

static void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
	if (!info || !data)
		return;
#if CONFIG_SENSOR_ESPNOW_CHANNEL_SCAN
	/* During channel scan: gateway beacon = ACK; lock on this channel. */
	if (s_scanning && s_scan_ack_sem != NULL && len >= 2 && data[0] == GATEWAY_PACKET_MAGIC) {
		xSemaphoreGive(s_scan_ack_sem);
		return;
	}
#endif
	/* Count other ESP-NOW sensors we hear (sensor packets from other MACs) */
	if (len >= (int)SENSOR_PACKET_SIZE && data[0] == SENSOR_PACKET_MAGIC) {
		if (memcmp(info->src_addr, s_self_mac, 6) != 0) {
			uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
			record_peer_seen(info->src_addr, now_ms);
		}
		return;
	}
	if (len < (int)CMD_PACKET_SIZE)
		return;
	if (data[0] != CMD_PACKET_MAGIC)
		return;
	if (data[1] == CMD_TYPE_BLINK) {
		if (s_blink_queue) {
			int dummy = 0;
			xQueueSend(s_blink_queue, &dummy, 0);
		}
		return;
	}
	if (data[1] == CMD_TYPE_RESET) {
		s_trigger_count = 0;
		s_last_motion_ms = 0;
		s_prev_motion = read_motion_level();
		s_motion_high_since_ms = 0;
		persist_motion_state();
		return;
	}
	if (len >= (int)CMD_BLE_LOG_PACKET_SIZE && data[1] == CMD_TYPE_SET_BLE_LOG) {
		uint8_t enabled = data[2];
		if (enabled)
			ble_logger_start();
		else
			ble_logger_stop();
		if (s_nvs) {
			nvs_set_u8(s_nvs, NVS_BLE_LOG_KEY, enabled ? 1 : 0);
			nvs_commit(s_nvs);
		}
		ESP_LOGI(TAG, "BLE log %s (from gateway)", enabled ? "on" : "off");
		return;
	}
	if (len >= (int)CMD_WIFI_LOG_PACKET_SIZE && data[1] == CMD_TYPE_SET_WIFI_LOG) {
		uint8_t enabled = data[2];
		if (enabled)
			wifi_logger_start();
		else
			wifi_logger_stop();
		if (s_nvs) {
			nvs_set_u8(s_nvs, NVS_WIFI_LOG_KEY, enabled ? 1 : 0);
			nvs_commit(s_nvs);
		}
		ESP_LOGI(TAG, "WiFi log %s (from gateway)", enabled ? "on" : "off");
		return;
	}
	if (len >= (int)CMD_LABEL_PACKET_SIZE && data[1] == CMD_TYPE_SET_LABEL) {
		const cmd_label_packet_t *cmd = (const cmd_label_packet_t *)data;
		strncpy(s_label, cmd->label, sizeof(s_label) - 1);
		s_label[sizeof(s_label) - 1] = '\0';
		if (s_nvs) {
			nvs_set_str(s_nvs, NVS_LABEL_KEY, s_label);
			nvs_commit(s_nvs);
		}
		ESP_LOGI(TAG, "Label set to \"%s\" (from gateway)", s_label);
		return;
	}
	if (len >= (int)CMD_LOCATION_PACKET_SIZE && data[1] == CMD_TYPE_SET_LOCATION) {
		const cmd_location_packet_t *cmd = (const cmd_location_packet_t *)data;
		s_is_outdoor = (cmd->is_outdoor != 0) ? 1 : 0;
		if (s_nvs) {
			nvs_set_u8(s_nvs, NVS_IS_OUTDOOR_KEY, s_is_outdoor);
			nvs_commit(s_nvs);
		}
		ESP_LOGI(TAG, "Location set to %s (from gateway)", s_is_outdoor ? "outdoor" : "indoor");
		return;
	}
}

#if CONFIG_SENSOR_ESPNOW_CHANNEL_SCAN
/** Try one channel: set WiFi channel, add broadcast peer, send probe, wait for ACK. Returns true if ACK received. */
static bool try_channel_and_wait_ack(uint8_t channel)
{
	esp_wifi_set_channel((int)channel, WIFI_SECOND_CHAN_NONE);
	vTaskDelay(pdMS_TO_TICKS(10));
	esp_now_del_peer(s_broadcast_mac);
	esp_now_peer_info_t peer = { 0 };
	memcpy(peer.peer_addr, s_broadcast_mac, 6);
	peer.channel = channel;
	peer.ifidx = WIFI_IF_STA;
	peer.encrypt = false;
	esp_err_t err = esp_now_add_peer(&peer);
	if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
		return false;
	}
	sensor_packet_t probe = { 0 };
	probe.magic = SENSOR_PACKET_MAGIC;
	probe.version = SENSOR_PACKET_VERSION;
	s_scanning = true;
	xSemaphoreTake(s_scan_ack_sem, 0); /* clear any stale */
	err = esp_now_send(s_broadcast_mac, (const uint8_t *)&probe, SENSOR_PACKET_SIZE);
	if (err != ESP_OK) {
		s_scanning = false;
		return false;
	}
	BaseType_t ack = xSemaphoreTake(s_scan_ack_sem, pdMS_TO_TICKS(ESPNOW_SCAN_WAIT_MS));
	s_scanning = false;
	return (ack == pdTRUE);
}
#endif

static void wifi_init_esp_now(void)
{
	memset(s_peers_seen, 0, sizeof(s_peers_seen));
	if (s_peers_mux == NULL)
		s_peers_mux = xSemaphoreCreateMutex();
	esp_netif_create_default_wifi_sta();
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);
	esp_wifi_set_storage(WIFI_STORAGE_RAM);
	esp_wifi_set_mode(WIFI_MODE_STA);
	esp_wifi_start();
	esp_wifi_get_mac(WIFI_IF_STA, s_self_mac);
	vTaskDelay(pdMS_TO_TICKS(200));
	esp_err_t err = esp_now_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
		return;
	}
	/* Register recv_cb before channel scan so we receive gateway beacon (same as sensor_unit_c6). */
	esp_now_register_recv_cb(esp_now_recv_cb);

#if CONFIG_SENSOR_ESPNOW_CHANNEL_SCAN
	s_espnow_channel = 0;
	s_scan_ack_sem = xSemaphoreCreateBinary();
	if (s_scan_ack_sem != NULL) {
		esp_now_register_send_cb(esp_now_scan_send_cb);
		uint8_t last_ch = 0;
		if (s_nvs && nvs_get_u8(s_nvs, NVS_ESPNOW_CHANNEL_KEY, &last_ch) == ESP_OK &&
		    last_ch >= ESPNOW_SCAN_CHANNEL_MIN && last_ch <= ESPNOW_SCAN_CHANNEL_MAX) {
			if (try_channel_and_wait_ack(last_ch)) {
				s_espnow_channel = last_ch;
				ESP_LOGI(TAG, "ESP-NOW channel %u (from NVS, ACK ok)", (unsigned)last_ch);
				goto scan_done;
			}
		}
		for (uint8_t ch = ESPNOW_SCAN_CHANNEL_MIN; ch <= ESPNOW_SCAN_CHANNEL_MAX; ch++) {
			if (try_channel_and_wait_ack(ch)) {
				s_espnow_channel = ch;
				if (s_nvs) {
					nvs_set_u8(s_nvs, NVS_ESPNOW_CHANNEL_KEY, ch);
					nvs_commit(s_nvs);
				}
				ESP_LOGI(TAG, "ESP-NOW channel %u (scan ACK)", (unsigned)ch);
				break;
			}
		}
		if (s_espnow_channel < ESPNOW_SCAN_CHANNEL_MIN || s_espnow_channel > ESPNOW_SCAN_CHANNEL_MAX) {
			s_espnow_channel = (uint8_t)ESPNOW_CHANNEL_DEFAULT;
			ESP_LOGW(TAG, "No gateway ACK on 1-%d; using channel %d", ESPNOW_SCAN_CHANNEL_MAX, (int)s_espnow_channel);
		}
scan_done:
		esp_now_unregister_send_cb();
		vSemaphoreDelete(s_scan_ack_sem);
		s_scan_ack_sem = NULL;
	} else {
		s_espnow_channel = (uint8_t)ESPNOW_CHANNEL_DEFAULT;
	}
#else
	s_espnow_channel = (uint8_t)ESPNOW_CHANNEL_DEFAULT;
#endif

	esp_wifi_set_channel((int)s_espnow_channel, WIFI_SECOND_CHAN_NONE);
	vTaskDelay(pdMS_TO_TICKS(50));
	esp_now_del_peer(s_broadcast_mac);
	esp_now_peer_info_t peer = { 0 };
	memcpy(peer.peer_addr, s_broadcast_mac, 6);
	peer.channel = s_espnow_channel;
	peer.ifidx = WIFI_IF_STA;
	peer.encrypt = false;
	err = esp_now_add_peer(&peer);
	if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
		ESP_LOGE(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(err));
		return;
	}
#if CONFIG_SENSOR_LED_GPIO >= 0
	s_blink_queue = xQueueCreate(2, sizeof(int));
	if (s_blink_queue != NULL) {
		gpio_config_t io = {
			.pin_bit_mask = (1ULL << CONFIG_SENSOR_LED_GPIO),
			.mode = GPIO_MODE_OUTPUT,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE,
		};
		gpio_config(&io);
		gpio_set_level(CONFIG_SENSOR_LED_GPIO, LED_OFF);
		/* Boot indication: two short blinks (active-low LED) */
		for (int i = 0; i < 2; i++) {
			gpio_set_level(CONFIG_SENSOR_LED_GPIO, LED_ON);
			vTaskDelay(pdMS_TO_TICKS(120));
			gpio_set_level(CONFIG_SENSOR_LED_GPIO, LED_OFF);
			vTaskDelay(pdMS_TO_TICKS(100));
		}
#if (portNUM_PROCESSORS > 1)
		xTaskCreatePinnedToCore(blink_task, "blink", 1536, NULL, 5, NULL, 1);  /* APP_CPU */
#else
		xTaskCreate(blink_task, "blink", 1536, NULL, 5, NULL);
#endif
		ESP_LOGI(TAG, "LED blink on GPIO %d (gateway command)", CONFIG_SENSOR_LED_GPIO);
	}
#endif
	s_esp_now_ok = true;
	ESP_LOGI(TAG, "ESP-NOW sender ready (channel %d)", (int)s_espnow_channel);
}

static void esp_now_send_stop(void)
{
	if (!s_esp_now_ok)
		return;
	esp_now_unregister_recv_cb();
	esp_now_deinit();
	s_esp_now_ok = false;
	ESP_LOGI(TAG, "ESP-NOW disabled");
}

static int8_t bme_i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr)
{
	const bme68x_i2c_ctx_t *ctx = (const bme68x_i2c_ctx_t *)intf_ptr;
	if (!ctx || !data || len == 0) {
		return BME68X_E_NULL_PTR;
	}
	esp_err_t err = i2c_master_write_read_device(ctx->port, ctx->addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
	return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static int8_t bme_i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr)
{
	const bme68x_i2c_ctx_t *ctx = (const bme68x_i2c_ctx_t *)intf_ptr;
	if (!ctx || !data || len == 0) {
		return BME68X_E_NULL_PTR;
	}
	uint8_t buf[1 + 32];
	if (len > sizeof(buf) - 1) {
		return BME68X_E_INVALID_LENGTH;
	}
	buf[0] = reg;
	memcpy(&buf[1], data, len);
	esp_err_t err = i2c_master_write_to_device(ctx->port, ctx->addr, buf, len + 1, pdMS_TO_TICKS(100));
	return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static void bme_delay_us(uint32_t period, void *intf_ptr)
{
	(void)intf_ptr;
	if (period >= 1000) {
		vTaskDelay(pdMS_TO_TICKS((period + 999) / 1000));
	} else {
		esp_rom_delay_us(period);
	}
}

static void bme680_init(void)
{
	int sda = CONFIG_SENSOR_BME_I2C_SDA_GPIO;
	int scl = CONFIG_SENSOR_BME_I2C_SCL_GPIO;
	if (sda < 0 || scl < 0) {
		ESP_LOGW(TAG, "BME680 disabled (SDA/SCL set to -1). Set SDA/SCL in menuconfig (e.g. D4=4 D5=5) to enable.");
		return;
	}
	if (sda > 48 || scl > 48) {
		ESP_LOGW(TAG, "BME680 disabled: SDA=%d SCL=%d invalid (use 0-48). Set correct GPIOs in menuconfig.", sda, scl);
		return;
	}
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = sda,
		.scl_io_num = scl,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = CONFIG_SENSOR_BME_I2C_FREQ_HZ,
	};
	esp_err_t err = i2c_param_config(CONFIG_SENSOR_BME_I2C_PORT, &conf);
	if (err != ESP_OK) {
		ESP_LOGW(TAG, "BME680 I2C param_config failed: %s", esp_err_to_name(err));
		return;
	}
	err = i2c_driver_install(CONFIG_SENSOR_BME_I2C_PORT, conf.mode, 0, 0, 0);
	if (err != ESP_OK) {
		ESP_LOGW(TAG, "BME680 I2C driver_install failed: %s (no BME or conflict)", esp_err_to_name(err));
		return;
	}
	s_bme_ctx.port = CONFIG_SENSOR_BME_I2C_PORT;
	s_bme_ctx.addr = (uint8_t)CONFIG_SENSOR_BME_I2C_ADDR;
	s_bme_dev.intf = BME68X_I2C_INTF;
	s_bme_dev.intf_ptr = &s_bme_ctx;
	s_bme_dev.read = bme_i2c_read;
	s_bme_dev.write = bme_i2c_write;
	s_bme_dev.delay_us = bme_delay_us;
	s_bme_dev.amb_temp = 25;
	int8_t rslt = bme68x_init(&s_bme_dev);
	if (rslt != BME68X_OK) {
		ESP_LOGW(TAG, "BME680 init failed: %d", (int)rslt);
		return;
	}
	s_bme_conf.filter = BME68X_FILTER_SIZE_3;
	s_bme_conf.os_temp = BME68X_OS_8X;
	s_bme_conf.os_pres = BME68X_OS_4X;
	s_bme_conf.os_hum = BME68X_OS_2X;
	s_bme_conf.odr = BME68X_ODR_NONE;
	rslt = bme68x_set_conf(&s_bme_conf, &s_bme_dev);
	if (rslt != BME68X_OK) {
		ESP_LOGW(TAG, "BME680 set_conf failed: %d", (int)rslt);
		return;
	}
	s_bme_heatr.enable = BME68X_ENABLE;
	s_bme_heatr.heatr_temp = 320;
	s_bme_heatr.heatr_dur = 150;
	rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &s_bme_heatr, &s_bme_dev);
	if (rslt != BME68X_OK) {
		ESP_LOGW(TAG, "BME680 heater config failed: %d", (int)rslt);
		return;
	}
	s_bme_ready = true;
	ESP_LOGI(TAG, "BME680 ready on I2C addr 0x%02X", (unsigned)s_bme_ctx.addr);
}

#if CONFIG_SENSOR_MOISTURE_ENABLE
static void moisture_init(void)
{
	adc_oneshot_unit_init_cfg_t unit_cfg = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	if (adc_oneshot_new_unit(&unit_cfg, &s_moisture_adc_handle) != ESP_OK) {
		ESP_LOGW(TAG, "Moisture ADC unit init failed");
		return;
	}
	adc_oneshot_chan_cfg_t chan_cfg = {
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_12,
	};
	const int adc_channels[] = {
		CONFIG_SENSOR_MOISTURE_CH0_ADC,
#if CONFIG_SENSOR_MOISTURE_NUM_CHANNELS > 1
		CONFIG_SENSOR_MOISTURE_CH1_ADC,
#endif
#if CONFIG_SENSOR_MOISTURE_NUM_CHANNELS > 2
		CONFIG_SENSOR_MOISTURE_CH2_ADC,
#endif
#if CONFIG_SENSOR_MOISTURE_NUM_CHANNELS > 3
		CONFIG_SENSOR_MOISTURE_CH3_ADC,
#endif
	};
	s_moisture_num_channels = (int)(sizeof(adc_channels) / sizeof(adc_channels[0]));
	if (s_moisture_num_channels > SENSOR_MOISTURE_CHANNELS)
		s_moisture_num_channels = SENSOR_MOISTURE_CHANNELS;
	for (int i = 0; i < SENSOR_MOISTURE_CHANNELS; i++)
		s_moisture_ready[i] = false;
	for (int i = 0; i < s_moisture_num_channels; i++) {
		s_moisture_channels[i] = (adc_channel_t)adc_channels[i];
		if (adc_oneshot_config_channel(s_moisture_adc_handle, s_moisture_channels[i], &chan_cfg) != ESP_OK) {
			ESP_LOGW(TAG, "Moisture ADC channel %d config failed", adc_channels[i]);
			continue;
		}
		s_moisture_ready[i] = true;
		ESP_LOGI(TAG, "Moisture sensor %d ready on ADC1 channel %d (D%d on XIAO S3)", i + 1, adc_channels[i], i);
	}
}

static int read_moisture_raw(int idx)
{
	if (idx < 0 || idx >= s_moisture_num_channels || !s_moisture_ready[idx] || s_moisture_adc_handle == NULL)
		return -1;
	int raw = 0;
	if (adc_oneshot_read(s_moisture_adc_handle, s_moisture_channels[idx], &raw) != ESP_OK)
		return -1;
	return raw;
}

static void get_moisture_cal(int idx, int *out_dry, int *out_wet)
{
	int dry = 2700, wet = 1000;
	switch (idx) {
	case 0: dry = CONFIG_SENSOR_MOISTURE_RAW_DRY_CH0; wet = CONFIG_SENSOR_MOISTURE_RAW_WET_CH0; break;
	case 1: dry = CONFIG_SENSOR_MOISTURE_RAW_DRY_CH1; wet = CONFIG_SENSOR_MOISTURE_RAW_WET_CH1; break;
#if CONFIG_SENSOR_MOISTURE_NUM_CHANNELS > 2
	case 2: dry = CONFIG_SENSOR_MOISTURE_RAW_DRY_CH2; wet = CONFIG_SENSOR_MOISTURE_RAW_WET_CH2; break;
#endif
#if CONFIG_SENSOR_MOISTURE_NUM_CHANNELS > 3
	case 3: dry = CONFIG_SENSOR_MOISTURE_RAW_DRY_CH3; wet = CONFIG_SENSOR_MOISTURE_RAW_WET_CH3; break;
#endif
	default: break;
	}
	*out_dry = dry;
	*out_wet = wet;
}

static float read_moisture_channel(int idx)
{
	int raw = read_moisture_raw(idx);
	if (raw < 0) return -1.0f;
	if (raw > 4095) raw = 4095;
	int raw_dry, raw_wet;
	get_moisture_cal(idx, &raw_dry, &raw_wet);
	if (raw_dry <= raw_wet)
		return (4095 - raw) * 100.0f / 4095.0f;
	float pct = (float)(raw_dry - raw) / (float)(raw_dry - raw_wet) * 100.0f;
	if (pct < 0.0f) pct = 0.0f;
	if (pct > 100.0f) pct = 100.0f;
	return pct;
}
#endif

#if CONFIG_SENSOR_TDS_ENABLE
static void tds_init(void)
{
	adc_oneshot_chan_cfg_t chan_cfg = {
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_12,
	};
	s_tds_channel = (adc_channel_t)CONFIG_SENSOR_TDS_ADC_CHANNEL;

#if CONFIG_SENSOR_MOISTURE_ENABLE
	if (s_moisture_adc_handle != NULL) {
		if (adc_oneshot_config_channel(s_moisture_adc_handle, s_tds_channel, &chan_cfg) == ESP_OK) {
			s_tds_adc_handle = s_moisture_adc_handle;
			s_tds_ready = true;
			ESP_LOGI(TAG, "TDS sensor on ADC1 channel %d (shared with moisture)", (int)s_tds_channel);
		} else {
			ESP_LOGW(TAG, "TDS channel %d config failed", (int)s_tds_channel);
		}
		return;
	}
#endif
	adc_oneshot_unit_init_cfg_t unit_cfg = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	if (adc_oneshot_new_unit(&unit_cfg, &s_tds_adc_handle) != ESP_OK) {
		ESP_LOGW(TAG, "TDS ADC unit init failed");
		return;
	}
	if (adc_oneshot_config_channel(s_tds_adc_handle, s_tds_channel, &chan_cfg) != ESP_OK) {
		ESP_LOGW(TAG, "TDS channel %d config failed", (int)s_tds_channel);
		return;
	}
	s_tds_ready = true;
	ESP_LOGI(TAG, "TDS sensor ready on ADC1 channel %d (XIAO S3)", (int)s_tds_channel);
}

static float read_tds_ppm(void)
{
	if (!s_tds_ready || s_tds_adc_handle == NULL)
		return SENSOR_TDS_INVALID;
	int raw = 0;
	if (adc_oneshot_read(s_tds_adc_handle, s_tds_channel, &raw) != ESP_OK)
		return SENSOR_TDS_INVALID;
	if (raw <= 0)
		return 0.0f;
	float v = (float)raw * 3.3f / 4095.0f;
	float ppm = v * (float)CONFIG_SENSOR_TDS_PPM_PER_VOLT;
	if (ppm < 0.0f) ppm = 0.0f;
	return ppm;
}
#endif

__attribute__((unused))
static void motion_gpio_init(void)
{
#if CONFIG_SENSOR_MOTION_GPIO_SRC
	int motion_gpio = CONFIG_SENSOR_MOTION_GPIO;
	if (motion_gpio < 0) {
		ESP_LOGW(TAG, "Motion sensor disabled (GPIO -1)");
		return;
	}
	gpio_config_t io = {
		.pin_bit_mask = (1ULL << (unsigned)motion_gpio),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&io);
	s_motion_ready = true;
#endif
}

static void sensor_hw_init(void)
{
#if CONFIG_SENSOR_MOTION_MMWAVE
	if (ld2410_init(CONFIG_SENSOR_MMWAVE_UART_NUM, CONFIG_SENSOR_MMWAVE_TX_GPIO,
			CONFIG_SENSOR_MMWAVE_RX_GPIO, (uint32_t)CONFIG_SENSOR_MMWAVE_BAUD)) {
		s_motion_ready = true;
		ESP_LOGI(TAG, "Motion: mmWave (LD2410) UART%d TX=%d RX=%d %d baud - waiting for first frame",
			CONFIG_SENSOR_MMWAVE_UART_NUM, CONFIG_SENSOR_MMWAVE_TX_GPIO, CONFIG_SENSOR_MMWAVE_RX_GPIO, CONFIG_SENSOR_MMWAVE_BAUD);
	} else {
		ESP_LOGE(TAG, "Motion: mmWave init failed - check UART pins TX=%d RX=%d and baud %d",
			CONFIG_SENSOR_MMWAVE_TX_GPIO, CONFIG_SENSOR_MMWAVE_RX_GPIO, CONFIG_SENSOR_MMWAVE_BAUD);
	}
#else
	motion_gpio_init();
#endif
	bme680_init();
#if CONFIG_SENSOR_MOISTURE_ENABLE
	moisture_init();
	ESP_LOGI(TAG, "Moisture: %d channel(s) on D0/D1 (XIAO S3)", s_moisture_num_channels);
#endif
#if (CONFIG_SENSOR_DS18B20_GPIO >= 0)
	ds18b20_init(CONFIG_SENSOR_DS18B20_GPIO);
#endif
#if CONFIG_SENSOR_TDS_ENABLE
	tds_init();
#endif
	ESP_LOGI(TAG, "Sensor HW init done (packet size=%u, version=%d)",
		(unsigned)SENSOR_PACKET_SIZE, SENSOR_PACKET_VERSION);
}

void esp_now_send_init(void)
{
	bool espnow_enabled = (bool)DEFAULT_ESPNOW_ENABLED;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &s_nvs) == ESP_OK) {
		uint32_t last_ms = 0;
		if (nvs_get_u32(s_nvs, NVS_LAST_MOTION_KEY, &last_ms) == ESP_OK) {
			s_last_motion_ms = last_ms;
		}
		uint32_t trig = 0;
		if (nvs_get_u32(s_nvs, NVS_TRIGGER_COUNT_KEY, &trig) == ESP_OK) {
			s_trigger_count = trig;
		}
		uint8_t v = DEFAULT_ESPNOW_ENABLED ? 1 : 0;
		if (nvs_get_u8(s_nvs, NVS_KEY_ESPNOW_EN, &v) == ESP_OK) {
			espnow_enabled = (v != 0);
		}
		uint8_t ble_log = 0;
		if (nvs_get_u8(s_nvs, NVS_BLE_LOG_KEY, &ble_log) == ESP_OK && ble_log) {
			ble_logger_start();
		}
		uint8_t wifi_log = 0;
		if (nvs_get_u8(s_nvs, NVS_WIFI_LOG_KEY, &wifi_log) == ESP_OK && wifi_log) {
			wifi_logger_start();
		}
		size_t label_len = sizeof(s_label);
		if (nvs_get_str(s_nvs, NVS_LABEL_KEY, s_label, &label_len) != ESP_OK) {
			s_label[0] = '\0';
		}
		s_label[sizeof(s_label) - 1] = '\0';
		uint8_t outdoor = 0;
		if (nvs_get_u8(s_nvs, NVS_IS_OUTDOOR_KEY, &outdoor) == ESP_OK) {
			s_is_outdoor = (outdoor != 0) ? 1 : 0;
		}
	} else {
		s_nvs = 0;
	}
	sensor_hw_init();
	ESP_LOGI(TAG, "Sensor HW init done, starting WiFi/ESP-NOW");
	if (espnow_enabled) {
		wifi_init_esp_now();
	} else {
		ESP_LOGI(TAG, "ESP-NOW disabled by settings");
	}
}

bool esp_now_send_ready(void)
{
	return s_esp_now_ok;
}

bool esp_now_send_is_enabled(void)
{
	return s_esp_now_ok;
}

void esp_now_send_set_enabled(bool enabled)
{
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
		nvs_set_u8(h, NVS_KEY_ESPNOW_EN, enabled ? 1 : 0);
		nvs_commit(h);
		nvs_close(h);
	}
	if (enabled) {
		if (!s_esp_now_ok)
			wifi_init_esp_now();
	} else {
		esp_now_send_stop();
	}
}

int esp_now_send_peers_seen_count(void)
{
	if (!s_peers_mux)
		return 0;
	uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
	int count = 0;
	if (xSemaphoreTake(s_peers_mux, pdMS_TO_TICKS(50)) != pdTRUE)
		return 0;
	for (int i = 0; i < MAX_PEERS_SEEN; i++) {
		if (s_peers_seen[i].last_seen_ms == 0)
			continue;
		if (now_ms - s_peers_seen[i].last_seen_ms < PEER_STALE_MS)
			count++;
	}
	xSemaphoreGive(s_peers_mux);
	return count;
}

static bool read_bme_values(float *temperature, float *humidity, float *pressure, float *gas)
{
	if (!s_bme_ready)
		return false;
	int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &s_bme_dev);
	if (rslt != BME68X_OK)
		return false;
	uint32_t dur_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &s_bme_conf, &s_bme_dev);
	bme_delay_us(dur_us + 10000, &s_bme_ctx);
	struct bme68x_data data = { 0 };
	uint8_t n = 0;
	rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n, &s_bme_dev);
	if (rslt != BME68X_OK || n == 0)
		return false;
	*temperature = data.temperature;
	*humidity = data.humidity;
	*pressure = data.pressure / 100.0f;
	*gas = data.gas_resistance / 1000.0f;
	return true;
}

static void update_bme_cache_if_needed(uint64_t now_ms)
{
	if (!s_has_bme_cache || (now_ms - s_last_bme_poll_ms) >= BME_POLL_INTERVAL_MS) {
		float t = 0.0f, h = 0.0f, p = 0.0f, g = 0.0f;
		bool env_ok = read_bme_values(&t, &h, &p, &g);
		if (env_ok) {
			s_cached_temperature = t;
			s_cached_humidity = h;
			s_cached_pressure = p;
			s_cached_gas = g;
			s_has_bme_cache = true;
		} else if (!s_has_bme_cache) {
			s_cached_temperature = 0.0f;
			s_cached_humidity = 0.0f;
			s_cached_pressure = 0.0f;
			s_cached_gas = 0.0f;
		}
		s_last_bme_poll_ms = now_ms;
	}
}

#if (CONFIG_SENSOR_DS18B20_GPIO >= 0)
static void update_ds18b20_cache_if_needed(uint64_t now_ms)
{
	if (!ds18b20_is_ready())
		return;
	if (s_has_ds18b20_cache && (now_ms - s_last_ds18b20_poll_ms) < DS18B20_POLL_INTERVAL_MS)
		return;
	float t = ds18b20_read_temp_c();
	if (t > -126.0f) {
		t += (float)CONFIG_SENSOR_DS18B20_OFFSET_TENTHS * 0.1f;
		s_cached_ds18b20_temp = t;
		s_has_ds18b20_cache = true;
	} else if (!s_has_ds18b20_cache) {
		s_cached_ds18b20_temp = 0.0f;
	}
	s_last_ds18b20_poll_ms = now_ms;
}
#endif

static uint8_t read_motion_level(void)
{
	if (!s_motion_ready)
		return 0;
#if CONFIG_SENSOR_MOTION_MMWAVE
	return ld2410_get_motion_level();
#else
	return gpio_get_level(CONFIG_SENSOR_MOTION_GPIO) ? 1 : 0;
#endif
}

/** Fill packet with current sensor values (motion + BME680 + DS18B20 + TDS + moisture). */
static void fill_sensor_packet(sensor_packet_t *p)
{
	uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
	p->magic = SENSOR_PACKET_MAGIC;
	p->version = SENSOR_PACKET_VERSION;
	strncpy(p->label, s_label, sizeof(p->label) - 1);
	p->label[sizeof(p->label) - 1] = '\0';
	p->stream_host[0] = '\0';
	p->is_outdoor = s_is_outdoor;
	p->motion = read_motion_level();
	update_bme_cache_if_needed(now_ms);
	p->temperature = s_cached_temperature;
#if (CONFIG_SENSOR_DS18B20_GPIO >= 0)
	if (!s_bme_ready && ds18b20_is_ready()) {
		update_ds18b20_cache_if_needed(now_ms);
		p->temperature = s_cached_ds18b20_temp;
	}
	update_ds18b20_cache_if_needed(now_ms);
#endif
	p->temperature_water = SENSOR_TEMP_WATER_INVALID;
#if (CONFIG_SENSOR_DS18B20_GPIO >= 0)
	update_ds18b20_cache_if_needed(now_ms);
	if (s_has_ds18b20_cache)
		p->temperature_water = s_cached_ds18b20_temp;
#endif
	p->humidity = s_cached_humidity;
	p->pressure = s_cached_pressure;
	p->gas = s_cached_gas;
	p->tds_ppm = SENSOR_TDS_INVALID;
#if CONFIG_SENSOR_TDS_ENABLE
	if (s_tds_ready)
		p->tds_ppm = read_tds_ppm();
#endif
	for (int i = 0; i < SENSOR_MOISTURE_CHANNELS; i++)
		p->moisture[i] = -1.0f;
#if CONFIG_SENSOR_MOISTURE_ENABLE
	for (int i = 0; i < s_moisture_num_channels; i++)
		p->moisture[i] = read_moisture_channel(i);
#endif
	for (int i = 0; i < SENSOR_MOISTURE_CHANNELS; i++)
		p->plant_label[i][0] = '\0';
	p->uptime_ms = (uint32_t)now_ms;
#if CONFIG_SENSOR_MOTION_MMWAVE
	/* Send all 6 mmwave fields every packet (98-byte S3 format) so gateway/dashboard get state, distances, energy, detection_dist. */
	{
		ld2410_report_t r = { 0 };
		ld2410_get_report(&r);
		p->mmwave_state = r.state;
		p->mmwave_moving_cm = r.moving_dist_cm;
		p->mmwave_stationary_cm = r.stationary_dist_cm;
		p->mmwave_moving_energy = r.moving_energy;
		p->mmwave_stationary_energy = r.stationary_energy;
		p->mmwave_detection_dist_cm = r.detection_dist_cm;
		if (r.has_data) {
			if ((r.state != 0) != (s_prev_motion != 0)) {
				const char *state_str = r.state ? "presence" : "clear";
				unsigned state_val = (unsigned)r.state;
				unsigned move_cm = (unsigned)r.moving_dist_cm;
				unsigned stat_cm = (unsigned)r.stationary_dist_cm;
				ESP_LOGI(TAG, "mmWave %s state=%u move=%u cm stat=%u cm", state_str, state_val, move_cm, stat_cm);
			}
			s_last_mmwave_nodata_log_ms = 0; /* reset so we log again if data stops */
		} else if (s_motion_ready && (now_ms - s_last_mmwave_nodata_log_ms >= 30000)) {
			s_last_mmwave_nodata_log_ms = now_ms;
			ESP_LOGW(TAG, "LD2410 no frame yet: check wiring (sensor TX->MCU RX=%d, sensor RX->MCU TX=%d), baud %d",
				(int)CONFIG_SENSOR_MMWAVE_RX_GPIO, (int)CONFIG_SENSOR_MMWAVE_TX_GPIO, (int)CONFIG_SENSOR_MMWAVE_BAUD);
		}
	}
#else
	p->mmwave_state = 0;
	p->mmwave_moving_cm = 0;
	p->mmwave_stationary_cm = 0;
	p->mmwave_moving_energy = 0;
	p->mmwave_stationary_energy = 0;
	p->mmwave_detection_dist_cm = 0;
#endif
	if (p->motion == 0) {
		s_motion_high_since_ms = 0;
	} else if (s_prev_motion == 0) {
		if (CONFIG_SENSOR_MOTION_CONFIRM_MS == 0) {
			s_last_motion_ms = p->uptime_ms;
			s_trigger_count++;
			persist_motion_state();
			s_prev_motion = 1;
		} else {
			if (s_motion_high_since_ms == 0)
				s_motion_high_since_ms = (uint32_t)now_ms;
			if ((uint32_t)now_ms - s_motion_high_since_ms >= (uint32_t)CONFIG_SENSOR_MOTION_CONFIRM_MS) {
				s_last_motion_ms = p->uptime_ms;
				s_trigger_count++;
				persist_motion_state();
				s_motion_high_since_ms = 0;
				s_prev_motion = 1;
			}
		}
	}
	if (p->motion == 0)
		s_prev_motion = 0;
	else if (CONFIG_SENSOR_MOTION_CONFIRM_MS == 0 || s_motion_high_since_ms == 0 ||
		 (uint32_t)now_ms - s_motion_high_since_ms >= (uint32_t)CONFIG_SENSOR_MOTION_CONFIRM_MS)
		s_prev_motion = (uint8_t)p->motion;
	if (s_last_motion_ms > p->uptime_ms) {
		s_last_motion_ms = 0;
	}
	p->last_motion_ms = s_last_motion_ms;
	p->trigger_count = s_trigger_count;

	if (ble_logger_is_enabled()) {
		ble_logger_stats_t ble = { 0 };
		ble_logger_get_stats(&ble);
		p->ble_seen_count = ble.seen_count;
		p->ble_last_rssi_dbm = ble.has_addr ? ble.rssi_dbm : 0;
		if (ble.has_addr) {
			memcpy(p->ble_last_addr, ble.addr, sizeof(p->ble_last_addr));
		} else {
			memset(p->ble_last_addr, 0, sizeof(p->ble_last_addr));
		}
	} else {
		p->ble_seen_count = 0;
		p->ble_last_rssi_dbm = 0;
		memset(p->ble_last_addr, 0, sizeof(p->ble_last_addr));
	}
}

void esp_now_send_packet(void)
{
	if (!s_esp_now_ok)
		return;
	sensor_packet_t pkt;
	fill_sensor_packet(&pkt);
	esp_err_t err = esp_now_send(s_broadcast_mac, (const uint8_t *)&pkt, SENSOR_PACKET_SIZE);
	if (err != ESP_OK)
		ESP_LOGW(TAG, "esp_now_send failed: %s", esp_err_to_name(err));
	s_send_count++;
	if (s_send_count <= 3 || (s_send_count % 30) == 0) {
		char tw_buf[16], tds_buf[16];
		float tw = pkt.temperature_water, tds = pkt.tds_ppm;
		int tw_ok = (tw > -500.0f && tw < 200.0f);
		int tds_ok = (tds >= 0.0f);
		if (tw_ok) snprintf(tw_buf, sizeof(tw_buf), "%.1f", (double)tw);
		else snprintf(tw_buf, sizeof(tw_buf), "-");
		if (tds_ok) snprintf(tds_buf, sizeof(tds_buf), "%.0f", (double)tds);
		else snprintf(tds_buf, sizeof(tds_buf), "-");
#if CONFIG_SENSOR_MOTION_MMWAVE
		ESP_LOGI(TAG, "pkt #%lu: motion=%u T=%.1f T_water=%s H=%.1f P=%.1f gas=%.1f TDS=%s soil=[%.1f,%.1f,%.1f,%.1f] mmw=%u %u/%ucm trig=%lu (sz=%u ch=%d)",
			(unsigned long)s_send_count, (unsigned)pkt.motion,
			(double)pkt.temperature, tw_buf, (double)pkt.humidity,
			(double)pkt.pressure, (double)pkt.gas, tds_buf,
			(double)pkt.moisture[0], (double)pkt.moisture[1],
			(double)pkt.moisture[2], (double)pkt.moisture[3],
			(unsigned)pkt.mmwave_state, (unsigned)pkt.mmwave_moving_cm, (unsigned)pkt.mmwave_stationary_cm,
			(unsigned long)pkt.trigger_count,
			(unsigned)SENSOR_PACKET_SIZE, (int)s_espnow_channel);
#else
#if CONFIG_SENSOR_MOISTURE_ENABLE
		{
			int r0 = read_moisture_raw(0), r1 = (s_moisture_num_channels > 1) ? read_moisture_raw(1) : -1;
			int r2 = (s_moisture_num_channels > 2) ? read_moisture_raw(2) : -1, r3 = (s_moisture_num_channels > 3) ? read_moisture_raw(3) : -1;
			ESP_LOGI(TAG, "pkt #%lu: motion=%u T=%.1f T_water=%s H=%.1f P=%.1f gas=%.1f TDS=%s soil=[%.1f,%.1f,%.1f,%.1f] raw=[%d,%d,%d,%d] trig=%lu (sz=%u ch=%d)",
				(unsigned long)s_send_count, (unsigned)pkt.motion,
				(double)pkt.temperature, tw_buf, (double)pkt.humidity,
				(double)pkt.pressure, (double)pkt.gas, tds_buf,
				(double)pkt.moisture[0], (double)pkt.moisture[1],
				(double)pkt.moisture[2], (double)pkt.moisture[3],
				r0, r1, r2, r3,
				(unsigned long)pkt.trigger_count,
				(unsigned)SENSOR_PACKET_SIZE, (int)s_espnow_channel);
		}
#else
		ESP_LOGI(TAG, "pkt #%lu: motion=%u T=%.1f T_water=%s H=%.1f P=%.1f gas=%.1f TDS=%s soil=[%.1f,%.1f,%.1f,%.1f] trig=%lu (sz=%u ch=%d)",
			(unsigned long)s_send_count, (unsigned)pkt.motion,
			(double)pkt.temperature, tw_buf, (double)pkt.humidity,
			(double)pkt.pressure, (double)pkt.gas, tds_buf,
			(double)pkt.moisture[0], (double)pkt.moisture[1],
			(double)pkt.moisture[2], (double)pkt.moisture[3],
			(unsigned long)pkt.trigger_count,
			(unsigned)SENSOR_PACKET_SIZE, (int)s_espnow_channel);
#endif
#endif
	}
}

/** Send one packet when motion goes 0→1 and confirm time has passed (debounce). */
void esp_now_send_packet_on_motion_trigger(void)
{
	if (!s_esp_now_ok)
		return;
	uint8_t motion = read_motion_level();
	if (motion == 0) {
		s_motion_high_since_ms = 0;
		return;
	}
	if (motion != 1 || s_prev_motion != 0)
		return;
	uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
	if (CONFIG_SENSOR_MOTION_CONFIRM_MS > 0) {
		if (s_motion_high_since_ms == 0)
			s_motion_high_since_ms = now_ms;
		if (now_ms - s_motion_high_since_ms < (uint32_t)CONFIG_SENSOR_MOTION_CONFIRM_MS)
			return;
	}
	sensor_packet_t pkt;
	fill_sensor_packet(&pkt);
	esp_err_t err = esp_now_send(s_broadcast_mac, (const uint8_t *)&pkt, SENSOR_PACKET_SIZE);
	if (err != ESP_OK)
		ESP_LOGW(TAG, "esp_now_send failed: %s", esp_err_to_name(err));
}

/** Send one packet when motion goes 1→0 so dashboard clears quickly. */
void esp_now_send_packet_on_motion_cleared(void)
{
	if (!s_esp_now_ok)
		return;
	if (read_motion_level() != 0 || s_prev_motion != 1)
		return;
	sensor_packet_t pkt;
	fill_sensor_packet(&pkt);
	esp_err_t err = esp_now_send(s_broadcast_mac, (const uint8_t *)&pkt, SENSOR_PACKET_SIZE);
	if (err != ESP_OK)
		ESP_LOGW(TAG, "esp_now_send failed: %s", esp_err_to_name(err));
}

void esp_now_send_wifi_scan_if_due(void)
{
	if (!s_esp_now_ok || !wifi_logger_is_enabled())
		return;
	wifi_scan_packet_t pkt;
	int n = wifi_logger_try_scan_and_fill(&pkt);
	if (n <= 0)
		return;
	esp_err_t err = esp_now_send(s_broadcast_mac, (const uint8_t *)&pkt, WIFI_SCAN_PACKET_SIZE);
	if (err != ESP_OK)
		ESP_LOGW(TAG, "wifi_scan send failed: %s", esp_err_to_name(err));
}
