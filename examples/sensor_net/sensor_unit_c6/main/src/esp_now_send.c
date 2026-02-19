/**
 * ESP-NOW sender for sensor unit: sends sensor_packet_t to broadcast (gateway receives on 2.4 GHz).
 * Also receives gateway→node command packets (e.g. blink LED). Uses same channel (6) as gateway AP.
 */
#include "packet.h"
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
#include "ble_logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "esp_now_send";

#define ESPNOW_CHANNEL_DEFAULT 6
#define ESPNOW_SCAN_CHANNEL_MIN 1
#define ESPNOW_SCAN_CHANNEL_MAX 14
#define ESPNOW_SCAN_WAIT_MS     400
#define BLINK_MS 120
#define BLINK_COUNT 3
#define BME_POLL_INTERVAL_MS 5000
#define DS18B20_POLL_INTERVAL_MS 5000

static const uint8_t s_broadcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static bool s_esp_now_ok = false;
static nvs_handle_t s_nvs = 0;
/** Current ESP-NOW channel (from scan or default). */
static uint8_t s_espnow_channel = ESPNOW_CHANNEL_DEFAULT;
static SemaphoreHandle_t s_scan_ack_sem = NULL;
static volatile bool s_scanning = false;
static uint32_t s_last_motion_ms = 0;
static uint32_t s_trigger_count = 0;
static uint8_t s_prev_motion = 0;
#if (CONFIG_SENSOR_MOTION_DEBOUNCE_MS > 0 || CONFIG_SENSOR_MOTION_COOLDOWN_MS > 0)
static uint8_t s_filtered_motion = 0;
static uint32_t s_motion_high_since_ms = 0;
static uint32_t s_cooldown_until_ms = 0;
static uint8_t s_prev_filtered_motion = 0;
static esp_timer_handle_t s_motion_timer = NULL;
#define MOTION_POLL_MS 50
#endif
static uint64_t s_last_bme_poll_ms = 0;
static bool s_has_bme_cache = false;
static float s_cached_temperature = 0.0f;
static float s_cached_humidity = 0.0f;
static float s_cached_pressure = 0.0f;
static float s_cached_gas = 0.0f;
static bool s_motion_ready = false;
static bool s_bme_ready = false;
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
#define NVS_TRIGGER_COUNT_KEY "trigger_count"
#define NVS_ESPNOW_CHANNEL_KEY "espnow_ch"
/* NVS keys for plant labels: "plbl0" .. "plbl3" */
#define NVS_PLANT_LABEL_PREFIX "plbl"
static char s_plant_labels[SENSOR_MOISTURE_CHANNELS][SENSOR_PLANT_LABEL_LEN];

static QueueHandle_t s_blink_queue = NULL;

#if CONFIG_SENSOR_LED_GPIO >= 0
static void blink_task(void *arg)
{
	(void)arg;
	int level = 0;
	for (;;) {
		if (xQueueReceive(s_blink_queue, &level, portMAX_DELAY) != pdTRUE)
			continue;
		for (int i = 0; i < BLINK_COUNT; i++) {
			gpio_set_level(CONFIG_SENSOR_LED_GPIO, 1);
			vTaskDelay(pdMS_TO_TICKS(BLINK_MS));
			gpio_set_level(CONFIG_SENSOR_LED_GPIO, 0);
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

static void plant_labels_load(void)
{
	for (int i = 0; i < SENSOR_MOISTURE_CHANNELS; i++) {
		s_plant_labels[i][0] = '\0';
		if (!s_nvs) continue;
		char key[8];
		snprintf(key, sizeof(key), "%s%d", NVS_PLANT_LABEL_PREFIX, i);
		size_t len = SENSOR_PLANT_LABEL_LEN;
		nvs_get_str(s_nvs, key, s_plant_labels[i], &len);
		s_plant_labels[i][SENSOR_PLANT_LABEL_LEN - 1] = '\0';
	}
}

static void plant_label_save(int ch, const char *label)
{
	if (ch < 0 || ch >= SENSOR_MOISTURE_CHANNELS) return;
	memset(s_plant_labels[ch], 0, SENSOR_PLANT_LABEL_LEN);
	if (label) {
		strncpy(s_plant_labels[ch], label, SENSOR_PLANT_LABEL_LEN - 1);
	}
	if (!s_nvs) return;
	char key[8];
	snprintf(key, sizeof(key), "%s%d", NVS_PLANT_LABEL_PREFIX, ch);
	nvs_set_str(s_nvs, key, s_plant_labels[ch]);
	nvs_commit(s_nvs);
	ESP_LOGI(TAG, "Plant label ch%d = \"%s\"", ch, s_plant_labels[ch]);
}

static void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
	(void)info;
	if (!data || len < 2)
		return;
	/* During channel scan: gateway beacon = ACK; lock on this channel. */
	if (s_scanning && s_scan_ack_sem != NULL && data[0] == GATEWAY_PACKET_MAGIC) {
		xSemaphoreGive(s_scan_ack_sem);
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
		persist_motion_state();
		return;
	}
	if (data[1] == CMD_TYPE_SET_PLANT_LABEL && len >= (int)CMD_PLANT_LABEL_PACKET_SIZE) {
		cmd_plant_label_packet_t cmd;
		memcpy(&cmd, data, sizeof(cmd));
		cmd.label[SENSOR_PLANT_LABEL_LEN - 1] = '\0';
		plant_label_save(cmd.channel, cmd.label);
		return;
	}
}

/** Try one channel: set WiFi channel, add broadcast peer, send probe, wait for gateway beacon. */
static bool try_channel_and_wait_ack(uint8_t channel)
{
	esp_wifi_set_channel((int)channel, WIFI_SECOND_CHAN_NONE);
	vTaskDelay(pdMS_TO_TICKS(10));
	esp_now_del_peer((uint8_t *)s_broadcast_mac);
	esp_now_peer_info_t peer = { 0 };
	memcpy(peer.peer_addr, s_broadcast_mac, 6);
	peer.channel = channel;
	peer.ifidx = WIFI_IF_STA;
	peer.encrypt = false;
	esp_err_t add_err = esp_now_add_peer(&peer);
	if (add_err != ESP_OK && add_err != ESP_ERR_ESPNOW_EXIST)
		return false;
	sensor_packet_t probe = { 0 };
	probe.magic = SENSOR_PACKET_MAGIC;
	probe.version = SENSOR_PACKET_VERSION;
	s_scanning = true;
	xSemaphoreTake(s_scan_ack_sem, 0);
	esp_err_t err = esp_now_send(s_broadcast_mac, (const uint8_t *)&probe, SENSOR_PACKET_SIZE);
	if (err != ESP_OK) {
		s_scanning = false;
		return false;
	}
	BaseType_t ack = xSemaphoreTake(s_scan_ack_sem, pdMS_TO_TICKS(ESPNOW_SCAN_WAIT_MS));
	s_scanning = false;
	return (ack == pdTRUE);
}

static void wifi_init_esp_now(void)
{
	esp_netif_create_default_wifi_sta();
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);
	esp_wifi_set_storage(WIFI_STORAGE_RAM);
	esp_wifi_set_mode(WIFI_MODE_STA);
	esp_wifi_start();
	vTaskDelay(pdMS_TO_TICKS(200));
	esp_err_t err = esp_now_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
		return;
	}
	esp_now_register_recv_cb(esp_now_recv_cb);

	s_scan_ack_sem = xSemaphoreCreateBinary();
	if (s_scan_ack_sem != NULL) {
		s_espnow_channel = 0;
		uint8_t last_ch = 0;
		if (s_nvs && nvs_get_u8(s_nvs, NVS_ESPNOW_CHANNEL_KEY, &last_ch) == ESP_OK &&
		    last_ch >= ESPNOW_SCAN_CHANNEL_MIN && last_ch <= ESPNOW_SCAN_CHANNEL_MAX) {
			if (try_channel_and_wait_ack(last_ch)) {
				s_espnow_channel = last_ch;
				ESP_LOGI(TAG, "ESP-NOW channel %u (from NVS, ACK ok)", (unsigned)last_ch);
			}
		}
		if (s_espnow_channel == 0) {
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
		}
		vSemaphoreDelete(s_scan_ack_sem);
		s_scan_ack_sem = NULL;
	}
	if (s_espnow_channel < ESPNOW_SCAN_CHANNEL_MIN || s_espnow_channel > ESPNOW_SCAN_CHANNEL_MAX) {
		s_espnow_channel = (uint8_t)ESPNOW_CHANNEL_DEFAULT;
		ESP_LOGW(TAG, "No gateway ACK on ch 1-%u; using channel %d", (unsigned)ESPNOW_SCAN_CHANNEL_MAX, (int)s_espnow_channel);
	}

	esp_wifi_set_channel((int)s_espnow_channel, WIFI_SECOND_CHAN_NONE);
	vTaskDelay(pdMS_TO_TICKS(50));
	esp_now_del_peer((uint8_t *)s_broadcast_mac);
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
		gpio_set_level(CONFIG_SENSOR_LED_GPIO, 0);
		xTaskCreate(blink_task, "blink", 1536, NULL, 5, NULL);
		ESP_LOGI(TAG, "LED blink on GPIO %d (gateway command)", CONFIG_SENSOR_LED_GPIO);
	}
#endif
	s_esp_now_ok = true;
	ESP_LOGI(TAG, "ESP-NOW sender ready (channel %d)", (int)s_espnow_channel);
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
	if (CONFIG_SENSOR_BME_I2C_SDA_GPIO < 0 || CONFIG_SENSOR_BME_I2C_SCL_GPIO < 0) {
		ESP_LOGW(TAG, "BME680 disabled (SDA/SCL set to -1)");
		return;
	}
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = CONFIG_SENSOR_BME_I2C_SDA_GPIO,
		.scl_io_num = CONFIG_SENSOR_BME_I2C_SCL_GPIO,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = CONFIG_SENSOR_BME_I2C_FREQ_HZ,
	};
	ESP_ERROR_CHECK(i2c_param_config(CONFIG_SENSOR_BME_I2C_PORT, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(CONFIG_SENSOR_BME_I2C_PORT, conf.mode, 0, 0, 0));
	s_bme_ctx.port = CONFIG_SENSOR_BME_I2C_PORT;
	s_bme_dev.intf = BME68X_I2C_INTF;
	s_bme_dev.intf_ptr = &s_bme_ctx;
	s_bme_dev.read = bme_i2c_read;
	s_bme_dev.write = bme_i2c_write;
	s_bme_dev.delay_us = bme_delay_us;
	s_bme_dev.amb_temp = 25;

	ESP_LOGI(TAG, "BME680 I2C on SDA=%d SCL=%d (port %d)",
		CONFIG_SENSOR_BME_I2C_SDA_GPIO, CONFIG_SENSOR_BME_I2C_SCL_GPIO,
		CONFIG_SENSOR_BME_I2C_PORT);

	uint8_t addrs[2] = { (uint8_t)CONFIG_SENSOR_BME_I2C_ADDR,
		(uint8_t)(CONFIG_SENSOR_BME_I2C_ADDR == 0x76 ? 0x77 : 0x76) };
	int8_t rslt = BME68X_E_COM_FAIL;
	bool ok = false;
	for (int i = 0; i < 2; i++) {
		s_bme_ctx.addr = addrs[i];
		rslt = bme68x_init(&s_bme_dev);
		if (rslt == BME68X_OK) {
			ok = true;
			break;
		}
		ESP_LOGW(TAG, "BME680 init failed at 0x%02X: %d", (unsigned)addrs[i], (int)rslt);
	}
	if (!ok) {
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

#if (CONFIG_SENSOR_MOTION_DEBOUNCE_MS > 0 || CONFIG_SENSOR_MOTION_COOLDOWN_MS > 0)
static void motion_poll_timer_cb(void *arg)
{
	(void)arg;
	if (!s_motion_ready)
		return;
	uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
	uint8_t raw = gpio_get_level(CONFIG_SENSOR_MOTION_GPIO) ? 1 : 0;
	if (raw == 0) {
		s_motion_high_since_ms = 0;
		s_filtered_motion = 0;
	} else {
		if (s_motion_high_since_ms == 0)
			s_motion_high_since_ms = now_ms;
		if ((now_ms - s_motion_high_since_ms) >= (uint32_t)CONFIG_SENSOR_MOTION_DEBOUNCE_MS) {
			s_filtered_motion = 1;
			if (s_prev_filtered_motion == 0 &&
			    (CONFIG_SENSOR_MOTION_COOLDOWN_MS == 0 || now_ms >= s_cooldown_until_ms)) {
				s_last_motion_ms = now_ms;
				s_trigger_count++;
				persist_motion_state();
				s_cooldown_until_ms = CONFIG_SENSOR_MOTION_COOLDOWN_MS > 0
					? now_ms + (uint32_t)CONFIG_SENSOR_MOTION_COOLDOWN_MS
					: 0;
			}
		}
	}
	s_prev_filtered_motion = s_filtered_motion;
}
#endif

static void motion_gpio_init(void)
{
	if (CONFIG_SENSOR_MOTION_GPIO < 0) {
		ESP_LOGW(TAG, "Motion sensor disabled (GPIO -1)");
		return;
	}
	gpio_config_t io = {
		.pin_bit_mask = (1ULL << CONFIG_SENSOR_MOTION_GPIO),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&io);
	s_motion_ready = true;
#if (CONFIG_SENSOR_MOTION_DEBOUNCE_MS > 0 || CONFIG_SENSOR_MOTION_COOLDOWN_MS > 0)
	const esp_timer_create_args_t args = {
		.callback = &motion_poll_timer_cb,
		.arg = NULL,
		.dispatch_method = ESP_TIMER_TASK,
		.name = "motion_poll",
	};
	if (esp_timer_create(&args, &s_motion_timer) == ESP_OK &&
	    esp_timer_start_periodic(s_motion_timer, MOTION_POLL_MS * 1000) == ESP_OK) {
		ESP_LOGI(TAG, "Motion debounce %d ms, cooldown %d ms", CONFIG_SENSOR_MOTION_DEBOUNCE_MS, CONFIG_SENSOR_MOTION_COOLDOWN_MS);
	}
#endif
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
		ESP_LOGI(TAG, "Moisture sensor %d ready on ADC1 channel %d", i + 1, adc_channels[i]);
	}
}

/* Read raw ADC for channel (for logging/calibration). Returns -1 on error. */
static int read_moisture_raw(int idx)
{
	if (idx < 0 || idx >= s_moisture_num_channels || !s_moisture_ready[idx] || s_moisture_adc_handle == NULL)
		return -1;
	int raw = 0;
	if (adc_oneshot_read(s_moisture_adc_handle, s_moisture_channels[idx], &raw) != ESP_OK)
		return -1;
	return raw;
}

/* Per-channel dry/wet calibration (higher raw = drier). */
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

/* Grove capacitive v2: higher raw = drier. Per-channel two-point calibration. */
static float read_moisture_channel(int idx)
{
	int raw = read_moisture_raw(idx);
	if (raw < 0) return -1.0f;
	if (raw > 4095) raw = 4095;
	int raw_dry, raw_wet;
	get_moisture_cal(idx, &raw_dry, &raw_wet);
	if (raw_dry <= raw_wet) {
		/* Fallback: full-scale inverse (legacy) */
		return (4095 - raw) * 100.0f / 4095.0f;
	}
	/* Linear map: raw_dry -> 0%, raw_wet -> 100% */
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
	ESP_LOGI(TAG, "TDS sensor ready on ADC1 channel %d (D2)", (int)s_tds_channel);
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

static void sensor_hw_init(void)
{
	motion_gpio_init();
	bme680_init();
#if defined(CONFIG_SENSOR_MOISTURE_ENABLE) && CONFIG_SENSOR_MOISTURE_ENABLE
	moisture_init();
	ESP_LOGI(TAG, "Moisture: %d channel(s) configured", s_moisture_num_channels);
	for (int i = 0; i < s_moisture_num_channels; i++) {
		ESP_LOGI(TAG, "  sensor %d: ADC1 ch %d, ready=%d",
			i + 1, (int)s_moisture_channels[i], (int)s_moisture_ready[i]);
	}
#else
	ESP_LOGI(TAG, "Grove moisture: DISABLED (enable in menuconfig to send soil data)");
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
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &s_nvs) == ESP_OK) {
		uint32_t last_ms = 0;
		if (nvs_get_u32(s_nvs, NVS_LAST_MOTION_KEY, &last_ms) == ESP_OK) {
			s_last_motion_ms = last_ms;
		}
		uint32_t trig = 0;
		if (nvs_get_u32(s_nvs, NVS_TRIGGER_COUNT_KEY, &trig) == ESP_OK) {
			s_trigger_count = trig;
		}
	} else {
		s_nvs = 0;
	}
	plant_labels_load();
	wifi_init_esp_now();
	sensor_hw_init();
}

bool esp_now_send_ready(void)
{
	return s_esp_now_ok;
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
	} else if (!s_has_ds18b20_cache)
		s_cached_ds18b20_temp = 0.0f;
	s_last_ds18b20_poll_ms = now_ms;
}
#endif

static uint8_t read_motion_level(void)
{
	if (!s_motion_ready)
		return 0;
	return gpio_get_level(CONFIG_SENSOR_MOTION_GPIO) ? 1 : 0;
}

#if (CONFIG_SENSOR_MOTION_DEBOUNCE_MS > 0 || CONFIG_SENSOR_MOTION_COOLDOWN_MS > 0)
static uint8_t get_motion_for_packet(void)
{
	return s_filtered_motion;
}
#else
static uint8_t get_motion_for_packet(void)
{
	return read_motion_level();
}
#endif

/** Fill packet with current sensor values (RCWL-0516 + BME680 + DS18B20).
 *  Air temp = BME (e.g. I2C on D0); water temp = DS18B20 1-Wire (e.g. D1). */
static void fill_sensor_packet(sensor_packet_t *p)
{
	memset(p, 0, sizeof(sensor_packet_t));
	uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
	p->magic = SENSOR_PACKET_MAGIC;
	p->version = SENSOR_PACKET_VERSION;
	p->motion = get_motion_for_packet();
	update_bme_cache_if_needed(now_ms);
	p->temperature = s_cached_temperature;           /* air: BME (e.g. D0) */
	p->temperature_water = SENSOR_TEMP_WATER_INVALID;
#if (CONFIG_SENSOR_DS18B20_GPIO >= 0)
	if (ds18b20_is_ready()) {
		update_ds18b20_cache_if_needed(now_ms);
		p->temperature_water = s_cached_ds18b20_temp; /* water: DS18B20 (e.g. D1) */
		if (!s_bme_ready)
			p->temperature = s_cached_ds18b20_temp;
	}
#endif
	p->humidity = s_cached_humidity;
	p->pressure = s_cached_pressure;
	p->gas = s_cached_gas;
	p->tds_ppm = SENSOR_TDS_INVALID;
#if CONFIG_SENSOR_TDS_ENABLE
	if (s_tds_ready)
		p->tds_ppm = read_tds_ppm();
#endif
	/* When CONFIG_SENSOR_MOISTURE_ENABLE=n we must not send real moisture; use -1 (disabled). */
	for (int i = 0; i < SENSOR_MOISTURE_CHANNELS; i++)
		p->moisture[i] = -1.0f;
#if defined(CONFIG_SENSOR_MOISTURE_ENABLE) && CONFIG_SENSOR_MOISTURE_ENABLE
	for (int i = 0; i < s_moisture_num_channels; i++)
		p->moisture[i] = read_moisture_channel(i);
#endif
	for (int i = 0; i < SENSOR_MOISTURE_CHANNELS; i++)
		memcpy(p->plant_label[i], s_plant_labels[i], SENSOR_PLANT_LABEL_LEN);
	p->mmwave_state = 0;
	p->mmwave_moving_cm = 0;
	p->mmwave_stationary_cm = 0;
	p->mmwave_moving_energy = 0;
	p->mmwave_stationary_energy = 0;
	p->mmwave_detection_dist_cm = 0;
	p->uptime_ms = (uint32_t)now_ms;
#if (CONFIG_SENSOR_MOTION_DEBOUNCE_MS > 0 || CONFIG_SENSOR_MOTION_COOLDOWN_MS > 0)
	/* Timer updates s_last_motion_ms and s_trigger_count; just apply wrap fix. */
#else
	if (p->motion == 1 && s_prev_motion == 0) {
		s_last_motion_ms = p->uptime_ms;
		s_trigger_count++;
		persist_motion_state();
	}
	s_prev_motion = p->motion;
#endif
	if (s_last_motion_ms > p->uptime_ms) {
		s_last_motion_ms = 0;
	}
	p->last_motion_ms = s_last_motion_ms;
	p->trigger_count = s_trigger_count;

	ble_logger_stats_t ble = { 0 };
	ble_logger_get_stats(&ble);
	p->ble_seen_count = ble.seen_count;
	p->ble_last_rssi_dbm = ble.has_addr ? ble.rssi_dbm : 0;
	if (ble.has_addr) {
		memcpy(p->ble_last_addr, ble.addr, sizeof(p->ble_last_addr));
	} else {
		memset(p->ble_last_addr, 0, sizeof(p->ble_last_addr));
	}
}

static uint32_t s_send_count = 0;

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
#if defined(CONFIG_SENSOR_MOISTURE_ENABLE) && CONFIG_SENSOR_MOISTURE_ENABLE
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
	}
}
