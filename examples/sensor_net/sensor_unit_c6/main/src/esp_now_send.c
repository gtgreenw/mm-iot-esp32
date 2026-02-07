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
#include "bme68x.h"
#include "ble_logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

static const char *TAG = "esp_now_send";

#define ESPNOW_CHANNEL 6
#define BLINK_MS 120
#define BLINK_COUNT 3
#define BME_POLL_INTERVAL_MS 5000

static uint8_t s_broadcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static bool s_esp_now_ok = false;
static nvs_handle_t s_nvs = 0;
static uint32_t s_last_motion_ms = 0;
static uint32_t s_trigger_count = 0;
static uint8_t s_prev_motion = 0;
static uint64_t s_last_bme_poll_ms = 0;
static bool s_has_bme_cache = false;
static float s_cached_temperature = 0.0f;
static float s_cached_humidity = 0.0f;
static float s_cached_pressure = 0.0f;
static float s_cached_gas = 0.0f;
static bool s_motion_ready = false;
static bool s_bme_ready = false;

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

static void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
	(void)info;
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
	}
}

static void wifi_init_esp_now(void)
{
	esp_netif_create_default_wifi_sta();
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);
	esp_wifi_set_storage(WIFI_STORAGE_RAM);
	esp_wifi_set_mode(WIFI_MODE_STA);
	esp_wifi_start();
	esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
	vTaskDelay(pdMS_TO_TICKS(200));
	esp_err_t err = esp_now_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
		return;
	}
	esp_now_peer_info_t peer = { 0 };
	memcpy(peer.peer_addr, s_broadcast_mac, 6);
	peer.channel = ESPNOW_CHANNEL;
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
	esp_now_register_recv_cb(esp_now_recv_cb);
	s_esp_now_ok = true;
	ESP_LOGI(TAG, "ESP-NOW sender ready (channel %d)", ESPNOW_CHANNEL);
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
}

static void sensor_hw_init(void)
{
	motion_gpio_init();
	bme680_init();
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

static uint8_t read_motion_level(void)
{
	if (!s_motion_ready)
		return 0;
	return gpio_get_level(CONFIG_SENSOR_MOTION_GPIO) ? 1 : 0;
}

/** Fill packet with current sensor values (RCWL-0516 + BME680). */
static void fill_sensor_packet(sensor_packet_t *p)
{
	uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
	p->magic = SENSOR_PACKET_MAGIC;
	p->version = SENSOR_PACKET_VERSION;
	p->motion = read_motion_level();
	update_bme_cache_if_needed(now_ms);
	p->temperature = s_cached_temperature;
	p->humidity = s_cached_humidity;
	p->pressure = s_cached_pressure;
	p->gas = s_cached_gas;
	p->uptime_ms = (uint32_t)now_ms;
	if (p->motion == 1 && s_prev_motion == 0) {
		s_last_motion_ms = p->uptime_ms;
		s_trigger_count++;
		persist_motion_state();
	}
	s_prev_motion = p->motion;
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

void esp_now_send_packet(void)
{
	if (!s_esp_now_ok)
		return;
	sensor_packet_t pkt;
	fill_sensor_packet(&pkt);
	esp_err_t err = esp_now_send(s_broadcast_mac, (const uint8_t *)&pkt, SENSOR_PACKET_SIZE);
	if (err != ESP_OK)
		ESP_LOGW(TAG, "esp_now_send failed: %s", esp_err_to_name(err));
}
