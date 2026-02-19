/**
 * DS18B20 1-Wire temperature sensor driver (bit-bang).
 * Single sensor, Skip ROM. 4.7k pull-up between DATA and VCC.
 */
#include "ds18b20.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ds18b20";

static int s_gpio = -1;
static bool s_ready = false;

/* 1-Wire timing (µs). DS18B20 datasheet. Open-drain needs longer recovery/sample. */
#define T_RESET_LOW  480
#define T_PRESENCE   70
#define T_SLOT_LOW   6
#define T_WRITE_1    6
#define T_READ_SAMPLE 14   /* sample just before 15µs (0 low), max time for 1 to rise */
#define T_RECOVERY   60    /* recovery between slots for open-drain rise */

/* Open-drain: drive low only; release = high-Z so external 4.7k pulls high. */
static inline void ow_low(void)
{
	gpio_set_direction(s_gpio, GPIO_MODE_OUTPUT_OD);
	gpio_set_level(s_gpio, 0);
}

static inline void ow_release(void)
{
	gpio_set_direction(s_gpio, GPIO_MODE_OUTPUT_OD);
	gpio_set_level(s_gpio, 1); /* high-Z, pull-up brings line high */
}

/* Presence: INPUT + pull-up so unplugged (floating) line reads high = no presence. */
static inline int ow_read_presence(void)
{
	gpio_set_direction(s_gpio, GPIO_MODE_INPUT);
	gpio_set_pull_mode(s_gpio, GPIO_PULLUP_ONLY);
	esp_rom_delay_us(10);
	int v = gpio_get_level(s_gpio) ? 1 : 0;
	gpio_set_pull_mode(s_gpio, GPIO_FLOATING);
	gpio_set_direction(s_gpio, GPIO_MODE_OUTPUT_OD);
	gpio_set_level(s_gpio, 1);
	return v;
}

static bool ow_reset(void)
{
	ow_low();
	esp_rom_delay_us(T_RESET_LOW);
	ow_release();
	esp_rom_delay_us(T_PRESENCE);
	int p = ow_read_presence();
	esp_rom_delay_us(T_RESET_LOW - T_PRESENCE);
	return (p == 0); /* presence = bus pulled low by slave */
}

static void ow_write_byte(uint8_t byte)
{
	for (int i = 0; i < 8; i++) {
		ow_low();
		esp_rom_delay_us(byte & 1 ? T_WRITE_1 : 60);
		ow_release();
		esp_rom_delay_us(T_RECOVERY);
		byte >>= 1;
	}
}

static uint8_t ow_read_byte(void)
{
	uint8_t b = 0;
	for (int i = 0; i < 8; i++) {
		ow_low();
		esp_rom_delay_us(1);
		ow_release();
		/* C6: INPUT + pull-up during read so we see line when slave releases (1-bit). */
		gpio_set_direction(s_gpio, GPIO_MODE_INPUT);
		gpio_set_pull_mode(s_gpio, GPIO_PULLUP_ONLY);
		esp_rom_delay_us(T_READ_SAMPLE);
		b |= (uint8_t)((gpio_get_level(s_gpio) ? 1 : 0) << i);
		gpio_set_pull_mode(s_gpio, GPIO_FLOATING);
		gpio_set_direction(s_gpio, GPIO_MODE_OUTPUT_OD);
		gpio_set_level(s_gpio, 1);
		esp_rom_delay_us(T_RECOVERY);
	}
	return b;
}

bool ds18b20_init(int gpio_num)
{
	if (gpio_num < 0) {
		s_ready = false;
		return false;
	}
	s_gpio = gpio_num;
	gpio_config_t io = {
		.pin_bit_mask = (1ULL << s_gpio),
		.mode = GPIO_MODE_OUTPUT_OD,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&io);
	gpio_set_level(s_gpio, 1); /* release bus (high-Z) so external 4.7k pulls up */
	/* XIAO ESP32-C6: set pin to INPUT once so reads work (Seeed forum #293778) */
	gpio_set_direction(s_gpio, GPIO_MODE_INPUT);
	esp_rom_delay_us(10000);
	gpio_set_direction(s_gpio, GPIO_MODE_OUTPUT_OD);
	gpio_set_level(s_gpio, 1);
	/* Let bus and external pull-up settle (helps with cable / cold start) */
	for (int i = 0; i < 5000; i++) {
		esp_rom_delay_us(1);
	}
	/* Retry presence; only claim "found" after a valid scratchpad read (avoids false detect when unplugged). */
	bool ok = false;
	for (int attempt = 0; attempt < 3 && !ok; attempt++) {
		if (attempt > 0) {
			for (int i = 0; i < 500; i++)
				esp_rom_delay_us(1);
		}
		if (!ow_reset())
			continue;
		ow_write_byte(0xCC);
		ow_write_byte(0x44);
		for (int i = 0; i < 750; i++)
			esp_rom_delay_us(1000);
		if (!ow_reset())
			continue;
		ow_write_byte(0xCC);
		ow_write_byte(0xBE);
		uint8_t buf[9];
		for (int i = 0; i < 9; i++)
			buf[i] = ow_read_byte();
		/* Valid device = scratchpad not all 0xFF (unplugged/floating) */
		if (buf[0] != 0xFF || buf[1] != 0xFF) {
			ok = true;
			break;
		}
	}
	s_ready = ok;
	if (ok) {
		ESP_LOGI(TAG, "DS18B20 found on GPIO %d", s_gpio);
	} else {
		ESP_LOGW(TAG, "DS18B20 no presence on GPIO %d (check 4.7k pull-up)", s_gpio);
	}
	return ok;
}

bool ds18b20_is_ready(void)
{
	return s_ready && s_gpio >= 0;
}

float ds18b20_read_temp_c(void)
{
	if (!s_ready || s_gpio < 0)
		return -127.0f;

	if (!ow_reset())
		return -127.0f;
	ow_write_byte(0xCC); /* Skip ROM */
	ow_write_byte(0x44); /* Convert T */

	/* 12-bit conversion: 750 ms */
	for (int i = 0; i < 750; i++)
		esp_rom_delay_us(1000);

	if (!ow_reset())
		return -127.0f;
	ow_write_byte(0xCC); /* Skip ROM */
	ow_write_byte(0xBE); /* Read Scratchpad */

	uint8_t buf[9];
	for (int i = 0; i < 9; i++)
		buf[i] = ow_read_byte();

	/* All 0xFF = no device / bus floating; treat as read failure */
	if (buf[0] == 0xFF && buf[1] == 0xFF)
		return -127.0f;
	/* CRC could be checked (buf[8]) - skip for simplicity */
	int16_t raw = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
	float c = (float)raw * 0.0625f;
	ESP_LOGD(TAG, "scratchpad %02X %02X ... raw=%d °C=%.2f", buf[0], buf[1], (int)raw, (double)c);
	return c;
}
