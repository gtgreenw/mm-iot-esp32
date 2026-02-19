/**
 * LD2410 / 24GHz mmWave for XIAO – UART parser.
 * Frame: F4 F3 F2 F1 [len 2B LE] [type 1B] 0xAA [target data] 0x55 0x00 F8 F7 F6 F5.
 * Target basic (type 0x02): state 1B, move_dist 2B, move_energy 1B, stat_dist 2B, stat_energy 1B, det_dist 2B (LE).
 */
#include "ld2410.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "ld2410";

#define LD2410_FRAME_HEADER      0xF1F2F3F4u
#define LD2410_FRAME_HEADER_ALT  0xFAFBFCFDu  /* Some Seeed/HLK modules use FD FC FB FA */
#define LD2410_FRAME_TAIL       0xF5F6F7F8u
#define LD2410_RX_BUF_SIZE      256
#define LD2410_TASK_STACK       2048
#define LD2410_TASK_PRIO        (configMAX_PRIORITIES - 2)

static volatile ld2410_report_t s_report;
static SemaphoreHandle_t s_report_mux;
static int s_uart_num = -1;
static TaskHandle_t s_task_handle;
static bool s_running;

static void ld2410_parse_target_basic(const uint8_t *p, size_t len)
{
	/* Target basic: state(1) + move_dist(2) + move_energy(1) + stat_dist(2) + stat_energy(1) + det_dist(2) = 9 bytes */
	if (len < 9)
		return;
	uint8_t state = p[0];
	uint16_t move_dist = (uint16_t)p[1] | ((uint16_t)p[2] << 8);
	uint8_t move_energy = p[3];
	uint16_t stat_dist = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
	uint8_t stat_energy = p[6];
	uint16_t det_dist = (uint16_t)p[7] | ((uint16_t)p[8] << 8);

	if (xSemaphoreTake(s_report_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
		bool was_first = !s_report.has_data;
		s_report.state = state;
		s_report.moving_dist_cm = move_dist;
		s_report.moving_energy = move_energy;
		s_report.stationary_dist_cm = stat_dist;
		s_report.stationary_energy = stat_energy;
		s_report.detection_dist_cm = det_dist;
		s_report.has_data = true;
		xSemaphoreGive(s_report_mux);
		if (was_first) {
			ESP_LOGI(TAG, "first frame: state=%u move=%u cm stat=%u cm", (unsigned)state, (unsigned)move_dist, (unsigned)stat_dist);
		}
	}
}

static void ld2410_parse_frame(const uint8_t *payload, size_t payload_len)
{
	/* Caller passes payload only (after 6-byte header); payload_len is from header. */
	if (payload_len < 6)
		return;
	uint8_t data_type = payload[0];
	if (payload[1] != 0xAA)
		return;
	/* Payload: type(1) 0xAA(1) target_data(9) 0x55(1) 0x00(1) for basic. */
	size_t data_len = payload_len - 4; /* exclude type, 0xAA, 0x55, 0x00 */
	if (data_len < 9)
		return;
	const uint8_t *data = payload + 2;
	if (data_type == 0x02) {
		ld2410_parse_target_basic(data, data_len);
	}
	/* 0x01 = engineering mode; we only use basic (0x02). */
}

static void ld2410_task(void *arg)
{
	uint8_t *buf = malloc(LD2410_RX_BUF_SIZE);
	if (!buf) {
		ESP_LOGE(TAG, "no mem for rx buf");
		s_running = false;
		vTaskDelete(NULL);
		return;
	}
	size_t head = 0;
	int uart = (int)(intptr_t)arg;
	static bool s_first_rx_logged = false;
	int empty_reads = 0;

	while (s_running) {
		int n = uart_read_bytes(uart, buf + head, (LD2410_RX_BUF_SIZE - head) - 1, pdMS_TO_TICKS(100));
		if (n <= 0) {
			/* Keep partial data; only clear after many timeouts to avoid holding garbage forever */
			if (head > 0 && ++empty_reads >= 30) {
				head = 0;
				empty_reads = 0;
			}
			continue;
		}
		empty_reads = 0;
		if (!s_first_rx_logged) {
			s_first_rx_logged = true;
			uint8_t *p = buf + head;
			int show = n < 6 ? n : 6;
			ESP_LOGI(TAG, "rx first data: %d bytes, hex %02x %02x %02x %02x %02x %02x",
				n, show >= 1 ? p[0] : 0, show >= 2 ? p[1] : 0, show >= 3 ? p[2] : 0,
				show >= 4 ? p[3] : 0, show >= 5 ? p[4] : 0, show >= 6 ? p[5] : 0);
		}
		size_t total = head + (size_t)n;
		buf[total] = 0;
		/* Search for frame header: F1 F2 F3 F4 or FD FC FB FA (variant) in stream. */
		for (size_t i = 0; i + 6 <= total; i++) {
			uint32_t h = (uint32_t)buf[i] | ((uint32_t)buf[i+1]<<8) | ((uint32_t)buf[i+2]<<16) | ((uint32_t)buf[i+3]<<24);
			if (h != LD2410_FRAME_HEADER && h != LD2410_FRAME_HEADER_ALT)
				continue;
			uint16_t plen = (uint16_t)buf[i+4] | ((uint16_t)buf[i+5]<<8);
			size_t frame_len = 6 + plen + 4;
			if (i + frame_len > total)
				break;
			uint32_t t = (uint32_t)buf[i+6+plen] | ((uint32_t)buf[i+6+plen+1]<<8)
				| ((uint32_t)buf[i+6+plen+2]<<16) | ((uint32_t)buf[i+6+plen+3]<<24);
			if (t == LD2410_FRAME_TAIL)
				ld2410_parse_frame(buf + i + 6, plen);
			/* Advance past this frame and retry. */
			memmove(buf, buf + i + frame_len, total - (i + frame_len));
			total -= (i + frame_len);
			head = (size_t)0;
			i = (size_t)-1;
		}
		if (total >= LD2410_RX_BUF_SIZE - 1)
			head = 0;
		else
			head = total;
	}
	free(buf);
	vTaskDelete(NULL);
}

bool ld2410_init(int uart_num, int tx_gpio, int rx_gpio, uint32_t baud)
{
	if (s_uart_num >= 0) {
		ESP_LOGW(TAG, "already inited");
		return true;
	}
	s_report_mux = xSemaphoreCreateMutex();
	if (!s_report_mux) {
		ESP_LOGE(TAG, "mutex create failed");
		return false;
	}
	memset((void *)&s_report, 0, sizeof(s_report));

	uart_config_t uart_cfg = {
		.baud_rate = (int)baud,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};
	esp_err_t err = uart_driver_install(uart_num, LD2410_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_driver_install %s", esp_err_to_name(err));
		vSemaphoreDelete(s_report_mux);
		return false;
	}
	err = uart_param_config(uart_num, &uart_cfg);
	if (err != ESP_OK) {
		uart_driver_delete(uart_num);
		vSemaphoreDelete(s_report_mux);
		return false;
	}
	err = uart_set_pin(uart_num, tx_gpio, rx_gpio, -1, -1);
	if (err != ESP_OK) {
		uart_driver_delete(uart_num);
		vSemaphoreDelete(s_report_mux);
		return false;
	}

	s_uart_num = uart_num;
	s_running = true;
	BaseType_t ok = xTaskCreate(ld2410_task, "ld2410", LD2410_TASK_STACK, (void *)(intptr_t)uart_num, LD2410_TASK_PRIO, &s_task_handle);
	if (ok != pdPASS) {
		uart_driver_delete(uart_num);
		s_uart_num = -1;
		s_running = false;
		vSemaphoreDelete(s_report_mux);
		ESP_LOGE(TAG, "task create failed");
		return false;
	}
	ESP_LOGI(TAG, "LD2410 UART%d TX=%d RX=%d %lu baud", uart_num, tx_gpio, rx_gpio, (unsigned long)baud);
	return true;
}

void ld2410_deinit(void)
{
	s_running = false;
	if (s_task_handle)
		vTaskDelay(pdMS_TO_TICKS(100));
	s_task_handle = NULL;
	if (s_uart_num >= 0) {
		uart_driver_delete(s_uart_num);
		s_uart_num = -1;
	}
	if (s_report_mux) {
		vSemaphoreDelete(s_report_mux);
		s_report_mux = NULL;
	}
}

uint8_t ld2410_get_motion_level(void)
{
	uint8_t level = 0;
	if (s_report_mux && xSemaphoreTake(s_report_mux, pdMS_TO_TICKS(10)) == pdTRUE) {
		level = (s_report.state != LD2410_STATE_NONE) ? 1 : 0;
		xSemaphoreGive(s_report_mux);
	}
	return level;
}

void ld2410_get_report(ld2410_report_t *out)
{
	if (!out)
		return;
	if (s_report_mux && xSemaphoreTake(s_report_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
		memcpy(out, (const void *)&s_report, sizeof(ld2410_report_t));
		xSemaphoreGive(s_report_mux);
	} else {
		memset(out, 0, sizeof(*out));
	}
}
