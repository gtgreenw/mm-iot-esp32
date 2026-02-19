/**
 * PDM microphone stream for Seeed Studio XIAO ESP32-S3-Sense (integrated PDM mic).
 * Pins per Seeed wiki: CLK=GPIO42, DATA (DIN)=GPIO41. 16 kHz, 16-bit mono PCM.
 * Uses ESP32-S3 hardware PDM-to-PCM converter (I2S0) for decoded audio.
 */
#include "mic_stream.h"
#include "camera_unit_settings.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_idf_version.h"
#include "driver/i2s_common.h"
#include "driver/i2s_pdm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>

/* Async handler API (httpd_req_async_handler_begin/complete) exists from ESP-IDF 5.2 */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#define MIC_STREAM_USE_ASYNC  1
#else
#define MIC_STREAM_USE_ASYNC  0
#endif

#if SOC_I2S_SUPPORTS_PDM_RX

static const char *TAG = "mic_stream";

/* Sample rate options: 0=16kHz, 1=8kHz, 2=3kHz. Takes effect at init (reboot). */
static const uint32_t s_mic_rates_hz[] = { 16000, 8000, 3000 };

/* XIAO ESP32-S3-Sense: integrated PDM mic (MP34DT06JTR), 16-bit mono */
#define MIC_GPIO_CLK        42   /* PDM clock */
#define MIC_GPIO_DATA       41   /* PDM data in */
#define MIC_READ_CHUNK      1024
#define MIC_READ_TIMEOUT_MS 100

/* ESP32-S3 PDM-to-PCM converter is on I2S0; must use I2S_NUM_0 for decoded PCM output */
#define MIC_I2S_PORT        I2S_NUM_0

static i2s_chan_handle_t s_rx_handle = NULL;
static bool s_ready = false;
static uint32_t s_sample_rate_hz = 16000;

/* 44-byte WAV header for 16-bit mono; data size 0x7FFFFFFF for streaming (browser keeps reading) */
static void build_wav_header(uint8_t *out, uint32_t sample_rate_hz)
{
	uint32_t data_size = 0x7FFFFFFF;
	uint32_t riff_size = 36 + data_size;
	uint32_t byte_rate = sample_rate_hz * 2;
	memcpy(out, "RIFF", 4);
	out[4] = (uint8_t)(riff_size);
	out[5] = (uint8_t)(riff_size >> 8);
	out[6] = (uint8_t)(riff_size >> 16);
	out[7] = (uint8_t)(riff_size >> 24);
	memcpy(out + 8, "WAVEfmt ", 8);
	out[16] = 16;
	out[17] = 0;
	out[18] = 0;
	out[19] = 0;
	out[20] = 1;   /* PCM */
	out[21] = 0;
	out[22] = 1;   /* mono */
	out[23] = 0;
	out[24] = (uint8_t)(sample_rate_hz);
	out[25] = (uint8_t)(sample_rate_hz >> 8);
	out[26] = (uint8_t)(sample_rate_hz >> 16);
	out[27] = (uint8_t)(sample_rate_hz >> 24);
	out[28] = (uint8_t)(byte_rate);
	out[29] = (uint8_t)(byte_rate >> 8);
	out[30] = (uint8_t)(byte_rate >> 16);
	out[31] = (uint8_t)(byte_rate >> 24);
	out[32] = 2;   /* block align */
	out[33] = 0;
	out[34] = 16;  /* bits per sample */
	out[35] = 0;
	memcpy(out + 36, "data", 4);
	out[40] = (uint8_t)(data_size);
	out[41] = (uint8_t)(data_size >> 8);
	out[42] = (uint8_t)(data_size >> 16);
	out[43] = (uint8_t)(data_size >> 24);
}

#if MIC_STREAM_USE_ASYNC

#define AUDIO_TASK_STACK  3072
#define AUDIO_TASK_PRIO   5

typedef struct {
	httpd_req_t *req_orig;
	httpd_req_t *req_async;
	uint8_t gain_pct;
} audio_ctx_t;

static void audio_task(void *pv)
{
	audio_ctx_t *ctx = (audio_ctx_t *)pv;
	httpd_req_t *req = ctx->req_async;
	uint8_t gain_pct = ctx->gain_pct;

	httpd_resp_set_type(req, "audio/wav");
	httpd_resp_set_hdr(req, "Cache-Control", "no-store");

	uint8_t wav_header[44];
	build_wav_header(wav_header, s_sample_rate_hz);
	if (httpd_resp_send_chunk(req, (const char *)wav_header, sizeof(wav_header)) != ESP_OK) {
		httpd_req_async_handler_complete(ctx->req_orig);
		free(ctx);
		vTaskDelete(NULL);
		return;
	}

	uint8_t buf[MIC_READ_CHUNK];
	esp_err_t res;
	while (s_ready && s_rx_handle) {
		size_t bytes_read = 0;
		res = i2s_channel_read(s_rx_handle, buf, sizeof(buf), &bytes_read, MIC_READ_TIMEOUT_MS);
		if (res != ESP_OK || bytes_read == 0)
			break;
		if (gain_pct != 100 && (bytes_read & 1) == 0) {
			for (size_t i = 0; i < bytes_read; i += 2) {
				int32_t v = (int16_t)(buf[i] | (buf[i + 1] << 8));
				v = (v * (int32_t)gain_pct) / 100;
				if (v > 32767) v = 32767;
				if (v < -32768) v = -32768;
				int16_t s = (int16_t)v;
				buf[i] = (uint8_t)(s & 0xff);
				buf[i + 1] = (uint8_t)(s >> 8);
			}
		}
		if (httpd_resp_send_chunk(req, (const char *)buf, bytes_read) != ESP_OK)
			break;
	}
	httpd_resp_send_chunk(req, NULL, 0);
	httpd_req_async_handler_complete(ctx->req_orig);
	free(ctx);
	vTaskDelete(NULL);
}

static esp_err_t audio_handler(httpd_req_t *req)
{
	if (!s_ready || !s_rx_handle) {
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Mic not available");
		return ESP_FAIL;
	}
	uint8_t gain_pct = 100;
	camera_unit_settings_get_mic_gain(&gain_pct);

	httpd_req_t *req_async = NULL;
	esp_err_t err = httpd_req_async_handler_begin(req, &req_async);
	if (err != ESP_OK || req_async == NULL) {
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Async begin failed");
		return ESP_FAIL;
	}

	audio_ctx_t *ctx = (audio_ctx_t *)malloc(sizeof(audio_ctx_t));
	if (ctx == NULL) {
		httpd_req_async_handler_complete(req);
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
		return ESP_FAIL;
	}
	ctx->req_orig = req;
	ctx->req_async = req_async;
	ctx->gain_pct = gain_pct;

	if (xTaskCreate(audio_task, "audio", AUDIO_TASK_STACK, ctx, AUDIO_TASK_PRIO, NULL) != pdPASS) {
		free(ctx);
		httpd_req_async_handler_complete(req);
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Task create failed");
		return ESP_FAIL;
	}
	return ESP_OK;
}

#else /* blocking fallback for ESP-IDF < 5.2 */

static esp_err_t audio_handler(httpd_req_t *req)
{
	if (!s_ready || !s_rx_handle) {
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Mic not available");
		return ESP_FAIL;
	}
	uint8_t gain_pct = 100;
	camera_unit_settings_get_mic_gain(&gain_pct);

	httpd_resp_set_type(req, "audio/wav");
	httpd_resp_set_hdr(req, "Cache-Control", "no-store");

	uint8_t wav_header[44];
	build_wav_header(wav_header, s_sample_rate_hz);
	if (httpd_resp_send_chunk(req, (const char *)wav_header, sizeof(wav_header)) != ESP_OK)
		return ESP_FAIL;

	uint8_t buf[MIC_READ_CHUNK];
	esp_err_t res;
	while (true) {
		size_t bytes_read = 0;
		res = i2s_channel_read(s_rx_handle, buf, sizeof(buf), &bytes_read, MIC_READ_TIMEOUT_MS);
		if (res != ESP_OK || bytes_read == 0)
			break;
		if (gain_pct != 100 && (bytes_read & 1) == 0) {
			for (size_t i = 0; i < bytes_read; i += 2) {
				int32_t v = (int16_t)(buf[i] | (buf[i + 1] << 8));
				v = (v * (int32_t)gain_pct) / 100;
				if (v > 32767) v = 32767;
				if (v < -32768) v = -32768;
				int16_t s = (int16_t)v;
				buf[i] = (uint8_t)(s & 0xff);
				buf[i + 1] = (uint8_t)(s >> 8);
			}
		}
		if (httpd_resp_send_chunk(req, (const char *)buf, bytes_read) != ESP_OK)
			break;
	}
	httpd_resp_send_chunk(req, NULL, 0);
	return ESP_OK;
}

#endif /* MIC_STREAM_USE_ASYNC */

bool mic_stream_init(void)
{
	uint8_t rate_sel = 0;
	camera_unit_settings_get_mic_sample_rate(&rate_sel);
	if (rate_sel > 2) rate_sel = 0;
	s_sample_rate_hz = s_mic_rates_hz[rate_sel];

	i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(MIC_I2S_PORT, I2S_ROLE_MASTER);
	chan_cfg.dma_frame_num = 240;
	chan_cfg.auto_clear = true;
	esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &s_rx_handle);
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "I2S channel alloc failed: %s", esp_err_to_name(ret));
		return false;
	}

	/* PCM format: I2S0 has PDM-to-PCM on ESP32-S3; slot default is PCM when SOC_I2S_SUPPORTS_PDM2PCM */
	i2s_pdm_rx_config_t pdm_rx_cfg = {
		.clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(s_sample_rate_hz),
		.slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
		.gpio_cfg = {
			.clk = MIC_GPIO_CLK,
			.din = MIC_GPIO_DATA,
			.invert_flags = { .clk_inv = 0 },
		},
	};
	ret = i2s_channel_init_pdm_rx_mode(s_rx_handle, &pdm_rx_cfg);
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "I2S PDM RX init failed at %lu Hz: %s", (unsigned long)s_sample_rate_hz, esp_err_to_name(ret));
		i2s_del_channel(s_rx_handle);
		s_rx_handle = NULL;
		return false;
	}
	ret = i2s_channel_enable(s_rx_handle);
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "I2S enable failed: %s", esp_err_to_name(ret));
		i2s_del_channel(s_rx_handle);
		s_rx_handle = NULL;
		return false;
	}
	s_ready = true;
	ESP_LOGI(TAG, "Mic init OK (XIAO Sense): %lu Hz 16-bit mono PCM, /audio", (unsigned long)s_sample_rate_hz);
	return true;
}

bool mic_stream_ready(void)
{
	return s_ready;
}

void mic_stream_register_uri(void *httpd_handle)
{
	httpd_handle_t server = (httpd_handle_t)httpd_handle;
	if (!server || !s_ready) return;
	static const httpd_uri_t uri_audio = {
		.uri    = "/audio",
		.method = HTTP_GET,
		.handler = audio_handler,
		.user_ctx = NULL,
	};
	httpd_register_uri_handler(server, &uri_audio);
}

#else /* !SOC_I2S_SUPPORTS_PDM_RX */

bool mic_stream_init(void)
{
	ESP_LOGW("mic_stream", "PDM RX not supported on this target");
	return false;
}

bool mic_stream_ready(void)
{
	return false;
}

void mic_stream_register_uri(void *httpd_handle)
{
	(void)httpd_handle;
}

#endif
