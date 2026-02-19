/**
 * Camera stream for camera sensor unit (XIAO ESP32-S3-Sense, OV2640).
 * MJPEG at /live/stream (embedded by /live page; no standalone /stream).
 * /snapshot = single JPEG.
 * Query: ?quality=low|medium|high|auto (default medium). auto = adapt quality to keep ~20 fps.
 * Compression: low/auto use HVGA (480x320) for smaller frames; medium/high use VGA. Adaptive can drop to HVGA when struggling.
 * CONFIG_SENSOR_CAMERA_ENABLE is always y for this firmware.
 */
#include "camera_stream.h"
#include "camera_unit_settings.h"
#include "esp_http_server.h"
#include "esp_idf_version.h"
#include <stdbool.h>

/* Async handler API exists from ESP-IDF 5.2 */
#if defined(CONFIG_SENSOR_CAMERA_ENABLE) && CONFIG_SENSOR_CAMERA_ENABLE
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#define CAMERA_STREAM_USE_ASYNC  1
#else
#define CAMERA_STREAM_USE_ASYNC  0
#endif
#endif

#if defined(CONFIG_SENSOR_CAMERA_ENABLE) && CONFIG_SENSOR_CAMERA_ENABLE
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#endif

#if defined(CONFIG_SENSOR_CAMERA_ENABLE) && CONFIG_SENSOR_CAMERA_ENABLE
static const char *TAG = "camera_stream";

/* XIAO ESP32-S3-Sense camera slot (Seeed wiki): DVP Y9=48, Y8=11, Y7=12, Y6=14, Y5=16, Y4=18, Y3=17, Y2=15; XCLK=10, PCLK=13, VSYNC=38, HREF=47; SDA=40, SCL=39; PWDN/RESET not connected (-1). */
#define XCLK_FREQ_HZ  20000000

#define STREAM_TARGET_FPS   20
#define FRAME_INTERVAL_US   (1000000 / STREAM_TARGET_FPS)

/* Adaptive: adjust quality every ADAPT_EVERY frames; slow threshold = drop quality, fast count = raise quality. */
#define ADAPT_EVERY         5
#define SLOW_FRAME_US       (1000000 / 18)   /* below ~18 fps -> reduce quality */
#define FAST_FRAME_US       (1000000 / 22)   /* above ~22 fps for a few frames -> try better quality */
#define ADAPT_QUALITY_MIN   4   /* best quality in auto when link keeps up (lower = better; 4 better than 6) */
#define ADAPT_QUALITY_MAX   32
#define ADAPT_FAST_COUNT    3
/* When adaptive quality hits max and we're still slow, drop to HVGA for extra compression. */
#define ADAPT_RESOLUTION_DROP_AT_MAX  1

/* Quality levels: low, medium, high (OV2640). auto = adaptive in stream loop. */
typedef enum { QUALITY_LOW = 0, QUALITY_MEDIUM, QUALITY_HIGH, QUALITY_AUTO, QUALITY_COUNT } stream_quality_t;
static const int s_quality_jpeg[] = { 25, 12, 8 };  /* low=25, medium=12, high=8 (same as sensor_unit_s3) */

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static bool s_camera_ok = false;
/** Ref count: stream/snapshot handlers increment on entry, decrement on exit. When 0, camera is deinited for power saving. */
static uint32_t s_camera_ref = 0;
static SemaphoreHandle_t s_camera_mutex = NULL;

/** Parse URI for ?quality=low|medium|high|auto. Default from NVS (camera_unit_settings). */
static stream_quality_t parse_quality_from_uri(const char *uri)
{
	if (uri) {
		const char *q = strstr(uri, "quality=");
		if (q) {
			q += 8;
			if (strncmp(q, "low", 3) == 0)   return QUALITY_LOW;
			if (strncmp(q, "high", 4) == 0)  return QUALITY_HIGH;
			if (strncmp(q, "auto", 4) == 0)  return QUALITY_AUTO;
			return QUALITY_MEDIUM;
		}
	}
	uint8_t nvs_q = QUALITY_MEDIUM;
	camera_unit_settings_get_quality(&nvs_q);
	return (stream_quality_t)(nvs_q <= QUALITY_HIGH ? nvs_q : QUALITY_MEDIUM);
}

void camera_stream_ensure_mutex(void)
{
	if (s_camera_mutex == NULL)
		s_camera_mutex = xSemaphoreCreateMutex();
}

bool camera_stream_init(void)
{
	camera_stream_ensure_mutex();
	if (s_camera_mutex == NULL)
		return false;
	camera_config_t config = {
		.pin_pwdn  = -1,
		.pin_reset = -1,
		.pin_xclk  = 10,
		.pin_sccb_sda = 40,
		.pin_sccb_scl = 39,
		.pin_d7 = 48,
		.pin_d6 = 11,
		.pin_d5 = 12,
		.pin_d4 = 14,
		.pin_d3 = 16,
		.pin_d2 = 18,
		.pin_d1 = 17,
		.pin_d0 = 15,
		.pin_vsync = 38,
		.pin_href  = 47,
		.pin_pclk  = 13,
		.xclk_freq_hz = XCLK_FREQ_HZ,
		.ledc_timer = LEDC_TIMER_0,
		.ledc_channel = LEDC_CHANNEL_0,
		.pixel_format = PIXFORMAT_JPEG,
		.frame_size = FRAMESIZE_VGA,
		.jpeg_quality = 12,
		.fb_count = 1,
		.grab_mode = CAMERA_GRAB_WHEN_EMPTY,
	};
	esp_err_t err = esp_camera_init(&config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
		return false;
	}
	/* OV2640 needs a short delay after init before mirror/flip register writes apply reliably. */
	vTaskDelay(pdMS_TO_TICKS(150));
	sensor_t *sensor = esp_camera_sensor_get();
	uint8_t orient = 0;
	camera_unit_settings_get_orientation(&orient);
	bool mirror = false;
	camera_unit_settings_get_mirror(&mirror);
	/* Orientation 1 = 180° (hmirror + vflip). Mirror image = horizontal flip. */
	int flip_h = (orient == 1) ? 1 : (mirror ? 1 : 0);
	int flip_v = (orient == 1) ? 1 : 0;
	if (!sensor) {
		ESP_LOGW(TAG, "Camera orientation: no sensor (orient=%u)", (unsigned)orient);
	} else {
		if (sensor->set_hmirror) {
			int r = sensor->set_hmirror(sensor, flip_h);
			ESP_LOGI(TAG, "Camera orient=%u mirror=%d -> hmirror=%d (ret=%d)", (unsigned)orient, mirror ? 1 : 0, flip_h, r);
		} else {
			ESP_LOGW(TAG, "Camera orientation: sensor has no set_hmirror");
		}
		if (sensor->set_vflip) {
			int r = sensor->set_vflip(sensor, flip_v);
			(void)r;
		} else {
			ESP_LOGW(TAG, "Camera orientation: sensor has no set_vflip");
		}
	}
	s_camera_ok = true;
	ESP_LOGI(TAG, "Camera init OK (XIAO ESP32-S3-Sense)");
	return true;
}

bool camera_stream_get_one_jpeg(uint8_t *buf, size_t buf_size, size_t *out_len)
{
	if (!s_camera_ok || !buf || !out_len || buf_size == 0) {
		if (out_len) *out_len = 0;
		return false;
	}
	camera_fb_t *fb = esp_camera_fb_get();
	if (!fb) return false;
	size_t jpg_len = 0;
	uint8_t *jpg_buf = fb->buf;
	uint8_t *alloc_buf = NULL;
	if (fb->format != PIXFORMAT_JPEG) {
		if (!frame2jpg(fb, 12, &alloc_buf, &jpg_len)) {
			esp_camera_fb_return(fb);
			return false;
		}
		jpg_buf = alloc_buf;
	} else {
		jpg_len = fb->len;
	}
	bool ok = (jpg_len <= buf_size && jpg_len > 0);
	if (ok) {
		memcpy(buf, jpg_buf, jpg_len);
		*out_len = jpg_len;
	}
	if (alloc_buf) free(alloc_buf);
	esp_camera_fb_return(fb);
	return ok;
}

/** Caller must hold s_camera_mutex. Deinit camera when ref goes to 0 (power saving). */
static void camera_stream_deinit_locked(void)
{
	esp_camera_deinit();
	s_camera_ok = false;
	ESP_LOGI(TAG, "Camera deinit (no viewers)");
}

#if CAMERA_STREAM_USE_ASYNC

/** Context for stream task: run MJPEG in separate task so HTTP server can serve other pages. */
typedef struct {
	httpd_req_t *req_orig;
	httpd_req_t *req_async;
	stream_quality_t quality;
} stream_ctx_t;

#define STREAM_TASK_STACK  4096
#define STREAM_TASK_PRIO   5

static void stream_task(void *pv)
{
	stream_ctx_t *ctx = (stream_ctx_t *)pv;
	httpd_req_t *req = ctx->req_async;
	camera_fb_t *fb = NULL;
	esp_err_t res = ESP_OK;
	size_t jpg_len = 0;
	uint8_t *jpg_buf = NULL;
	char part_buf[64];
	stream_quality_t quality = ctx->quality;
	int jpeg_quality;
	bool adaptive = (quality == QUALITY_AUTO);
	int fast_frames = 0;
	int frame_count = 0;
	bool use_hvga = (quality == QUALITY_LOW || quality == QUALITY_AUTO);
	sensor_t *sensor = esp_camera_sensor_get();

	if (adaptive)
		jpeg_quality = 10;
	else
		jpeg_quality = s_quality_jpeg[quality];

	res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
	if (res != ESP_OK)
		goto done;

	if (sensor && sensor->set_framesize) {
		if (sensor->set_framesize(sensor, use_hvga ? FRAMESIZE_HVGA : FRAMESIZE_VGA) == 0 && use_hvga)
			ESP_LOGI(TAG, "Stream: HVGA (compression for performance)");
	}

	while (true) {
		int64_t frame_start_us = esp_timer_get_time();

		uint8_t orient = 0;
		camera_unit_settings_get_orientation(&orient);
		bool mirror = false;
		camera_unit_settings_get_mirror(&mirror);
		int flip_h = (orient == 1) ? 1 : (mirror ? 1 : 0);
		int flip_v = (orient == 1) ? 1 : 0;
		if (sensor && sensor->set_hmirror)
			sensor->set_hmirror(sensor, flip_h);
		if (sensor && sensor->set_vflip)
			sensor->set_vflip(sensor, flip_v);

		fb = esp_camera_fb_get();
		if (!fb) {
			ESP_LOGE(TAG, "Camera capture failed");
			res = ESP_FAIL;
			break;
		}
		if (sensor && sensor->set_quality)
			sensor->set_quality(sensor, jpeg_quality);
		if (fb->format != PIXFORMAT_JPEG) {
			int q = (jpeg_quality <= 10) ? (80 - jpeg_quality * 5) : (100 - jpeg_quality);
			if (q < 10) q = 10;
			if (q > 95) q = 95;
			bool ok = frame2jpg(fb, q, &jpg_buf, &jpg_len);
			if (!ok) {
				esp_camera_fb_return(fb);
				res = ESP_FAIL;
				break;
			}
		} else {
			jpg_len = fb->len;
			jpg_buf = fb->buf;
		}

		res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
		if (res == ESP_OK) {
			size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned)jpg_len);
			res = httpd_resp_send_chunk(req, part_buf, hlen);
		}
		if (res == ESP_OK)
			res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len);

		if (fb->format != PIXFORMAT_JPEG && jpg_buf)
			free(jpg_buf);
		esp_camera_fb_return(fb);
		if (res != ESP_OK)
			break;

		int64_t now_us = esp_timer_get_time();
		int64_t elapsed_us = now_us - frame_start_us;

		if (adaptive) {
			if (elapsed_us > (int64_t)SLOW_FRAME_US) {
				if (jpeg_quality < ADAPT_QUALITY_MAX) {
					jpeg_quality += 3;
					if (jpeg_quality > ADAPT_QUALITY_MAX) jpeg_quality = ADAPT_QUALITY_MAX;
				}
#if ADAPT_RESOLUTION_DROP_AT_MAX
				if (jpeg_quality >= ADAPT_QUALITY_MAX && sensor && sensor->set_framesize && !use_hvga) {
					if (sensor->set_framesize(sensor, FRAMESIZE_HVGA) == 0) {
						use_hvga = true;
						ESP_LOGI(TAG, "adapt: drop to HVGA for more compression");
					}
				}
#endif
				fast_frames = 0;
			} else if (elapsed_us < (int64_t)FAST_FRAME_US) {
				fast_frames++;
				if ((frame_count % ADAPT_EVERY) == 0 && fast_frames >= ADAPT_FAST_COUNT && jpeg_quality > ADAPT_QUALITY_MIN) {
					jpeg_quality--;
					fast_frames = 0;
				}
			} else {
				fast_frames = 0;
			}
		}
		frame_count++;

		if (elapsed_us < FRAME_INTERVAL_US) {
			int64_t delay_us = FRAME_INTERVAL_US - elapsed_us;
			if (delay_us >= 1000)
				vTaskDelay(pdMS_TO_TICKS((uint32_t)(delay_us / 1000)));
			else if (delay_us > 0)
				esp_rom_delay_us((uint32_t)delay_us);
		}
	}

	/* Final empty chunk if we sent any headers (required by chunked encoding) */
	(void)httpd_resp_send_chunk(req, NULL, 0);

done:
	/* complete() must receive the async request (from begin); it frees that copy. req_orig can be invalid if client disconnected. */
	if (ctx->req_async != NULL) {
		httpd_req_async_handler_complete(ctx->req_async);
		ctx->req_async = NULL;
	}
	if (xSemaphoreTake(s_camera_mutex, portMAX_DELAY) == pdTRUE) {
		if (s_camera_ref > 0)
			s_camera_ref--;
		if (s_camera_ref == 0)
			camera_stream_deinit_locked();
		xSemaphoreGive(s_camera_mutex);
	}
	free(ctx);
	vTaskDelete(NULL);
}

static esp_err_t stream_handler(httpd_req_t *req)
{
	/* Ref count: init on first viewer, deinit when last viewer leaves (power saving). */
	if (xSemaphoreTake(s_camera_mutex, portMAX_DELAY) == pdTRUE) {
		if (!s_camera_ok && !camera_stream_init()) {
			xSemaphoreGive(s_camera_mutex);
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera init failed");
			return ESP_FAIL;
		}
		s_camera_ref++;
		xSemaphoreGive(s_camera_mutex);
	}

	httpd_req_t *req_async = NULL;
	esp_err_t err = httpd_req_async_handler_begin(req, &req_async);
	if (err != ESP_OK || req_async == NULL) {
		if (xSemaphoreTake(s_camera_mutex, portMAX_DELAY) == pdTRUE) {
			if (s_camera_ref > 0) s_camera_ref--;
			if (s_camera_ref == 0) camera_stream_deinit_locked();
			xSemaphoreGive(s_camera_mutex);
		}
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Async begin failed");
		return ESP_FAIL;
	}

	stream_ctx_t *ctx = (stream_ctx_t *)malloc(sizeof(stream_ctx_t));
	if (ctx == NULL) {
		httpd_req_async_handler_complete(req_async);
		if (xSemaphoreTake(s_camera_mutex, portMAX_DELAY) == pdTRUE) {
			if (s_camera_ref > 0) s_camera_ref--;
			if (s_camera_ref == 0) camera_stream_deinit_locked();
			xSemaphoreGive(s_camera_mutex);
		}
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
		return ESP_FAIL;
	}
	ctx->req_orig = req;
	ctx->req_async = req_async;
	ctx->quality = parse_quality_from_uri(req->uri);

	if (xTaskCreate(stream_task, "stream", STREAM_TASK_STACK, ctx, STREAM_TASK_PRIO, NULL) != pdPASS) {
		free(ctx);
		httpd_req_async_handler_complete(req_async);  /* async handle from begin; complete frees it */
		if (xSemaphoreTake(s_camera_mutex, portMAX_DELAY) == pdTRUE) {
			if (s_camera_ref > 0) s_camera_ref--;
			if (s_camera_ref == 0) camera_stream_deinit_locked();
			xSemaphoreGive(s_camera_mutex);
		}
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Task create failed");
		return ESP_FAIL;
	}
	/* Handler returns immediately so the HTTP server task can accept / and /api/settings. */
	return ESP_OK;
}

#else /* blocking fallback for ESP-IDF < 5.2 */

static esp_err_t stream_handler(httpd_req_t *req)
{
	camera_fb_t *fb = NULL;
	esp_err_t res = ESP_OK;
	size_t jpg_len = 0;
	uint8_t *jpg_buf = NULL;
	char part_buf[64];

	if (xSemaphoreTake(s_camera_mutex, portMAX_DELAY) == pdTRUE) {
		if (!s_camera_ok && !camera_stream_init()) {
			xSemaphoreGive(s_camera_mutex);
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera init failed");
			return ESP_FAIL;
		}
		s_camera_ref++;
		xSemaphoreGive(s_camera_mutex);
	}

	res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
	if (res != ESP_OK) {
		if (xSemaphoreTake(s_camera_mutex, portMAX_DELAY) == pdTRUE) {
			if (s_camera_ref > 0) s_camera_ref--;
			if (s_camera_ref == 0) camera_stream_deinit_locked();
			xSemaphoreGive(s_camera_mutex);
		}
		return res;
	}

	stream_quality_t quality = parse_quality_from_uri(req->uri);
	int jpeg_quality = (quality == QUALITY_AUTO) ? 10 : s_quality_jpeg[quality];
	bool adaptive = (quality == QUALITY_AUTO);
	int fast_frames = 0;
	int frame_count = 0;
	bool use_hvga = (quality == QUALITY_LOW || quality == QUALITY_AUTO);
	sensor_t *sensor = esp_camera_sensor_get();

	if (sensor && sensor->set_framesize)
		sensor->set_framesize(sensor, use_hvga ? FRAMESIZE_HVGA : FRAMESIZE_VGA);

	while (true) {
		int64_t frame_start_us = esp_timer_get_time();
		uint8_t orient = 0;
		camera_unit_settings_get_orientation(&orient);
		bool mirror = false;
		camera_unit_settings_get_mirror(&mirror);
		int flip_h = (orient == 1) ? 1 : (mirror ? 1 : 0);
		int flip_v = (orient == 1) ? 1 : 0;
		if (sensor && sensor->set_hmirror)
			sensor->set_hmirror(sensor, flip_h);
		if (sensor && sensor->set_vflip)
			sensor->set_vflip(sensor, flip_v);

		fb = esp_camera_fb_get();
		if (!fb) {
			res = ESP_FAIL;
			break;
		}
		if (sensor && sensor->set_quality)
			sensor->set_quality(sensor, jpeg_quality);
		if (fb->format != PIXFORMAT_JPEG) {
			int q = (jpeg_quality <= 10) ? (80 - jpeg_quality * 5) : (100 - jpeg_quality);
			if (q < 10) q = 10;
			if (q > 95) q = 95;
			if (!frame2jpg(fb, q, &jpg_buf, &jpg_len)) {
				esp_camera_fb_return(fb);
				res = ESP_FAIL;
				break;
			}
		} else {
			jpg_len = fb->len;
			jpg_buf = fb->buf;
		}

		res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
		if (res == ESP_OK) {
			size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned)jpg_len);
			res = httpd_resp_send_chunk(req, part_buf, hlen);
		}
		if (res == ESP_OK)
			res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len);

		if (fb->format != PIXFORMAT_JPEG && jpg_buf)
			free(jpg_buf);
		esp_camera_fb_return(fb);
		if (res != ESP_OK)
			break;

		int64_t elapsed_us = esp_timer_get_time() - frame_start_us;
		if (adaptive) {
			if (elapsed_us > (int64_t)SLOW_FRAME_US && jpeg_quality < ADAPT_QUALITY_MAX) {
				jpeg_quality += 3;
				if (jpeg_quality > ADAPT_QUALITY_MAX) jpeg_quality = ADAPT_QUALITY_MAX;
			} else if (elapsed_us < (int64_t)FAST_FRAME_US) {
				fast_frames++;
				if ((frame_count % ADAPT_EVERY) == 0 && fast_frames >= ADAPT_FAST_COUNT && jpeg_quality > ADAPT_QUALITY_MIN) {
					jpeg_quality--;
					fast_frames = 0;
				}
			} else {
				fast_frames = 0;
			}
		}
		frame_count++;

		if (elapsed_us < FRAME_INTERVAL_US) {
			int64_t delay_us = FRAME_INTERVAL_US - elapsed_us;
			if (delay_us >= 1000)
				vTaskDelay(pdMS_TO_TICKS((uint32_t)(delay_us / 1000)));
			else if (delay_us > 0)
				esp_rom_delay_us((uint32_t)delay_us);
		}
	}

	(void)httpd_resp_send_chunk(req, NULL, 0);
	if (xSemaphoreTake(s_camera_mutex, portMAX_DELAY) == pdTRUE) {
		if (s_camera_ref > 0) s_camera_ref--;
		if (s_camera_ref == 0) camera_stream_deinit_locked();
		xSemaphoreGive(s_camera_mutex);
	}
	return res;
}

#endif /* CAMERA_STREAM_USE_ASYNC */

static esp_err_t snapshot_handler(httpd_req_t *req)
{
	/* Ref count: init on first use, deinit when last user leaves (power saving). */
	if (xSemaphoreTake(s_camera_mutex, portMAX_DELAY) == pdTRUE) {
		if (!s_camera_ok && !camera_stream_init()) {
			xSemaphoreGive(s_camera_mutex);
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera init failed");
			return ESP_FAIL;
		}
		s_camera_ref++;
		xSemaphoreGive(s_camera_mutex);
	}

	camera_fb_t *fb = esp_camera_fb_get();
	if (!fb) {
		if (xSemaphoreTake(s_camera_mutex, portMAX_DELAY) == pdTRUE) {
			if (s_camera_ref > 0) s_camera_ref--;
			if (s_camera_ref == 0) camera_stream_deinit_locked();
			xSemaphoreGive(s_camera_mutex);
		}
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Capture failed");
		return ESP_FAIL;
	}
	size_t jpg_len = fb->len;
	uint8_t *jpg_buf = fb->buf;
	uint8_t *alloc_buf = NULL;
	if (fb->format != PIXFORMAT_JPEG) {
		if (!frame2jpg(fb, 12, &alloc_buf, &jpg_len)) {
			esp_camera_fb_return(fb);
			if (xSemaphoreTake(s_camera_mutex, portMAX_DELAY) == pdTRUE) {
				if (s_camera_ref > 0) s_camera_ref--;
				if (s_camera_ref == 0) camera_stream_deinit_locked();
				xSemaphoreGive(s_camera_mutex);
			}
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JPEG conversion failed");
			return ESP_FAIL;
		}
		jpg_buf = alloc_buf;
	}
	httpd_resp_set_type(req, "image/jpeg");
	httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=snapshot.jpg");
	esp_err_t res = httpd_resp_send(req, (const char *)jpg_buf, jpg_len);
	if (alloc_buf) free(alloc_buf);
	esp_camera_fb_return(fb);

	/* Snapshot done: release ref; deinit if no other viewers. */
	if (xSemaphoreTake(s_camera_mutex, portMAX_DELAY) == pdTRUE) {
		if (s_camera_ref > 0) s_camera_ref--;
		if (s_camera_ref == 0) camera_stream_deinit_locked();
		xSemaphoreGive(s_camera_mutex);
	}
	return res;
}

static const httpd_uri_t uri_stream = {
	.uri       = "/live/stream",
	.method    = HTTP_GET,
	.handler   = stream_handler,
	.user_ctx  = NULL,
};
static const httpd_uri_t uri_snapshot = {
	.uri       = "/snapshot",
	.method    = HTTP_GET,
	.handler   = snapshot_handler,
	.user_ctx  = NULL,
};

void camera_stream_register_uri(httpd_handle_t server)
{
	if (!server) return;
	/* Always register stream/snapshot; camera init is lazy on first request (power saving). */
	httpd_register_uri_handler(server, &uri_stream);
	httpd_register_uri_handler(server, &uri_snapshot);
}

#else /* !CONFIG_SENSOR_CAMERA_ENABLE */

void camera_stream_ensure_mutex(void)
{
}

bool camera_stream_init(void)
{
	return false;
}

void camera_stream_register_uri(httpd_handle_t server)
{
	(void)server;
}

bool camera_stream_get_one_jpeg(uint8_t *buf, size_t buf_size, size_t *out_len)
{
	(void)buf;
	(void)buf_size;
	if (out_len) *out_len = 0;
	return false;
}

#endif /* CONFIG_SENSOR_CAMERA_ENABLE */
