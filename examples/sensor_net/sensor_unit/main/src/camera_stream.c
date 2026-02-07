/**
 * Optional camera stream for sensor unit (Xiao ESP32-S3-Sense, OV2640).
 * MJPEG over HTTP at /stream for gateway dashboard embedding.
 * When CONFIG_SENSOR_CAMERA_ENABLE is n, provides stubs (no camera).
 */
#include "camera_stream.h"
#include "esp_http_server.h"
#include <stdbool.h>

#if defined(CONFIG_SENSOR_CAMERA_ENABLE) && CONFIG_SENSOR_CAMERA_ENABLE
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <inttypes.h>
#include <string.h>
#include <stddef.h>
#endif

#if defined(CONFIG_SENSOR_CAMERA_ENABLE) && CONFIG_SENSOR_CAMERA_ENABLE
static const char *TAG = "camera_stream";

/* Xiao ESP32-S3-Sense camera slot (Seeed wiki): DVP Y9=48, Y8=11, Y7=12, Y6=14, Y5=16, Y4=18, Y3=17, Y2=15; XCLK=10, PCLK=13, VSYNC=38, HREF=47; SDA=40, SCL=39; PWDN/RESET not connected (-1). */
#define XCLK_FREQ_HZ  20000000

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static bool s_camera_ok = false;

bool camera_stream_init(void)
{
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
	s_camera_ok = true;
	ESP_LOGI(TAG, "Camera init OK (Xiao ESP32-S3-Sense)");
	return true;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
	camera_fb_t *fb = NULL;
	esp_err_t res = ESP_OK;
	size_t jpg_len = 0;
	uint8_t *jpg_buf = NULL;
	char part_buf[64];
	int64_t last_frame = esp_timer_get_time();

	if (!s_camera_ok) {
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera not available");
		return ESP_FAIL;
	}

	res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
	if (res != ESP_OK)
		return res;

	while (true) {
		fb = esp_camera_fb_get();
		if (!fb) {
			ESP_LOGE(TAG, "Camera capture failed");
			res = ESP_FAIL;
			break;
		}
		if (fb->format != PIXFORMAT_JPEG) {
			bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
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

		int64_t now = esp_timer_get_time();
		int64_t frame_ms = (now - last_frame) / 1000;
		last_frame = now;
		ESP_LOGD(TAG, "MJPEG frame %u KB, %" PRId64 " ms", (unsigned)(jpg_len / 1024), frame_ms);
	}
	return res;
}

static const httpd_uri_t uri_stream = {
	.uri       = "/stream",
	.method    = HTTP_GET,
	.handler   = stream_handler,
	.user_ctx  = NULL,
};

void camera_stream_register_uri(httpd_handle_t server)
{
	if (server && s_camera_ok)
		httpd_register_uri_handler(server, &uri_stream);
}

#else /* !CONFIG_SENSOR_CAMERA_ENABLE */

bool camera_stream_init(void)
{
	return false;
}

void camera_stream_register_uri(httpd_handle_t server)
{
	(void)server;
}

#endif /* CONFIG_SENSOR_CAMERA_ENABLE */
