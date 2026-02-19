/**
 * Camera unit settings in NVS (same namespace as HaLow: gateway).
 * Default stream quality: high when not set.
 */
#include "camera_unit_settings.h"
#include "nvs.h"

#define NVS_NAMESPACE       "gateway"
#define KEY_CAM_QUALITY     "cam_quality"
#define KEY_CAM_MIRROR      "cam_mirror"
#define KEY_CAM_ORIENT      "cam_orient"
#define KEY_ESPNOW_EN       "espnow_en"
#define KEY_LED_EN          "led_en"
#define KEY_MIC_GAIN        "mic_gain"
#define KEY_MIC_RATE        "mic_rate"

bool camera_unit_settings_get_quality(uint8_t *out)
{
	if (!out) return false;
	*out = CAMERA_QUALITY_HIGH;
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
		return false;
	uint8_t q = CAMERA_QUALITY_HIGH;
	esp_err_t e = nvs_get_u8(h, KEY_CAM_QUALITY, &q);
	nvs_close(h);
	if (e == ESP_OK && q <= CAMERA_QUALITY_HIGH)
		*out = q;
	return (e == ESP_OK);
}

bool camera_unit_settings_set_quality(uint8_t q)
{
	if (q > CAMERA_QUALITY_HIGH) return false;
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
		return false;
	nvs_set_u8(h, KEY_CAM_QUALITY, q);
	esp_err_t e = nvs_commit(h);
	nvs_close(h);
	return (e == ESP_OK);
}

bool camera_unit_settings_get_mirror(bool *out)
{
	if (!out) return false;
	*out = false;
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
		return false;
	uint8_t v = 0;
	esp_err_t e = nvs_get_u8(h, KEY_CAM_MIRROR, &v);
	nvs_close(h);
	if (e == ESP_OK)
		*out = (v != 0);
	return (e == ESP_OK || e == ESP_ERR_NVS_NOT_FOUND);
}

bool camera_unit_settings_set_mirror(bool on)
{
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
		return false;
	nvs_set_u8(h, KEY_CAM_MIRROR, on ? 1 : 0);
	esp_err_t e = nvs_commit(h);
	nvs_close(h);
	return (e == ESP_OK);
}

bool camera_unit_settings_get_orientation(uint8_t *out)
{
	if (!out) return false;
	*out = 0;
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
		return false;
	uint8_t v = 0;
	esp_err_t e = nvs_get_u8(h, KEY_CAM_ORIENT, &v);
	nvs_close(h);
	if (e == ESP_OK && v <= 3)
		*out = v;
	return (e == ESP_OK || e == ESP_ERR_NVS_NOT_FOUND);
}

bool camera_unit_settings_set_orientation(uint8_t orient)
{
	if (orient > 3) return false;
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
		return false;
	nvs_set_u8(h, KEY_CAM_ORIENT, orient);
	esp_err_t e = nvs_commit(h);
	nvs_close(h);
	return (e == ESP_OK);
}

bool camera_unit_settings_get_espnow(bool *out)
{
	if (!out) return false;
	*out = true;
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
		return false;
	uint8_t v = 1;
	esp_err_t e = nvs_get_u8(h, KEY_ESPNOW_EN, &v);
	nvs_close(h);
	if (e == ESP_OK)
		*out = (v != 0);
	return (e == ESP_OK || e == ESP_ERR_NVS_NOT_FOUND);
}

bool camera_unit_settings_set_espnow(bool on)
{
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
		return false;
	nvs_set_u8(h, KEY_ESPNOW_EN, on ? 1 : 0);
	esp_err_t e = nvs_commit(h);
	nvs_close(h);
	return (e == ESP_OK);
}

bool camera_unit_settings_get_led_enabled(bool *out)
{
	if (!out) return false;
	*out = true;
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
		return false;
	uint8_t v = 1;
	esp_err_t e = nvs_get_u8(h, KEY_LED_EN, &v);
	nvs_close(h);
	if (e == ESP_OK)
		*out = (v != 0);
	return (e == ESP_OK || e == ESP_ERR_NVS_NOT_FOUND);
}

bool camera_unit_settings_set_led_enabled(bool on)
{
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
		return false;
	nvs_set_u8(h, KEY_LED_EN, on ? 1 : 0);
	esp_err_t e = nvs_commit(h);
	nvs_close(h);
	return (e == ESP_OK);
}

bool camera_unit_settings_get_mic_gain(uint8_t *out)
{
	if (!out) return false;
	*out = 100;
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
		return false;
	uint8_t v = 100;
	esp_err_t e = nvs_get_u8(h, KEY_MIC_GAIN, &v);
	nvs_close(h);
	if (e == ESP_OK)
		*out = v;
	return (e == ESP_OK || e == ESP_ERR_NVS_NOT_FOUND);
}

bool camera_unit_settings_set_mic_gain(uint8_t gain)
{
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
		return false;
	nvs_set_u8(h, KEY_MIC_GAIN, gain);
	esp_err_t e = nvs_commit(h);
	nvs_close(h);
	return (e == ESP_OK);
}

/** mic_sample_rate: 0=16kHz, 1=8kHz, 2=3kHz */
bool camera_unit_settings_get_mic_sample_rate(uint8_t *out)
{
	if (!out) return false;
	*out = 0;
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
		return false;
	uint8_t v = 0;
	esp_err_t e = nvs_get_u8(h, KEY_MIC_RATE, &v);
	nvs_close(h);
	if (e == ESP_OK && v <= 2)
		*out = v;
	return (e == ESP_OK || e == ESP_ERR_NVS_NOT_FOUND);
}

bool camera_unit_settings_set_mic_sample_rate(uint8_t rate)
{
	if (rate > 2) return false;
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
		return false;
	nvs_set_u8(h, KEY_MIC_RATE, rate);
	esp_err_t e = nvs_commit(h);
	nvs_close(h);
	return (e == ESP_OK);
}
