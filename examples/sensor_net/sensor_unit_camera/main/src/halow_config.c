/**
 * HaLow WiFi credentials and "configured" flag in NVS.
 * Same NVS namespace as gateway so config can be shared.
 */
#include "halow_config.h"
#include "nvs.h"
#include <string.h>

#define NVS_NAMESPACE   "gateway"
#define KEY_CONFIGURED  "halow_ok"
#define KEY_SSID       "halow_ssid"
#define KEY_PASSPHRASE "halow_pass"
#define KEY_LINK       "halow_link"

#define SSID_MAX       32
#define PASS_MAX       64

bool halow_config_is_configured(void)
{
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
		return false;
	uint8_t ok = 0;
	nvs_get_u8(h, KEY_CONFIGURED, &ok);
	nvs_close(h);
	return (ok != 0);
}

bool halow_config_load(char *ssid, size_t ssid_size, char *passphrase, size_t pass_size)
{
	if (!ssid || ssid_size == 0 || !passphrase || pass_size == 0)
		return false;
	ssid[0] = passphrase[0] = '\0';
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
		return false;
	size_t len;
	esp_err_t e;
	len = ssid_size;
	e = nvs_get_str(h, KEY_SSID, ssid, &len);
	if (e != ESP_OK) { nvs_close(h); return false; }
	len = pass_size;
	e = nvs_get_str(h, KEY_PASSPHRASE, passphrase, &len);
	nvs_close(h);
	return (e == ESP_OK);
}

bool halow_config_save(const char *ssid, const char *passphrase)
{
	if (!ssid || !passphrase)
		return false;
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
		return false;
	nvs_set_u8(h, KEY_CONFIGURED, 1);
	nvs_set_str(h, KEY_SSID, ssid);
	nvs_set_str(h, KEY_PASSPHRASE, passphrase);
	esp_err_t e = nvs_commit(h);
	nvs_close(h);
	return (e == ESP_OK);
}

void halow_config_clear(void)
{
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
		return;
	nvs_set_u8(h, KEY_CONFIGURED, 0);
	nvs_erase_key(h, KEY_SSID);
	nvs_erase_key(h, KEY_PASSPHRASE);
	nvs_commit(h);
	nvs_close(h);
}

bool halow_config_link_enabled(void)
{
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
		return true;
	uint8_t val = 1;
	nvs_get_u8(h, KEY_LINK, &val);
	nvs_close(h);
	return (val != 0);
}

bool halow_config_set_link_enabled(bool enabled)
{
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
		return false;
	nvs_set_u8(h, KEY_LINK, enabled ? 1 : 0);
	esp_err_t e = nvs_commit(h);
	nvs_close(h);
	return (e == ESP_OK);
}
