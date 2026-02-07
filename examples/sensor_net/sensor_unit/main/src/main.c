/**
 * Sensor unit for ESP-Motion (mm-iot-esp32). Optional camera for gateway dashboard.
 *
 * Two modes:
 * - Setup mode: No HaLow config in NVS. 2.4 GHz AP (WPA2) with DHCP; user joins AP,
 *   opens http://192.168.4.1, sets HaLow SSID/password, saves → device reboots.
 * - Running mode: Connects to HaLow, sends sensor packets via ESP-NOW (broadcast).
 *   If CONFIG_SENSOR_CAMERA_ENABLE: serves MJPEG at http://<sensor-ip>/stream for
 *   gateway dashboard (Xiao ESP32-S3-Sense).
 *
 * Build from sensor_unit: idf.py set-target esp32s3 && idf.py build
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "halow_config.h"
#include "packet.h"
#include "esp_now_send.h"
#include "mmipal.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/ip4_addr.h"
#include "lwip/def.h"

#include "camera_stream.h"

extern bool start_halow_connection_with_timeout(uint32_t timeout_ms);

static const char *TAG = "sensor_unit";
static httpd_handle_t s_server = NULL;
#define FW_VERSION "1.0.1"

#define HALOW_CONNECT_TIMEOUT_MS  (30 * 1000)
#define SENSOR_SEND_INTERVAL_MS   2000
#define AP_SSID_SETUP     "ESP-Sensor-EN"
#define AP_PASSWORD_SETUP "sensor123"
#define ESPNOW_CHANNEL    6

#define AP_IP_NBO  (PP_HTONL(LWIP_MAKEU32(192, 168, 4, 1)))

static void log_boot_banner(void)
{
	ESP_LOGI(TAG,
		"\n"
		" ███████╗███████╗███╗   ██╗███████╗ ██████╗ ██████╗\n"
		" ██╔════╝██╔════╝████╗  ██║██╔════╝██╔═══██╗██╔══██╗\n"
		" ███████╗█████╗  ██╔██╗ ██║███████╗██║   ██║██████╔╝\n"
		" ╚════██║██╔══╝  ██║╚██╗██║╚════██║██║   ██║██╔══██╗\n"
		" ███████║███████╗██║ ╚████║███████║╚██████╔╝██║  ██║\n"
		" ╚══════╝╚══════╝╚═╝  ╚═══╝╚══════╝ ╚═════╝ ╚═╝  ╚═╝\n"
		"      N E T   ::   H a L o W   S E N S O R   N O D E\n"
		"      ESP-NOW uplink | live telemetry | cyberpunk mode\n"
		"      version " FW_VERSION "\n");
}

static void start_ap_for_setup(void)
{
	esp_netif_create_default_wifi_ap();
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);
	esp_wifi_set_storage(WIFI_STORAGE_RAM);
	esp_wifi_set_mode(WIFI_MODE_AP);
	wifi_config_t ap_cfg = { 0 };
	ap_cfg.ap.channel = ESPNOW_CHANNEL;
	ap_cfg.ap.max_connection = 4;
	ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
	strncpy((char *)ap_cfg.ap.ssid, AP_SSID_SETUP, sizeof(ap_cfg.ap.ssid) - 1);
	ap_cfg.ap.ssid[sizeof(ap_cfg.ap.ssid) - 1] = '\0';
	ap_cfg.ap.ssid_len = strlen((char *)ap_cfg.ap.ssid);
	strncpy((char *)ap_cfg.ap.password, AP_PASSWORD_SETUP, sizeof(ap_cfg.ap.password) - 1);
	ap_cfg.ap.password[sizeof(ap_cfg.ap.password) - 1] = '\0';
	esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
	esp_wifi_start();
	ESP_LOGI(TAG, "Setup AP online: %s", AP_SSID_SETUP);
}

static const char *setup_html =
	"<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
	"<title>Sensor Unit Setup</title>"
	"<style>*{box-sizing:border-box}body{font-family:system-ui,sans-serif;margin:0;padding:16px;background:#1a1a2e;color:#eee}"
	"h1{font-size:1.25rem}.f{max-width:320px;margin:12px 0}.f label{display:block;margin-bottom:4px;color:#aaa}.f input{width:100%;padding:10px;border:1px solid #444;background:#2a2a4e;color:#eee;border-radius:6px}"
	"button{padding:10px 20px;margin-top:12px;border:1px solid #444;background:#4a6a8e;color:#eee;border-radius:6px;cursor:pointer}.err{color:#f88;margin-top:8px}</style></head><body>"
	"<h1>Sensor Unit – Setup</h1>"
	"<p style=\"color:#888\">Enter the same HaLow SSID and password as your gateway. After saving, the device will reboot and join HaLow.</p>"
	"<form id=\"f\" class=\"f\"><label>HaLow SSID</label><input type=\"text\" id=\"ssid\" name=\"ssid\" required maxlength=\"32\" placeholder=\"e.g. Halow1\">"
	"<label>HaLow password</label><input type=\"password\" id=\"pass\" name=\"pass\" maxlength=\"63\" placeholder=\"Passphrase\">"
	"<button type=\"submit\">Save and reboot</button><span id=\"err\" class=\"err\"></span></form>"
	"<script>document.getElementById('f').onsubmit=function(e){e.preventDefault();var s=document.getElementById('ssid').value.trim();var p=document.getElementById('pass').value;"
	"if(!s){document.getElementById('err').textContent='SSID required';return;}fetch('/api/setup',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid:s,passphrase:p})}).then(function(r){if(r.ok){document.getElementById('err').textContent='Saving… device rebooting.';}else{document.getElementById('err').textContent='Save failed';}}).catch(function(){document.getElementById('err').textContent='Request failed';});};</script></body></html>";

static esp_err_t handler_get_setup(httpd_req_t *req)
{
	httpd_resp_set_type(req, "text/html");
	httpd_resp_send(req, setup_html, strlen(setup_html));
	return ESP_OK;
}

static esp_err_t handler_post_api_setup(httpd_req_t *req)
{
	char body[256];
	int r = httpd_req_recv(req, body, sizeof(body) - 1);
	if (r <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad"); return ESP_FAIL; }
	body[r] = '\0';
	char *ssid = strstr(body, "\"ssid\":\"");
	if (!ssid) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad"); return ESP_FAIL; }
	ssid += 8;
	char *ssid_end = ssid;
	while (*ssid_end && *ssid_end != '"') { if (*ssid_end == '\\') ssid_end++; ssid_end++; }
	char *pass = strstr(ssid_end, "\"passphrase\":\"");
	if (pass)
		pass += 14;
	else {
		pass = strstr(ssid_end, "\"pass\":\"");
		if (pass) pass += 8;
	}
	char ssid_buf[33], pass_buf[65];
	size_t sl = (size_t)(ssid_end - ssid); if (sl >= sizeof(ssid_buf)) sl = sizeof(ssid_buf) - 1;
	memcpy(ssid_buf, ssid, sl); ssid_buf[sl] = '\0';
	pass_buf[0] = '\0';
	if (pass) {
		char *pass_end = pass;
		while (*pass_end && *pass_end != '"') { if (*pass_end == '\\') pass_end++; pass_end++; }
		size_t pl = (size_t)(pass_end - pass); if (pl >= sizeof(pass_buf)) pl = sizeof(pass_buf) - 1;
		memcpy(pass_buf, pass, pl); pass_buf[pl] = '\0';
	}
	if (!halow_config_save(ssid_buf, pass_buf)) {
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "NVS save failed");
		return ESP_FAIL;
	}
	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, "{\"ok\":true}", 10);
	vTaskDelay(pdMS_TO_TICKS(500));
	esp_restart();
	return ESP_OK;
}

static void run_setup_mode(void)
{
	ESP_LOGI(TAG, "Setup mode: HaLow not configured. Join %s (WPA2) and open http://192.168.4.1", AP_SSID_SETUP);
	esp_err_t err = esp_netif_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Network init failed: %s", esp_err_to_name(err));
		return;
	}
	esp_event_loop_create_default();
	start_ap_for_setup();
	vTaskDelay(pdMS_TO_TICKS(1000));
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.max_uri_handlers = 8;
	config.stack_size = 4096;
	if (httpd_start(&s_server, &config) == ESP_OK) {
		httpd_uri_t u_root = { .uri = "/", .method = HTTP_GET, .handler = handler_get_setup, .user_ctx = NULL };
		httpd_uri_t u_setup = { .uri = "/api/setup", .method = HTTP_POST, .handler = handler_post_api_setup, .user_ctx = NULL };
		httpd_register_uri_handler(s_server, &u_root);
		httpd_register_uri_handler(s_server, &u_setup);
		ESP_LOGI(TAG, "HTTP portal listening on port %d", config.server_port);
	} else
		ESP_LOGE(TAG, "HTTP server start failed");
}

static void set_halow_default_netif_cb(void *arg)
{
	(void)arg;
	struct netif *netif;
	for (netif = netif_list; netif != NULL; netif = netif->next) {
		if (!netif_is_up(netif)) continue;
#if LWIP_IPV4
		const ip4_addr_t *ip4 = netif_ip4_addr(netif);
		if (ip4 == NULL) continue;
		if (ip4->addr == AP_IP_NBO) continue;
		netif_set_default(netif);
		ESP_LOGI(TAG, "Route: default netif set to HaLow");
		return;
#endif
	}
}

static void run_running_mode(void)
{
	ESP_LOGI(TAG, "Boot: powering HaLow radio...");
	vTaskDelay(pdMS_TO_TICKS(3000));

	ESP_LOGI(TAG, "Link: connecting to HaLow (timeout %lu s)...", (unsigned long)(HALOW_CONNECT_TIMEOUT_MS / 1000));
	if (!start_halow_connection_with_timeout(HALOW_CONNECT_TIMEOUT_MS)) {
		ESP_LOGW(TAG, "Link failed: timeout. Clearing config and rebooting to setup.");
		halow_config_clear();
		vTaskDelay(pdMS_TO_TICKS(500));
		esp_restart();
	}
	ESP_LOGI(TAG, "Link up: HaLow connected");
	vTaskDelay(pdMS_TO_TICKS(2000));

	esp_err_t err = esp_netif_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Network init failed: %s", esp_err_to_name(err));
		return;
	}
	/* Prefer HaLow for default route (HTTP server on HaLow IP). */
	tcpip_callback(set_halow_default_netif_cb, NULL);
	esp_now_send_init();
	vTaskDelay(pdMS_TO_TICKS(500));

#if defined(CONFIG_SENSOR_CAMERA_ENABLE) && CONFIG_SENSOR_CAMERA_ENABLE
	bool camera_ok = camera_stream_init();
	if (camera_ok)
		ESP_LOGI(TAG, "Camera online: MJPEG at http://<this-ip>/stream");
#endif

	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.max_uri_handlers = 16;
	config.stack_size = 8192;
	if (httpd_start(&s_server, &config) == ESP_OK) {
#if defined(CONFIG_SENSOR_CAMERA_ENABLE) && CONFIG_SENSOR_CAMERA_ENABLE
		camera_stream_register_uri(s_server);
#endif
		ESP_LOGI(TAG, "HTTP server listening on port %d", config.server_port);
	} else
		ESP_LOGE(TAG, "HTTP server start failed");

	struct mmipal_ip_config ip_cfg;
	int got_ip = 0;
	for (int i = 0; i < 15 && !got_ip; i++) {
		if (i > 0) vTaskDelay(pdMS_TO_TICKS(1000));
		if (mmipal_get_ip_config(&ip_cfg) != MMIPAL_SUCCESS) continue;
		if (ip_cfg.ip_addr[0] != '\0' && strcmp(ip_cfg.ip_addr, "0.0.0.0") != 0) {
			ESP_LOGI(TAG, "HaLow IP: %s  (camera: http://%s/stream)", ip_cfg.ip_addr, ip_cfg.ip_addr);
			got_ip = 1;
		}
	}
	if (!got_ip)
		ESP_LOGW(TAG, "DHCP: no HaLow IP after 15 s.");

	/* Send sensor packets periodically */
	while (1) {
		esp_now_send_packet();
		vTaskDelay(pdMS_TO_TICKS(SENSOR_SEND_INTERVAL_MS));
	}
}

void app_main(void)
{
	log_boot_banner();
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ESP_ERROR_CHECK(nvs_flash_init());
	}
	esp_event_loop_create_default();

	ESP_LOGI(TAG, "ESP-Motion sensor unit ready (HaLow + ESP-NOW) [Xiao ESP32-S3-Sense]");
	if (!halow_config_is_configured())
		run_setup_mode();
	else
		run_running_mode();
}
