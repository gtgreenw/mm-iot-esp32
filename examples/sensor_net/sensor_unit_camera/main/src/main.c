/**
 * Camera sensor unit for sensor_net: XIAO ESP32-S3-Sense.
 * HaLow STA + MJPEG at /live + minimal ESP-NOW (no BME/motion).
 *
 * - Config window: At boot, WiFi AP is on for 5 minutes. Open http://192.168.4.1 to set
 *   HaLow network name/password and camera/unit settings. After 5 min, AP turns off and
 *   unit connects to HaLow (or reboots if not configured).
 * - Running mode: Connects to HaLow, video at http://<sensor-ip>/live, ESP-NOW for gateway.
 *
 * Build: idf.py set-target esp32s3 && idf.py build
 */
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "halow_config.h"
#include "camera_unit_settings.h"
#include "packet.h"
#include "esp_now_send_camera.h"

#include "mm_app_common.h"
#include "mmipal.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/ip4_addr.h"
#include "lwip/def.h"
#include "camera_stream.h"
#include "mic_stream.h"

extern bool start_halow_connection_with_timeout(uint32_t timeout_ms);

static const char *TAG = "camera_unit";
static httpd_handle_t s_server = NULL;
#define FW_VERSION "1.1.0"

#define HALOW_CONNECT_TIMEOUT_MS  (30 * 1000)
#define CAMERA_ESPNOW_SEND_MS     (10 * 1000)
#define AP_SSID_SETUP     "ESP-Sensor-EN"
#define AP_PASSWORD_SETUP "sensor123"
#define ESPNOW_CHANNEL    6
#define CONFIG_WIFI_MINUTES 5
#define CONFIG_WIFI_MS    (CONFIG_WIFI_MINUTES * 60 * 1000)

#define AP_IP_NBO  (PP_HTONL(LWIP_MAKEU32(192, 168, 4, 1)))

static volatile bool s_config_window_done = false;
/** Default HaLow credentials (match mm_app_loadconfig.c); used when not configured and to save after first connect. */
#define DEFAULT_HALOW_SSID   "halow"
#define DEFAULT_HALOW_PASS   "letmein111"
/** Returns false if connection failed; never returns if connected (runs main loop). */
static bool run_running_mode(void);
static TimerHandle_t s_config_timer = NULL;

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
		"      N E T   ::   H a L o W   C A M E R A   N O D E\n"
		"      XIAO ESP32-S3-Sense | /live | Config WiFi: %d min\n"
		"      version " FW_VERSION "\n", CONFIG_WIFI_MINUTES);
}

static void start_ap_for_config(void)
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
	esp_wifi_start();
	/* Start at minimum TX power (2 dBm); restore to 4 dBm 5 s after config window is up */
	esp_wifi_set_max_tx_power(8);  /* 8 = 2 dBm min (ESP-IDF quarter-dBm) */
	ESP_LOGI(TAG, "Config AP online: %s — open http://192.168.4.1 (for %d min)", AP_SSID_SETUP, CONFIG_WIFI_MINUTES);
}

/* Settings page: HaLow SSID/pass, camera quality, HaLow link on/off. */
static const char *settings_page_html =
	"<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
	"<title>Camera Unit – Settings</title>"
	"<style>*{box-sizing:border-box}body{font-family:system-ui,sans-serif;margin:0;padding:16px;background:#1a1a2e;color:#eee}"
	"h1{font-size:1.25rem}.f{max-width:360px;margin:12px 0}.f label{display:block;margin-bottom:4px;color:#aaa}.f input,.f select{width:100%;padding:10px;border:1px solid #444;background:#2a2a4e;color:#eee;border-radius:6px}.f label.cb{display:flex;align-items:center;gap:0.5em;margin-top:8px;margin-bottom:4px}.f label.cb input[type=checkbox]{flex-shrink:0}"
	".f label.cb input[type=checkbox]{-webkit-appearance:none;appearance:none;width:20px;height:20px;min-width:20px;min-height:20px;border:2px solid #6a9;background:#2a2a4e;border-radius:4px;margin:0;cursor:pointer}"
	".f label.cb input[type=checkbox]:checked{background:#2d5a3a;border-color:#4a9;background-image:url('data:image/svg+xml,<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 12 10\"><path fill=\"none\" stroke=\"%236cf\" stroke-width=\"2\" d=\"M1 5l4 4 6-8\"/></svg>');background-size:12px 10px;background-repeat:no-repeat;background-position:center}"
	".stat{color:#9af;font-size:0.9rem;margin-bottom:0.5em}"
	"button{padding:10px 20px;margin:8px 8px 0 0;border:1px solid #444;background:#4a6a8e;color:#eee;border-radius:6px;cursor:pointer;touch-action:manipulation}.reboot-btn{background:#6a4a4e}"
	".err{color:#f88;margin-top:8px}.msg{color:#8f8;margin-top:8px}section{margin-top:20px;padding-top:16px;border-top:1px solid #333}</style></head><body>"
	"<h1>Camera Unit – Settings</h1>"
	"<p class=\"msg\">WiFi config is on for 5 minutes after boot. Set HaLow network and camera options below.</p>"
	"<p style=\"margin:12px 0\"><a href=\"/live\" style=\"padding:8px 16px;background:#4a6a8e;border-radius:6px;text-decoration:none;color:#fff;display:inline-block\">▶ Live (video + audio)</a></p>"
	"<section><h2>HaLow network</h2>"
	"<form id=\"fHalow\" class=\"f\"><label>HaLow network name (SSID)</label><input type=\"text\" id=\"ssid\" name=\"ssid\" maxlength=\"32\" placeholder=\"e.g. Halow1\">"
	"<label>HaLow password</label><input type=\"password\" id=\"pass\" name=\"pass\" maxlength=\"63\" placeholder=\"Passphrase\">"
	"<button type=\"submit\">Save HaLow &amp; reboot</button><span id=\"errHalow\" class=\"err\"></span></form></section>"
	"<section><h2>Camera &amp; unit</h2>"
	"<form id=\"fSettings\" class=\"f\"><label>Stream quality</label><select id=\"quality\" name=\"quality\"><option value=\"0\">Low</option><option value=\"1\">Medium</option><option value=\"2\" selected>High</option></select>"
	"<label style=\"display:block;margin-top:1em;color:#aaa\">Camera orientation</label><select id=\"camera_orientation\" name=\"camera_orientation\" style=\"width:100%%;padding:10px;border:1px solid #444;background:#2a2a4e;color:#eee;border-radius:6px;margin-top:4px\"><option value=\"0\">Normal (0°)</option><option value=\"1\">180°</option><option value=\"2\">90° clockwise</option><option value=\"3\">90° counter-clockwise</option></select>"
	"<p style=\"color:#888;font-size:0.85em;margin-top:4px\">Orientation may require Reboot to take effect.</p>"
	"<label class=\"cb\"><input type=\"checkbox\" id=\"link_enabled\" name=\"link_enabled\" checked> HaLow link enabled</label>"
	"<label class=\"cb\"><input type=\"checkbox\" id=\"mirror\" name=\"mirror\"> Mirror image</label>"
	"<p class=\"stat\">ESP-NOW sensors visible: <strong id=\"espnowPeersCount\">-</strong></p>"
	"<label class=\"cb\"><input type=\"checkbox\" id=\"espnow_enable\" name=\"espnow_enable\"> Enable ESP-NOW</label>"
	"<p style=\"color:#888;font-size:0.85em;margin-top:2px\">Send/relay sensor data on 2.4 GHz channel 6. Applies immediately.</p>"
	"<label class=\"cb\"><input type=\"checkbox\" id=\"led_enabled\" name=\"led_enabled\" checked> HaLow link LED</label>"
	"<p style=\"color:#888;font-size:0.85em;margin-top:2px\">Boot/connecting/connected blink. Takes effect after reboot.</p>"
	"<label style=\"display:block;margin-top:1em;color:#aaa\">Microphone gain (0–255, 100=normal)</label><input type=\"number\" id=\"mic_gain\" name=\"mic_gain\" min=\"0\" max=\"255\" value=\"100\" style=\"width:100%%;padding:10px;border:1px solid #444;background:#2a2a4e;color:#eee;border-radius:6px;margin-top:4px\">"
	"<label style=\"display:block;margin-top:1em;color:#aaa\">Microphone sample rate</label><select id=\"mic_sample_rate\" name=\"mic_sample_rate\" style=\"width:100%%;padding:10px;border:1px solid #444;background:#2a2a4e;color:#eee;border-radius:6px;margin-top:4px\"><option value=\"0\" selected>16 kHz</option><option value=\"1\">8 kHz</option><option value=\"2\">3 kHz</option></select>"
	"<p style=\"color:#888;font-size:0.85em;margin-top:4px\">Sample rate takes effect after reboot.</p>"
	"<button type=\"submit\">Save settings</button><span id=\"errSettings\" class=\"err\"></span><span id=\"msgSettings\" class=\"msg\"></span></form></section>"
	"<section><h2>Device</h2>"
	"<p><button type=\"button\" id=\"rebootBtn\" class=\"reboot-btn\">Reboot</button> <span id=\"rebootMsg\" class=\"msg\"></span></p>"
	"<p style=\"color:#888;font-size:0.85em\">Use after changing camera orientation or when settings do not apply.</p></section>"
	"<script>"
	"(function run(){"
	"function load(){fetch('/api/settings').then(function(r){return r.json();}).then(function(d){"
	"document.getElementById('ssid').value=d.ssid||'';"
	"document.getElementById('quality').value=String(d.quality!==undefined?d.quality:2);"
	"var o=(d.camera_orientation!==undefined)?d.camera_orientation:0;var co=document.getElementById('camera_orientation');if(co)co.value=String(o<=3?o:0);"
	"document.getElementById('link_enabled').checked=d.link_enabled!==false;"
	"document.getElementById('mirror').checked=d.mirror===true;"
	"var e=document.getElementById('espnow_enable');if(e)e.checked=d.espnow_enabled!==false;"
	"var le=document.getElementById('led_enabled');if(le)le.checked=d.led_enabled!==false;"
	"var mg=document.getElementById('mic_gain');if(mg)mg.value=String(d.mic_gain!=null&&d.mic_gain<=255?d.mic_gain:100);"
	"var msr=document.getElementById('mic_sample_rate');if(msr){var r=(d.mic_sample_rate!=null&&d.mic_sample_rate<=2)?d.mic_sample_rate:0;msr.value=String(r);}"
	"var p=document.getElementById('espnowPeersCount');if(p)p.textContent=d.sensors_visible!=null?String(d.sensors_visible):'-';}).catch(function(){});}"
	"function updatePeersCount(){fetch('/api/settings').then(function(r){return r.json();}).then(function(d){var el=document.getElementById('espnowPeersCount');if(el)el.textContent=d.sensors_visible!=null?String(d.sensors_visible):'-';}).catch(function(){});}"
	"var fHalow=document.getElementById('fHalow'),fSettings=document.getElementById('fSettings');"
	"if(fHalow){fHalow.addEventListener('submit',function(e){e.preventDefault();var s=document.getElementById('ssid').value.trim(),p=document.getElementById('pass').value;"
	"if(!s){document.getElementById('errHalow').textContent='SSID required';return;}fetch('/api/setup',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid:s,passphrase:p})}).then(function(r){if(r.ok){document.getElementById('errHalow').textContent='Saving… rebooting.';}else{document.getElementById('errHalow').textContent='Save failed';}}).catch(function(){document.getElementById('errHalow').textContent='Request failed';});});}"
	"if(fSettings){fSettings.addEventListener('submit',function(e){e.preventDefault();var q=parseInt(document.getElementById('quality').value,10),l=document.getElementById('link_enabled').checked,m=document.getElementById('mirror').checked,orient=parseInt(document.getElementById('camera_orientation').value,10);if(orient>3)orient=0;var en=document.getElementById('espnow_enable');var espnow=(en&&en.checked);var le=document.getElementById('led_enabled');var ledOn=(le&&le.checked);var mgEl=document.getElementById('mic_gain');var micGain=mgEl?Math.min(255,Math.max(0,parseInt(mgEl.value,10)||100)):100;var msrEl=document.getElementById('mic_sample_rate');var micRate=msrEl?Math.min(2,Math.max(0,parseInt(msrEl.value,10)||0)):0;"
	"fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({quality:q,link_enabled:l,mirror:m,camera_orientation:orient,espnow_enabled:espnow,led_enabled:ledOn,mic_gain:micGain,mic_sample_rate:micRate})}).then(function(r){if(r.ok){document.getElementById('msgSettings').textContent='Saved.';}else{document.getElementById('errSettings').textContent='Save failed';}}).catch(function(){document.getElementById('errSettings').textContent='Request failed';});});}"
	"var rebootBtn=document.getElementById('rebootBtn');if(rebootBtn){rebootBtn.addEventListener('click',function(){rebootBtn.disabled=true;document.getElementById('rebootMsg').textContent='Rebooting…';fetch('/api/reboot',{method:'POST'}).catch(function(){});});}"
	"load();setInterval(updatePeersCount,3000);})();</script></body></html>";

/* /live: video stream only, full viewport, fullscreen button. */
static const char *live_page_html =
	"<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><meta http-equiv=\"Cache-Control\" content=\"no-store\">"
	"<title>Live</title>"
	"<style>*{box-sizing:border-box}body{margin:0;background:#000;display:flex;align-items:center;justify-content:center;min-height:100vh;overflow:hidden}"
	"#container{position:relative;width:100%;height:100vh;min-height:100%;display:flex;align-items:center;justify-content:center;background:#000}"
	"#vid{width:100%;height:100%;max-width:100%;max-height:100%;object-fit:cover;display:block}"
	".fs-btn{position:fixed;top:12px;right:12px;z-index:9999;padding:10px 16px;background:rgba(0,0,0,0.7);color:#fff;border:1px solid #666;border-radius:6px;cursor:pointer;font-size:14px;pointer-events:auto;touch-action:manipulation}"
	".fs-btn:hover{background:rgba(60,60,60,0.9)}"
	"#container:fullscreen,#container:-webkit-full-screen,#container:-ms-fullscreen{width:100vw;height:100vh;display:flex;align-items:center;justify-content:center;background:#000}"
	"#container:fullscreen #vid,#container:-webkit-full-screen #vid,#container:-ms-fullscreen #vid{width:100%;height:100%;max-width:100%;max-height:100%;object-fit:cover;display:block}"
	"</style></head><body>"
	"<div id=\"container\">"
	"<img id=\"vid\" alt=\"Live\">"
	"<button type=\"button\" class=\"fs-btn\" id=\"fsBtn\" onclick=\"doFullscreen()\">Fullscreen</button>"
	"</div>"
	"<script>"
	"(function(){"
	"var container=document.getElementById('container'),fsBtn=document.getElementById('fsBtn');"
	"function doFullscreen(){var el=document.fullscreenElement||document.webkitFullscreenElement;if(el){var exit=document.exitFullscreen||document.webkitExitFullscreen;if(exit)exit.call(document);return;}var req=container.requestFullscreen||container.webkitRequestFullscreen;if(req)req.call(container);}"
	"window.doFullscreen=doFullscreen;"
	"function setFsLabel(){if(fsBtn)fsBtn.textContent=(document.fullscreenElement||document.webkitFullscreenElement)?'Exit fullscreen':'Fullscreen';}"
	"document.addEventListener('fullscreenchange',setFsLabel);document.addEventListener('webkitfullscreenchange',setFsLabel);"
	"window.addEventListener('load',function(){var v=document.getElementById('vid');if(v)v.src='/live/stream';});"
	"})();</script></body></html>";

static esp_err_t handler_get_live(httpd_req_t *req)
{
	httpd_resp_set_type(req, "text/html");
	httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
	httpd_resp_send(req, live_page_html, strlen(live_page_html));
	return ESP_OK;
}

static esp_err_t handler_get_root(httpd_req_t *req)
{
	httpd_resp_set_type(req, "text/html");
	httpd_resp_send(req, settings_page_html, strlen(settings_page_html));
	return ESP_OK;
}

static esp_err_t handler_get_api_settings(httpd_req_t *req)
{
	char ssid[33], pass_dummy[65], buf[400];
	ssid[0] = '\0';
	if (halow_config_is_configured())
		halow_config_load(ssid, sizeof(ssid), pass_dummy, sizeof(pass_dummy));
	uint8_t quality = CAMERA_QUALITY_MED;
	camera_unit_settings_get_quality(&quality);
	bool link_enabled = halow_config_link_enabled();
	/* Escape SSID for JSON (backslash and quote) */
	char ssid_esc[65];
	size_t k = 0;
	for (size_t i = 0; ssid[i] != '\0' && k < sizeof(ssid_esc) - 2; i++) {
		if (ssid[i] == '\\' || ssid[i] == '"') ssid_esc[k++] = '\\';
		ssid_esc[k++] = ssid[i];
	}
	ssid_esc[k] = '\0';
	bool mirror = false;
	camera_unit_settings_get_mirror(&mirror);
	uint8_t camera_orientation = 0;
	camera_unit_settings_get_orientation(&camera_orientation);
	bool espnow_enabled = true;
	camera_unit_settings_get_espnow(&espnow_enabled);
	bool led_enabled = true;
	camera_unit_settings_get_led_enabled(&led_enabled);
	uint8_t mic_gain = 100;
	camera_unit_settings_get_mic_gain(&mic_gain);
	uint8_t mic_sample_rate = 0;
	camera_unit_settings_get_mic_sample_rate(&mic_sample_rate);
	int sensors_visible = esp_now_send_camera_peers_seen_count();
	int n = snprintf(buf, sizeof(buf),
		"{\"ssid\":\"%s\",\"quality\":%u,\"link_enabled\":%s,\"mirror\":%s,\"camera_orientation\":%u,\"espnow_enabled\":%s,\"led_enabled\":%s,\"mic_gain\":%u,\"mic_sample_rate\":%u,\"sensors_visible\":%d}",
		ssid_esc, (unsigned)quality, link_enabled ? "true" : "false", mirror ? "true" : "false",
		(unsigned)camera_orientation, espnow_enabled ? "true" : "false", led_enabled ? "true" : "false",
		(unsigned)mic_gain, (unsigned)mic_sample_rate, sensors_visible);
	if (n < 0 || (size_t)n >= sizeof(buf)) n = 0;
	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, buf, (size_t)n);
	return ESP_OK;
}

static esp_err_t handler_post_api_settings(httpd_req_t *req)
{
	char body[256];
	int r = httpd_req_recv(req, body, sizeof(body) - 1);
	if (r <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad"); return ESP_FAIL; }
	body[r] = '\0';
	uint8_t quality = CAMERA_QUALITY_MED;
	bool link_enabled = true;
	bool mirror = false;
	bool espnow_enabled = true;
	char *q = strstr(body, "\"quality\":");
	if (q) { quality = (uint8_t)atoi(q + 10); if (quality > 2) quality = 2; }
	q = strstr(body, "\"link_enabled\":");
	if (q) link_enabled = (strstr(q, "true") != NULL);
	q = strstr(body, "\"mirror\":");
	if (q) mirror = (strstr(q, "true") != NULL);
	uint8_t camera_orientation = 0;
	q = strstr(body, "\"camera_orientation\":");
	if (q) { camera_orientation = (uint8_t)atoi(q + 22); if (camera_orientation > 3) camera_orientation = 0; }
	q = strstr(body, "\"espnow_enabled\":");
	if (q) espnow_enabled = (strstr(q, "true") != NULL);
	bool led_enabled = true;
	q = strstr(body, "\"led_enabled\":");
	if (q) led_enabled = (strstr(q, "true") != NULL);
	uint8_t mic_gain = 100;
	q = strstr(body, "\"mic_gain\":");
	if (q) { int v = atoi(q + 12); if (v >= 0 && v <= 255) mic_gain = (uint8_t)v; }
	uint8_t mic_sample_rate = 0;
	q = strstr(body, "\"mic_sample_rate\":");
	if (q) { int v = atoi(q + 19); if (v >= 0 && v <= 2) mic_sample_rate = (uint8_t)v; }
	camera_unit_settings_set_quality(quality);
	halow_config_set_link_enabled(link_enabled);
	camera_unit_settings_set_mirror(mirror);
	camera_unit_settings_set_led_enabled(led_enabled);
	camera_unit_settings_set_mic_gain(mic_gain);
	camera_unit_settings_set_mic_sample_rate(mic_sample_rate);
	if (!camera_unit_settings_set_orientation(camera_orientation)) {
		ESP_LOGW(TAG, "Settings: failed to save camera_orientation=%u to NVS", (unsigned)camera_orientation);
	} else {
		ESP_LOGI(TAG, "Settings: camera_orientation=%u saved", (unsigned)camera_orientation);
	}
	esp_now_send_camera_set_enabled(espnow_enabled);
	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, "{\"ok\":true}", 10);
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

static esp_err_t handler_post_api_reboot(httpd_req_t *req)
{
	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, "{\"ok\":true}", 10);
	vTaskDelay(pdMS_TO_TICKS(800));
	esp_restart();
	return ESP_OK;
}

static void config_timer_cb(TimerHandle_t t)
{
	(void)t;
	s_config_window_done = true;
	ESP_LOGI(TAG, "Config window (%d min) ended; closing AP", CONFIG_WIFI_MINUTES);
}

#define WIFI_TX_POWER_SETTLE_S  5
static void config_ap_tx_restore_cb(TimerHandle_t t)
{
	(void)t;
	esp_wifi_set_max_tx_power(4 * 4);  /* 4 dBm */
	ESP_LOGI(TAG, "Config AP WiFi TX power restored to 4 dBm");
}

static void run_config_window(void)
{
	/* Event loop already created in app_main(); do not call esp_event_loop_create_default() again (crashes). */
	esp_err_t err = esp_netif_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Network init failed: %s", esp_err_to_name(err));
		return;
	}
	start_ap_for_config();
	vTaskDelay(pdMS_TO_TICKS(1000));

	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.max_uri_handlers = 12;
	config.stack_size = 8192;
	config.max_open_sockets = 6;
	config.lru_purge_enable = true;
	config.recv_wait_timeout = 15;
	config.send_wait_timeout = 15;
	if (httpd_start(&s_server, &config) == ESP_OK) {
		httpd_uri_t u_root   = { .uri = "/", .method = HTTP_GET,  .handler = handler_get_root,         .user_ctx = NULL };
		httpd_uri_t u_get    = { .uri = "/api/settings", .method = HTTP_GET,  .handler = handler_get_api_settings,  .user_ctx = NULL };
		httpd_uri_t u_set    = { .uri = "/api/settings", .method = HTTP_POST, .handler = handler_post_api_settings, .user_ctx = NULL };
		httpd_uri_t u_setup  = { .uri = "/api/setup",    .method = HTTP_POST, .handler = handler_post_api_setup,   .user_ctx = NULL };
		httpd_uri_t u_reboot = { .uri = "/api/reboot",   .method = HTTP_POST, .handler = handler_post_api_reboot,  .user_ctx = NULL };
		httpd_register_uri_handler(s_server, &u_root);
		httpd_register_uri_handler(s_server, &u_get);
		httpd_register_uri_handler(s_server, &u_set);
		httpd_register_uri_handler(s_server, &u_setup);
		httpd_register_uri_handler(s_server, &u_reboot);
		ESP_LOGI(TAG, "HTTP config portal on http://192.168.4.1");
		/* Restore WiFi TX power 5 s after config window is up */
		TimerHandle_t tx_restore = xTimerCreate("ap_tx", pdMS_TO_TICKS(WIFI_TX_POWER_SETTLE_S * 1000), pdFALSE, NULL, config_ap_tx_restore_cb);
		if (tx_restore != NULL)
			xTimerStart(tx_restore, 0);
	} else {
		ESP_LOGE(TAG, "HTTP server start failed");
	}

	s_config_timer = xTimerCreate("config_timer", pdMS_TO_TICKS(CONFIG_WIFI_MS), pdFALSE, NULL, config_timer_cb);
	if (s_config_timer != NULL)
		xTimerStart(s_config_timer, 0);

	while (!s_config_window_done)
		vTaskDelay(pdMS_TO_TICKS(500));

	if (s_config_timer != NULL) {
		xTimerStop(s_config_timer, 0);
		xTimerDelete(s_config_timer, 0);
		s_config_timer = NULL;
	}
	if (s_server != NULL) {
		httpd_stop(s_server);
		s_server = NULL;
	}
	esp_wifi_stop();
	esp_wifi_deinit();
	esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
	if (ap_netif != NULL)
		esp_netif_destroy(ap_netif);

	if (halow_config_is_configured())
		run_running_mode();
	else {
		ESP_LOGI(TAG, "HaLow not configured; reboot to open config again");
		esp_restart();
	}
}

/** HaLow (Morse) netif name in LwIP (mmnetif.c sets name[0]='M', name[1]='M'). */
#define HALOW_NETIF_NAME0  'M'
#define HALOW_NETIF_NAME1  'M'

static void set_halow_default_netif_cb(void *arg)
{
	(void)arg;
	struct netif *netif;
	struct netif *fallback = NULL;
	for (netif = netif_list; netif != NULL; netif = netif->next) {
		if (!netif_is_up(netif)) continue;
#if LWIP_IPV4
		const ip4_addr_t *ip4 = netif_ip4_addr(netif);
		if (ip4 == NULL) continue;
		if (ip4->addr == AP_IP_NBO) continue; /* skip AP */
		/* Prefer HaLow (MM) netif so HTTP server is reachable on HaLow IP */
		if (netif->name[0] == HALOW_NETIF_NAME0 && netif->name[1] == HALOW_NETIF_NAME1) {
			netif_set_default(netif);
			ESP_LOGI(TAG, "Route: default netif set to HaLow (MM)");
			return;
		}
		fallback = netif;
#endif
	}
	if (fallback != NULL) {
		netif_set_default(fallback);
		ESP_LOGI(TAG, "Route: default netif set to HaLow");
	}
}

/* Delays to reduce crashes when connecting to HaLow without touching the
 * HaLow chip shield (EMI/power droop during RF; touching shield grounds it). */
#define HALOW_PRE_CONNECT_MS  5000   /* 5 s settle before HaLow RF bring-up */
#define HALOW_POST_PRE_MS    500    /* extra delay after pre-connect before init */
#define HALOW_PRE_CAMERA_MS  2000

static bool run_running_mode(void)
{
	/* 0. Let power rails settle before HaLow radio (reduces EMI-related crashes) */
	ESP_LOGI(TAG, "Boot: powering HaLow radio...");
	vTaskDelay(pdMS_TO_TICKS(HALOW_PRE_CONNECT_MS));
	vTaskDelay(pdMS_TO_TICKS(HALOW_POST_PRE_MS));

	/* 1. Initialize HaLow stack and connect (gateway: app_wlan_init + app_wlan_start_with_timeout) */
	/* Suppress log output during connect so HaLow GPIO ISR path does not block in UART (int WDT panic). */
	esp_log_level_set("*", ESP_LOG_WARN);
	esp_log_level_set(TAG, ESP_LOG_INFO);
	ESP_LOGI(TAG, "Connecting to HaLow (timeout %lu s)...", (unsigned long)(HALOW_CONNECT_TIMEOUT_MS / 1000));
	if (!start_halow_connection_with_timeout(HALOW_CONNECT_TIMEOUT_MS)) {
		if (!app_wlan_halow_available()) {
			ESP_LOGW(TAG, "HaLow unavailable (e.g. firmware did not boot); opening config AP (5 min), then reboot.");
			run_config_window();
			esp_restart();
		}
		ESP_LOGW(TAG, "HaLow link failed: timeout");
		if (halow_config_is_configured()) {
			ESP_LOGW(TAG, "Clearing saved config and rebooting for reconfigure.");
			halow_config_clear();
			vTaskDelay(pdMS_TO_TICKS(500));
			esp_restart();
		}
		return false;
	}
	esp_log_level_set("*", ESP_LOG_INFO);
	ESP_LOGI(TAG, "Link up: HaLow connected");
	if (!halow_config_is_configured()) {
		if (halow_config_save(DEFAULT_HALOW_SSID, DEFAULT_HALOW_PASS))
			ESP_LOGI(TAG, "Saved default credentials to NVS for next boot");
	}

	/* 2. Stabilize HaLow stack (gateway: 5 s after connect) */
	ESP_LOGI(TAG, "Stabilizing HaLow stack (5 s)...");
	vTaskDelay(pdMS_TO_TICKS(5000));

	/* 3. ESP netif and default route (gateway does this in start_2ghz_ap; we set default to HaLow) */
	esp_err_t err = esp_netif_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Network init failed: %s", esp_err_to_name(err));
		return false;
	}
	tcpip_callback(set_halow_default_netif_cb, NULL);
	vTaskDelay(pdMS_TO_TICKS(500));
	vTaskDelay(pdMS_TO_TICKS(HALOW_PRE_CAMERA_MS));   /* Avoid camera/SD/mic + HaLow RF overlap */

	/* 4. ESP-NOW (gateway: esp_now_rcv_init after start_2ghz_ap) */
	esp_now_send_camera_init();

	/* 5. Wait for HaLow IP so webserver/stream are reachable on HaLow network */
	struct mmipal_ip_config ip_cfg;
	int got_ip = 0;
	for (int i = 0; i < 15 && !got_ip; i++) {
		if (i > 0) vTaskDelay(pdMS_TO_TICKS(1000));
		if (mmipal_get_ip_config(&ip_cfg) != MMIPAL_SUCCESS) continue;
		if (ip_cfg.ip_addr[0] != '\0' && strcmp(ip_cfg.ip_addr, "0.0.0.0") != 0) {
			got_ip = 1;
		}
	}
	if (!got_ip) {
		ESP_LOGW(TAG, "DHCP: no HaLow IP after 15 s.; starting server anyway");
		ip_cfg.ip_addr[0] = '\0';
	}

	/* 6. Start HTTP server first so / and /api/settings work even if camera/mic init fail */
	ESP_LOGI(TAG, "Starting HTTP server (port 80)...");
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.max_uri_handlers = 20;
	config.stack_size = 8192;
	config.max_open_sockets = 13;  /* 13 usable; 3 reserved for system (16 max total) */
	config.lru_purge_enable = true;   /* Reclaim idle sockets so /stream doesn't starve page requests */
	config.recv_wait_timeout = 15;    /* Tolerate slow HaLow link (avoid premature disconnect) */
	config.send_wait_timeout = 15;
#if (portNUM_PROCESSORS > 1)
	config.core_id = 0;   /* Pin HTTP to PRO_CPU so requests stay responsive alongside HaLow */
#endif
	esp_err_t httpd_err = httpd_start(&s_server, &config);
	if (httpd_err != ESP_OK) {
		ESP_LOGE(TAG, "HTTP server start failed: %s (free heap %lu)", esp_err_to_name(httpd_err), (unsigned long)esp_get_free_heap_size());
	} else {
		httpd_uri_t u_root   = { .uri = "/", .method = HTTP_GET,  .handler = handler_get_root,         .user_ctx = NULL };
		httpd_uri_t u_live   = { .uri = "/live", .method = HTTP_GET,  .handler = handler_get_live,           .user_ctx = NULL };
		httpd_uri_t u_live_slash = { .uri = "/live/", .method = HTTP_GET,  .handler = handler_get_live, .user_ctx = NULL };
		httpd_uri_t u_get    = { .uri = "/api/settings", .method = HTTP_GET,  .handler = handler_get_api_settings,  .user_ctx = NULL };
		httpd_uri_t u_set    = { .uri = "/api/settings", .method = HTTP_POST, .handler = handler_post_api_settings, .user_ctx = NULL };
		httpd_uri_t u_setup  = { .uri = "/api/setup",    .method = HTTP_POST, .handler = handler_post_api_setup,   .user_ctx = NULL };
		httpd_uri_t u_reboot = { .uri = "/api/reboot",   .method = HTTP_POST, .handler = handler_post_api_reboot,  .user_ctx = NULL };
		httpd_register_uri_handler(s_server, &u_root);
		httpd_register_uri_handler(s_server, &u_live);
		httpd_register_uri_handler(s_server, &u_live_slash);
		httpd_register_uri_handler(s_server, &u_get);
		httpd_register_uri_handler(s_server, &u_set);
		httpd_register_uri_handler(s_server, &u_setup);
		httpd_register_uri_handler(s_server, &u_reboot);
		ESP_LOGI(TAG, "HTTP server listening on port %d (HaLow)", config.server_port);
	}

	/* 7. Peripherals: camera (lazy init on first /live or /snapshot), mic — then register URIs */
	bool mic_ok = mic_stream_init();
	camera_stream_ensure_mutex();  /* mutex for lazy camera ref-count; must exist before first /live or /snapshot */
	if (s_server != NULL) {
		camera_stream_register_uri(s_server);
		mic_stream_register_uri(s_server);
		if (ip_cfg.ip_addr[0] != '\0') {
			ESP_LOGI(TAG, "  HaLow: http://%s/  (settings)", ip_cfg.ip_addr);
			ESP_LOGI(TAG, "  Live:  http://%s/live", ip_cfg.ip_addr);
			if (mic_ok)
				ESP_LOGI(TAG, "  Audio: http://%s/audio", ip_cfg.ip_addr);
		}
	}

	if (ip_cfg.ip_addr[0] == '\0')
		ESP_LOGI(TAG, "Camera: init on first /live or /snapshot (power saving when idle)");
	if (mic_ok && ip_cfg.ip_addr[0] == '\0')
		ESP_LOGI(TAG, "Mic online (http://<halow-ip>/audio)");

	ESP_LOGI(TAG, "Boot complete; main loop running");
	/* Restore WiFi TX power 5 s after everything has settled */
	vTaskDelay(pdMS_TO_TICKS(WIFI_TX_POWER_SETTLE_S * 1000));
	esp_now_send_camera_restore_wifi_tx_power();
	esp_now_send_camera_packet();
	uint64_t last_esp_now_ms = (uint64_t)(esp_timer_get_time() / 1000);
	for (;;) {
		uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
		if (now_ms - last_esp_now_ms >= CAMERA_ESPNOW_SEND_MS) {
			last_esp_now_ms = now_ms;
			esp_now_send_camera_packet();
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void app_main(void)
{
	/* 1. Boot banner (same order as main gateway iperf.c) */
	log_boot_banner();

	esp_log_level_set("httpd", ESP_LOG_WARN);
	esp_log_level_set("httpd_uri", ESP_LOG_WARN);
	esp_log_level_set("httpd_txrx", ESP_LOG_WARN);
	/* esp32-camera still uses legacy I2C driver (driver/i2c.h); suppress deprecation warning until component migrates to i2c_master.h */
	esp_log_level_set("i2c", ESP_LOG_ERROR);

	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ESP_ERROR_CHECK(nvs_flash_init());
	}
	esp_event_loop_create_default();

	ESP_LOGI(TAG, "Camera sensor unit ready [XIAO ESP32-S3-Sense]");

	/* 2. Connect to HaLow or fall back to config AP (gateway: setup_halow_or_fallback) */
	if (halow_config_is_configured()) {
		ESP_LOGI(TAG, "HaLow configured: connecting...");
		run_running_mode();
		return;
	}
	ESP_LOGI(TAG, "No NVS config: trying default SSID \"%s\"", DEFAULT_HALOW_SSID);
	if (!run_running_mode()) {
		ESP_LOGW(TAG, "Default credentials failed; opening config AP for setup");
		run_config_window();
	}
}
