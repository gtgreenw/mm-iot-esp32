#include "sensor_gateway_http.h"
#include "esp_now_rcv.h"
#include "mmipal.h"
#include "mmwlan.h"
#include "mm_app_common.h"
#include "settings.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "time_sync.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/inet.h"
#include <inttypes.h>
#include <string.h>
#include <stdio.h>

#define NVS_NAMESPACE "gateway"
#define NVS_CAMERAS_KEY "cameras"
#define MAX_CAMERAS 4
#define CAMERA_URL_LEN 128

static esp_err_t handler_get_gateway(httpd_req_t *req)
{
    const char *html = sensor_gateway_get_dashboard_html();
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

static void json_escape(const char *in, char *out, size_t out_size)
{
    size_t j = 0;
    for (; in && *in && j < out_size - 2; in++) {
        if (*in == '"' || *in == '\\') { out[j++] = '\\'; out[j++] = *in; }
        else if (*in >= 32 && *in < 127) out[j++] = *in;
    }
    out[j] = '\0';
}

static esp_err_t handler_get_api_halow(httpd_req_t *req)
{
    bridge_settings_t st;
    settings_load(&st);

    struct mmipal_ip_config ip_cfg;
    bool halow_up = (mmipal_get_ip_config(&ip_cfg) == MMIPAL_SUCCESS &&
                     ip_cfg.ip_addr[0] && strcmp(ip_cfg.ip_addr, "0.0.0.0") != 0);
    static bool halow_up_last = false;
    static int64_t halow_up_since_us = 0;
    int64_t now_us = esp_timer_get_time();
    if (halow_up && !halow_up_last) {
        halow_up_since_us = now_us;
    } else if (!halow_up) {
        halow_up_since_us = 0;
    }
    halow_up_last = halow_up;

    int32_t rssi = mmwlan_get_rssi();
    char rssi_buf[16];
    if (rssi != (int32_t)INT32_MIN) {
        snprintf(rssi_buf, sizeof(rssi_buf), "%ld", (long)rssi);
    } else {
        snprintf(rssi_buf, sizeof(rssi_buf), "null");
    }

    char esc_ssid[SETTINGS_MAX_SSID * 2];
    char esc_ip[64];
    char esc_gw[64];
    char esc_mac[32];
    json_escape(st.halow_ssid, esc_ssid, sizeof(esc_ssid));
    json_escape(halow_up ? ip_cfg.ip_addr : "", esc_ip, sizeof(esc_ip));
    json_escape(halow_up ? ip_cfg.gateway_addr : "", esc_gw, sizeof(esc_gw));
    uint8_t mac[MMWLAN_MAC_ADDR_LEN] = {0};
    char mac_buf[18] = "";
    const char *mac_str = "";
    if (mmwlan_get_mac_addr(mac) == MMWLAN_SUCCESS) {
        snprintf(mac_buf, sizeof(mac_buf), "%02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        mac_str = mac_buf;
    }
    json_escape(mac_str, esc_mac, sizeof(esc_mac));
    char uptime_buf[24];
    if (halow_up && halow_up_since_us > 0 && now_us >= halow_up_since_us) {
        int64_t uptime_s = (now_us - halow_up_since_us) / 1000000;
        snprintf(uptime_buf, sizeof(uptime_buf), "%ld", (long)uptime_s);
    } else {
        snprintf(uptime_buf, sizeof(uptime_buf), "null");
    }

    uint8_t bw = app_wlan_get_op_bw_mhz();
    char bw_buf[16];
    if (bw > 0) {
        snprintf(bw_buf, sizeof(bw_buf), "%u", (unsigned)bw);
    } else {
        snprintf(bw_buf, sizeof(bw_buf), "null");
    }
    char buf[416];
    int len = snprintf(buf, sizeof(buf),
        "{\"up\":%s,\"ssid\":\"%s\",\"ip\":\"%s\",\"gateway\":\"%s\",\"mac\":\"%s\","
        "\"link_uptime_s\":%s,\"rssi_dbm\":%s,\"bw_mhz\":%s}",
        halow_up ? "true" : "false", esc_ssid, esc_ip, esc_gw, esc_mac,
        uptime_buf, rssi_buf, bw_buf);
    if (len <= 0 || len >= (int)sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Stats failed");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, (ssize_t)len);
    return ESP_OK;
}

static esp_err_t handler_post_api_halow_reconnect(httpd_req_t *req)
{
    (void)req;
    bool ok = app_wlan_request_reconnect();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, ok ? "{\"ok\":true}" : "{\"ok\":false}", ok ? 10 : 11);
    return ok ? ESP_OK : ESP_FAIL;
}

static esp_err_t handler_get_api_wifi2g(httpd_req_t *req)
{
    bridge_settings_t st;
    settings_load(&st);

    wifi_config_t ap_cfg = {0};
    bool ap_ok = (esp_wifi_get_config(WIFI_IF_AP, &ap_cfg) == ESP_OK);
    const char *ap_ssid_raw = ap_ok && ap_cfg.ap.ssid[0] ? (char *)ap_cfg.ap.ssid : st.ap_ssid;
    uint8_t ap_channel = ap_ok ? ap_cfg.ap.channel : 0;

    wifi_sta_list_t sta_list = {0};
    int ap_clients = 0;
    if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK) {
        ap_clients = (int)sta_list.num;
    }

    char ap_ip_buf[32] = "";
    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif) {
        esp_netif_ip_info_t ip;
        if (esp_netif_get_ip_info(ap_netif, &ip) == ESP_OK) {
            snprintf(ap_ip_buf, sizeof(ap_ip_buf), IPSTR, IP2STR(&ip.ip));
        }
    }

    wifi_ap_record_t sta_info = {0};
    bool sta_connected = (esp_wifi_sta_get_ap_info(&sta_info) == ESP_OK);
    const char *sta_ssid_raw = sta_connected ? (char *)sta_info.ssid : "";
    int sta_rssi = sta_connected ? (int)sta_info.rssi : 0;

    char sta_ip_buf[32] = "";
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif) {
        esp_netif_ip_info_t ip;
        if (esp_netif_get_ip_info(sta_netif, &ip) == ESP_OK) {
            snprintf(sta_ip_buf, sizeof(sta_ip_buf), IPSTR, IP2STR(&ip.ip));
        }
    }

    char esc_ap_ssid[SETTINGS_MAX_SSID * 2];
    char esc_sta_ssid[SETTINGS_MAX_SSID * 2];
    char esc_ap_ip[32 * 2];
    char esc_sta_ip[32 * 2];
    json_escape(ap_ssid_raw, esc_ap_ssid, sizeof(esc_ap_ssid));
    json_escape(sta_ssid_raw, esc_sta_ssid, sizeof(esc_sta_ssid));
    json_escape(ap_ip_buf, esc_ap_ip, sizeof(esc_ap_ip));
    json_escape(sta_ip_buf, esc_sta_ip, sizeof(esc_sta_ip));

    const char *mode = (st.backhaul_mode == BACKHAUL_MODE_WIFI_2G) ? "wifi2g" : "halow";
    char rssi_buf[16];
    if (sta_connected) {
        snprintf(rssi_buf, sizeof(rssi_buf), "%d", sta_rssi);
    } else {
        snprintf(rssi_buf, sizeof(rssi_buf), "null");
    }
    char buf[512];
    int len = snprintf(
        buf, sizeof(buf),
        "{\"ap\":{\"ssid\":\"%s\",\"channel\":%u,\"clients\":%d,\"ip\":\"%s\"},"
        "\"sta\":{\"connected\":%s,\"ssid\":\"%s\",\"rssi_dbm\":%s,\"ip\":\"%s\"},"
        "\"backhaul_mode\":\"%s\"}",
        esc_ap_ssid, (unsigned)ap_channel, ap_clients, esc_ap_ip,
        sta_connected ? "true" : "false", esc_sta_ssid, rssi_buf, esc_sta_ip, mode);
    if (len <= 0 || len >= (int)sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Stats failed");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, (ssize_t)len);
    return ESP_OK;
}

static esp_err_t handler_get_api_sensors(httpd_req_t *req)
{
    static char buf[2048];
    int n = esp_now_rcv_node_count();
    int len = snprintf(buf, sizeof(buf), "{\"local\":null,\"nodes\":[");
    for (int i = 0; i < n && len < (int)sizeof(buf) - 256; i++) {
        const node_entry_t *e = esp_now_rcv_get_node(i);
        if (!e) continue;
        const sensor_packet_t *p = &e->pkt;
        char lbl[96];
        json_escape(esp_now_rcv_get_label(e->mac), lbl, sizeof(lbl));
        char ble_addr[20];
        if (p->ble_last_addr[0] || p->ble_last_addr[1] || p->ble_last_addr[2] ||
            p->ble_last_addr[3] || p->ble_last_addr[4] || p->ble_last_addr[5]) {
            snprintf(ble_addr, sizeof(ble_addr), "%02X:%02X:%02X:%02X:%02X:%02X",
                     p->ble_last_addr[0], p->ble_last_addr[1], p->ble_last_addr[2],
                     p->ble_last_addr[3], p->ble_last_addr[4], p->ble_last_addr[5]);
        } else {
            ble_addr[0] = '\0';
        }
        bool env_missing = (p->temperature == 0.0f && p->humidity == 0.0f &&
            p->pressure == 0.0f && p->gas == 0.0f);
        char temp_buf[16];
        char hum_buf[16];
        char pres_buf[16];
        char gas_buf[16];
        const char *temp_str = "null";
        const char *hum_str = "null";
        const char *pres_str = "null";
        const char *gas_str = "null";
        if (!env_missing) {
            snprintf(temp_buf, sizeof(temp_buf), "%.2f", (double)p->temperature);
            snprintf(hum_buf, sizeof(hum_buf), "%.2f", (double)p->humidity);
            snprintf(pres_buf, sizeof(pres_buf), "%.2f", (double)p->pressure);
            snprintf(gas_buf, sizeof(gas_buf), "%.2f", (double)p->gas);
            temp_str = temp_buf;
            hum_str = hum_buf;
            pres_str = pres_buf;
            gas_str = gas_buf;
        }
        len += snprintf(buf + len, sizeof(buf) - len,
            "%s{\"mac\":\"%s\",\"label\":\"%s\",\"motion\":%u,\"trigger_count\":%lu,\"last_motion_ms\":%lu,"
            "\"rssi_dbm\":%d,\"ble_seen\":%u,\"ble_last_addr\":\"%s\",\"ble_last_rssi\":%d,"
            "\"temperature\":%s,\"humidity\":%s,\"pressure\":%s,\"gas\":%s,\"uptime_ms\":%lu}",
            i ? "," : "", e->mac, lbl, (unsigned)p->motion, (unsigned long)e->trigger_count,
            (unsigned long)e->last_motion_uptime_ms, (int)e->rssi_dbm,
            (unsigned)p->ble_seen_count, ble_addr, (int)p->ble_last_rssi_dbm,
            temp_str, hum_str, pres_str, gas_str, (unsigned long)p->uptime_ms);
    }
    len += snprintf(buf + len, sizeof(buf) - len, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

static esp_err_t handler_post_api_sensors_reset(httpd_req_t *req)
{
    char body[128];
    int r = httpd_req_recv(req, body, sizeof(body) - 1);
    if (r <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad"); return ESP_FAIL; }
    body[r] = '\0';
    char *mac = strstr(body, "\"mac\":\"");
    if (!mac) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad"); return ESP_FAIL; }
    mac += 7;
    char *mac_end = mac;
    while (*mac_end && *mac_end != '"') { if (*mac_end == '\\') mac_end++; mac_end++; }
    char mac_buf[NODE_MAC_LEN];
    size_t mac_len = mac_end - mac; if (mac_len >= sizeof(mac_buf)) mac_len = sizeof(mac_buf) - 1;
    memcpy(mac_buf, mac, mac_len); mac_buf[mac_len] = '\0';
    bool ok = esp_now_rcv_send_reset(mac_buf);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, ok ? "{\"ok\":true}" : "{\"ok\":false}", ok ? 10 : 11);
    return ok ? ESP_OK : ESP_FAIL;
}

static esp_err_t handler_get_api_log(httpd_req_t *req)
{
    static char buf[8192];
    int n = sensor_log_count();
    int len = snprintf(buf, sizeof(buf), "{\"entries\":[");
    for (int i = 0; i < n && len < (int)sizeof(buf) - 200; i++) {
        const sensor_log_entry_t *e = sensor_log_get(i);
        if (!e) continue;
        const sensor_packet_t *p = &e->pkt;
        char lbl[96];
        json_escape(esp_now_rcv_get_label(e->mac), lbl, sizeof(lbl));
        bool env_missing = (p->temperature == 0.0f && p->humidity == 0.0f &&
            p->pressure == 0.0f && p->gas == 0.0f);
        char temp_buf[16];
        char hum_buf[16];
        char pres_buf[16];
        char gas_buf[16];
        const char *temp_str = "null";
        const char *hum_str = "null";
        const char *pres_str = "null";
        const char *gas_str = "null";
        if (!env_missing) {
            snprintf(temp_buf, sizeof(temp_buf), "%.2f", (double)p->temperature);
            snprintf(hum_buf, sizeof(hum_buf), "%.2f", (double)p->humidity);
            snprintf(pres_buf, sizeof(pres_buf), "%.2f", (double)p->pressure);
            snprintf(gas_buf, sizeof(gas_buf), "%.2f", (double)p->gas);
            temp_str = temp_buf;
            hum_str = hum_buf;
            pres_str = pres_buf;
            gas_str = gas_buf;
        }
        len += snprintf(buf + len, sizeof(buf) - len,
            "%s{\"mac\":\"%s\",\"label\":\"%s\",\"ts_ms\":%" PRId64 ",\"motion\":%u,\"temperature\":%s,\"humidity\":%s,"
            "\"pressure\":%s,\"gas\":%s,\"uptime_ms\":%lu}",
            i ? "," : "", e->mac, lbl, e->ts_ms, (unsigned)p->motion,
            temp_str, hum_str, pres_str, gas_str, (unsigned long)p->uptime_ms);
    }
    len += snprintf(buf + len, sizeof(buf) - len, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

static esp_err_t handler_post_api_log_clear(httpd_req_t *req)
{
    sensor_log_clear();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", 10);
    return ESP_OK;
}

static esp_err_t handler_get_api_debug(httpd_req_t *req)
{
    static char buf[256];
    size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    size_t min_free_heap = esp_get_minimum_free_heap_size();
    size_t used_heap = (total_heap > free_heap) ? (total_heap - free_heap) : 0;
    unsigned used_pct = total_heap ? (unsigned)((used_heap * 100U) / total_heap) : 0;
    int64_t time_ms = time_sync_get_epoch_ms();
    int time_valid = (time_ms >= 0) ? 1 : 0;
    int len = snprintf(
        buf, sizeof(buf),
        "{\"node_count\":%d,\"heap_total\":%u,\"heap_free\":%u,\"heap_min_free\":%u,"
        "\"heap_used\":%u,\"heap_used_pct\":%u,\"time_ms\":%" PRId64 ",\"time_valid\":%d}",
        esp_now_rcv_node_count(),
        (unsigned)total_heap, (unsigned)free_heap, (unsigned)min_free_heap,
        (unsigned)used_heap, used_pct, time_ms, time_valid);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

static esp_err_t handler_get_api_labels(httpd_req_t *req)
{
    static char buf[1024];
    int n = esp_now_rcv_node_count();
    int len = snprintf(buf, sizeof(buf), "{\"labels\":{");
    for (int i = 0; i < n && len < (int)sizeof(buf) - 128; i++) {
        const node_entry_t *e = esp_now_rcv_get_node(i);
        if (!e) continue;
        char lbl[96];
        json_escape(esp_now_rcv_get_label(e->mac), lbl, sizeof(lbl));
        len += snprintf(buf + len, sizeof(buf) - len, "%s\"%s\":\"%s\"", i ? "," : "", e->mac, lbl);
    }
    len += snprintf(buf + len, sizeof(buf) - len, "}}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

static esp_err_t handler_post_api_labels(httpd_req_t *req)
{
    char body[256];
    int r = httpd_req_recv(req, body, sizeof(body) - 1);
    if (r <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad"); return ESP_FAIL; }
    body[r] = '\0';
    char *mac = strstr(body, "\"mac\":\"");
    if (!mac) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad"); return ESP_FAIL; }
    mac += 7;
    char *mac_end = mac;
    while (*mac_end && *mac_end != '"') { if (*mac_end == '\\') mac_end++; mac_end++; }
    char *lbl = strstr(mac_end, "\"label\":\"");
    if (!lbl) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad"); return ESP_FAIL; }
    lbl += 9;
    char *lbl_end = lbl;
    while (*lbl_end && *lbl_end != '"') { if (*lbl_end == '\\') lbl_end++; lbl_end++; }
    char mac_buf[NODE_MAC_LEN];
    char lbl_buf[64];
    size_t mac_len = mac_end - mac; if (mac_len >= sizeof(mac_buf)) mac_len = sizeof(mac_buf) - 1;
    memcpy(mac_buf, mac, mac_len); mac_buf[mac_len] = '\0';
    size_t lbl_len = lbl_end - lbl; if (lbl_len >= sizeof(lbl_buf)) lbl_len = sizeof(lbl_buf) - 1;
    memcpy(lbl_buf, lbl, lbl_len); lbl_buf[lbl_len] = '\0';
    esp_now_rcv_set_label(mac_buf, lbl_buf);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", 10);
    return ESP_OK;
}

static void cameras_load(char urls[][CAMERA_URL_LEN], int *count)
{
    nvs_handle_t h;
    *count = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    char all[512];
    size_t len = sizeof(all);
    if (nvs_get_str(h, NVS_CAMERAS_KEY, all, &len) != ESP_OK) { nvs_close(h); return; }
    nvs_close(h);
    char *p = all;
    while (*count < MAX_CAMERAS && p && *p) {
        char *end = strchr(p, '\n');
        size_t n = end ? (size_t)(end - p) : strlen(p);
        if (n > 0 && n < CAMERA_URL_LEN) {
            memcpy(urls[*count], p, n);
            urls[*count][n] = '\0';
            (*count)++;
        }
        p = end ? end + 1 : NULL;
    }
}

static void cameras_save(const char urls[][CAMERA_URL_LEN], int count)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    char all[512] = "";
    for (int i = 0; i < count && strlen(all) < sizeof(all) - CAMERA_URL_LEN - 2; i++) {
        strlcat(all, urls[i], sizeof(all));
        strlcat(all, "\n", sizeof(all));
    }
    nvs_set_str(h, NVS_CAMERAS_KEY, all);
    nvs_commit(h);
    nvs_close(h);
}

static esp_err_t handler_get_api_cameras(httpd_req_t *req)
{
    static char urls[MAX_CAMERAS][CAMERA_URL_LEN];
    int count = 0;
    cameras_load(urls, &count);
    static char buf[600];
    int len = snprintf(buf, sizeof(buf), "{\"urls\":[");
    for (int i = 0; i < count; i++) {
        char esc[256];
        json_escape(urls[i], esc, sizeof(esc));
        len += snprintf(buf + len, sizeof(buf) - len, "%s\"%s\"", i ? "," : "", esc);
    }
    len += snprintf(buf + len, sizeof(buf) - len, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

static esp_err_t handler_post_api_cameras(httpd_req_t *req)
{
    static char body[512];
    int r = httpd_req_recv(req, body, sizeof(body) - 1);
    if (r <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad"); return ESP_FAIL; }
    body[r] = '\0';
    static char urls[MAX_CAMERAS][CAMERA_URL_LEN];
    int count = 0;
    char *p = body;
    while (count < MAX_CAMERAS && (p = strchr(p, '"')) != NULL) {
        p++;
        char *end = p;
        while (*end && *end != '"') { if (*end == '\\') end++; end++; }
        size_t n = end - p;
        if (n > 0 && n < CAMERA_URL_LEN) {
            memcpy(urls[count], p, n); urls[count][n] = '\0';
            count++;
        }
        p = end + 1;
    }
    cameras_save(urls, count);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", 10);
    return ESP_OK;
}

static const httpd_uri_t uri_root =          { .uri = "/", .method = HTTP_GET, .handler = handler_get_gateway };
static const httpd_uri_t uri_gateway =       { .uri = "/gateway", .method = HTTP_GET, .handler = handler_get_gateway };
static const httpd_uri_t uri_sensors =       { .uri = "/api/sensors", .method = HTTP_GET, .handler = handler_get_api_sensors };
static const httpd_uri_t uri_sensors_reset = { .uri = "/api/sensors/reset", .method = HTTP_POST, .handler = handler_post_api_sensors_reset };
static const httpd_uri_t uri_log_get =       { .uri = "/api/log", .method = HTTP_GET, .handler = handler_get_api_log };
static const httpd_uri_t uri_log_clear =     { .uri = "/api/log/clear", .method = HTTP_POST, .handler = handler_post_api_log_clear };
static const httpd_uri_t uri_debug =         { .uri = "/api/debug", .method = HTTP_GET, .handler = handler_get_api_debug };
static const httpd_uri_t uri_halow =         { .uri = "/api/halow", .method = HTTP_GET, .handler = handler_get_api_halow };
static const httpd_uri_t uri_halow_reconnect = { .uri = "/api/halow/reconnect", .method = HTTP_POST, .handler = handler_post_api_halow_reconnect };
static const httpd_uri_t uri_wifi2g =        { .uri = "/api/wifi2g", .method = HTTP_GET, .handler = handler_get_api_wifi2g };
static const httpd_uri_t uri_labels_get =    { .uri = "/api/labels", .method = HTTP_GET, .handler = handler_get_api_labels };
static const httpd_uri_t uri_labels_post =   { .uri = "/api/labels", .method = HTTP_POST, .handler = handler_post_api_labels };
static const httpd_uri_t uri_cameras_get =   { .uri = "/api/cameras", .method = HTTP_GET, .handler = handler_get_api_cameras };
static const httpd_uri_t uri_cameras_post =  { .uri = "/api/cameras", .method = HTTP_POST, .handler = handler_post_api_cameras };

void sensor_gateway_http_register(httpd_handle_t server)
{
    httpd_register_uri_handler(server, &uri_root);
    httpd_register_uri_handler(server, &uri_gateway);
    httpd_register_uri_handler(server, &uri_sensors);
    httpd_register_uri_handler(server, &uri_sensors_reset);
    httpd_register_uri_handler(server, &uri_log_get);
    httpd_register_uri_handler(server, &uri_log_clear);
    httpd_register_uri_handler(server, &uri_debug);
    httpd_register_uri_handler(server, &uri_halow);
    httpd_register_uri_handler(server, &uri_halow_reconnect);
    httpd_register_uri_handler(server, &uri_wifi2g);
    httpd_register_uri_handler(server, &uri_labels_get);
    httpd_register_uri_handler(server, &uri_labels_post);
    httpd_register_uri_handler(server, &uri_cameras_get);
    httpd_register_uri_handler(server, &uri_cameras_post);
}
