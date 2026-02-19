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

static void json_escape(const char *in, char *out, size_t out_size);

/* Format moisture[4] as JSON array: [12.34,null,null,null] */
static int fmt_moisture_array(const float *m, char *buf, size_t sz)
{
    int n = 0;
    n += snprintf(buf + n, sz - n, "[");
    for (int i = 0; i < SENSOR_MOISTURE_CHANNELS; i++) {
        if (i > 0) n += snprintf(buf + n, sz - n, ",");
        if (m[i] >= 0.0f)
            n += snprintf(buf + n, sz - n, "%.2f", (double)m[i]);
        else
            n += snprintf(buf + n, sz - n, "null");
    }
    n += snprintf(buf + n, sz - n, "]");
    return n;
}

/* Format plant_label[4][16] as JSON array: ["Tomato","Basil","",""] */
static int fmt_plant_labels(const char labels[][SENSOR_PLANT_LABEL_LEN], char *buf, size_t sz)
{
    int n = 0;
    n += snprintf(buf + n, sz - n, "[");
    for (int i = 0; i < SENSOR_MOISTURE_CHANNELS; i++) {
        if (i > 0) n += snprintf(buf + n, sz - n, ",");
        char esc[SENSOR_PLANT_LABEL_LEN * 2];
        json_escape(labels[i], esc, sizeof(esc));
        n += snprintf(buf + n, sz - n, "\"%s\"", esc);
    }
    n += snprintf(buf + n, sz - n, "]");
    return n;
}

#define NVS_NAMESPACE "gateway"
#define NVS_CAMERAS_KEY "cameras"
#define MAX_CAMERAS 4
#define CAMERA_URL_LEN 128

#define DASHBOARD_CHUNK_SIZE 4096

static esp_err_t handler_get_gateway(httpd_req_t *req)
{
    const char *html = sensor_gateway_get_dashboard_html();
    size_t total = sensor_gateway_get_dashboard_html_len();
    if (html == NULL || total == 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Dashboard not available");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "text/html");
    /* Send in chunks to avoid large single send (buffer/timeout on slow links) */
    size_t sent = 0;
    while (sent < total) {
        size_t n = (total - sent) > (size_t)DASHBOARD_CHUNK_SIZE
                   ? (size_t)DASHBOARD_CHUNK_SIZE
                   : (total - sent);
        if (httpd_resp_send_chunk(req, html + sent, n) != ESP_OK) {
            return ESP_FAIL;
        }
        sent += n;
    }
    if (httpd_resp_send_chunk(req, NULL, 0) != ESP_OK) {
        return ESP_FAIL;
    }
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
    static char buf[8192];
    int n = esp_now_rcv_node_count();
    uint32_t gateway_uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    int len = snprintf(buf, sizeof(buf), "{\"local\":null,\"gateway_uptime_ms\":%lu,\"nodes\":[",
                       (unsigned long)gateway_uptime_ms);
    for (int i = 0; i < n && len < (int)sizeof(buf) - 600; i++) {
        const node_entry_t *e = esp_now_rcv_get_node(i);
        if (!e) continue;
        const sensor_packet_t *p = &e->pkt;
        char lbl[96];
        json_escape(esp_now_rcv_get_label(e->mac), lbl, sizeof(lbl));
        const char *loc = esp_now_rcv_get_location(e->mac);
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
        char temp_water_buf[16];
        char tds_buf[16];
        char hum_buf[16];
        char pres_buf[16];
        char gas_buf[16];
        static char moisture_arr[80];
        static char plabel_arr[SENSOR_MOISTURE_CHANNELS * (SENSOR_PLANT_LABEL_LEN * 2 + 4) + 8];
        const char *temp_str = "null";
        const char *temp_water_str = "null";
        const char *tds_str = "null";
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
        /* Air temp: send whenever valid (e.g. from BME or DS18B20 fallback when BME not ready) */
        if (p->temperature > -200.0f && p->temperature < 200.0f) {
            snprintf(temp_buf, sizeof(temp_buf), "%.2f", (double)p->temperature);
            temp_str = temp_buf;
        }
        if (p->temperature_water > SENSOR_TEMP_WATER_INVALID) {
            snprintf(temp_water_buf, sizeof(temp_water_buf), "%.2f", (double)p->temperature_water);
            temp_water_str = temp_water_buf;
        }
        if (p->tds_ppm >= 0.0f) {
            snprintf(tds_buf, sizeof(tds_buf), "%.1f", (double)p->tds_ppm);
            tds_str = tds_buf;
        }
        {
            float m_copy[SENSOR_MOISTURE_CHANNELS];
            memcpy(m_copy, p->moisture, sizeof(m_copy));
            fmt_moisture_array(m_copy, moisture_arr, sizeof(moisture_arr));
        }
        {
            char lbl_copy[SENSOR_MOISTURE_CHANNELS][SENSOR_PLANT_LABEL_LEN];
            memcpy(lbl_copy, p->plant_label, sizeof(lbl_copy));
            fmt_plant_labels(lbl_copy, plabel_arr, sizeof(plabel_arr));
        }
        len += snprintf(buf + len, sizeof(buf) - len,
            "%s{\"mac\":\"%s\",\"label\":\"%s\",\"motion\":%u,\"trigger_count\":%lu,\"last_motion_ms\":%lu,"
            "\"last_motion_seen_ms\":%lu,\"last_seen_ms\":%lu,\"rssi_dbm\":%d,\"ble_seen\":%u,\"ble_last_addr\":\"%s\",\"ble_last_rssi\":%d,"
            "\"temperature\":%s,\"temperature_water\":%s,\"tds_ppm\":%s,\"humidity\":%s,\"pressure\":%s,\"gas\":%s,\"moisture\":%s,"
            "\"plant_labels\":%s,\"uptime_ms\":%lu,\"location\":\"%s\","
            "\"mmwave_state\":%u,\"mmwave_moving_cm\":%u,\"mmwave_stationary_cm\":%u,"
            "\"mmwave_moving_energy\":%u,\"mmwave_stationary_energy\":%u,\"mmwave_detection_dist_cm\":%u}",
            i ? "," : "", e->mac, lbl, (unsigned)p->motion, (unsigned long)e->trigger_count,
            (unsigned long)e->last_motion_uptime_ms, (unsigned long)e->last_motion_seen_ms, (unsigned long)e->last_ms, (int)e->rssi_dbm,
            (unsigned)p->ble_seen_count, ble_addr, (int)p->ble_last_rssi_dbm,
            temp_str, temp_water_str, tds_str, hum_str, pres_str, gas_str, moisture_arr,
            plabel_arr, (unsigned long)p->uptime_ms, loc,
            (unsigned)p->mmwave_state, (unsigned)p->mmwave_moving_cm, (unsigned)p->mmwave_stationary_cm,
            (unsigned)p->mmwave_moving_energy, (unsigned)p->mmwave_stationary_energy, (unsigned)p->mmwave_detection_dist_cm);
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
        char temp_water_buf[16];
        char tds_buf[16];
        char hum_buf[16];
        char pres_buf[16];
        char gas_buf[16];
        static char moisture_arr[80];
        const char *temp_str = "null";
        const char *temp_water_str = "null";
        const char *tds_str = "null";
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
        if (p->temperature_water > SENSOR_TEMP_WATER_INVALID) {
            snprintf(temp_water_buf, sizeof(temp_water_buf), "%.2f", (double)p->temperature_water);
            temp_water_str = temp_water_buf;
        }
        if (p->tds_ppm >= 0.0f) {
            snprintf(tds_buf, sizeof(tds_buf), "%.1f", (double)p->tds_ppm);
            tds_str = tds_buf;
        }
        {
            float m_copy[SENSOR_MOISTURE_CHANNELS];
            memcpy(m_copy, p->moisture, sizeof(m_copy));
            fmt_moisture_array(m_copy, moisture_arr, sizeof(moisture_arr));
        }
        const char *loc = esp_now_rcv_get_location(e->mac);
        len += snprintf(buf + len, sizeof(buf) - len,
            "%s{\"mac\":\"%s\",\"label\":\"%s\",\"ts_ms\":%" PRId64 ",\"motion\":%u,\"location\":\"%s\","
            "\"temperature\":%s,\"temperature_water\":%s,\"tds_ppm\":%s,\"humidity\":%s,\"pressure\":%s,\"gas\":%s,\"moisture\":%s,\"uptime_ms\":%lu,"
            "\"mmwave_state\":%u,\"mmwave_moving_cm\":%u,\"mmwave_stationary_cm\":%u,"
            "\"mmwave_moving_energy\":%u,\"mmwave_stationary_energy\":%u,\"mmwave_detection_dist_cm\":%u}",
            i ? "," : "", e->mac, lbl, e->ts_ms, (unsigned)p->motion, loc,
            temp_str, temp_water_str, tds_str, hum_str, pres_str, gas_str, moisture_arr, (unsigned long)p->uptime_ms,
            (unsigned)p->mmwave_state, (unsigned)p->mmwave_moving_cm, (unsigned)p->mmwave_stationary_cm,
            (unsigned)p->mmwave_moving_energy, (unsigned)p->mmwave_stationary_energy, (unsigned)p->mmwave_detection_dist_cm);
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
    static char buf[320];
    size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    size_t min_free_heap = esp_get_minimum_free_heap_size();
    size_t used_heap = (total_heap > free_heap) ? (total_heap - free_heap) : 0;
    unsigned used_pct = total_heap ? (unsigned)((used_heap * 100U) / total_heap) : 0;
    int64_t time_ms = time_sync_get_epoch_ms();
    int time_valid = (time_ms >= 0) ? 1 : 0;
    uint32_t gateway_uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    int len = snprintf(
        buf, sizeof(buf),
        "{\"node_count\":%d,\"gateway_count\":1,\"gateway_uptime_ms\":%lu,\"espnow_channel\":%d,\"espnow_enabled\":true,"
        "\"heap_total\":%u,\"heap_free\":%u,\"heap_min_free\":%u,"
        "\"heap_used\":%u,\"heap_used_pct\":%u,\"time_ms\":%" PRId64 ",\"time_valid\":%d}",
        esp_now_rcv_node_count(), (unsigned long)gateway_uptime_ms, esp_now_rcv_get_channel(),
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

static esp_err_t handler_post_api_location(httpd_req_t *req)
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
    char *loc = strstr(mac_end, "\"location\":\"");
    if (!loc) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad"); return ESP_FAIL; }
    loc += 12;
    char *loc_end = loc;
    while (*loc_end && *loc_end != '"') { if (*loc_end == '\\') loc_end++; loc_end++; }
    char mac_buf[NODE_MAC_LEN];
    char loc_buf[16];
    size_t mac_len = mac_end - mac; if (mac_len >= sizeof(mac_buf)) mac_len = sizeof(mac_buf) - 1;
    memcpy(mac_buf, mac, mac_len); mac_buf[mac_len] = '\0';
    size_t loc_len = loc_end - loc; if (loc_len >= sizeof(loc_buf)) loc_len = sizeof(loc_buf) - 1;
    memcpy(loc_buf, loc, loc_len); loc_buf[loc_len] = '\0';
    esp_now_rcv_set_location(mac_buf, loc_buf);
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

/* UI skin: dashboard expects {"skin":"...","font_size":"..."} */
static esp_err_t handler_get_api_ui_skin(httpd_req_t *req)
{
    char skin[32], font_size[16];
    esp_now_rcv_get_ui_skin(skin, sizeof(skin), font_size, sizeof(font_size));
    char esc_skin[64], esc_fs[32];
    json_escape(skin, esc_skin, sizeof(esc_skin));
    json_escape(font_size, esc_fs, sizeof(esc_fs));
    char buf[128];
    int len = snprintf(buf, sizeof(buf), "{\"skin\":\"%s\",\"font_size\":\"%s\"}", esc_skin, esc_fs);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

static esp_err_t handler_post_api_ui_skin(httpd_req_t *req)
{
    char body[128];
    int r = httpd_req_recv(req, body, sizeof(body) - 1);
    if (r <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad"); return ESP_FAIL; }
    body[r] = '\0';
    char skin[32] = "", font_size[16] = "";
    char *sp = strstr(body, "\"skin\":\"");
    if (sp) {
        sp += 8;
        char *se = sp; while (*se && *se != '"') se++;
        size_t n = se - sp; if (n >= sizeof(skin)) n = sizeof(skin) - 1;
        memcpy(skin, sp, n); skin[n] = '\0';
    }
    char *fp = strstr(body, "\"font_size\":\"");
    if (fp) {
        fp += 13;
        char *fe = fp; while (*fe && *fe != '"') fe++;
        size_t n = fe - fp; if (n >= sizeof(font_size)) n = sizeof(font_size) - 1;
        memcpy(font_size, fp, n); font_size[n] = '\0';
    }
    esp_now_rcv_set_ui_skin(skin, font_size);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", 10);
    return ESP_OK;
}

/* BLE log: gateway has no BLE log; return empty. Dashboard expects {"entries":[...]} */
static esp_err_t handler_get_api_ble_log(httpd_req_t *req)
{
    (void)req;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"entries\":[]}", 14);
    return ESP_OK;
}

static esp_err_t handler_post_api_ble_log_clear(httpd_req_t *req)
{
    (void)req;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", 10);
    return ESP_OK;
}

/* BLE whitelist: stub (no persistence on gateway). Dashboard expects {"entries":[{mac,label},...]} */
static esp_err_t handler_get_api_ble_whitelist(httpd_req_t *req)
{
    (void)req;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"entries\":[]}", 14);
    return ESP_OK;
}

static esp_err_t handler_post_api_ble_whitelist(httpd_req_t *req)
{
    (void)req;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", 10);
    return ESP_OK;
}

static esp_err_t handler_post_api_ble_whitelist_remove(httpd_req_t *req)
{
    (void)req;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", 10);
    return ESP_OK;
}

static esp_err_t handler_post_api_ble_whitelist_capture(httpd_req_t *req)
{
    (void)req;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true,\"count\":0}", 20);
    return ESP_OK;
}

static esp_err_t handler_post_api_ble_whitelist_clear(httpd_req_t *req)
{
    (void)req;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", 10);
    return ESP_OK;
}

/* WiFi log: gateway has no wifi log; return empty. Dashboard expects {"entries":[...]} */
static esp_err_t handler_get_api_wifi_log(httpd_req_t *req)
{
    (void)req;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"entries\":[]}", 14);
    return ESP_OK;
}

static esp_err_t handler_post_api_wifi_log_clear(httpd_req_t *req)
{
    (void)req;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", 10);
    return ESP_OK;
}

static esp_err_t handler_post_api_wifi_log_enable(httpd_req_t *req)
{
    (void)req;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", 10);
    return ESP_OK;
}

/* GET /favicon.ico — browsers request this automatically; return 204 to avoid 404 log spam */
static esp_err_t handler_get_favicon(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t uri_favicon =       { .uri = "/favicon.ico", .method = HTTP_GET, .handler = handler_get_favicon };
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
static const httpd_uri_t uri_location_post = { .uri = "/api/location", .method = HTTP_POST, .handler = handler_post_api_location };
static const httpd_uri_t uri_cameras_get =   { .uri = "/api/cameras", .method = HTTP_GET, .handler = handler_get_api_cameras };
static const httpd_uri_t uri_cameras_post =  { .uri = "/api/cameras", .method = HTTP_POST, .handler = handler_post_api_cameras };
static const httpd_uri_t uri_ui_skin =       { .uri = "/api/ui_skin", .method = HTTP_GET, .handler = handler_get_api_ui_skin };
static const httpd_uri_t uri_ui_skin_post =  { .uri = "/api/ui_skin", .method = HTTP_POST, .handler = handler_post_api_ui_skin };
static const httpd_uri_t uri_ble_log =      { .uri = "/api/ble_log", .method = HTTP_GET, .handler = handler_get_api_ble_log };
static const httpd_uri_t uri_ble_log_clear = { .uri = "/api/ble_log/clear", .method = HTTP_POST, .handler = handler_post_api_ble_log_clear };
static const httpd_uri_t uri_ble_whitelist_get = { .uri = "/api/ble_whitelist", .method = HTTP_GET, .handler = handler_get_api_ble_whitelist };
static const httpd_uri_t uri_ble_whitelist_post = { .uri = "/api/ble_whitelist", .method = HTTP_POST, .handler = handler_post_api_ble_whitelist };
static const httpd_uri_t uri_ble_whitelist_remove = { .uri = "/api/ble_whitelist/remove", .method = HTTP_POST, .handler = handler_post_api_ble_whitelist_remove };
static const httpd_uri_t uri_ble_whitelist_capture = { .uri = "/api/ble_whitelist/capture", .method = HTTP_POST, .handler = handler_post_api_ble_whitelist_capture };
static const httpd_uri_t uri_ble_whitelist_clear = { .uri = "/api/ble_whitelist/clear", .method = HTTP_POST, .handler = handler_post_api_ble_whitelist_clear };
static const httpd_uri_t uri_wifi_log =      { .uri = "/api/wifi_log", .method = HTTP_GET, .handler = handler_get_api_wifi_log };
static const httpd_uri_t uri_wifi_log_clear = { .uri = "/api/wifi_log/clear", .method = HTTP_POST, .handler = handler_post_api_wifi_log_clear };
static const httpd_uri_t uri_wifi_log_enable = { .uri = "/api/wifi_log/enable", .method = HTTP_POST, .handler = handler_post_api_wifi_log_enable };

/* POST /api/plant_label — set a plant label on a sensor unit (stored in NVS on the sensor).
   Body: {"mac":"AA:BB:CC:DD:EE:FF","channel":0,"label":"Tomato"} */
static esp_err_t handler_post_api_plant_label(httpd_req_t *req)
{
    char body[192];
    int r = httpd_req_recv(req, body, sizeof(body) - 1);
    if (r <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad"); return ESP_FAIL; }
    body[r] = '\0';
    /* Parse mac */
    char *mp = strstr(body, "\"mac\":\"");
    if (!mp) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing mac"); return ESP_FAIL; }
    mp += 7;
    char *me = mp; while (*me && *me != '"') me++;
    char mac_buf[NODE_MAC_LEN];
    size_t ml = me - mp; if (ml >= sizeof(mac_buf)) ml = sizeof(mac_buf) - 1;
    memcpy(mac_buf, mp, ml); mac_buf[ml] = '\0';
    /* Parse channel */
    char *cp = strstr(body, "\"channel\":");
    if (!cp) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing channel"); return ESP_FAIL; }
    int ch = atoi(cp + 10);
    /* Parse label */
    char label_buf[SENSOR_PLANT_LABEL_LEN] = { 0 };
    char *lp = strstr(body, "\"label\":\"");
    if (lp) {
        lp += 9;
        char *le = lp; while (*le && *le != '"') { if (*le == '\\') le++; le++; }
        size_t ll = le - lp; if (ll >= sizeof(label_buf)) ll = sizeof(label_buf) - 1;
        memcpy(label_buf, lp, ll);
    }
    bool ok = esp_now_rcv_send_plant_label(mac_buf, ch, label_buf);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, ok ? "{\"ok\":true}" : "{\"ok\":false}");
    return ESP_OK;
}

static const httpd_uri_t uri_plant_label = { .uri = "/api/plant_label", .method = HTTP_POST, .handler = handler_post_api_plant_label };

void sensor_gateway_http_register(httpd_handle_t server)
{
    httpd_register_uri_handler(server, &uri_favicon);
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
    httpd_register_uri_handler(server, &uri_location_post);
    httpd_register_uri_handler(server, &uri_cameras_get);
    httpd_register_uri_handler(server, &uri_cameras_post);
    httpd_register_uri_handler(server, &uri_ui_skin);
    httpd_register_uri_handler(server, &uri_ui_skin_post);
    httpd_register_uri_handler(server, &uri_ble_log);
    httpd_register_uri_handler(server, &uri_ble_log_clear);
    httpd_register_uri_handler(server, &uri_ble_whitelist_get);
    httpd_register_uri_handler(server, &uri_ble_whitelist_post);
    httpd_register_uri_handler(server, &uri_ble_whitelist_remove);
    httpd_register_uri_handler(server, &uri_ble_whitelist_capture);
    httpd_register_uri_handler(server, &uri_ble_whitelist_clear);
    httpd_register_uri_handler(server, &uri_wifi_log);
    httpd_register_uri_handler(server, &uri_wifi_log_clear);
    httpd_register_uri_handler(server, &uri_wifi_log_enable);
    httpd_register_uri_handler(server, &uri_plant_label);
}
