#include "esp_now_rcv.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs.h"
#include "time_sync.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

#define ESPNOW_CHANNEL 6
#define NVS_NAMESPACE "gateway"
#define NVS_LABELS_KEY "slabels"
#define NVS_LABELS_MAX 512
#define NVS_LOG_KEY "slog"

static const char *TAG = "esp_now_rcv";

static node_entry_t s_nodes[MAX_NODES];
static int s_node_count = 0;
static nvs_handle_t s_nvs = 0;

static sensor_log_entry_t s_log[SENSOR_LOG_MAX];
static int s_log_count = 0;
static int s_log_head = 0;

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  motion;
    float    temperature;
    float    humidity;
    float    pressure;
    float    gas;
    uint32_t uptime_ms;
} sensor_packet_v1_t;

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  motion;
    float    temperature;
    float    humidity;
    float    pressure;
    float    gas;
    uint32_t last_motion_ms;
    uint16_t ble_seen_count;
    int8_t   ble_last_rssi_dbm;
    uint8_t  ble_last_addr[6];
    uint32_t uptime_ms;
} sensor_packet_v3_t;

static int find_node(const char *mac)
{
    for (int i = 0; i < s_node_count; i++)
        if (strcmp(s_nodes[i].mac, mac) == 0)
            return i;
    return -1;
}

static bool env_missing_packet(const sensor_packet_t *p)
{
    return (p->temperature == 0.0f && p->humidity == 0.0f &&
            p->pressure == 0.0f && p->gas == 0.0f);
}

void store_node(const uint8_t *mac_addr, const sensor_packet_t *p, int8_t rssi_dbm, bool has_trigger_count)
{
    char mac_str[NODE_MAC_LEN];
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);
    int i = find_node(mac_str);
    if (i < 0) {
        if (s_node_count >= MAX_NODES) {
            i = 0;
            for (int j = 1; j < s_node_count; j++)
                if (s_nodes[j].last_ms < s_nodes[i].last_ms)
                    i = j;
        } else {
            i = s_node_count++;
        }
        strncpy(s_nodes[i].mac, mac_str, NODE_MAC_LEN - 1);
        s_nodes[i].mac[NODE_MAC_LEN - 1] = '\0';
        s_nodes[i].trigger_count = has_trigger_count ? p->trigger_count : 0;
        s_nodes[i].last_motion = 0;
        s_nodes[i].last_motion_uptime_ms = 0;
        s_nodes[i].rssi_dbm = rssi_dbm;
        s_nodes[i].last_env_log_ms = 0;
    }
    uint8_t prev = s_nodes[i].last_motion;
    memcpy(&s_nodes[i].pkt, p, sizeof(sensor_packet_t));
    s_nodes[i].last_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_nodes[i].rssi_dbm = rssi_dbm;
    if (p->last_motion_ms != 0) {
        s_nodes[i].last_motion_uptime_ms = p->last_motion_ms;
    } else if (p->motion == 1) {
        s_nodes[i].last_motion_uptime_ms = p->uptime_ms;
    }
    if (has_trigger_count) {
        s_nodes[i].trigger_count = p->trigger_count;
    } else if (p->motion == 1 && prev == 0) {
        s_nodes[i].trigger_count++;
    }
    s_nodes[i].last_motion = p->motion;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    bool env_missing = env_missing_packet(p);
    bool should_log = false;
    if (p->motion == 1) {
        should_log = true;
    } else if (!env_missing) {
        if (s_nodes[i].last_env_log_ms == 0 ||
            (now_ms - s_nodes[i].last_env_log_ms) >= 300000) {
            should_log = true;
        }
    }
    if (should_log) {
        sensor_log_append(mac_addr, p);
        if (!env_missing) {
            s_nodes[i].last_env_log_ms = now_ms;
        }
    }
}

static void log_persist(void)
{
    if (s_nvs == 0) return;
    uint8_t buf[8 + sizeof(s_log)];
    uint32_t *p32 = (uint32_t *)buf;
    p32[0] = (uint32_t)s_log_count;
    p32[1] = (uint32_t)s_log_head;
    memcpy(buf + 8, s_log, sizeof(s_log));
    nvs_set_blob(s_nvs, NVS_LOG_KEY, buf, sizeof(buf));
    nvs_commit(s_nvs);
}

void sensor_log_append(const uint8_t *mac_addr, const sensor_packet_t *p)
{
    char mac_str[NODE_MAC_LEN];
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);
    sensor_log_entry_t *e = &s_log[s_log_head];
    strncpy(e->mac, mac_str, NODE_MAC_LEN - 1);
    e->mac[NODE_MAC_LEN - 1] = '\0';
    int64_t epoch_ms = time_sync_get_epoch_ms();
    if (epoch_ms >= 0) {
        e->ts_ms = epoch_ms;
    } else {
        e->ts_ms = (int64_t)(esp_timer_get_time() / 1000);
    }
    memcpy(&e->pkt, p, sizeof(sensor_packet_t));
    s_log_head = (s_log_head + 1) % SENSOR_LOG_MAX;
    if (s_log_count < SENSOR_LOG_MAX) s_log_count++;
    log_persist();
}

int sensor_log_count(void) { return s_log_count; }

const sensor_log_entry_t *sensor_log_get(int i)
{
    if (i < 0 || i >= s_log_count) return NULL;
    int idx = (s_log_head - s_log_count + i + SENSOR_LOG_MAX) % SENSOR_LOG_MAX;
    return &s_log[idx];
}

void sensor_log_clear(void)
{
    s_log_count = 0;
    s_log_head = 0;
    if (s_nvs) {
        nvs_erase_key(s_nvs, NVS_LOG_KEY);
        nvs_commit(s_nvs);
    }
}

static void log_load(void)
{
    if (s_nvs == 0) return;
    uint8_t buf[8 + sizeof(s_log)];
    size_t len = sizeof(buf);
    if (nvs_get_blob(s_nvs, NVS_LOG_KEY, buf, &len) != ESP_OK) return;
    if (len < 8) return;
    uint32_t *p32 = (uint32_t *)buf;
    s_log_count = (int)p32[0];
    s_log_head = (int)p32[1];
    if (s_log_count > SENSOR_LOG_MAX) s_log_count = SENSOR_LOG_MAX;
    if (s_log_head >= SENSOR_LOG_MAX) s_log_head = 0;
    if (len >= 8 + sizeof(s_log)) memcpy(s_log, buf + 8, sizeof(s_log));
}

static void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (!info || !data || len < 3) return;
    if (data[0] != SENSOR_PACKET_MAGIC) return;
    sensor_packet_t pkt = {0};
    bool has_trigger_count = false;
    if (data[1] == SENSOR_PACKET_VERSION && len == (int)SENSOR_PACKET_SIZE) {
        memcpy(&pkt, data, sizeof(pkt));
        has_trigger_count = true;
    } else if (data[1] == 3 && len == (int)sizeof(sensor_packet_v3_t)) {
        const sensor_packet_v3_t *p3 = (const sensor_packet_v3_t *)data;
        pkt.magic = p3->magic;
        pkt.version = p3->version;
        pkt.motion = p3->motion;
        pkt.temperature = p3->temperature;
        pkt.humidity = p3->humidity;
        pkt.pressure = p3->pressure;
        pkt.gas = p3->gas;
        pkt.last_motion_ms = p3->last_motion_ms;
        pkt.ble_seen_count = p3->ble_seen_count;
        pkt.ble_last_rssi_dbm = p3->ble_last_rssi_dbm;
        memcpy(pkt.ble_last_addr, p3->ble_last_addr, sizeof(pkt.ble_last_addr));
        pkt.uptime_ms = p3->uptime_ms;
        pkt.trigger_count = 0;
    } else if (data[1] == 1 && len == (int)sizeof(sensor_packet_v1_t)) {
        const sensor_packet_v1_t *p1 = (const sensor_packet_v1_t *)data;
        pkt.magic = p1->magic;
        pkt.version = p1->version;
        pkt.motion = p1->motion;
        pkt.temperature = p1->temperature;
        pkt.humidity = p1->humidity;
        pkt.pressure = p1->pressure;
        pkt.gas = p1->gas;
        pkt.uptime_ms = p1->uptime_ms;
        pkt.last_motion_ms = 0;
        pkt.trigger_count = 0;
        pkt.ble_seen_count = 0;
        pkt.ble_last_rssi_dbm = 0;
        memset(pkt.ble_last_addr, 0, sizeof(pkt.ble_last_addr));
    } else {
        return;
    }
    int8_t rssi = -127;
    if (info->rx_ctrl) {
        rssi = info->rx_ctrl->rssi;
    }
    store_node(info->src_addr, &pkt, rssi, has_trigger_count);
}

void esp_now_rcv_init(void)
{
    memset(s_nodes, 0, sizeof(s_nodes));
    s_node_count = 0;
    memset(s_log, 0, sizeof(s_log));
    s_log_count = 0;
    s_log_head = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &s_nvs) != ESP_OK)
        s_nvs = 0;
    log_load();

    esp_err_t ret = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "set_channel(%d) failed: %s (AP should already be on ch6)",
                 ESPNOW_CHANNEL, esp_err_to_name(ret));
    } else {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ret = esp_now_init();
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_ESPNOW_EXIST) {
            esp_now_deinit();
            ret = esp_now_init();
        }
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = esp_now_register_recv_cb(esp_now_recv_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_register_recv_cb failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "ESP-NOW gateway ready on 2.4 GHz channel %d", ESPNOW_CHANNEL);
}

int esp_now_rcv_node_count(void) { return s_node_count; }

const node_entry_t *esp_now_rcv_get_node(int i)
{
    if (i < 0 || i >= s_node_count) return NULL;
    return &s_nodes[i];
}

static bool mac_string_to_bytes(const char *mac_str, uint8_t *mac_out)
{
    unsigned int a[6];
    if (sscanf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X",
               &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]) != 6)
        return false;
    for (int i = 0; i < 6; i++)
        mac_out[i] = (uint8_t)a[i];
    return true;
}

bool esp_now_rcv_send_reset(const char *mac)
{
    uint8_t peer_mac[6];
    if (!mac_string_to_bytes(mac, peer_mac))
        return false;
    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, peer_mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_AP;
    peer.encrypt = false;
    esp_err_t err = esp_now_add_peer(&peer);
    if (err == ESP_ERR_ESPNOW_EXIST) {
        err = esp_now_mod_peer(&peer);
    }
    if (err != ESP_OK)
        return false;
    cmd_packet_t cmd = { .magic = CMD_PACKET_MAGIC, .cmd_type = CMD_TYPE_RESET };
    err = esp_now_send(peer_mac, (const uint8_t *)&cmd, CMD_PACKET_SIZE);
    return (err == ESP_OK);
}

static void labels_get(char *buf, size_t buf_size)
{
    if (s_nvs == 0 || buf_size == 0) { buf[0] = '\0'; return; }
    size_t len = buf_size;
    nvs_get_str(s_nvs, NVS_LABELS_KEY, buf, &len);
    if (len >= buf_size) buf[0] = '\0';
}

static void labels_set(const char *buf)
{
    if (s_nvs == 0) return;
    nvs_set_str(s_nvs, NVS_LABELS_KEY, buf);
    nvs_commit(s_nvs);
}

const char *esp_now_rcv_get_label(const char *mac)
{
    static char ret[64];
    char all[NVS_LABELS_MAX];
    labels_get(all, sizeof(all));
    size_t mac_len = strlen(mac);
    char *p = strstr(all, mac);
    if (!p || (p != all && p[-1] != '\n')) return "";
    if (p[mac_len] != '=') return "";
    p += mac_len + 1;
    char *end = strchr(p, '\n');
    size_t n = end ? (size_t)(end - p) : strlen(p);
    if (n >= sizeof(ret)) n = sizeof(ret) - 1;
    memcpy(ret, p, n);
    ret[n] = '\0';
    return ret;
}

void esp_now_rcv_set_label(const char *mac, const char *label)
{
    char all[NVS_LABELS_MAX];
    labels_get(all, sizeof(all));
    char line[128];
    snprintf(line, sizeof(line), "%s=%s\n", mac, label ? label : "");
    char *idx = strstr(all, mac);
    if (idx && (idx == all || idx[-1] == '\n') && idx[strlen(mac)] == '=') {
        char *line_end = strchr(idx, '\n');
        size_t rest = line_end ? strlen(line_end + 1) + 1 : 0;
        memmove(idx, line_end ? line_end + 1 : idx + strlen(idx), rest);
        all[strlen(all)] = '\0';
    }
    size_t cur = strlen(all);
    size_t add = strlen(line);
    if (cur + add < sizeof(all)) {
        strcat(all, line);
        labels_set(all);
    }
}
