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

#define NVS_NAMESPACE "gateway"
#define ESPNOW_CHANNEL_DEFAULT 6
#define NVS_LABELS_KEY "slabels"
#define NVS_LABELS_MAX 512
#define NVS_LOCATIONS_KEY "sloc"
#define NVS_LOCATIONS_MAX 256
#define NVS_LOG_KEY "slog"
#define NVS_UI_SKIN_KEY "ui_skin"
#define NVS_UI_FONT_KEY "ui_font"

static const char *TAG = "esp_now_rcv";

static node_entry_t s_nodes[MAX_NODES];
static int s_node_count = 0;
static nvs_handle_t s_nvs = 0;
/** Current channel (from WiFi AP/STA); used for ESP-NOW and beacon. */
static int s_espnow_channel = 0;
static const uint8_t s_broadcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static sensor_log_entry_t s_log[SENSOR_LOG_MAX];
static int s_log_count = 0;
static int s_log_head = 0;

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
        s_nodes[i].last_motion_seen_ms = 0;
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
    if (p->motion == 1) {
        s_nodes[i].last_motion_seen_ms = (uint32_t)(esp_timer_get_time() / 1000);
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

/* Old v5 struct (single moisture) for backward-compat parsing. */
typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  motion;
    float    temperature;
    float    humidity;
    float    pressure;
    float    gas;
    float    moisture;
    uint32_t last_motion_ms;
    uint32_t trigger_count;
    uint16_t ble_seen_count;
    int8_t   ble_last_rssi_dbm;
    uint8_t  ble_last_addr[6];
    uint32_t uptime_ms;
} sensor_packet_v5_t;

/* Old v6 struct (4 moisture, no plant labels). */
typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  motion;
    float    temperature;
    float    humidity;
    float    pressure;
    float    gas;
    float    moisture[SENSOR_MOISTURE_CHANNELS];
    uint32_t last_motion_ms;
    uint32_t trigger_count;
    uint16_t ble_seen_count;
    int8_t   ble_last_rssi_dbm;
    uint8_t  ble_last_addr[6];
    uint32_t uptime_ms;
} sensor_packet_v6_t;

/* sensor_unit_s3 (XIAO + Seeed mmWave): v6 with label, stream_host, mmwave, is_outdoor. */
#define SENSOR_LABEL_MAX 32
#define SENSOR_STREAM_HOST_MAX 16
typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  motion;
    float    temperature;
    float    humidity;
    float    pressure;
    float    gas;
    uint32_t last_motion_ms;
    uint32_t trigger_count;
    uint16_t ble_seen_count;
    int8_t   ble_last_rssi_dbm;
    uint8_t  ble_last_addr[6];
    uint32_t uptime_ms;
    char     label[SENSOR_LABEL_MAX];
    char     stream_host[SENSOR_STREAM_HOST_MAX];
    uint8_t  mmwave_state;
    uint16_t mmwave_moving_cm;
    uint16_t mmwave_stationary_cm;
    uint8_t  mmwave_moving_energy;
    uint8_t  mmwave_stationary_energy;
    uint16_t mmwave_detection_dist_cm;
    uint8_t  is_outdoor;
} sensor_packet_v6_s3_t;

/* S3 extended: 98-byte base + moisture[4] + plant_label[4][16] = 178 bytes */
typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  motion;
    float    temperature;
    float    humidity;
    float    pressure;
    float    gas;
    uint32_t last_motion_ms;
    uint32_t trigger_count;
    uint16_t ble_seen_count;
    int8_t   ble_last_rssi_dbm;
    uint8_t  ble_last_addr[6];
    uint32_t uptime_ms;
    char     label[SENSOR_LABEL_MAX];
    char     stream_host[SENSOR_STREAM_HOST_MAX];
    uint8_t  mmwave_state;
    uint16_t mmwave_moving_cm;
    uint16_t mmwave_stationary_cm;
    uint8_t  mmwave_moving_energy;
    uint8_t  mmwave_stationary_energy;
    uint16_t mmwave_detection_dist_cm;
    float    moisture[SENSOR_MOISTURE_CHANNELS];
    char     plant_label[SENSOR_MOISTURE_CHANNELS][SENSOR_PLANT_LABEL_LEN];
    uint8_t  is_outdoor;
} sensor_packet_v6_s3_ext_t;

/* S3 v7: v6 ext + temperature_water + tds_ppm = 186 bytes */
typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  motion;
    float    temperature;
    float    humidity;
    float    pressure;
    float    gas;
    uint32_t last_motion_ms;
    uint32_t trigger_count;
    uint16_t ble_seen_count;
    int8_t   ble_last_rssi_dbm;
    uint8_t  ble_last_addr[6];
    uint32_t uptime_ms;
    char     label[SENSOR_LABEL_MAX];
    char     stream_host[SENSOR_STREAM_HOST_MAX];
    uint8_t  mmwave_state;
    uint16_t mmwave_moving_cm;
    uint16_t mmwave_stationary_cm;
    uint8_t  mmwave_moving_energy;
    uint8_t  mmwave_stationary_energy;
    uint16_t mmwave_detection_dist_cm;
    float    moisture[SENSOR_MOISTURE_CHANNELS];
    char     plant_label[SENSOR_MOISTURE_CHANNELS][SENSOR_PLANT_LABEL_LEN];
    uint8_t  is_outdoor;
    float    temperature_water;
    float    tds_ppm;
} sensor_packet_v7_s3_t;

static void init_pkt_defaults(sensor_packet_t *pkt)
{
    memset(pkt, 0, sizeof(*pkt));
    pkt->temperature_water = SENSOR_TEMP_WATER_INVALID;
    pkt->tds_ppm = SENSOR_TDS_INVALID;
    for (int i = 0; i < SENSOR_MOISTURE_CHANNELS; i++) {
        pkt->moisture[i] = -1.0f;
        pkt->plant_label[i][0] = '\0';
    }
}

static void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (!info || !data || len < 3) return;
    if (data[0] != SENSOR_PACKET_MAGIC) return;

    sensor_packet_t pkt;
    init_pkt_defaults(&pkt);

    if (data[1] == SENSOR_PACKET_VERSION && len == (int)SENSOR_PACKET_SIZE) {
        /* v9 packet (C6: temperature + temperature_water + tds_ppm + full struct) */
        memcpy(&pkt, data, (size_t)len);
    } else if (data[1] == SENSOR_PACKET_VERSION_V8 && len == (int)SENSOR_PACKET_V8_SIZE) {
        /* v8 packet (no tds_ppm) */
        memcpy(&pkt, data, (size_t)len);
        pkt.tds_ppm = SENSOR_TDS_INVALID;
    } else if (data[1] == SENSOR_PACKET_VERSION_V7 && len == (int)SENSOR_PACKET_V7_WIRE_SIZE) {
        /* v7 packet (4 moisture + plant labels, no mmwave on wire) */
        memcpy(&pkt, data, SENSOR_PACKET_V7_WIRE_SIZE);
    } else if (data[1] == 7 && len == (int)SENSOR_PACKET_V7_S3_SIZE) {
        /* sensor_unit_s3 v7: 186 bytes with temperature_water + tds_ppm */
        sensor_packet_v7_s3_t s7;
        memset(&s7, 0, sizeof(s7));
        memcpy(&s7, data, (size_t)(len < (int)sizeof(s7) ? len : (int)sizeof(s7)));
        pkt.magic = s7.magic;
        pkt.version = SENSOR_PACKET_VERSION;
        pkt.motion = s7.motion;
        pkt.temperature = s7.temperature;
        pkt.humidity = s7.humidity;
        pkt.pressure = s7.pressure;
        pkt.gas = s7.gas;
        pkt.last_motion_ms = s7.last_motion_ms;
        pkt.trigger_count = s7.trigger_count;
        pkt.ble_seen_count = s7.ble_seen_count;
        pkt.ble_last_rssi_dbm = s7.ble_last_rssi_dbm;
        memcpy(pkt.ble_last_addr, s7.ble_last_addr, 6);
        pkt.uptime_ms = s7.uptime_ms;
        pkt.mmwave_state = s7.mmwave_state;
        pkt.mmwave_moving_cm = s7.mmwave_moving_cm;
        pkt.mmwave_stationary_cm = s7.mmwave_stationary_cm;
        pkt.mmwave_moving_energy = s7.mmwave_moving_energy;
        pkt.mmwave_stationary_energy = s7.mmwave_stationary_energy;
        pkt.mmwave_detection_dist_cm = s7.mmwave_detection_dist_cm;
        for (int i = 0; i < SENSOR_MOISTURE_CHANNELS; i++)
            pkt.moisture[i] = s7.moisture[i];
        for (int i = 0; i < SENSOR_MOISTURE_CHANNELS; i++) {
            strncpy(pkt.plant_label[i], s7.plant_label[i], SENSOR_PLANT_LABEL_LEN - 1);
            pkt.plant_label[i][SENSOR_PLANT_LABEL_LEN - 1] = '\0';
        }
        pkt.temperature_water = s7.temperature_water;
        pkt.tds_ppm = s7.tds_ppm;
        if (s7.label[0] != '\0') {
            char mac_str[NODE_MAC_LEN];
            snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                     info->src_addr[0], info->src_addr[1], info->src_addr[2],
                     info->src_addr[3], info->src_addr[4], info->src_addr[5]);
            const char *cur = esp_now_rcv_get_label(mac_str);
            if (!cur || cur[0] == '\0')
                esp_now_rcv_set_label(mac_str, s7.label);
        }
    } else if (data[1] == 6 && len == (int)SENSOR_PACKET_V6_S3_EXT_SIZE) {
        /* sensor_unit_s3 with moisture (D0/D1) + plant_label: 178 bytes */
        sensor_packet_v6_s3_ext_t s3e;
        memset(&s3e, 0, sizeof(s3e));
        memcpy(&s3e, data, (size_t)(len < (int)sizeof(s3e) ? len : (int)sizeof(s3e)));
        pkt.magic = s3e.magic;
        pkt.version = SENSOR_PACKET_VERSION;
        pkt.motion = s3e.motion;
        pkt.temperature = s3e.temperature;
        pkt.humidity = s3e.humidity;
        pkt.pressure = s3e.pressure;
        pkt.gas = s3e.gas;
        pkt.last_motion_ms = s3e.last_motion_ms;
        pkt.trigger_count = s3e.trigger_count;
        pkt.ble_seen_count = s3e.ble_seen_count;
        pkt.ble_last_rssi_dbm = s3e.ble_last_rssi_dbm;
        memcpy(pkt.ble_last_addr, s3e.ble_last_addr, 6);
        pkt.uptime_ms = s3e.uptime_ms;
        pkt.mmwave_state = s3e.mmwave_state;
        pkt.mmwave_moving_cm = s3e.mmwave_moving_cm;
        pkt.mmwave_stationary_cm = s3e.mmwave_stationary_cm;
        pkt.mmwave_moving_energy = s3e.mmwave_moving_energy;
        pkt.mmwave_stationary_energy = s3e.mmwave_stationary_energy;
        pkt.mmwave_detection_dist_cm = s3e.mmwave_detection_dist_cm;
        for (int i = 0; i < SENSOR_MOISTURE_CHANNELS; i++)
            pkt.moisture[i] = s3e.moisture[i];
        for (int i = 0; i < SENSOR_MOISTURE_CHANNELS; i++) {
            strncpy(pkt.plant_label[i], s3e.plant_label[i], SENSOR_PLANT_LABEL_LEN - 1);
            pkt.plant_label[i][SENSOR_PLANT_LABEL_LEN - 1] = '\0';
        }
        if (s3e.label[0] != '\0') {
            char mac_str[NODE_MAC_LEN];
            snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                     info->src_addr[0], info->src_addr[1], info->src_addr[2],
                     info->src_addr[3], info->src_addr[4], info->src_addr[5]);
            const char *cur = esp_now_rcv_get_label(mac_str);
            if (!cur || cur[0] == '\0')
                esp_now_rcv_set_label(mac_str, s3e.label);
        }
    } else if (data[1] == 6 && (len == (int)SENSOR_PACKET_V6_S3_SIZE || len == (int)SENSOR_PACKET_V6_S3_SIZE_LEGACY)) {
        /* sensor_unit_s3 (XIAO + Seeed mmWave): v6 with label, mmwave, is_outdoor; 98 bytes or legacy 94 (no moisture) */
        sensor_packet_v6_s3_t s3;
        memset(&s3, 0, sizeof(s3));
        memcpy(&s3, data, (size_t)(len < (int)sizeof(s3) ? len : (int)sizeof(s3)));
        pkt.magic = s3.magic;
        pkt.version = SENSOR_PACKET_VERSION;
        pkt.motion = s3.motion;
        pkt.temperature = s3.temperature;
        pkt.humidity = s3.humidity;
        pkt.pressure = s3.pressure;
        pkt.gas = s3.gas;
        pkt.last_motion_ms = s3.last_motion_ms;
        pkt.trigger_count = s3.trigger_count;
        pkt.ble_seen_count = s3.ble_seen_count;
        pkt.ble_last_rssi_dbm = s3.ble_last_rssi_dbm;
        memcpy(pkt.ble_last_addr, s3.ble_last_addr, 6);
        pkt.uptime_ms = s3.uptime_ms;
        pkt.mmwave_state = s3.mmwave_state;
        pkt.mmwave_moving_cm = s3.mmwave_moving_cm;
        pkt.mmwave_stationary_cm = s3.mmwave_stationary_cm;
        /* 94-byte legacy (e.g. sensor_unit_camera) omits energy + detection_dist on wire; only 98-byte S3 has them */
        if (len == (int)SENSOR_PACKET_V6_S3_SIZE) {
            pkt.mmwave_moving_energy = s3.mmwave_moving_energy;
            pkt.mmwave_stationary_energy = s3.mmwave_stationary_energy;
            pkt.mmwave_detection_dist_cm = s3.mmwave_detection_dist_cm;
        } else {
            pkt.mmwave_moving_energy = 0;
            pkt.mmwave_stationary_energy = 0;
            pkt.mmwave_detection_dist_cm = 0;
        }
        strncpy(pkt.plant_label[0], s3.label, SENSOR_PLANT_LABEL_LEN - 1);
        pkt.plant_label[0][SENSOR_PLANT_LABEL_LEN - 1] = '\0';
        /* Sync sensor label to gateway NVS when gateway has no label for this MAC */
        if (s3.label[0] != '\0') {
            char mac_str[NODE_MAC_LEN];
            snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                     info->src_addr[0], info->src_addr[1], info->src_addr[2],
                     info->src_addr[3], info->src_addr[4], info->src_addr[5]);
            const char *cur = esp_now_rcv_get_label(mac_str);
            if (!cur || cur[0] == '\0')
                esp_now_rcv_set_label(mac_str, s3.label);
        }
    } else if (data[1] == 6 && len == (int)SENSOR_PACKET_V6_SIZE) {
        /* v6 packet (4 moisture, no labels) */
        sensor_packet_v6_t v6;
        memcpy(&v6, data, sizeof(v6));
        pkt.magic = v6.magic;
        pkt.version = SENSOR_PACKET_VERSION;
        pkt.motion = v6.motion;
        pkt.temperature = v6.temperature;
        pkt.humidity = v6.humidity;
        pkt.pressure = v6.pressure;
        pkt.gas = v6.gas;
        for (int i = 0; i < SENSOR_MOISTURE_CHANNELS; i++)
            pkt.moisture[i] = v6.moisture[i];
        pkt.last_motion_ms = v6.last_motion_ms;
        pkt.trigger_count = v6.trigger_count;
        pkt.ble_seen_count = v6.ble_seen_count;
        pkt.ble_last_rssi_dbm = v6.ble_last_rssi_dbm;
        memcpy(pkt.ble_last_addr, v6.ble_last_addr, 6);
        pkt.uptime_ms = v6.uptime_ms;
    } else if (data[1] == 5 && len == (int)SENSOR_PACKET_V5_SIZE) {
        /* Old v5 packet (single moisture) */
        sensor_packet_v5_t v5;
        memcpy(&v5, data, sizeof(v5));
        pkt.magic = v5.magic;
        pkt.version = SENSOR_PACKET_VERSION;
        pkt.motion = v5.motion;
        pkt.temperature = v5.temperature;
        pkt.humidity = v5.humidity;
        pkt.pressure = v5.pressure;
        pkt.gas = v5.gas;
        pkt.moisture[0] = v5.moisture;
        pkt.last_motion_ms = v5.last_motion_ms;
        pkt.trigger_count = v5.trigger_count;
        pkt.ble_seen_count = v5.ble_seen_count;
        pkt.ble_last_rssi_dbm = v5.ble_last_rssi_dbm;
        memcpy(pkt.ble_last_addr, v5.ble_last_addr, 6);
        pkt.uptime_ms = v5.uptime_ms;
    } else {
        return;
    }

    int8_t rssi = -127;
    if (info->rx_ctrl) {
        rssi = info->rx_ctrl->rssi;
    }
    static uint32_t rx_log_count;
    rx_log_count++;
    if (rx_log_count <= 5 || (rx_log_count % 15) == 0) {
        char mac_str[NODE_MAC_LEN], tw_buf[16], tds_buf[16];
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 info->src_addr[0], info->src_addr[1], info->src_addr[2],
                 info->src_addr[3], info->src_addr[4], info->src_addr[5]);
        int tw_ok = (pkt.temperature_water > -500.0f && pkt.temperature_water < 200.0f);
        int tds_ok = (pkt.tds_ppm >= 0.0f);
        if (tw_ok) snprintf(tw_buf, sizeof(tw_buf), "%.1f", (double)pkt.temperature_water);
        else snprintf(tw_buf, sizeof(tw_buf), "-");
        if (tds_ok) snprintf(tds_buf, sizeof(tds_buf), "%.0f", (double)pkt.tds_ppm);
        else snprintf(tds_buf, sizeof(tds_buf), "-");
        ESP_LOGI(TAG, "rx %s: motion=%u T=%.1f T_water=%s H=%.1f P=%.1f gas=%.1f TDS=%s rssi=%d",
                 mac_str, (unsigned)pkt.motion, (double)pkt.temperature,
                 tw_buf, (double)pkt.humidity, (double)pkt.pressure, (double)pkt.gas,
                 tds_buf, (int)rssi);
    }
    store_node(info->src_addr, &pkt, rssi, true);
    /* Beacon so sensors scanning for channel can lock onto this channel */
    if (s_espnow_channel >= 1 && s_espnow_channel <= 14) {
        uint8_t beacon[2] = { GATEWAY_PACKET_MAGIC, (uint8_t)s_espnow_channel };
        esp_now_send(s_broadcast_mac, beacon, sizeof(beacon));
    }
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

    /* Use current WiFi channel (AP follows STA in AP+STA mode); do not force channel. */
    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    esp_err_t ch_ret = esp_wifi_get_channel(&primary, &second);
    if (ch_ret == ESP_OK && primary >= 1 && primary <= 14) {
        s_espnow_channel = (int)primary;
        ESP_LOGI(TAG, "ESP-NOW using current WiFi channel %d", s_espnow_channel);
    } else {
        s_espnow_channel = ESPNOW_CHANNEL_DEFAULT;
        ESP_LOGW(TAG, "esp_wifi_get_channel failed (%s), using channel %d",
                 esp_err_to_name(ch_ret), s_espnow_channel);
    }

    esp_err_t ret = esp_now_init();
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
    /* Add broadcast peer so we can send gateway beacon (for sensor channel scan).
     * Use WIFI_IF_STA when gateway is STA-only (no AP); otherwise AP so sensors see the beacon. */
    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_err_t mode_ret = esp_wifi_get_mode(&mode);
    wifi_interface_t ifidx = WIFI_IF_AP;
    if (mode_ret == ESP_OK && mode == WIFI_MODE_STA) {
        ifidx = WIFI_IF_STA;
        ESP_LOGI(TAG, "ESP-NOW using STA interface (gateway is STA-only)");
    }
    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, s_broadcast_mac, 6);
    peer.channel = (uint8_t)s_espnow_channel;
    peer.ifidx = ifidx;
    peer.encrypt = false;
    esp_err_t add_ret = esp_now_add_peer(&peer);
    if (add_ret != ESP_OK && add_ret != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGW(TAG, "esp_now_add_peer broadcast failed: %s", esp_err_to_name(add_ret));
    }

    ret = esp_now_register_recv_cb(esp_now_recv_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_register_recv_cb failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "ESP-NOW gateway ready on 2.4 GHz channel %d", s_espnow_channel);
}

void esp_now_rcv_start_deferred(void)
{
    /* Re-read channel (e.g. after STA connected) so ESP-NOW matches actual WiFi channel. */
    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    if (esp_wifi_get_channel(&primary, &second) != ESP_OK || primary < 1 || primary > 14) {
        return;
    }
    if ((int)primary == s_espnow_channel) {
        return;
    }
    s_espnow_channel = (int)primary;
    ESP_LOGI(TAG, "ESP-NOW channel updated to %d (WiFi channel)", s_espnow_channel);
    esp_now_del_peer((uint8_t *)s_broadcast_mac);
    wifi_mode_t mode = WIFI_MODE_NULL;
    wifi_interface_t ifidx = WIFI_IF_AP;
    if (esp_wifi_get_mode(&mode) == ESP_OK && mode == WIFI_MODE_STA) {
        ifidx = WIFI_IF_STA;
    }
    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, s_broadcast_mac, 6);
    peer.channel = (uint8_t)s_espnow_channel;
    peer.ifidx = ifidx;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
}

int esp_now_rcv_get_channel(void)
{
    return s_espnow_channel;
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
    peer.channel = (uint8_t)(s_espnow_channel >= 1 && s_espnow_channel <= 14 ? s_espnow_channel : ESPNOW_CHANNEL_DEFAULT);
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

bool esp_now_rcv_send_plant_label(const char *mac, int channel, const char *label)
{
    if (channel < 0 || channel >= SENSOR_MOISTURE_CHANNELS) return false;
    uint8_t peer_mac[6];
    if (!mac_string_to_bytes(mac, peer_mac))
        return false;
    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, peer_mac, 6);
    peer.channel = (uint8_t)(s_espnow_channel >= 1 && s_espnow_channel <= 14 ? s_espnow_channel : ESPNOW_CHANNEL_DEFAULT);
    wifi_mode_t mode = WIFI_MODE_NULL;
    peer.ifidx = WIFI_IF_AP;
    if (esp_wifi_get_mode(&mode) == ESP_OK && mode == WIFI_MODE_STA)
        peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_err_t err = esp_now_add_peer(&peer);
    if (err == ESP_ERR_ESPNOW_EXIST) {
        err = esp_now_mod_peer(&peer);
    }
    if (err != ESP_OK)
        return false;
    cmd_plant_label_packet_t cmd = { 0 };
    cmd.magic = CMD_PACKET_MAGIC;
    cmd.cmd_type = CMD_TYPE_SET_PLANT_LABEL;
    cmd.channel = (uint8_t)channel;
    if (label) {
        strncpy(cmd.label, label, SENSOR_PLANT_LABEL_LEN - 1);
    }
    err = esp_now_send(peer_mac, (const uint8_t *)&cmd, CMD_PLANT_LABEL_PACKET_SIZE);
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

static void locations_get(char *buf, size_t buf_size)
{
    if (s_nvs == 0 || buf_size == 0) { buf[0] = '\0'; return; }
    size_t len = buf_size;
    nvs_get_str(s_nvs, NVS_LOCATIONS_KEY, buf, &len);
    if (len >= buf_size) buf[0] = '\0';
}

static void locations_set(const char *buf)
{
    if (s_nvs == 0) return;
    nvs_set_str(s_nvs, NVS_LOCATIONS_KEY, buf);
    nvs_commit(s_nvs);
}

const char *esp_now_rcv_get_location(const char *mac)
{
    static char ret[16];
    char all[NVS_LOCATIONS_MAX];
    locations_get(all, sizeof(all));
    size_t mac_len = strlen(mac);
    char *p = strstr(all, mac);
    if (!p || (p != all && p[-1] != '\n')) return "indoor";
    if (p[mac_len] != '=') return "indoor";
    p += mac_len + 1;
    char *end = strchr(p, '\n');
    size_t n = end ? (size_t)(end - p) : strlen(p);
    if (n >= sizeof(ret)) n = sizeof(ret) - 1;
    memcpy(ret, p, n);
    ret[n] = '\0';
    if (strcmp(ret, "outdoor") != 0) return "indoor";
    return ret;
}

void esp_now_rcv_set_location(const char *mac, const char *location)
{
    char all[NVS_LOCATIONS_MAX];
    locations_get(all, sizeof(all));
    char line[64];
    snprintf(line, sizeof(line), "%s=%s\n", mac, (location && strcmp(location, "outdoor") == 0) ? "outdoor" : "indoor");
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
        locations_set(all);
    }
}

void esp_now_rcv_get_ui_skin(char *skin, size_t skin_sz, char *font_size, size_t fs_sz)
{
    if (skin_sz) skin[0] = '\0';
    if (fs_sz) font_size[0] = '\0';
    if (s_nvs == 0) goto defaults;
    {
        size_t len = skin_sz;
        if (nvs_get_str(s_nvs, NVS_UI_SKIN_KEY, skin, &len) != ESP_OK)
            skin[0] = '\0';
    }
    {
        size_t len = fs_sz;
        if (nvs_get_str(s_nvs, NVS_UI_FONT_KEY, font_size, &len) != ESP_OK)
            font_size[0] = '\0';
    }
defaults:
    if (skin[0] == '\0' && skin_sz > 0)
        strncpy(skin, "cyberpunk", skin_sz - 1);
    if (font_size[0] == '\0' && fs_sz > 0)
        strncpy(font_size, "medium", fs_sz - 1);
}

void esp_now_rcv_set_ui_skin(const char *skin, const char *font_size)
{
    if (s_nvs == 0) return;
    if (skin && skin[0])
        nvs_set_str(s_nvs, NVS_UI_SKIN_KEY, skin);
    if (font_size && font_size[0])
        nvs_set_str(s_nvs, NVS_UI_FONT_KEY, font_size);
    nvs_commit(s_nvs);
}
