/*
 * Sensor Net Gateway – Apple HomeKit bridge
 * Exposes ESP-NOW sensor nodes as HomeKit Temperature, Humidity, and Motion accessories.
 */
#include "sensor_homekit.h"
#include "esp_now_rcv.h"
#include "packet.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#if CONFIG_SENSOR_NET_HOMEKIT

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>
#include <string.h>
#include <stdio.h>

static const char *TAG = "sensor_homekit";

#define BRIDGE_TASK_STACK 4096
#define BRIDGE_TASK_PRIO  (tskIDLE_PRIORITY + 2)
#define UPDATE_INTERVAL_MS 10000

/* Per-node service handles for value updates (index = order at bridge start) */
typedef struct {
    hap_serv_t *temp_serv;
    hap_serv_t *hum_serv;
    hap_serv_t *motion_serv;
} node_servs_t;

static node_servs_t s_node_servs[MAX_NODES];
static char s_node_names[MAX_NODES][64];
static int s_node_count;
static TimerHandle_t s_update_timer;

static int bridge_identify(hap_acc_t *ha)
{
    (void)ha;
    ESP_LOGI(TAG, "Bridge identified");
    return HAP_SUCCESS;
}

static int accessory_identify(hap_acc_t *ha)
{
    hap_serv_t *hs = hap_acc_get_serv_by_uuid(ha, HAP_SERV_UUID_ACCESSORY_INFORMATION);
    if (hs) {
        hap_char_t *hc = hap_serv_get_char_by_uuid(hs, HAP_CHAR_UUID_NAME);
        if (hc) {
            const hap_val_t *val = hap_char_get_val(hc);
            if (val && val->s)
                ESP_LOGI(TAG, "Accessory %s identified", val->s);
        }
    }
    return HAP_SUCCESS;
}

static void update_sensor_values(TimerHandle_t t)
{
    (void)t;
    for (int i = 0; i < s_node_count; i++) {
        const node_entry_t *e = esp_now_rcv_get_node(i);
        if (!e) continue;

        const sensor_packet_t *p = &e->pkt;
        hap_val_t val;
        hap_char_t *hc;

        if (s_node_servs[i].temp_serv) {
            val.f = p->temperature;
            hc = hap_serv_get_char_by_uuid(s_node_servs[i].temp_serv, HAP_CHAR_UUID_CURRENT_TEMPERATURE);
            if (hc) hap_char_update_val(hc, &val);
        }
        if (s_node_servs[i].hum_serv) {
            val.f = p->humidity;
            hc = hap_serv_get_char_by_uuid(s_node_servs[i].hum_serv, HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY);
            if (hc) hap_char_update_val(hc, &val);
        }
        if (s_node_servs[i].motion_serv) {
            val.b = (p->motion != 0) || (p->mmwave_state != 0);
            hc = hap_serv_get_char_by_uuid(s_node_servs[i].motion_serv, HAP_CHAR_UUID_MOTION_DETECTED);
            if (hc) hap_char_update_val(hc, &val);
        }
    }
}

static void bridge_task(void *arg)
{
    (void)arg;

    hap_init(HAP_TRANSPORT_WIFI);

    hap_acc_cfg_t bridge_cfg = {
        .name = "Sensor Gateway",
        .manufacturer = "Morse Micro",
        .model = "SensorNet",
        .serial_num = "1",
        .fw_rev = "1.0",
        .hw_rev = NULL,
        .pv = "1.1.0",
        .cid = HAP_CID_BRIDGE,
        .identify_routine = bridge_identify,
    };
    hap_acc_t *bridge_acc = hap_acc_create(&bridge_cfg);
    if (!bridge_acc) {
        ESP_LOGE(TAG, "Failed to create bridge accessory");
        vTaskDelete(NULL);
        return;
    }
    hap_acc_add_wifi_transport_service(bridge_acc, 0);
    hap_add_accessory(bridge_acc);

    s_node_count = esp_now_rcv_node_count();
    if (s_node_count > MAX_NODES)
        s_node_count = MAX_NODES;

    memset(s_node_servs, 0, sizeof(s_node_servs));

    for (int i = 0; i < s_node_count; i++) {
        const node_entry_t *e = esp_now_rcv_get_node(i);
        if (!e) continue;

        const sensor_packet_t *p = &e->pkt;
        const char *label = esp_now_rcv_get_label(e->mac);
        char *name = s_node_names[i];
        if (label && label[0] != '\0')
            snprintf(name, 64, "%s", label);
        else
            snprintf(name, 64, "Sensor %s", e->mac);

        hap_acc_cfg_t acc_cfg = {
            .name = name,
            .manufacturer = "Sensor Net",
            .model = "Node",
            .serial_num = e->mac,
            .fw_rev = "1.0",
            .hw_rev = NULL,
            .pv = "1.1.0",
            .cid = HAP_CID_SENSOR,
            .identify_routine = accessory_identify,
        };
        hap_acc_t *acc = hap_acc_create(&acc_cfg);
        if (!acc) continue;

        /* Temperature sensor (0.0 until node reports); update timer will refresh */
        {
            hap_serv_t *ts = hap_serv_temperature_sensor_create(p->temperature);
            if (ts) {
                hap_serv_add_char(ts, hap_char_name_create(name));
                hap_acc_add_serv(acc, ts);
                s_node_servs[i].temp_serv = ts;
            }
        }
        /* Humidity sensor */
        {
            hap_serv_t *hs = hap_serv_humidity_sensor_create(p->humidity);
            if (hs) {
                hap_serv_add_char(hs, hap_char_name_create(name));
                hap_acc_add_serv(acc, hs);
                s_node_servs[i].hum_serv = hs;
            }
        }
        /* Motion sensor */
        {
            bool motion = (p->motion != 0) || (p->mmwave_state != 0);
            hap_serv_t *ms = hap_serv_motion_sensor_create(motion);
            if (ms) {
                hap_serv_add_char(ms, hap_char_name_create(name));
                hap_acc_add_serv(acc, ms);
                s_node_servs[i].motion_serv = ms;
            }
        }

        int aid = hap_get_unique_aid(e->mac);
        hap_add_bridged_accessory(acc, aid);
    }

    hap_set_setup_code(CONFIG_SENSOR_NET_HOMEKIT_SETUP_CODE);
    hap_set_setup_id(CONFIG_SENSOR_NET_HOMEKIT_SETUP_ID);

    hap_start();

    s_update_timer = xTimerCreate("hk_update", pdMS_TO_TICKS(UPDATE_INTERVAL_MS),
                                  pdTRUE, NULL, update_sensor_values);
    if (s_update_timer)
        xTimerStart(s_update_timer, 0);

    ESP_LOGI(TAG, "HomeKit bridge started; %d accessories, setup %s",
             s_node_count, CONFIG_SENSOR_NET_HOMEKIT_SETUP_CODE);

    vTaskDelete(NULL);
}

void sensor_homekit_start(void)
{
    xTaskCreate(bridge_task, "hap_bridge", BRIDGE_TASK_STACK, NULL, BRIDGE_TASK_PRIO, NULL);
}

#else

void sensor_homekit_start(void)
{
    (void)0;
}

#endif /* CONFIG_SENSOR_NET_HOMEKIT */
