#include <string.h>
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "sdkconfig.h"
#include "settings.h"
#include "web_config.h"
#include "lwip/inet.h"
#include "lwip/def.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/tcpip.h"
#include "lwip/err.h"
#include "mmipal.h"
#include "dns_forwarder.h"

/* NAPT is required so 192.168.4.x clients can reach HaLow (e.g. 10.41.0.1). */
#if !defined(CONFIG_LWIP_IPV4_NAPT) || !CONFIG_LWIP_IPV4_NAPT
#error "NAPT required for bridge. Delete sdkconfig, then run: idf.py fullclean && idf.py build"
#endif

static const char *TAG = "NAT_BRIDGE";

static esp_netif_t *s_ap_netif = NULL;
static esp_netif_t *s_sta_netif = NULL;
static TimerHandle_t s_route_fix_timer = NULL;
static bool s_halow_backhaul = true;
static bool s_wifi_backhaul_enabled = false;

/* 10.41.0.0/24 in network byte order: subnet check for HaLow netif */
#define HALOW_SUBNET_NBO   (PP_HTONL(LWIP_MAKEU32(10, 41, 0, 0)))
#define HALOW_SUBNET_MASK  (PP_HTONL(LWIP_MAKEU32(255, 255, 255, 0)))
#define HALOW_GW_NBO       (PP_HTONL(LWIP_MAKEU32(10, 41, 0, 1)))
/* 192.168.4.1 in network byte order (AP netif) */
#define AP_IP_NBO          (PP_HTONL(LWIP_MAKEU32(192, 168, 4, 1)))

/** Runs on LwIP tcpip thread: set default netif to the one that is NOT the AP (backhaul). */
static void set_backhaul_default_netif_cb(void *arg)
{
    (void)arg;
    struct netif *netif;
    int count = 0;
    for (netif = netif_list; netif != NULL; netif = netif->next) {
        if (!netif_is_up(netif)) {
            continue;
        }
#if LWIP_IPV4
        const ip4_addr_t *ip4 = netif_ip4_addr(netif);
        if (ip4 == NULL) {
            continue;
        }
        count++;
        /* Skip the AP netif (192.168.4.1); set default to the other one (backhaul) */
        if (ip4->addr == AP_IP_NBO) {
            continue;
        }
        netif_set_default(netif);
        ESP_LOGI(TAG, "Default netif set to non-AP (backhaul) so NAPT and DNS use it for internet.");
        return;
#endif
    }
    ESP_LOGW(TAG, "No non-AP netif found (netifs seen: %d); default unchanged, internet may not work.", count);
}

/** Runs on LwIP tcpip thread: set default gateway on HaLow netif if missing (so NAPT can reach internet). */
static void ensure_halow_default_route_cb(void *arg)
{
    (void)arg;
    if (!s_halow_backhaul) {
        return;
    }
    struct netif *netif;
    for (netif = netif_list; netif != NULL; netif = netif->next) {
        if (!netif_is_up(netif)) {
            continue;
        }
#if LWIP_IPV4
        const ip4_addr_t *ip4 = netif_ip4_addr(netif);
        if (ip4 == NULL) {
            continue;
        }
        /* HaLow netif is 10.41.0.x (not 192.168.4.x) */
        if ((ip4->addr & HALOW_SUBNET_MASK) != HALOW_SUBNET_NBO) {
            continue;
        }
        if (ip4_addr_get_u32(netif_ip4_gw(netif)) != 0) {
            /* Gateway already set (e.g. from DHCP) */
            continue;
        }
        ip4_addr_t gw;
        ip4_addr_set_u32(&gw, HALOW_GW_NBO);
        netif_set_addr(netif, netif_ip4_addr(netif), netif_ip4_netmask(netif), &gw);
        ESP_LOGI(TAG, "HaLow netif default gateway set to 10.41.0.1 (was missing).");
        break;
#endif
    }
}

static void delayed_route_fix_timer_cb(TimerHandle_t t)
{
    (void)t;
    /* Retry default netif (in case HaLow had no IP at AP start) then ensure gateway */
    tcpip_callback(set_backhaul_default_netif_cb, NULL);
    tcpip_callback(ensure_halow_default_route_cb, NULL);
}

void nat_router_refresh_halow_default_route(void)
{
    if (!s_halow_backhaul) {
        return;
    }
    /* Re-run default netif and gateway so NAPT/DNS use HaLow again after reconnect. */
    tcpip_callback(set_backhaul_default_netif_cb, NULL);
    tcpip_callback(ensure_halow_default_route_cb, NULL);
}

static void wifi_ap_start_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data)
{
    if (event_id != WIFI_EVENT_AP_START || s_ap_netif == NULL) {
        return;
    }
    /* 2.4 GHz TX power: API uses 0.25 dBm units; range [8, 84] = 2–21 dBm */
    bridge_settings_t st;
    settings_load(&st);
    int8_t dbm = st.ap_tx_power_dbm;
    if (dbm < 2) dbm = 2;
    if (dbm > 20) dbm = 20;
    int8_t quarter_dbm = (int8_t)(dbm * 4);
    esp_err_t ret = esp_wifi_set_max_tx_power(quarter_dbm);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "2.4 GHz TX power set to %d dBm", (int)dbm);
    } else {
        ESP_LOGW(TAG, "set_max_tx_power failed: %s", esp_err_to_name(ret));
    }

    ret = esp_netif_napt_enable(s_ap_netif);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NAPT enabled on AP (traffic to backhaul uses the uplink IP as source).");
    } else if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "NAPT not supported: enable CONFIG_LWIP_IPV4_NAPT in menuconfig, then fullclean and rebuild.");
    } else {
        ESP_LOGW(TAG, "NAPT enable failed: %s", esp_err_to_name(ret));
    }

    /* Ensure LwIP default netif is backhaul so NAPT output and DNS forwarder go out uplink, not AP */
    tcpip_callback(set_backhaul_default_netif_cb, NULL);

    /* Clients get 192.168.4.1 as DNS (default); forwarder handles it so DNS does not go through NAPT */
    dns_forwarder_start("8.8.8.8");

    if (s_halow_backhaul) {
        /* Log HaLow default route; if gateway is missing, set it so forwarded traffic can reach internet */
        struct mmipal_ip_config ip_cfg;
        if (mmipal_get_ip_config(&ip_cfg) == MMIPAL_SUCCESS) {
            ESP_LOGI(TAG, "HaLow IP %s gateway %s (default route for NAPT).", ip_cfg.ip_addr, ip_cfg.gateway_addr);
            if (strcmp(ip_cfg.gateway_addr, "0.0.0.0") == 0 || !ip_cfg.gateway_addr[0]) {
                if (tcpip_callback(ensure_halow_default_route_cb, NULL) != ERR_OK) {
                    ESP_LOGW(TAG, "Could not schedule default-route fix.");
                }
            }
        }
        /* Retry default netif + gateway after 3s (HaLow DHCP may complete after AP start) */
        if (s_route_fix_timer != NULL) {
            xTimerStart(s_route_fix_timer, 0);
        }
    }
}

static void wifi_sta_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (s_wifi_backhaul_enabled) {
            esp_wifi_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_backhaul_enabled) {
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        tcpip_callback(set_backhaul_default_netif_cb, NULL);
    }
}

void start_2ghz_ap(void) {
    ESP_LOGI(TAG, "Starting 2.4GHz AP (ESP-IDF v5.1.1 Manual Radio Mode)...");
    s_halow_backhaul = true;
    s_sta_netif = NULL;
    s_wifi_backhaul_enabled = false;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_err_t event_ret = esp_event_loop_create_default();
    if (event_ret != ESP_OK && event_ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(event_ret);
    }

    /* esp_netif IPC semaphores are required by esp_netif_create_default_wifi_ap().
     * LwIP was already inited by mmipal; this only creates api_lock_sem/api_sync_sem
     * (and skips tcpip_init if tcpip thread already exists). */
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Create default AP netif (192.168.4.1) and start DHCP server so clients get an IP */
    s_ap_netif = esp_netif_create_default_wifi_ap();
    if (s_ap_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create AP netif");
        return;
    }
    /* Enable NAPT when AP starts (so netif is up); required for 192.168.4.x -> HaLow gateway */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_AP_START,
                                                        &wifi_ap_start_handler, NULL, NULL));

    s_route_fix_timer = xTimerCreate("route_fix", pdMS_TO_TICKS(3000), pdFALSE, NULL, delayed_route_fix_timer_cb);
    if (s_route_fix_timer == NULL) {
        ESP_LOGW(TAG, "Could not create route-fix timer.");
    }

    bridge_settings_t s;
    settings_load(&s);
    if (!s.ap_ssid[0]) {
        strncpy(s.ap_ssid, "XIAO_S3_HALOW", SETTINGS_MAX_SSID - 1);
        s.ap_ssid[SETTINGS_MAX_SSID - 1] = '\0';
    }
    if (!s.ap_pass[0]) {
        strncpy(s.ap_pass, "letmein111", SETTINGS_MAX_PASS - 1);
        s.ap_pass[SETTINGS_MAX_PASS - 1] = '\0';
    }

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    {
        esp_err_t bw_ret = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40);
        if (bw_ret != ESP_OK) {
            ESP_LOGW(TAG, "AP HT40 bandwidth set failed: %s; falling back to HT20", esp_err_to_name(bw_ret));
            bw_ret = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
            if (bw_ret != ESP_OK) {
                ESP_LOGW(TAG, "AP HT20 bandwidth set failed: %s", esp_err_to_name(bw_ret));
            }
        } else {
            ESP_LOGI(TAG, "AP bandwidth set to HT40");
        }
    }

    wifi_config_t ap_config = {
        .ap = {
            .channel = 6,
            .max_connection = 8,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)ap_config.ap.ssid, s.ap_ssid, sizeof(ap_config.ap.ssid) - 1);
    ap_config.ap.ssid_len = strlen((char *)ap_config.ap.ssid);
    strncpy((char *)ap_config.ap.password, s.ap_pass, sizeof(ap_config.ap.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* NAPT is enabled in wifi_ap_start_handler when WIFI_EVENT_AP_START fires */

    ESP_LOGI(TAG, "2.4GHz Wi-Fi Radio '%s' is now active.", s.ap_ssid);

    (void)start_web_config_server();
}

void start_2ghz_apsta_backhaul(void)
{
    ESP_LOGI(TAG, "Starting 2.4GHz AP + STA backhaul (HaLow disabled)...");
    s_halow_backhaul = false;
    s_wifi_backhaul_enabled = false;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_err_t event_ret = esp_event_loop_create_default();
    if (event_ret != ESP_OK && event_ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(event_ret);
    }

    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
        return;
    }

    s_ap_netif = esp_netif_create_default_wifi_ap();
    if (s_ap_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create AP netif");
        return;
    }
    s_sta_netif = esp_netif_create_default_wifi_sta();
    if (s_sta_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create STA netif");
        return;
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_AP_START,
                                                        &wifi_ap_start_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_START,
                                                        &wifi_sta_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                                                        &wifi_sta_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_sta_event_handler, NULL, NULL));

    s_route_fix_timer = xTimerCreate("route_fix", pdMS_TO_TICKS(3000), pdFALSE, NULL, delayed_route_fix_timer_cb);
    if (s_route_fix_timer == NULL) {
        ESP_LOGW(TAG, "Could not create route-fix timer.");
    }

    bridge_settings_t s;
    settings_load(&s);
    if (!s.ap_ssid[0]) {
        strncpy(s.ap_ssid, "XIAO_S3_HALOW", SETTINGS_MAX_SSID - 1);
        s.ap_ssid[SETTINGS_MAX_SSID - 1] = '\0';
    }
    if (!s.ap_pass[0]) {
        strncpy(s.ap_pass, "letmein111", SETTINGS_MAX_PASS - 1);
        s.ap_pass[SETTINGS_MAX_PASS - 1] = '\0';
    }

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    {
        esp_err_t bw_ret = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40);
        if (bw_ret != ESP_OK) {
            ESP_LOGW(TAG, "AP HT40 bandwidth set failed: %s; falling back to HT20", esp_err_to_name(bw_ret));
            bw_ret = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
            if (bw_ret != ESP_OK) {
                ESP_LOGW(TAG, "AP HT20 bandwidth set failed: %s", esp_err_to_name(bw_ret));
            }
        } else {
            ESP_LOGI(TAG, "AP bandwidth set to HT40");
        }
    }

    wifi_config_t ap_config = {
        .ap = {
            .channel = 6,
            .max_connection = 8,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)ap_config.ap.ssid, s.ap_ssid, sizeof(ap_config.ap.ssid) - 1);
    ap_config.ap.ssid_len = strlen((char *)ap_config.ap.ssid);
    strncpy((char *)ap_config.ap.password, s.ap_pass, sizeof(ap_config.ap.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    if (s.wifi_backhaul_ssid[0]) {
        wifi_config_t sta_config = { 0 };
        strncpy((char *)sta_config.sta.ssid, s.wifi_backhaul_ssid, sizeof(sta_config.sta.ssid) - 1);
        strncpy((char *)sta_config.sta.password, s.wifi_backhaul_pass, sizeof(sta_config.sta.password) - 1);
        sta_config.sta.threshold.authmode = s.wifi_backhaul_pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
        sta_config.sta.pmf_cfg.capable = true;
        sta_config.sta.pmf_cfg.required = false;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
        s_wifi_backhaul_enabled = true;
    } else {
        ESP_LOGW(TAG, "2.4GHz backhaul SSID is empty; STA will not connect.");
    }

    ESP_ERROR_CHECK(esp_wifi_start());
    if (s_wifi_backhaul_enabled) {
        ESP_ERROR_CHECK(esp_wifi_connect());
    }

    ESP_LOGI(TAG, "2.4GHz Wi-Fi AP '%s' is now active (backhaul on STA).", s.ap_ssid);
    (void)start_web_config_server();
}