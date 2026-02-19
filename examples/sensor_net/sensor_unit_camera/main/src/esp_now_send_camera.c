/**
 * ESP-NOW for camera sensor unit (XIAO ESP32-S3-Sense).
 * Same capabilities as sensor_unit: sends sensor_packet_t (zeros + uptime; no BME/motion),
 * handles gateway commands (blink, reset), and acts as mesh hop by relaying other nodes' packets.
 * Gateway blink command uses the HaLow link LED (halow_led_request_blink).
 * Also sends sensor packet over HaLow mesh so gateway receives it when mesh overlay is enabled.
 */
#include "packet.h"
#include "esp_now_send_camera.h"
#include "camera_unit_settings.h"
#include "mm_app_common.h"
#include "halow_mesh.h"
#include "halow_mesh_overlay.h"
#include "mmpkt.h"
#include "mmipal.h"
#include "mmwlan.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include <string.h>

/* HaLow (Morse) netif name in LwIP (mmnetif.c sets name[0]='M', name[1]='M'). */
#define HALOW_NETIF_NAME0  'M'
#define HALOW_NETIF_NAME1  'M'

#define ETH_HDR_LEN 14
#define MESH_BROADCAST_MAC { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }

static const char *TAG = "esp_now_camera";

#define ESPNOW_CHANNEL_DEFAULT CONFIG_ESPNOW_CHANNEL
#define ESPNOW_SCAN_CHANNEL_MIN 1
#define ESPNOW_SCAN_CHANNEL_MAX 13
#define ESPNOW_SCAN_WAIT_MS    80
#define MAX_PEERS_SEEN  32
#define NVS_NAMESPACE   "sensor"
#define NVS_ESPNOW_CHANNEL_KEY "espnow_ch"
#define NVS_LABEL_KEY "label"
#define NVS_IS_OUTDOOR_KEY "outdoor"
#define PEER_STALE_MS   (5 * 60 * 1000)  /* 5 min */
/* Minimum sensor packet length (v4 format without label/stream_host). sensor_unit_c6 sends v4. */
#define SENSOR_PACKET_MIN_LEN  40

static uint8_t s_broadcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t s_self_mac[6] = { 0 };
static bool s_esp_now_ok = false;
/** Current ESP-NOW channel (fixed or from scan). */
static uint8_t s_espnow_channel = ESPNOW_CHANNEL_DEFAULT;
#if CONFIG_SENSOR_ESPNOW_CHANNEL_SCAN
static SemaphoreHandle_t s_scan_ack_sem = NULL;
static volatile bool s_scanning = false;
#endif
static QueueHandle_t s_blink_queue = NULL;

typedef struct {
	uint8_t mac[6];
	uint32_t last_seen_ms;
} peer_seen_t;
static peer_seen_t s_peers_seen[MAX_PEERS_SEEN];
static SemaphoreHandle_t s_peers_mux = NULL;
#define FWD_TO_HALOW_LOG_MS  (15 * 1000)  /* log at most every 15 s */
static uint32_t s_last_fwd_halow_log_ms = 0;

/** Fill stream_host with this unit's HaLow (MM) IP so the gateway dashboard links to the camera, not the gateway. */
static void fill_stream_host(char *out, size_t out_len)
{
	if (!out || out_len == 0) {
		return;
	}
	out[0] = '\0';
	/* Prefer HaLow netif IP from LwIP so we always send this device's IP, never the gateway's. */
	for (struct netif *netif = netif_list; netif != NULL; netif = netif->next) {
		if (!netif_is_up(netif)) continue;
		if (netif->name[0] != HALOW_NETIF_NAME0 || netif->name[1] != HALOW_NETIF_NAME1) continue;
#if LWIP_IPV4
		const ip4_addr_t *ip4 = netif_ip4_addr(netif);
		if (ip4 != NULL && !ip4_addr_isany(ip4)) {
			const char *s = ip4addr_ntoa(ip4);
			if (s && s[0] != '\0') {
				strncpy(out, s, out_len - 1);
				out[out_len - 1] = '\0';
				return;
			}
		}
#endif
		break;
	}
	/* Fallback to mmipal (must be this unit's HaLow IP, not gateway). */
	struct mmipal_ip_config ip_cfg;
	if (mmipal_get_ip_config(&ip_cfg) == MMIPAL_SUCCESS &&
	    ip_cfg.ip_addr[0] != '\0' && strcmp(ip_cfg.ip_addr, "0.0.0.0") != 0) {
		strncpy(out, ip_cfg.ip_addr, out_len - 1);
		out[out_len - 1] = '\0';
		return;
	}
	/* Fallback: use any netif with an IP (e.g. WiFi STA/AP) so the camera shows on Video when HaLow has no IP yet. */
#if LWIP_IPV4
	for (struct netif *netif = netif_list; netif != NULL; netif = netif->next) {
		if (!netif_is_up(netif)) continue;
		const ip4_addr_t *ip4 = netif_ip4_addr(netif);
		if (ip4 == NULL || ip4_addr_isany(ip4)) continue;
		const char *s = ip4addr_ntoa(ip4);
		if (s && s[0] != '\0') {
			strncpy(out, s, out_len - 1);
			out[out_len - 1] = '\0';
			return;
		}
	}
#endif
}

static void send_halow_mesh_payload(const void *payload, size_t payload_len);

#if CONFIG_SENSOR_LED_GPIO >= 0
static void blink_task(void *arg)
{
	(void)arg;
	int level = 0;
	for (;;) {
		if (xQueueReceive(s_blink_queue, &level, portMAX_DELAY) != pdTRUE)
			continue;
		halow_led_request_blink();
	}
}
#endif

static void record_peer_seen(const uint8_t *mac, uint32_t now_ms)
{
	if (!s_peers_mux || !mac)
		return;
	if (xSemaphoreTake(s_peers_mux, pdMS_TO_TICKS(50)) != pdTRUE)
		return;
	int slot = -1;
	int oldest = -1;
	uint32_t oldest_ms = 0;
	for (int i = 0; i < MAX_PEERS_SEEN; i++) {
		if (memcmp(s_peers_seen[i].mac, mac, 6) == 0) {
			slot = i;
			break;
		}
		if (s_peers_seen[i].last_seen_ms == 0) {
			if (slot < 0)
				slot = i;
			break;
		}
		if (oldest < 0 || s_peers_seen[i].last_seen_ms < oldest_ms) {
			oldest = i;
			oldest_ms = s_peers_seen[i].last_seen_ms;
		}
	}
	if (slot < 0 && oldest >= 0)
		slot = oldest;
	if (slot >= 0) {
		memcpy(s_peers_seen[slot].mac, mac, 6);
		s_peers_seen[slot].last_seen_ms = now_ms;
	}
	xSemaphoreGive(s_peers_mux);
}

static void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
	if (!info || !data)
		return;
	/* Count any sensor packet (v4 or v5) from another node; relay/forward only full v5-sized packets */
	if (len >= SENSOR_PACKET_MIN_LEN && data[0] == SENSOR_PACKET_MAGIC &&
	    memcmp(info->src_addr, s_self_mac, 6) != 0) {
		uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
		record_peer_seen(info->src_addr, now_ms);
		if (len >= (int)SENSOR_PACKET_SIZE) {
			esp_err_t err = esp_now_send(s_broadcast_mac, data, len);
			if (err != ESP_OK)
				ESP_LOGW(TAG, "relay send: %s", esp_err_to_name(err));
			/* Forward to HaLow mesh so gateway receives sensor data with original source MAC */
			espnow_fwd_sensor_packet_t fwd;
			fwd.magic = ESPNOW_FWD_MAGIC;
			fwd.version = ESPNOW_FWD_VERSION;
			memcpy(fwd.src_mac, info->src_addr, 6);
			memcpy(&fwd.pkt, data, SENSOR_PACKET_SIZE);
			send_halow_mesh_payload(&fwd, ESPNOW_FWD_SENSOR_PACKET_SIZE);
			if (now_ms - s_last_fwd_halow_log_ms >= FWD_TO_HALOW_LOG_MS) {
				s_last_fwd_halow_log_ms = now_ms;
				ESP_LOGI(TAG, "Fwd sensor %02x:%02x:%02x:... to HaLow (gateway mesh)", info->src_addr[0], info->src_addr[1], info->src_addr[2]);
			}
		}
		return;
	}
	/* Gateway commands (same as sensor_unit) */
	if (len < (int)CMD_PACKET_SIZE || data[0] != CMD_PACKET_MAGIC)
		return;
	if (data[1] == CMD_TYPE_BLINK) {
		if (s_blink_queue) {
			int dummy = 0;
			xQueueSend(s_blink_queue, &dummy, 0);
		}
		return;
	}
	if (data[1] == CMD_TYPE_RESET) {
		/* No motion/trigger state on camera; no-op like sensor_unit clear */
		return;
	}
	if (len >= (int)CMD_LABEL_PACKET_SIZE && data[1] == CMD_TYPE_SET_LABEL) {
		const cmd_label_packet_t *cmd = (const cmd_label_packet_t *)data;
		nvs_handle_t nvs;
		if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
			nvs_set_str(nvs, NVS_LABEL_KEY, cmd->label);
			nvs_commit(nvs);
			nvs_close(nvs);
		}
		ESP_LOGI(TAG, "Label set (from gateway)");
		return;
	}
	if (len >= (int)CMD_LOCATION_PACKET_SIZE && data[1] == CMD_TYPE_SET_LOCATION) {
		const cmd_location_packet_t *cmd = (const cmd_location_packet_t *)data;
		uint8_t is_outdoor = (cmd->is_outdoor != 0) ? 1 : 0;
		nvs_handle_t nvs;
		if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
			nvs_set_u8(nvs, NVS_IS_OUTDOOR_KEY, is_outdoor);
			nvs_commit(nvs);
			nvs_close(nvs);
		}
		ESP_LOGI(TAG, "Location set to %s (from gateway)", is_outdoor ? "outdoor" : "indoor");
		return;
	}
}

#if CONFIG_SENSOR_ESPNOW_CHANNEL_SCAN
/** Send callback used during channel scan: ACK = gateway on this channel. */
static void esp_now_scan_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	(void)mac_addr;
	if (s_scanning && s_scan_ack_sem != NULL && status == ESP_NOW_SEND_SUCCESS) {
		xSemaphoreGive(s_scan_ack_sem);
	}
}

/** Try one channel: set WiFi channel, add broadcast peer, send probe, wait for ACK. Returns true if ACK received. */
static bool try_channel_and_wait_ack(uint8_t channel)
{
	esp_wifi_set_channel((int)channel, WIFI_SECOND_CHAN_NONE);
	vTaskDelay(pdMS_TO_TICKS(10));
	esp_now_del_peer(s_broadcast_mac);
	esp_now_peer_info_t peer = { 0 };
	memcpy(peer.peer_addr, s_broadcast_mac, 6);
	peer.channel = channel;
	peer.ifidx = WIFI_IF_STA;
	peer.encrypt = false;
	esp_err_t err = esp_now_add_peer(&peer);
	if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
		return false;
	}
	sensor_packet_t probe = { 0 };
	probe.magic = SENSOR_PACKET_MAGIC;
	probe.version = SENSOR_PACKET_VERSION;
	s_scanning = true;
	xSemaphoreTake(s_scan_ack_sem, 0); /* clear any stale */
	err = esp_now_send(s_broadcast_mac, (const uint8_t *)&probe, SENSOR_PACKET_SIZE);
	if (err != ESP_OK) {
		s_scanning = false;
		return false;
	}
	BaseType_t ack = xSemaphoreTake(s_scan_ack_sem, pdMS_TO_TICKS(ESPNOW_SCAN_WAIT_MS));
	s_scanning = false;
	return (ack == pdTRUE);
}
#endif

static void wifi_init_esp_now(void)
{
	esp_netif_create_default_wifi_sta();
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);
	esp_wifi_set_storage(WIFI_STORAGE_RAM);
	esp_wifi_set_mode(WIFI_MODE_STA);
	esp_wifi_start();
	/* Start at minimum TX power (2 dBm); main.c restores to 4 dBm 5 s after boot complete */
	esp_wifi_set_max_tx_power(8);  /* 8 = 2 dBm min (ESP-IDF quarter-dBm) */
	vTaskDelay(pdMS_TO_TICKS(100));
	esp_wifi_get_mac(WIFI_IF_STA, s_self_mac);
	esp_err_t err = esp_now_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
		return;
	}

#if CONFIG_SENSOR_ESPNOW_CHANNEL_SCAN
	s_espnow_channel = 0;
	s_scan_ack_sem = xSemaphoreCreateBinary();
	if (s_scan_ack_sem != NULL) {
		esp_now_register_send_cb(esp_now_scan_send_cb);
		nvs_handle_t nvs = 0;
		if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
			uint8_t last_ch = 0;
			if (nvs_get_u8(nvs, NVS_ESPNOW_CHANNEL_KEY, &last_ch) == ESP_OK &&
			    last_ch >= ESPNOW_SCAN_CHANNEL_MIN && last_ch <= ESPNOW_SCAN_CHANNEL_MAX) {
				if (try_channel_and_wait_ack(last_ch)) {
					s_espnow_channel = last_ch;
					ESP_LOGI(TAG, "ESP-NOW channel %u (from NVS, ACK ok)", (unsigned)last_ch);
					nvs_close(nvs);
					goto scan_done;
				}
			}
			nvs_close(nvs);
		}
		for (uint8_t ch = ESPNOW_SCAN_CHANNEL_MIN; ch <= ESPNOW_SCAN_CHANNEL_MAX; ch++) {
			if (try_channel_and_wait_ack(ch)) {
				s_espnow_channel = ch;
				if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
					nvs_set_u8(nvs, NVS_ESPNOW_CHANNEL_KEY, ch);
					nvs_commit(nvs);
					nvs_close(nvs);
				}
				ESP_LOGI(TAG, "ESP-NOW channel %u (scan ACK)", (unsigned)ch);
				break;
			}
		}
		if (s_espnow_channel < ESPNOW_SCAN_CHANNEL_MIN || s_espnow_channel > ESPNOW_SCAN_CHANNEL_MAX) {
			s_espnow_channel = (uint8_t)ESPNOW_CHANNEL_DEFAULT;
			ESP_LOGW(TAG, "No gateway ACK on 1-%d; using channel %d", ESPNOW_SCAN_CHANNEL_MAX, (int)s_espnow_channel);
		}
scan_done:
		esp_now_unregister_send_cb();
		vSemaphoreDelete(s_scan_ack_sem);
		s_scan_ack_sem = NULL;
	} else {
		s_espnow_channel = (uint8_t)ESPNOW_CHANNEL_DEFAULT;
	}
#else
	s_espnow_channel = (uint8_t)ESPNOW_CHANNEL_DEFAULT;
#endif

	esp_wifi_set_channel((int)s_espnow_channel, WIFI_SECOND_CHAN_NONE);
	vTaskDelay(pdMS_TO_TICKS(50));
	esp_now_del_peer(s_broadcast_mac);
	esp_now_peer_info_t peer = { 0 };
	memcpy(peer.peer_addr, s_broadcast_mac, 6);
	peer.channel = s_espnow_channel;
	peer.ifidx = WIFI_IF_STA;
	peer.encrypt = false;
	err = esp_now_add_peer(&peer);
	if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
		ESP_LOGE(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(err));
		return;
	}
#if CONFIG_SENSOR_LED_GPIO >= 0
	s_blink_queue = xQueueCreate(2, sizeof(int));
	if (s_blink_queue != NULL) {
		xTaskCreate(blink_task, "blink", 1536, NULL, 5, NULL);
		ESP_LOGI(TAG, "Blink command → HaLow link LED (GPIO %d)", CONFIG_SENSOR_LED_GPIO);
	}
#endif
	esp_now_register_recv_cb(esp_now_recv_cb);
	s_esp_now_ok = true;
	ESP_LOGI(TAG, "ESP-NOW ready (channel %d, relay hop)", (int)s_espnow_channel);
}

void esp_now_send_camera_restore_wifi_tx_power(void)
{
	if (!s_esp_now_ok)
		return;  /* WiFi STA (ESP-NOW) not running */
	/* 2.4 GHz TX power: 4 dBm (ESP-IDF quarter-dBm) */
	esp_err_t err = esp_wifi_set_max_tx_power(4 * 4);
	if (err == ESP_OK) {
		ESP_LOGI(TAG, "WiFi TX power restored to 4 dBm");
	} else {
		ESP_LOGW(TAG, "WiFi TX power restore failed: %s", esp_err_to_name(err));
	}
}

static void esp_now_send_camera_stop(void)
{
	if (!s_esp_now_ok)
		return;
	esp_now_unregister_recv_cb();
	esp_now_deinit();
	s_esp_now_ok = false;
	ESP_LOGI(TAG, "ESP-NOW disabled");
}

void esp_now_send_camera_init(void)
{
	bool espnow_enabled = true;
	camera_unit_settings_get_espnow(&espnow_enabled);
	memset(s_peers_seen, 0, sizeof(s_peers_seen));
	if (s_peers_mux == NULL)
		s_peers_mux = xSemaphoreCreateMutex();
	if (!espnow_enabled) {
		ESP_LOGI(TAG, "ESP-NOW disabled by settings");
		return;
	}
	wifi_init_esp_now();
}

bool esp_now_send_camera_ready(void)
{
	return s_esp_now_ok;
}

bool esp_now_send_camera_is_enabled(void)
{
	return s_esp_now_ok;
}

void esp_now_send_camera_set_enabled(bool enabled)
{
	camera_unit_settings_set_espnow(enabled);
	if (enabled) {
		if (!s_esp_now_ok)
			wifi_init_esp_now();
	} else {
		esp_now_send_camera_stop();
	}
}

int esp_now_send_camera_peers_seen_count(void)
{
	if (!s_peers_mux)
		return 0;
	uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
	int count = 0;
	if (xSemaphoreTake(s_peers_mux, pdMS_TO_TICKS(50)) != pdTRUE)
		return 0;
	for (int i = 0; i < MAX_PEERS_SEEN; i++) {
		if (s_peers_seen[i].last_seen_ms == 0)
			continue;
		if (now_ms - s_peers_seen[i].last_seen_ms < PEER_STALE_MS)
			count++;
	}
	xSemaphoreGive(s_peers_mux);
	return count;
}

/** Load label and is_outdoor from NVS (sensor namespace). */
static void load_label_and_location(char *label_out, size_t label_size, uint8_t *is_outdoor_out)
{
	label_out[0] = '\0';
	*is_outdoor_out = 0;
	nvs_handle_t nvs;
	if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK)
		return;
	size_t len = label_size;
	if (nvs_get_str(nvs, NVS_LABEL_KEY, label_out, &len) != ESP_OK)
		label_out[0] = '\0';
	label_out[label_size - 1] = '\0';
	uint8_t v = 0;
	if (nvs_get_u8(nvs, NVS_IS_OUTDOOR_KEY, &v) == ESP_OK)
		*is_outdoor_out = (v != 0) ? 1 : 0;
	nvs_close(nvs);
}

/** Fill packet same format as sensor_unit; no BME/motion so zeros. */
static void fill_camera_packet(sensor_packet_t *p)
{
	uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
	memset(p, 0, sizeof(*p));
	p->magic = SENSOR_PACKET_MAGIC;
	p->version = SENSOR_PACKET_VERSION;
	p->motion = 0;
	p->temperature = 0.0f;
	p->humidity = 0.0f;
	p->pressure = 0.0f;
	p->gas = 0.0f;
	p->last_motion_ms = 0;
	p->trigger_count = 0;
	p->ble_seen_count = 0;
	p->ble_last_rssi_dbm = 0;
	memset(p->ble_last_addr, 0, sizeof(p->ble_last_addr));
	p->uptime_ms = (uint32_t)now_ms;
	fill_stream_host(p->stream_host, sizeof(p->stream_host));
	load_label_and_location(p->label, sizeof(p->label), &p->is_outdoor);
}

/** Send arbitrary payload over HaLow mesh (ethertype 0x88B5). Gateway mesh_rx_cb receives it. */
static void send_halow_mesh_payload(const void *payload, size_t payload_len)
{
	static const uint8_t bcast[6] = MESH_BROADCAST_MAC;
	uint8_t halow_mac[6];
	if (!payload || payload_len == 0)
		return;
	if (mmwlan_get_mac_addr(halow_mac) != MMWLAN_SUCCESS)
		return;
	if (mmwlan_tx_wait_until_ready(500) != MMWLAN_SUCCESS)
		return;
	size_t mesh_payload_len = sizeof(halow_mesh_hdr_t) + payload_len;
	size_t frame_len = ETH_HDR_LEN + mesh_payload_len;
	struct mmpkt *txpkt = mmwlan_alloc_mmpkt_for_tx((uint32_t)frame_len, MMWLAN_TX_DEFAULT_QOS_TID);
	if (!txpkt)
		return;
	struct mmpktview *view = mmpkt_open(txpkt);
	if (!view) {
		mmpkt_release(txpkt);
		return;
	}
	uint8_t eth_hdr[ETH_HDR_LEN];
	memcpy(eth_hdr, bcast, 6);
	memcpy(eth_hdr + 6, halow_mac, 6);
	eth_hdr[12] = (uint8_t)(HALOW_MESH_OVERLAY_ETHERTYPE >> 8);
	eth_hdr[13] = (uint8_t)(HALOW_MESH_OVERLAY_ETHERTYPE & 0xFF);
	mmpkt_append_data(view, eth_hdr, sizeof(eth_hdr));
	halow_mesh_hdr_t hdr;
	hdr.magic = HALOW_MESH_MAGIC;
	hdr.version = HALOW_MESH_VERSION;
	hdr.msg_type = HALOW_MESH_MSG_DATA;
	hdr.ttl = HALOW_MESH_DEFAULT_TTL;
	hdr.hop_count = 0;
	hdr.reserved = 0;
	hdr.payload_len = (uint16_t)payload_len;
	memcpy(hdr.src, halow_mac, 6);
	memcpy(hdr.dest, bcast, 6);
	mmpkt_append_data(view, (const uint8_t *)&hdr, sizeof(hdr));
	mmpkt_append_data(view, (const uint8_t *)payload, payload_len);
	mmpkt_close(&view);
	struct mmwlan_tx_metadata meta = MMWLAN_TX_METADATA_INIT;
	if (mmwlan_tx_pkt(txpkt, &meta) != MMWLAN_SUCCESS)
		mmpkt_release(txpkt);
}

#define MESH_SEND_LOG_INTERVAL_MS  (30 * 1000)

/** Send camera's own sensor packet over HaLow mesh in wrapped format (gateway expects src_mac so dashboard shows camera). */
static void send_camera_packet_halow_mesh(const sensor_packet_t *pkt)
{
	espnow_fwd_sensor_packet_t fwd;
	fwd.magic = ESPNOW_FWD_MAGIC;
	fwd.version = ESPNOW_FWD_VERSION;
	memcpy(fwd.src_mac, s_self_mac, 6);
	memcpy(&fwd.pkt, pkt, SENSOR_PACKET_SIZE);
	send_halow_mesh_payload(&fwd, ESPNOW_FWD_SENSOR_PACKET_SIZE);
}

void esp_now_send_camera_packet(void)
{
	if (!s_esp_now_ok)
		return;
	sensor_packet_t pkt;
	fill_camera_packet(&pkt);
	/* Notify gateway via both HaLow mesh and ESP-NOW so the Video page shows this camera
	 * whether the gateway is in ESP-NOW range or only on the HaLow mesh. */
	send_camera_packet_halow_mesh(&pkt);
	esp_err_t err = esp_now_send(s_broadcast_mac, (const uint8_t *)&pkt, SENSOR_PACKET_SIZE);
	if (err != ESP_OK) {
		ESP_LOGW(TAG, "ESP-NOW send camera packet failed: %s", esp_err_to_name(err));
	}
	static uint32_t last_log_ms = 0;
	uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
	if (now_ms - last_log_ms >= MESH_SEND_LOG_INTERVAL_MS) {
		last_log_ms = now_ms;
		if (pkt.stream_host[0] != '\0') {
			ESP_LOGI(TAG, "Camera packet TX (uptime %lu ms) stream_host=%s [HaLow+ESP-NOW]", (unsigned long)pkt.uptime_ms, pkt.stream_host);
		} else {
			ESP_LOGI(TAG, "Sensor packet TX (uptime %lu ms) stream_host empty", (unsigned long)pkt.uptime_ms);
		}
	}
}
