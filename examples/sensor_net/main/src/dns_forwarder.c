/*
 * Copyright 2023 Morse Micro
 * SPDX-License-Identifier: Apache-2.0
 *
 * DNS forwarder: listen on UDP 53, forward queries to upstream, return replies to client.
 * Clients use 192.168.4.1 as DNS so traffic stays local; only Xiao->upstream uses HaLow.
 * Optional response cache to reduce repeat lookups over the slow HaLow link.
 */

#include <string.h>
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dns_forwarder.h"

static const char *TAG = "DNS_FWD";

#define DNS_PORT                 53
#define DNS_MAX_PAYLOAD          512
#define PENDING_MAX              16
#define CACHE_ENTRIES            16
#define CACHE_TTL_SEC            120
#define DNS_FORWARDER_STACK_WORDS 4096

typedef struct {
    uint16_t txid;
    struct sockaddr_in client;
    uint8_t used;
} pending_t;

typedef struct {
    uint32_t key;
    uint32_t expire_sec;
    uint16_t len;
    uint8_t  payload[DNS_MAX_PAYLOAD];
} cache_entry_t;

static pending_t s_pending[PENDING_MAX];
static cache_entry_t s_cache[CACHE_ENTRIES];
static struct sockaddr_in s_upstream;
static int s_sock = -1;
static TaskHandle_t s_task = NULL;

static int dns_question_len(const uint8_t *buf, int len)
{
    if (len < 13) return 0;
    int i = 12;
    while (i < len) {
        if (buf[i] == 0) return (i - 12) + 1 + 4;
        if ((buf[i] & 0xC0) == 0xC0) return 2 + 4;
        int labellen = buf[i];
        i += 1 + labellen;
    }
    return 0;
}

static uint32_t dns_question_hash(const uint8_t *buf, int len)
{
    int qlen = dns_question_len(buf, len);
    if (qlen <= 0 || 12 + qlen > len) return 0;
    uint32_t h = 0;
    for (int j = 0; j < qlen; j++) {
        h = (h * 31) + buf[12 + j];
    }
    return h;
}

static uint16_t get_dns_txid(const void *buf)
{
    const uint8_t *p = (const uint8_t *)buf;
    return (uint16_t)((p[0] << 8) | p[1]);
}

static void set_dns_txid(void *buf, uint16_t txid)
{
    uint8_t *p = (uint8_t *)buf;
    p[0] = (uint8_t)(txid >> 8);
    p[1] = (uint8_t)(txid & 0xFF);
}

static int is_dns_response(const void *buf, int len)
{
    if (len < 12) return 0;
    return (((const uint8_t *)buf)[2] & 0x80) != 0;
}

static void pending_put(uint16_t txid, const struct sockaddr_in *client)
{
    for (int i = 0; i < PENDING_MAX; i++) {
        if (!s_pending[i].used) {
            s_pending[i].txid = txid;
            s_pending[i].client = *client;
            s_pending[i].used = 1;
            return;
        }
    }
    s_pending[0].txid = txid;
    s_pending[0].client = *client;
    s_pending[0].used = 1;
}

static int pending_get(uint16_t txid, struct sockaddr_in *out_client)
{
    for (int i = 0; i < PENDING_MAX; i++) {
        if (s_pending[i].used && s_pending[i].txid == txid) {
            *out_client = s_pending[i].client;
            s_pending[i].used = 0;
            return 1;
        }
    }
    return 0;
}

static cache_entry_t *cache_lookup(uint32_t key, uint32_t now_sec)
{
    for (int i = 0; i < CACHE_ENTRIES; i++) {
        if (s_cache[i].len > 0 && s_cache[i].key == key && s_cache[i].expire_sec > now_sec) {
            return &s_cache[i];
        }
    }
    return NULL;
}

static void cache_store(uint32_t key, const uint8_t *payload, uint16_t len, uint32_t now_sec)
{
    if (len > DNS_MAX_PAYLOAD) return;
    /* Evict oldest or first empty */
    int slot = 0;
    uint32_t oldest = now_sec + 1;
    for (int i = 0; i < CACHE_ENTRIES; i++) {
        if (s_cache[i].len == 0) {
            slot = i;
            break;
        }
        if (s_cache[i].expire_sec < oldest) {
            oldest = s_cache[i].expire_sec;
            slot = i;
        }
    }
    s_cache[slot].key = key;
    s_cache[slot].expire_sec = now_sec + CACHE_TTL_SEC;
    s_cache[slot].len = len;
    memcpy(s_cache[slot].payload, payload, len);
}

static void dns_forwarder_task(void *pvParameters)
{
    const char *upstream_ip = (const char *)pvParameters;
    struct addrinfo hints = { 0 }, *res = NULL;
    uint8_t buf[DNS_MAX_PAYLOAD];
    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    if (lwip_getaddrinfo(upstream_ip, "53", &hints, &res) != 0 || res == NULL) {
        ESP_LOGE(TAG, "getaddrinfo %s failed", upstream_ip);
        vTaskDelete(NULL);
        return;
    }
    s_upstream = *(struct sockaddr_in *)res->ai_addr;
    lwip_freeaddrinfo(res);

    s_sock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket failed");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    lwip_setsockopt(s_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in bind_addr = { 0 };
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(DNS_PORT);
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    if (lwip_bind(s_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0) {
        ESP_LOGE(TAG, "bind 0.0.0.0:53 failed");
        lwip_close(s_sock);
        s_sock = -1;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "DNS forwarder: clients use 192.168.4.1 -> upstream %s:53 (cache %d entries, %ds TTL)", upstream_ip, CACHE_ENTRIES, CACHE_TTL_SEC);

    while (1) {
        int n = lwip_recvfrom(s_sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &fromlen);
        if (n <= 0 || n > (int)sizeof(buf)) {
            continue;
        }
        uint32_t now_sec = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS / 1000);

        if (is_dns_response(buf, n)) {
            uint16_t txid = get_dns_txid(buf);
            struct sockaddr_in client;
            if (pending_get(txid, &client)) {
                lwip_sendto(s_sock, buf, n, 0, (struct sockaddr *)&client, sizeof(client));
                /* Cache the response for future lookups */
                int qlen = dns_question_len(buf, n);
                if (qlen > 0) {
                    uint32_t key = dns_question_hash(buf, n);
                    cache_store(key, buf, (uint16_t)n, now_sec);
                }
            }
        } else {
            uint16_t txid = get_dns_txid(buf);
            int qlen = dns_question_len(buf, n);
            if (qlen > 0) {
                uint32_t key = dns_question_hash(buf, n);
                cache_entry_t *ce = cache_lookup(key, now_sec);
                if (ce != NULL) {
                    memcpy(buf, ce->payload, ce->len);
                    set_dns_txid(buf, txid);
                    lwip_sendto(s_sock, buf, ce->len, 0, (struct sockaddr *)&from, sizeof(from));
                    continue;
                }
            }
            pending_put(txid, &from);
            lwip_sendto(s_sock, buf, n, 0, (struct sockaddr *)&s_upstream, sizeof(s_upstream));
        }
    }
}

void dns_forwarder_start(const char *upstream_ip)
{
    if (upstream_ip == NULL || s_task != NULL) {
        return;
    }
    memset(s_pending, 0, sizeof(s_pending));
    memset(s_cache, 0, sizeof(s_cache));
    xTaskCreate(dns_forwarder_task, "dns_fwd", DNS_FORWARDER_STACK_WORDS,
                (void *)upstream_ip, 5, &s_task);
}
