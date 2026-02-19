#ifndef HALOW_MESH_H
#define HALOW_MESH_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HALOW_MESH_ADDR_LEN 6
#define HALOW_MESH_MAGIC 0x4D
#define HALOW_MESH_VERSION 1

#define HALOW_MESH_MSG_DATA 1
#define HALOW_MESH_MSG_DV_UPDATE 2

#ifndef HALOW_MESH_DEFAULT_TTL
#define HALOW_MESH_DEFAULT_TTL 8
#endif

#ifndef HALOW_MESH_ROUTE_TIMEOUT_MS
#define HALOW_MESH_ROUTE_TIMEOUT_MS 120000
#endif

#ifndef HALOW_MESH_MAX_COST
#define HALOW_MESH_MAX_COST 32
#endif

typedef int (*halow_mesh_send_fn)(const uint8_t *next_hop,
                                 const uint8_t *data,
                                 size_t len,
                                 void *ctx);

typedef void (*halow_mesh_rx_cb)(const uint8_t *src,
                                const uint8_t *payload,
                                size_t len,
                                void *ctx);

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t version;
    uint8_t msg_type;
    uint8_t ttl;
    uint8_t hop_count;
    uint8_t reserved;
    uint16_t payload_len;
    uint8_t src[HALOW_MESH_ADDR_LEN];
    uint8_t dest[HALOW_MESH_ADDR_LEN];
} halow_mesh_hdr_t;

typedef struct __attribute__((packed)) {
    uint8_t dest[HALOW_MESH_ADDR_LEN];
    uint8_t cost;
} halow_mesh_dv_entry_t;

typedef struct {
    uint8_t dest[HALOW_MESH_ADDR_LEN];
    uint8_t next_hop[HALOW_MESH_ADDR_LEN];
    uint8_t cost;
    uint32_t last_update_ms;
    bool valid;
} halow_mesh_route_t;

typedef struct {
    uint8_t local_addr[HALOW_MESH_ADDR_LEN];
    halow_mesh_route_t *routes;
    size_t route_count;
    size_t max_routes;
    halow_mesh_send_fn send_fn;
    void *send_ctx;
    halow_mesh_rx_cb rx_cb;
    void *rx_ctx;
    uint16_t seq;
} halow_mesh_t;

bool halow_mesh_init(halow_mesh_t *mesh,
                     const uint8_t local_addr[HALOW_MESH_ADDR_LEN],
                     halow_mesh_send_fn send_fn,
                     void *send_ctx,
                     size_t max_routes);

void halow_mesh_deinit(halow_mesh_t *mesh);

void halow_mesh_set_rx_cb(halow_mesh_t *mesh, halow_mesh_rx_cb cb, void *ctx);

int halow_mesh_send(halow_mesh_t *mesh,
                    const uint8_t dest[HALOW_MESH_ADDR_LEN],
                    const uint8_t *payload,
                    size_t payload_len);

int halow_mesh_handle_rx(halow_mesh_t *mesh,
                         const uint8_t rx_src[HALOW_MESH_ADDR_LEN],
                         const uint8_t *data,
                         size_t len);

size_t halow_mesh_build_dv_update(halow_mesh_t *mesh,
                                  uint8_t *out,
                                  size_t max_len);

void halow_mesh_tick(halow_mesh_t *mesh);

size_t halow_mesh_node_count(const halow_mesh_t *mesh);

#ifdef __cplusplus
}
#endif

#endif
