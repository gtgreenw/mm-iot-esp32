#ifndef HALOW_MESH_OVERLAY_H
#define HALOW_MESH_OVERLAY_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "halow_mesh.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HALOW_MESH_OVERLAY_ETHERTYPE 0x88B5

typedef struct {
    halow_mesh_t mesh;
    uint8_t local_mac[HALOW_MESH_ADDR_LEN];
} halow_mesh_overlay_t;

bool halow_mesh_overlay_init(halow_mesh_overlay_t *overlay, size_t max_routes);
void halow_mesh_overlay_deinit(halow_mesh_overlay_t *overlay);

void halow_mesh_overlay_set_rx_cb(halow_mesh_overlay_t *overlay,
                                  halow_mesh_rx_cb cb,
                                  void *ctx);

int halow_mesh_overlay_send(halow_mesh_overlay_t *overlay,
                            const uint8_t dest[HALOW_MESH_ADDR_LEN],
                            const uint8_t *payload,
                            size_t payload_len);

int halow_mesh_overlay_send_dv(halow_mesh_overlay_t *overlay);

void halow_mesh_overlay_tick(halow_mesh_overlay_t *overlay);

size_t halow_mesh_overlay_node_count(const halow_mesh_overlay_t *overlay);

#ifdef __cplusplus
}
#endif

#endif
