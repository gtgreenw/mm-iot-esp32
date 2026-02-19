#include "halow_mesh.h"

#include <string.h>
#include <stdlib.h>

#include "esp_heap_caps.h"
#include "mmosal.h"

#define HALOW_MESH_BROADCAST_ADDR "\xff\xff\xff\xff\xff\xff"

static bool addr_eq(const uint8_t a[HALOW_MESH_ADDR_LEN],
                    const uint8_t b[HALOW_MESH_ADDR_LEN])
{
    return memcmp(a, b, HALOW_MESH_ADDR_LEN) == 0;
}

static bool addr_is_broadcast(const uint8_t addr[HALOW_MESH_ADDR_LEN])
{
    return memcmp(addr, HALOW_MESH_BROADCAST_ADDR, HALOW_MESH_ADDR_LEN) == 0;
}

static void addr_copy(uint8_t dst[HALOW_MESH_ADDR_LEN],
                      const uint8_t src[HALOW_MESH_ADDR_LEN])
{
    memcpy(dst, src, HALOW_MESH_ADDR_LEN);
}

static halow_mesh_route_t *find_route(halow_mesh_t *mesh,
                                      const uint8_t dest[HALOW_MESH_ADDR_LEN])
{
    for (size_t i = 0; i < mesh->max_routes; ++i) {
        if (mesh->routes[i].valid && addr_eq(mesh->routes[i].dest, dest)) {
            return &mesh->routes[i];
        }
    }
    return NULL;
}

static halow_mesh_route_t *alloc_route(halow_mesh_t *mesh,
                                       const uint8_t dest[HALOW_MESH_ADDR_LEN])
{
    for (size_t i = 0; i < mesh->max_routes; ++i) {
        if (!mesh->routes[i].valid) {
            mesh->routes[i].valid = true;
            addr_copy(mesh->routes[i].dest, dest);
            mesh->route_count++;
            return &mesh->routes[i];
        }
    }
    return NULL;
}

static void update_route(halow_mesh_t *mesh,
                         const uint8_t dest[HALOW_MESH_ADDR_LEN],
                         const uint8_t next_hop[HALOW_MESH_ADDR_LEN],
                         uint8_t cost)
{
    if (cost == 0 || cost > HALOW_MESH_MAX_COST) {
        return;
    }

    halow_mesh_route_t *route = find_route(mesh, dest);
    if (!route) {
        route = alloc_route(mesh, dest);
        if (!route) {
            return;
        }
        addr_copy(route->next_hop, next_hop);
        route->cost = cost;
        route->last_update_ms = mmosal_get_time_ms();
        return;
    }

    if (route->cost > cost || addr_eq(route->next_hop, next_hop)) {
        addr_copy(route->next_hop, next_hop);
        route->cost = cost;
        route->last_update_ms = mmosal_get_time_ms();
    }
}

static int send_with_header(halow_mesh_t *mesh,
                            const uint8_t next_hop[HALOW_MESH_ADDR_LEN],
                            const uint8_t dest[HALOW_MESH_ADDR_LEN],
                            const uint8_t *payload,
                            size_t payload_len,
                            uint8_t msg_type,
                            uint8_t ttl,
                            uint8_t hop_count)
{
    if (!mesh || !mesh->send_fn || (!payload && payload_len > 0)) {
        return -1;
    }

    size_t total_len = sizeof(halow_mesh_hdr_t) + payload_len;
    uint8_t *buf = (uint8_t *)heap_caps_malloc(total_len, MALLOC_CAP_8BIT);
    if (!buf) {
        buf = (uint8_t *)malloc(total_len);
    }
    if (!buf) {
        return -1;
    }

    halow_mesh_hdr_t *hdr = (halow_mesh_hdr_t *)buf;
    hdr->magic = HALOW_MESH_MAGIC;
    hdr->version = HALOW_MESH_VERSION;
    hdr->msg_type = msg_type;
    hdr->ttl = ttl;
    hdr->hop_count = hop_count;
    hdr->reserved = 0;
    hdr->payload_len = (uint16_t)payload_len;
    addr_copy(hdr->src, mesh->local_addr);
    addr_copy(hdr->dest, dest);

    if (payload_len > 0 && payload) {
        memcpy(buf + sizeof(halow_mesh_hdr_t), payload, payload_len);
    }

    int rc = mesh->send_fn(next_hop, buf, total_len, mesh->send_ctx);
    free(buf);
    return rc;
}

bool halow_mesh_init(halow_mesh_t *mesh,
                     const uint8_t local_addr[HALOW_MESH_ADDR_LEN],
                     halow_mesh_send_fn send_fn,
                     void *send_ctx,
                     size_t max_routes)
{
    if (!mesh || !local_addr || !send_fn || max_routes == 0) {
        return false;
    }

    memset(mesh, 0, sizeof(*mesh));
    addr_copy(mesh->local_addr, local_addr);
    mesh->send_fn = send_fn;
    mesh->send_ctx = send_ctx;
    mesh->max_routes = max_routes;

    size_t bytes = sizeof(halow_mesh_route_t) * max_routes;
    mesh->routes = (halow_mesh_route_t *)heap_caps_malloc(bytes,
                                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!mesh->routes) {
        mesh->routes = (halow_mesh_route_t *)calloc(max_routes, sizeof(halow_mesh_route_t));
    } else {
        memset(mesh->routes, 0, bytes);
    }

    return mesh->routes != NULL;
}

void halow_mesh_deinit(halow_mesh_t *mesh)
{
    if (!mesh) {
        return;
    }
    if (mesh->routes) {
        free(mesh->routes);
        mesh->routes = NULL;
    }
    mesh->route_count = 0;
    mesh->max_routes = 0;
}

void halow_mesh_set_rx_cb(halow_mesh_t *mesh, halow_mesh_rx_cb cb, void *ctx)
{
    if (!mesh) {
        return;
    }
    mesh->rx_cb = cb;
    mesh->rx_ctx = ctx;
}

int halow_mesh_send(halow_mesh_t *mesh,
                    const uint8_t dest[HALOW_MESH_ADDR_LEN],
                    const uint8_t *payload,
                    size_t payload_len)
{
    if (!mesh || !dest || (!payload && payload_len > 0)) {
        return -1;
    }

    if (addr_is_broadcast(dest)) {
        return send_with_header(mesh,
                                (const uint8_t *)HALOW_MESH_BROADCAST_ADDR,
                                dest,
                                payload,
                                payload_len,
                                HALOW_MESH_MSG_DATA,
                                HALOW_MESH_DEFAULT_TTL,
                                0);
    }

    halow_mesh_route_t *route = find_route(mesh, dest);
    if (!route) {
        return -1;
    }

    return send_with_header(mesh,
                            route->next_hop,
                            dest,
                            payload,
                            payload_len,
                            HALOW_MESH_MSG_DATA,
                            HALOW_MESH_DEFAULT_TTL,
                            0);
}

static void update_neighbor_route(halow_mesh_t *mesh,
                                  const uint8_t neighbor[HALOW_MESH_ADDR_LEN])
{
    if (!mesh || !neighbor) {
        return;
    }
    update_route(mesh, neighbor, neighbor, 1);
}

int halow_mesh_handle_rx(halow_mesh_t *mesh,
                         const uint8_t rx_src[HALOW_MESH_ADDR_LEN],
                         const uint8_t *data,
                         size_t len)
{
    if (!mesh || !rx_src || !data || len < sizeof(halow_mesh_hdr_t)) {
        return -1;
    }

    const halow_mesh_hdr_t *hdr = (const halow_mesh_hdr_t *)data;
    if (hdr->magic != HALOW_MESH_MAGIC || hdr->version != HALOW_MESH_VERSION) {
        return -1;
    }
    if (sizeof(halow_mesh_hdr_t) + hdr->payload_len > len) {
        return -1;
    }

    update_neighbor_route(mesh, rx_src);

    const uint8_t *payload = data + sizeof(halow_mesh_hdr_t);
    size_t payload_len = hdr->payload_len;

    if (hdr->msg_type == HALOW_MESH_MSG_DV_UPDATE) {
        if (payload_len < 1) {
            return -1;
        }
        uint8_t count = payload[0];
        size_t needed = 1 + (size_t)count * sizeof(halow_mesh_dv_entry_t);
        if (payload_len < needed) {
            return -1;
        }
        const halow_mesh_dv_entry_t *entries =
            (const halow_mesh_dv_entry_t *)(payload + 1);
        for (uint8_t i = 0; i < count; ++i) {
            const halow_mesh_dv_entry_t *e = &entries[i];
            if (addr_eq(e->dest, mesh->local_addr)) {
                continue;
            }
            uint8_t new_cost = (uint8_t)(e->cost + 1);
            if (new_cost > HALOW_MESH_MAX_COST) {
                continue;
            }
            update_route(mesh, e->dest, rx_src, new_cost);
        }
        return 0;
    }

    if (hdr->msg_type != HALOW_MESH_MSG_DATA) {
        return -1;
    }

    if (addr_eq(hdr->dest, mesh->local_addr) || addr_is_broadcast(hdr->dest)) {
        update_route(mesh, hdr->src, rx_src, (uint8_t)(hdr->hop_count + 1));
        if (mesh->rx_cb) {
            mesh->rx_cb(hdr->src, payload, payload_len, mesh->rx_ctx);
        }
        return 0;
    }

    if (hdr->ttl <= 1) {
        return -1;
    }

    halow_mesh_route_t *route = find_route(mesh, hdr->dest);
    if (!route) {
        return -1;
    }
    if (addr_eq(route->next_hop, rx_src)) {
        return -1;
    }

    return send_with_header(mesh,
                            route->next_hop,
                            hdr->dest,
                            payload,
                            payload_len,
                            HALOW_MESH_MSG_DATA,
                            (uint8_t)(hdr->ttl - 1),
                            (uint8_t)(hdr->hop_count + 1));
}

size_t halow_mesh_build_dv_update(halow_mesh_t *mesh,
                                  uint8_t *out,
                                  size_t max_len)
{
    if (!mesh || !out || max_len < sizeof(halow_mesh_hdr_t) + 1) {
        return 0;
    }

    size_t header_len = sizeof(halow_mesh_hdr_t);
    size_t max_entries = (max_len - header_len - 1) / sizeof(halow_mesh_dv_entry_t);
    if (max_entries == 0) {
        return 0;
    }

    halow_mesh_hdr_t *hdr = (halow_mesh_hdr_t *)out;
    hdr->magic = HALOW_MESH_MAGIC;
    hdr->version = HALOW_MESH_VERSION;
    hdr->msg_type = HALOW_MESH_MSG_DV_UPDATE;
    hdr->ttl = 1;
    hdr->hop_count = 0;
    hdr->reserved = 0;
    addr_copy(hdr->src, mesh->local_addr);
    memset(hdr->dest, 0xff, HALOW_MESH_ADDR_LEN);

    uint8_t *payload = out + header_len;
    uint8_t count = 0;
    halow_mesh_dv_entry_t *entries = (halow_mesh_dv_entry_t *)(payload + 1);

    if (count < max_entries) {
        entries[count].cost = 0;
        addr_copy(entries[count].dest, mesh->local_addr);
        count++;
    }

    for (size_t i = 0; i < mesh->max_routes && count < max_entries; ++i) {
        if (!mesh->routes[i].valid) {
            continue;
        }
        entries[count].cost = mesh->routes[i].cost;
        addr_copy(entries[count].dest, mesh->routes[i].dest);
        count++;
    }

    payload[0] = count;
    hdr->payload_len = (uint16_t)(1 + count * sizeof(halow_mesh_dv_entry_t));
    return header_len + hdr->payload_len;
}

void halow_mesh_tick(halow_mesh_t *mesh)
{
    if (!mesh || !mesh->routes) {
        return;
    }
    uint32_t now_ms = mmosal_get_time_ms();
    for (size_t i = 0; i < mesh->max_routes; ++i) {
        if (!mesh->routes[i].valid) {
            continue;
        }
        uint32_t age = now_ms - mesh->routes[i].last_update_ms;
        if (age > HALOW_MESH_ROUTE_TIMEOUT_MS) {
            mesh->routes[i].valid = false;
            if (mesh->route_count > 0) {
                mesh->route_count--;
            }
        }
    }
}

size_t halow_mesh_node_count(const halow_mesh_t *mesh)
{
    if (!mesh || !mesh->routes) {
        return 0;
    }
    size_t count = 1;
    for (size_t i = 0; i < mesh->max_routes; ++i) {
        if (!mesh->routes[i].valid) {
            continue;
        }
        if (memcmp(mesh->routes[i].dest, mesh->local_addr, HALOW_MESH_ADDR_LEN) == 0) {
            continue;
        }
        count++;
    }
    return count;
}
