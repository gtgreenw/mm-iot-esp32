#include "halow_mesh_overlay.h"

#include <string.h>

#include "mmwlan.h"
#include "mmpkt.h"
#include "mmnetif.h"

#define ETH_HDR_LEN 14
#define ETH_ADDR_LEN 6

static void write_eth_header(uint8_t *buf,
                             const uint8_t *dst,
                             const uint8_t *src,
                             uint16_t ethertype)
{
    memcpy(buf, dst, ETH_ADDR_LEN);
    memcpy(buf + ETH_ADDR_LEN, src, ETH_ADDR_LEN);
    buf[12] = (uint8_t)(ethertype >> 8);
    buf[13] = (uint8_t)(ethertype & 0xff);
}

static int mesh_send_eth(const uint8_t *next_hop,
                         const uint8_t *data,
                         size_t len,
                         void *ctx)
{
    halow_mesh_overlay_t *overlay = (halow_mesh_overlay_t *)ctx;
    if (!overlay || !next_hop || !data || len == 0) {
        return -1;
    }

    enum mmwlan_status status = mmwlan_tx_wait_until_ready(MMWLAN_TX_DEFAULT_TIMEOUT_MS);
    if (status != MMWLAN_SUCCESS) {
        return -1;
    }

    size_t frame_len = ETH_HDR_LEN + len;
    struct mmpkt *pkt = mmwlan_alloc_mmpkt_for_tx(frame_len, MMWLAN_TX_DEFAULT_QOS_TID);
    if (!pkt) {
        return -1;
    }

    struct mmpktview *view = mmpkt_open(pkt);
    if (!view) {
        mmpkt_release(pkt);
        return -1;
    }

    uint8_t hdr[ETH_HDR_LEN];
    write_eth_header(hdr, next_hop, overlay->local_mac, HALOW_MESH_OVERLAY_ETHERTYPE);
    mmpkt_append_data(view, hdr, sizeof(hdr));
    mmpkt_append_data(view, data, len);
    mmpkt_close(&view);

    struct mmwlan_tx_metadata metadata = MMWLAN_TX_METADATA_INIT;
    status = mmwlan_tx_pkt(pkt, &metadata);
    return (status == MMWLAN_SUCCESS) ? 0 : -1;
}

static bool mesh_rx_ethertype(const uint8_t *dst,
                              const uint8_t *src,
                              uint16_t ethertype,
                              const uint8_t *payload,
                              size_t payload_len,
                              void *arg)
{
    (void)dst;
    (void)ethertype;
    halow_mesh_overlay_t *overlay = (halow_mesh_overlay_t *)arg;
    if (!overlay || !src || !payload) {
        return false;
    }

    if (halow_mesh_handle_rx(&overlay->mesh, src, payload, payload_len) == 0) {
        return true;
    }
    return false;
}

bool halow_mesh_overlay_init(halow_mesh_overlay_t *overlay, size_t max_routes)
{
    if (!overlay || max_routes == 0) {
        return false;
    }

    memset(overlay, 0, sizeof(*overlay));
    if (mmwlan_get_mac_addr(overlay->local_mac) != MMWLAN_SUCCESS) {
        return false;
    }

    if (!halow_mesh_init(&overlay->mesh, overlay->local_mac,
                         mesh_send_eth, overlay, max_routes)) {
        return false;
    }

    if (!mmnetif_register_ethertype_handler(HALOW_MESH_OVERLAY_ETHERTYPE,
                                            mesh_rx_ethertype, overlay)) {
        halow_mesh_deinit(&overlay->mesh);
        return false;
    }

    return true;
}

void halow_mesh_overlay_deinit(halow_mesh_overlay_t *overlay)
{
    if (!overlay) {
        return;
    }
    mmnetif_register_ethertype_handler(HALOW_MESH_OVERLAY_ETHERTYPE, NULL, NULL);
    halow_mesh_deinit(&overlay->mesh);
}

void halow_mesh_overlay_set_rx_cb(halow_mesh_overlay_t *overlay,
                                  halow_mesh_rx_cb cb,
                                  void *ctx)
{
    if (!overlay) {
        return;
    }
    halow_mesh_set_rx_cb(&overlay->mesh, cb, ctx);
}

int halow_mesh_overlay_send(halow_mesh_overlay_t *overlay,
                            const uint8_t dest[HALOW_MESH_ADDR_LEN],
                            const uint8_t *payload,
                            size_t payload_len)
{
    if (!overlay) {
        return -1;
    }
    return halow_mesh_send(&overlay->mesh, dest, payload, payload_len);
}

int halow_mesh_overlay_send_dv(halow_mesh_overlay_t *overlay)
{
    if (!overlay) {
        return -1;
    }

    uint8_t buf[256];
    size_t len = halow_mesh_build_dv_update(&overlay->mesh, buf, sizeof(buf));
    if (len == 0) {
        return -1;
    }
    static const uint8_t bcast[HALOW_MESH_ADDR_LEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    return mesh_send_eth(bcast, buf, len, overlay);
}

void halow_mesh_overlay_tick(halow_mesh_overlay_t *overlay)
{
    if (!overlay) {
        return;
    }
    halow_mesh_tick(&overlay->mesh);
}

size_t halow_mesh_overlay_node_count(const halow_mesh_overlay_t *overlay)
{
    if (!overlay) {
        return 0;
    }
    return halow_mesh_node_count(&overlay->mesh);
}
