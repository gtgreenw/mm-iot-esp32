/*
 * Copyright 2023 Morse Micro
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef NAT_ROUTER_H
#define NAT_ROUTER_H

/** Start the 2.4 GHz AP with NAPT and DHCP. Call once after HaLow link is up. */
void start_2ghz_ap(void);

/** Start 2.4 GHz AP + STA backhaul (HaLow disabled). */
void start_2ghz_apsta_backhaul(void);

/** Refresh LwIP default netif and HaLow gateway. Call when HaLow link comes back up after reconnect
 *  so NAPT and DNS use the HaLow interface for internet again. */
void nat_router_refresh_halow_default_route(void);

#endif /* NAT_ROUTER_H */
