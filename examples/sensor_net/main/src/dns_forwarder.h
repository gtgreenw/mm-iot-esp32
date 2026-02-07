/*
 * Copyright 2023 Morse Micro
 * SPDX-License-Identifier: Apache-2.0
 *
 * DNS forwarder: listen on UDP 53, forward to upstream. Clients use 192.168.4.1
 * as DNS (no NAPT needed for DNS). Call once after AP is up.
 */

#ifndef DNS_FORWARDER_H
#define DNS_FORWARDER_H

/** Start DNS forwarder (0.0.0.0:53 -> upstream). Upstream e.g. "8.8.8.8" or "10.41.0.1". */
void dns_forwarder_start(const char *upstream_ip);

#endif /* DNS_FORWARDER_H */
