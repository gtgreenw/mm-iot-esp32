/*
 * Copyright 2023 Morse Micro
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MM_APP_COMMON_H
#define MM_APP_COMMON_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void app_wlan_init(void);
void app_wlan_start(void);
/** Returns true if connected within timeout_ms; false on timeout. Use UINT32_MAX for no timeout. */
bool app_wlan_start_with_timeout(uint32_t timeout_ms);
void app_wlan_stop(void);
void app_wlan_arp_send(void);

#ifdef __cplusplus
}
#endif

#endif
