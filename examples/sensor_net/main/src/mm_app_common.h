/*
 * Copyright 2023 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * Morse Micro application helper routines for initializing/de-initializing the Wireless LAN
 * interface and IP configuration.
 *
 * @section APP_COMMON_API Application helper routines for Wireless LAN interface
 *
 * This file contains routines for loading the required WLAN and IP parameters (e.g., SSID,
 * password, and IP address) from the @ref MMCONFIG, initializing the WLAN
 * interface and network stack.
 *
 * Parameters of interest are listed in the table below. These can be set in the `config.hjson`
 * file for your project and then programmed to the config store either using the Platform IO
 * UI or through the command line. See the config.hjson for advanced parameters.
 *
 * | Parameter Name      | Description                                                           |
 * | ------------------- | --------------------------------------------------------------------- |
 * | `wlan.country_code` | 2 character country code specifying the regulatory domain to use      |
 * | `wlan.ssid`         | The SSID of the network                                               |
 * | `wlan.security`     | Security type of the network (`sae`, `owe`, or `open`)                |
 * | `wlan.password`     | The password of your network if security type is `sae`                |
 * | `ip.dhcp_enabled`   | For static IPv4 addresses set to `false`, for IPv4 DHCP set to `true` |
 * | `ip.ip_addr`        | IPv4 address to use if `ip.dhcp_enabled` is false                     |
 * | `ip.netmask`        | IPv4 Netmask to use if `ip.dhcp_enabled` is false                     |
 * | `ip.gateway`        | IP address of gateway to use if `ip.dhcp_enabled` is false            |
 * | `ip6.ip_addr`       | IPv6 address to use if `ip6.autoconfig` is false                      |
 * | `ip6.autoconfig`    | For IPv6 static address set to false, for IPv6 autoconfig set to true |
 *
 * To set the parameters in config store from the command line, follow the
 * @ref MMCONFIG_PROGRAMMING instructions.
 */

#include <stdbool.h>
#include <stdint.h>

/**
 * Initializes the WLAN interface using settings specified in the config store.
 *
 * If no settings are found, the defaults are used.
 *
 * @warning This must be called only once.
 */

void app_wlan_init(void);

/**
 * Starts the WLAN interface and connects to Wi-Fi using settings specified in the config store.
 *
 * If no settings are found, the defaults are used.
 */
void app_wlan_start(void);

/**
 * Starts the WLAN interface and waits for link-up up to timeout_ms.
 *
 * @return true if link-up occurred within timeout, false otherwise.
 */
bool app_wlan_start_with_timeout(uint32_t timeout_ms);

/**
 * Disconnects from Wi-Fi and de-initializes the WLAN interface.
 */
void app_wlan_stop(void);

/**
 * Request a HaLow reconnect (non-blocking). Returns false if not available.
 */
bool app_wlan_request_reconnect(void);

/**
 * Return last known HaLow operating bandwidth in MHz (0 if unknown).
 */
uint8_t app_wlan_get_op_bw_mhz(void);
