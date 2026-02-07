/*
 * Copyright 2023 Morse Micro
 * SPDX-License-Identifier: Apache-2.0
 *
 * From mm-iot-esp32 examples; WLAN init/start and IP stack for HaLow STA.
 */
#include <stdio.h>
#include <stdint.h>
#include "mmosal.h"
#include "mmhal.h"
#include "mmwlan.h"
#include "mmipal.h"
#include "mm_app_common.h"
#include "mm_app_loadconfig.h"
#include "lwip/inet.h"

#define HALOW_SCAN_DWELL_MS  100

static struct mmosal_semb *link_established = NULL;
static bool link_up = false;
static uint32_t ip_addr_u32 = 0;
static uint32_t gw_addr_u32 = 0;
uint8_t mac_addr[MMWLAN_MAC_ADDR_LEN];

static void sta_status_callback(enum mmwlan_sta_state sta_state)
{
	switch (sta_state) {
	case MMWLAN_STA_DISABLED:
		printf("WLAN STA disabled\n");
		break;
	case MMWLAN_STA_CONNECTING:
		printf("WLAN STA connecting\n");
		break;
	case MMWLAN_STA_CONNECTED:
		printf("WLAN STA connected\n");
		break;
	}
}

static void link_status_callback(const struct mmipal_link_status *link_status)
{
	if (link_status->link_state == MMIPAL_LINK_UP) {
		link_up = true;
		ip_addr_u32 = ipaddr_addr(link_status->ip_addr);
		gw_addr_u32 = ipaddr_addr(link_status->gateway);
		mmosal_semb_give(link_established);
		app_wlan_arp_send();
	} else {
		link_up = false;
	}
}

void app_wlan_init(void)
{
	enum mmwlan_status status;
	struct mmwlan_version version;

	MMOSAL_ASSERT(link_established == NULL);
	link_established = mmosal_semb_create("link_established");

	mmhal_init();
	mmwlan_init();
	mmwlan_set_health_check_interval(0, 0);
	mmwlan_set_channel_list(load_channel_list());

	status = mmwlan_set_sgi_enabled(true);
	if (status != MMWLAN_SUCCESS)
		printf("Warning: mmwlan_set_sgi_enabled(true) failed (%d)\n", (int)status);
	status = mmwlan_set_rts_threshold(2347);
	if (status != MMWLAN_SUCCESS)
		printf("Warning: mmwlan_set_rts_threshold(2347) failed (%d)\n", (int)status);
	{
		struct mmwlan_scan_config scan_config = { .dwell_time_ms = HALOW_SCAN_DWELL_MS };
		status = mmwlan_set_scan_config(&scan_config);
		if (status != MMWLAN_SUCCESS)
			printf("Warning: mmwlan_set_scan_config failed (%d)\n", (int)status);
	}

	struct mmipal_init_args mmipal_init_args = MMIPAL_INIT_ARGS_DEFAULT;
	load_mmipal_init_args(&mmipal_init_args);
	if (mmipal_init(&mmipal_init_args) != MMIPAL_SUCCESS)
		MMOSAL_ASSERT(false);
	mmipal_set_link_status_callback(link_status_callback);

	status = mmwlan_get_version(&version);
	MMOSAL_ASSERT(status == MMWLAN_SUCCESS);
	status = mmwlan_get_mac_addr(mac_addr);
	MMOSAL_ASSERT(status == MMWLAN_SUCCESS);
}

void app_wlan_start(void)
{
	(void)app_wlan_start_with_timeout(UINT32_MAX);
}

bool app_wlan_start_with_timeout(uint32_t timeout_ms)
{
	enum mmwlan_status status;
	struct mmwlan_sta_args sta_args = MMWLAN_STA_ARGS_INIT;

	load_mmwlan_sta_args(&sta_args);
	load_mmwlan_settings();

	printf("Attempting to connect to %s ", sta_args.ssid);
	if (sta_args.security_type == MMWLAN_SAE)
		printf("with passphrase %s", sta_args.passphrase);
	printf("\n");
	if (timeout_ms != UINT32_MAX)
		printf("Timeout %lu ms\n", (unsigned long)timeout_ms);

	status = mmwlan_sta_enable(&sta_args, sta_status_callback);
	MMOSAL_ASSERT(status == MMWLAN_SUCCESS);

	mmosal_semb_wait(link_established, timeout_ms);
	return link_up;
}

void app_wlan_stop(void)
{
	mmwlan_shutdown();
}

void app_wlan_arp_send(void)
{
	if (!link_up) return;
	uint8_t arp_packet[] = {
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
		0x08, 0x06,
		0x00, 0x01, 0x08, 0x00, 0x06, 0x04, 0x00, 0x01,
		mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
		((uint8_t *)&ip_addr_u32)[0], ((uint8_t *)&ip_addr_u32)[1], ((uint8_t *)&ip_addr_u32)[2], ((uint8_t *)&ip_addr_u32)[3],
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		((uint8_t *)&gw_addr_u32)[0], ((uint8_t *)&gw_addr_u32)[1], ((uint8_t *)&gw_addr_u32)[2], ((uint8_t *)&gw_addr_u32)[3],
	};
	mmwlan_tx(arp_packet, sizeof(arp_packet));
}
