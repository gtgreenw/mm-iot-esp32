/*
 * Copyright 2022-2023 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Throughput measurement using iperf.
 *
 * The Iperf parameters are specified using the defines in the file. Additional defines in
 * @c mm_app_loadconfig.c and @c mm_app_common.c are used to configure the network stack and WLAN
 * interface.
 *
 * @note It is assumed that you have followed the steps in the @ref GETTING_STARTED guide and are
 * therefore familiar with how to build, flash, and monitor an application using the MM-IoT-SDK
 * framework.
 *
 * This file demonstrates how to run iperf using the Morse Micro WLAN API.
 */


#include <endian.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "mmosal.h"
#include "mmwlan.h"
#include "mmipal.h"

#include "esp_err.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "mmosal.h"

#include "mmiperf.h"
#include "mm_app_common.h"
#include "time_sync.h"
#define FW_VERSION "1.0.1"
#include "nat_router.h"
#include "settings.h"
#include "esp_now_rcv.h"

/* ------------------------ Configuration options ------------------------ */

/* ANSI cyberpunk palette: neon cyan, magenta, green on dark */
#define RST  "\033[0m"
#define DIM  "\033[90m"
#define CYAN "\033[1;36m"
#define MAG  "\033[1;35m"
#define NEON "\033[1;32m"
#define WARN "\033[1;33m"

static bool setup_halow_or_fallback(void)
{
    /* Attempt HaLow connect; fallback to AP config UI after 30s. */
    if (app_wlan_start_with_timeout(30000)) {
        printf("%s>>%s Link up: HaLow connected.\n", DIM, RST);
        return true;
    }

    printf("%s>>%s Link failed: timeout. Starting 2.4 GHz AP for setup...\n", DIM, RST);
    start_2ghz_ap();
    esp_now_rcv_init();
    printf("%s>>%s Setup portal: %shttp://192.168.4.1/settings%s (scan/select HaLow, save & reboot).\n\n",
           DIM, RST, CYAN, RST);
    return false;
}

// #region agent log
static void debug_log(const char *hypothesis_id, const char *location, const char *message, const char *data_json)
{
    FILE *f = fopen("/Users/Gavin/esp/mm-iot-esp32/examples/halow_good/.cursor/debug.log", "a");
    if (!f) return;
    fprintf(f,
            "{\"sessionId\":\"debug-session\",\"runId\":\"pre-fix\",\"hypothesisId\":\"%s\","
            "\"location\":\"%s\",\"message\":\"%s\",\"data\":%s,\"timestamp\":%lu}\n",
            hypothesis_id, location, message, data_json,
            (unsigned long)mmosal_get_time_ms());
    fclose(f);
}
// #endregion

/** Iperf configurations. */
enum iperf_type
{
    IPERF_TCP_SERVER,   /**< TCP server (RX) */
    IPERF_UDP_SERVER,    /**< UDP server (RX) */
    IPERF_TCP_CLIENT,   /**< TCP client (TX) */
    IPERF_UDP_CLIENT,   /**< UDP client (TX) */
};

#ifndef IPERF_TYPE
/** Type of iperf instance to start. */
#define IPERF_TYPE                      IPERF_UDP_SERVER
#endif

#ifndef IPERF_SERVER_IP
/** IP address of server to connect to when in client mode. */
#define IPERF_SERVER_IP                 "192.168.1.1"
#endif

#ifndef IPERF_TIME_AMOUNT
/**
 * Duration for client transfers specified either in seconds or bytes.
 * If this is negative, it specifies a time in seconds; if positive, it
 * specifies the number of bytes to transmit.
 */
#define IPERF_TIME_AMOUNT               -30
#endif
#ifndef IPERF_UDP_TARGET_KBPS
/** UDP target bitrate (kbps) for iperf clients. */
#define IPERF_UDP_TARGET_KBPS           60000
#endif
#ifndef IPERF_SERVER_PORT
/** Specifies the port to listen on in server mode. */
#define IPERF_SERVER_PORT               5001
#endif

/* ------------------------ End of configuration options ------------------------ */

/** Array of power of 10 unit specifiers. */
static const char units[] = {' ', 'K', 'M', 'G', 'T'};

/**
 * Function to format a given number of bytes into an appropriate SI base. I.e if you give it 1400
 * it will return 1 with unit_index set to 1 for Kilo.
 *
 * @warning This uses power of 10 units (kilo, mega, giga, etc). Not to be confused with power of 2
 *          units (kibi, mebi, gibi, etc).
 *
 * @param[in]   bytes       Original number of bytes
 * @param[out]  unit_index  Index into the @ref units array. Must not be NULL
 *
 * @return Number of bytes formatted to the appropriate unit given by the unit index.
 */
static uint32_t format_bytes(uint64_t bytes, uint8_t *unit_index)
{
    MMOSAL_ASSERT(unit_index != NULL);
    *unit_index = 0;

    while (bytes >= 1000 && *unit_index < 4)
    {
        bytes /= 1000;
        (*unit_index)++;
    }

    return bytes;
}
/**
 * Handle a report at the end of an iperf transfer.
 *
 * @param report    The iperf report.
 * @param arg       Opaque argument specified when iperf was started.
 * @param handle    The iperf instance handle returned when iperf was started.
 */
static void iperf_report_handler(const struct mmiperf_report *report, void *arg,
                                 mmiperf_handle_t handle)
{
    (void)arg;
    (void)handle;

    uint8_t bytes_transferred_unit_index = 0;
    uint32_t bytes_transferred_formatted = format_bytes(report->bytes_transferred,
                                                        &bytes_transferred_unit_index);

    printf("\nIperf Report\n");
    printf("  Remote Address: %s:%d\n", report->remote_addr, report->remote_port);
    printf("  Local Address:  %s:%d\n", report->local_addr, report->local_port);
    printf("  Transferred: %lu %cBytes, duration: %lu ms, bandwidth: %lu kbps\n",
           bytes_transferred_formatted, units[bytes_transferred_unit_index],
           report->duration_ms, report->bandwidth_kbitpsec);
    printf("\n");

    if ((report->report_type == MMIPERF_UDP_DONE_SERVER) ||
        (report->report_type == MMIPERF_TCP_DONE_SERVER))
    {
        printf("Waiting for client to connect...\n");
    }
}

/** Start iperf as a TCP client. */
static void start_tcp_client(void)
{
    uint32_t server_port = IPERF_SERVER_PORT;
    struct mmiperf_client_args args = MMIPERF_CLIENT_ARGS_DEFAULT;

    /* Get the Server IP */
    strncpy(args.server_addr, IPERF_SERVER_IP, sizeof(args.server_addr));


    MMOSAL_ASSERT(server_port <= UINT16_MAX);
    args.server_port = server_port;

    int amount = IPERF_TIME_AMOUNT;
    args.amount = amount;
    if (args.amount < 0)
    {
        args.amount *= 100;
    }
    args.target_bw = IPERF_UDP_TARGET_KBPS;
    args.report_fn = iperf_report_handler;

    mmiperf_start_tcp_client(&args);
    printf("\nIperf TCP client started, waiting for completion...\n");
}

/** Start iperf as a UDP client. */
static void start_udp_client(void)
{
    uint32_t server_port = IPERF_SERVER_PORT;
    struct mmiperf_client_args args = MMIPERF_CLIENT_ARGS_DEFAULT;

    strncpy(args.server_addr, IPERF_SERVER_IP, sizeof(args.server_addr));

    MMOSAL_ASSERT(server_port <= UINT16_MAX);
    args.server_port = server_port;

    int amount = IPERF_TIME_AMOUNT;
    args.amount = amount;
    if (args.amount < 0)
    {
        args.amount *= 100;
    }
    args.report_fn = iperf_report_handler;

    mmiperf_start_udp_client(&args);
    printf("\nIperf UDP client started, waiting for completion...\n");
}

/** Start iperf as a TCP server. */
static void start_tcp_server(void)
{
    struct mmiperf_server_args args = MMIPERF_SERVER_ARGS_DEFAULT;

    uint32_t local_port = IPERF_SERVER_PORT;
    args.local_port = (uint16_t) local_port;

    args.report_fn = iperf_report_handler;

    mmiperf_handle_t iperf_handle = mmiperf_start_tcp_server(&args);
    if (iperf_handle == NULL)
    {
        printf("Failed to get local address\n");
        return;
    }
    printf("\nIperf TCP server started, waiting for client to connect...\n");
    struct mmipal_ip_config ip_config;
    enum mmipal_status status;
    status = mmipal_get_ip_config(&ip_config);
    if (status == MMIPAL_SUCCESS)
    {
        printf("Execute cmd on AP 'iperf -c %s -p %u -i 1' for IPv4\n",
               ip_config.ip_addr, args.local_port);
    }

    struct mmipal_ip6_config ip6_config;
    status = mmipal_get_ip6_config(&ip6_config);
    if (status == MMIPAL_SUCCESS)
    {
        printf("Execute cmd on AP 'iperf -c %s%%wlan0 -p %u -i 1 -V' for IPv6\n",
               ip6_config.ip6_addr[0], args.local_port);
    }
}

/** Start iperf as a UDP server. */
static void start_udp_server(void)
{
    struct mmiperf_server_args args = MMIPERF_SERVER_ARGS_DEFAULT;

    uint32_t local_port = IPERF_SERVER_PORT;
    args.local_port = (uint16_t) local_port;

    args.report_fn = iperf_report_handler;

    mmiperf_handle_t iperf_handle = mmiperf_start_udp_server(&args);
    if (iperf_handle == NULL)
    {
        printf("Failed to start iperf server\n");
        return;
    }

    printf("\nIperf UDP server started, waiting for client to connect...\n");
    struct mmipal_ip_config ip_config;
    enum mmipal_status status;
    status = mmipal_get_ip_config(&ip_config);
    if (status == MMIPAL_SUCCESS)
    {
        printf("Execute cmd on AP 'iperf -c %s -p %u -i 1 -u -b %uM' for IPv4\n",
               ip_config.ip_addr, args.local_port, (unsigned)(IPERF_UDP_TARGET_KBPS / 1000));
    }

    struct mmipal_ip6_config ip6_config;
    status = mmipal_get_ip6_config(&ip6_config);
    if (status == MMIPAL_SUCCESS)
    {
        printf("Execute cmd on AP 'iperf -c %s%%wlan0 -p %u -i 1 -V -u -b %uM' for IPv6\n",
               ip6_config.ip6_addr[0], args.local_port, (unsigned)(IPERF_UDP_TARGET_KBPS / 1000));
    }
}

/**
 * Main entry point to the application. This will be invoked in a thread once operating system
 * and hardware initialization has completed. It may return, but it does not have to.
 */
void app_main(void)
{
    // #region agent log
    debug_log("H2", "iperf.c:app_main", "console_banner_start", "{\"theme\":\"cyberpunk\"}");
    // #endregion

    printf("\n%s███████╗███████╗███╗   ██╗███████╗ ██████╗ ██████╗     %s%s\n", CYAN, MAG, RST);
    printf("%s██╔════╝██╔════╝████╗  ██║██╔════╝██╔═══██╗██╔══██╗    %s%s\n", CYAN, MAG, RST);
    printf("%s███████╗█████╗  ██╔██╗ ██║███████╗██║   ██║██████╔╝    %s%s\n", CYAN, MAG, RST);
    printf("%s╚════██║██╔══╝  ██║╚██╗██║╚════██║██║   ██║██╔══██╗    %s%s\n", CYAN, MAG, RST);
    printf("%s███████║███████╗██║ ╚████║███████║╚██████╔╝██║  ██║    %s%s\n", CYAN, MAG, RST);
    printf("%s╚══════╝╚══════╝╚═╝  ╚═══╝╚══════╝ ╚═════╝ ╚═╝  ╚═╝    %s%s\n", CYAN, MAG, RST);
    printf("%s      N E T   ::   H a L o W   G A T E W A Y   +   I P E R F%s\n", NEON, RST);
    printf("%s      2.4 GHz AP + ESP-NOW hub | cyberpunk mode online%s\n", DIM, RST);
    printf("%sVersion %s%s\n", DIM, FW_VERSION, RST);
    printf("%sBuilt %s %s%s\n\n", DIM, __DATE__, __TIME__, RST);

    // #region agent log
    debug_log("H2", "iperf.c:app_main", "console_banner_printed", "{\"banner\":\"shown\"}");
    // #endregion

    settings_init();
    time_sync_init();
    bridge_settings_t s;
    settings_load(&s);
    bool use_wifi_backhaul = (s.backhaul_mode == BACKHAUL_MODE_WIFI_2G);

    if (!use_wifi_backhaul) {
        /* 1. Initialize HaLow stack */
        app_wlan_init();

        /* 2. Attempt to connect, otherwise fall back to AP setup */
        if (!setup_halow_or_fallback()) {
            while (true) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }

        /* 3. Wait for the stack to settle */
        printf("%s>>%s Stabilizing HaLow stack (2 s)...%s\n", DIM, RST, RST);
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 4. Start the 2.4GHz AP (This function is in nat_router.c) */
        printf("\n%s>>%s Launching %s2.4 GHz AP%s\n", DIM, RST, MAG, RST);
        start_2ghz_ap();
        printf("%s>>%s Settings portal: %shttp://192.168.4.1/settings%s\n\n", DIM, RST, CYAN, RST);
    } else {
        printf("\n%s>>%s Backhaul: 2.4 GHz Wi-Fi (HaLow disabled)\n", WARN, RST);
        start_2ghz_apsta_backhaul();
        printf("%s>>%s Settings portal: %shttp://192.168.4.1/settings%s\n\n", DIM, RST, CYAN, RST);
    }

    /* 4b. Start ESP-NOW gateway on 2.4 GHz (sensor_net compatible). */
    esp_now_rcv_init();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_now_rcv_start_deferred();
    printf("%s>>%s Sensor hub: %shttp://192.168.4.1/%s\n\n", DIM, RST, CYAN, RST);

    enum iperf_type iperf_mode = IPERF_TYPE;
    bool iperf_server_allowed = s.iperf_server_enabled;
    bool iperf_is_server = (iperf_mode == IPERF_TCP_SERVER || iperf_mode == IPERF_UDP_SERVER);

    /* 4. Start the Iperf service based on the macro */
    if (iperf_is_server && !iperf_server_allowed) {
        printf("%s>>%s iperf server locked. Enable in settings to start.\n", DIM, RST);
    } else {
        switch (iperf_mode)
        {
        case IPERF_TCP_SERVER:
            start_tcp_server();
            break;
        case IPERF_UDP_SERVER:
            start_udp_server();
            break;
        case IPERF_UDP_CLIENT:
            start_udp_client();
            break;
        case IPERF_TCP_CLIENT:
            start_tcp_client();
            break;
        }
    }

    /* Keep the main task alive; avoid deleting app_main task. */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}