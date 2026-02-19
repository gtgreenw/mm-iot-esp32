/*
 * Copyright 2023 Morse Micro
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WEB_CONFIG_H
#define WEB_CONFIG_H

#include "esp_http_server.h"

/** Start the settings web server (bind to default AP IP). Call after AP is up. */
httpd_handle_t start_web_config_server(void);

#endif
