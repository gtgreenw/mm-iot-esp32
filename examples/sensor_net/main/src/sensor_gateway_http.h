#ifndef SENSOR_GATEWAY_HTTP_H
#define SENSOR_GATEWAY_HTTP_H

#include "esp_http_server.h"

const char *sensor_gateway_get_dashboard_html(void);
void sensor_gateway_http_register(httpd_handle_t server);

#endif
