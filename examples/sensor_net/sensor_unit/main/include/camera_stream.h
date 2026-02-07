#ifndef CAMERA_STREAM_H
#define CAMERA_STREAM_H

#include <stdbool.h>
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize camera for Xiao ESP32-S3-Sense (OV2640). Returns true on success. */
bool camera_stream_init(void);

/** Register MJPEG stream handler at URI /stream on the given server. */
void camera_stream_register_uri(httpd_handle_t server);

#ifdef __cplusplus
}
#endif

#endif
