#ifndef CAMERA_STREAM_H
#define CAMERA_STREAM_H

#include <stdbool.h>
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Ensure mutex exists (call at boot before register_uri; camera init remains lazy). */
void camera_stream_ensure_mutex(void);

/** Initialize camera for XIAO ESP32-S3-Sense (OV2640). Returns true on success. */
bool camera_stream_init(void);

/** Register MJPEG stream handler at URI /stream on the given server.
 *  Query: ?quality=low|medium|high (default medium). Stream is capped at 20 fps. */
void camera_stream_register_uri(httpd_handle_t server);

/** Capture one JPEG frame into buf (max buf_size bytes). Sets *out_len to actual size. Returns true on success. */
bool camera_stream_get_one_jpeg(uint8_t *buf, size_t buf_size, size_t *out_len);

#ifdef __cplusplus
}
#endif

#endif
