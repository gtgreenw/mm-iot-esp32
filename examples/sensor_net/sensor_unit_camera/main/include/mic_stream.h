#ifndef MIC_STREAM_H
#define MIC_STREAM_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize PDM microphone (XIAO Sense: DATA=41, CLK=42). 16 kHz, 16-bit mono. Call once at startup. */
bool mic_stream_init(void);

/** Return true if mic is initialized and ready. */
bool mic_stream_ready(void);

/** Register GET /audio handler on the given HTTP server. Streams raw PCM 16-bit LE 16 kHz mono. */
void mic_stream_register_uri(void *httpd_handle);

#ifdef __cplusplus
}
#endif

#endif
