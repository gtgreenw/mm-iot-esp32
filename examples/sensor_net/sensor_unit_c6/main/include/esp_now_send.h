#ifndef ESP_NOW_SEND_H
#define ESP_NOW_SEND_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void esp_now_send_init(void);
bool esp_now_send_ready(void);
void esp_now_send_packet(void);

#ifdef __cplusplus
}
#endif

#endif
