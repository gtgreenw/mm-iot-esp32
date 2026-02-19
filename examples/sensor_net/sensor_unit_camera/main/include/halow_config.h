#ifndef HALOW_CONFIG_H
#define HALOW_CONFIG_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

bool halow_config_is_configured(void);
bool halow_config_load(char *ssid, size_t ssid_size, char *passphrase, size_t pass_size);
bool halow_config_save(const char *ssid, const char *passphrase);
void halow_config_clear(void);

bool halow_config_link_enabled(void);
bool halow_config_set_link_enabled(bool enabled);

#ifdef __cplusplus
}
#endif

#endif
