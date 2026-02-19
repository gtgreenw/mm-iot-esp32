/*
 * Time sync helper (SNTP + NVS fallback)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

void time_sync_init(void);
void time_sync_start(void);
bool time_sync_has_time(void);
int64_t time_sync_get_epoch_ms(void);
