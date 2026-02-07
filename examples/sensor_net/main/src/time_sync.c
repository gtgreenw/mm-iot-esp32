/*
 * Time sync helper (SNTP + NVS fallback)
 */

#include "time_sync.h"

#include "lwip/apps/sntp.h"
#include "esp_timer.h"
#include "nvs.h"
#include <sys/time.h>
#include <time.h>

#define TIME_SYNC_NS "time"
#define TIME_SYNC_KEY_EPOCH "epoch"
#define TIME_SYNC_KEY_UPTIME "uptime_us"

#define TIME_SYNC_VALID_EPOCH 1600000000

static int64_t s_last_sync_epoch = 0;
static int64_t s_last_sync_uptime_us = 0;
static int64_t s_last_backup_uptime_us = 0;
static bool s_has_backup = false;
static bool s_started = false;

static void time_sync_save_backup(void)
{
    nvs_handle_t h;
    if (nvs_open(TIME_SYNC_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_i64(h, TIME_SYNC_KEY_EPOCH, s_last_sync_epoch);
    nvs_set_i64(h, TIME_SYNC_KEY_UPTIME, s_last_sync_uptime_us);
    nvs_commit(h);
    nvs_close(h);
}

#ifdef SNTP_SYNC_MODE_IMMED
static void time_sync_on_sync(struct timeval *tv)
{
    (void)tv;
    time_t now = time(NULL);
    if (now < TIME_SYNC_VALID_EPOCH) {
        return;
    }
    s_last_sync_epoch = (int64_t)now;
    s_last_sync_uptime_us = (int64_t)esp_timer_get_time();
    s_has_backup = true;
    time_sync_save_backup();
}
#endif

void time_sync_init(void)
{
    nvs_handle_t h;
    if (nvs_open(TIME_SYNC_NS, NVS_READONLY, &h) == ESP_OK) {
        int64_t epoch = 0;
        int64_t uptime = 0;
        if (nvs_get_i64(h, TIME_SYNC_KEY_EPOCH, &epoch) == ESP_OK &&
            nvs_get_i64(h, TIME_SYNC_KEY_UPTIME, &uptime) == ESP_OK &&
            epoch >= TIME_SYNC_VALID_EPOCH) {
            s_last_sync_epoch = epoch;
            s_last_sync_uptime_us = uptime;
            s_has_backup = true;
        }
        nvs_close(h);
    }

    setenv("TZ", "UTC0", 1);
    tzset();
}

void time_sync_start(void)
{
    if (s_started) {
        return;
    }
    s_started = true;

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
#ifdef SNTP_SYNC_MODE_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_setservername(0, "pool.ntp.org");
    sntp_setservername(1, "time.google.com");
    sntp_setservername(2, "time.cloudflare.com");
#ifdef SNTP_SYNC_MODE_IMMED
    sntp_set_time_sync_notification_cb(time_sync_on_sync);
#endif
    sntp_init();
}

bool time_sync_has_time(void)
{
    return time_sync_get_epoch_ms() >= 0;
}

int64_t time_sync_get_epoch_ms(void)
{
    struct timeval tv;
    if (gettimeofday(&tv, NULL) == 0 && tv.tv_sec >= TIME_SYNC_VALID_EPOCH) {
        int64_t now_us = (int64_t)esp_timer_get_time();
        if (s_last_backup_uptime_us == 0 || (now_us - s_last_backup_uptime_us) > 60000000LL) {
            s_last_sync_epoch = (int64_t)tv.tv_sec;
            s_last_sync_uptime_us = now_us;
            s_last_backup_uptime_us = now_us;
            s_has_backup = true;
            time_sync_save_backup();
        }
        return (int64_t)tv.tv_sec * 1000LL + (int64_t)tv.tv_usec / 1000LL;
    }
    if (!s_has_backup) {
        return -1;
    }
    int64_t elapsed_us = (int64_t)esp_timer_get_time() - s_last_sync_uptime_us;
    int64_t now_sec = s_last_sync_epoch + (elapsed_us / 1000000LL);
    return now_sec * 1000LL;
}
