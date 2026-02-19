/*
 * Weather fetch: Aviation Weather METAR API when online; cache severe in NVS for off-grid.
 * METAR provides current conditions only (no multi-day forecast).
 */
#include "weather_fetch.h"
#include "settings.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "nvs.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#define NVS_NAMESPACE     "weather"
static const char *TAG = "weather";
#define NVS_KEY_SEVERE   "severe"
#define METAR_HOST       "aviationweather.gov"
#define METAR_PATH_FMT   "/api/data/metar?ids=%s&format=json"
#define DEFAULT_STATION  "KSFO"
/* NWS API: Joplin, MO - https://forecast.weather.gov/MapClick.php?CityName=Joplin&state=MO&site=SGF&lat=37.0818&lon=-94.503
 * Use gridpoints forecast URL directly (SGF = Springfield, MO) so we make one NWS HTTPS request instead of two. */
#define NWS_FORECAST_URL "https://api.weather.gov/gridpoints/SGF/98,45/forecast"
/* NWS active alerts for Joplin, MO (lat,lon) – fog, severe, etc. */
#define NWS_ALERTS_URL   "https://api.weather.gov/alerts/active?point=37.0818,-94.503"
/* NWS requires a descriptive User-Agent with contact info (see api.weather.gov docs). */
#define NWS_USER_AGENT   "SensorGateway/1.0 (https://github.com/; embedded-weather)"

#define RESPONSE_BUF_SIZE 2048
/* NWS forecast JSON (14 periods + detailedForecast text) often exceeds 8KB. */
#define NWS_RESPONSE_BUF 24576
/* NWS alerts GeoJSON (a few alerts) – 8KB is enough. */
#define NWS_ALERTS_BUF   8192

/* Minimum free internal heap before attempting HTTPS (mbedTLS dynamic buffer ~15KB+). */
#define WEATHER_MIN_FREE_INTERNAL_HEAP  (32 * 1024)

/* Cloud cover from METAR -> WMO-like code (0–4) for dashboard icon. */
static int metar_cover_to_code(const char *cover)
{
    if (!cover) return 0;
    if (strcmp(cover, "SKC") == 0 || strcmp(cover, "CLR") == 0) return 0;
    if (strcmp(cover, "FEW") == 0) return 1;
    if (strcmp(cover, "SCT") == 0) return 2;
    if (strcmp(cover, "BKN") == 0) return 3;
    if (strcmp(cover, "OVC") == 0) return 4;
    return 0;
}

static const char *metar_cover_desc(const char *cover)
{
    if (!cover) return "—";
    if (strcmp(cover, "SKC") == 0 || strcmp(cover, "CLR") == 0) return "Clear";
    if (strcmp(cover, "FEW") == 0) return "Few clouds";
    if (strcmp(cover, "SCT") == 0) return "Partly cloudy";
    if (strcmp(cover, "BKN") == 0) return "Mostly cloudy";
    if (strcmp(cover, "OVC") == 0) return "Overcast";
    return "—";
}

/* Approximate relative humidity from temp and dewpoint (C). */
static int rh_from_temp_dew(float temp_c, float dew_c)
{
    if (temp_c < -40.f || temp_c > 60.f) return 0;
    double es_t = 6.112 * exp((17.67 * (double)temp_c) / ((double)temp_c + 243.5));
    double es_d = 6.112 * exp((17.67 * (double)dew_c) / ((double)dew_c + 243.5));
    double rh = 100.0 * (es_d / es_t);
    if (rh < 0) return 0;
    if (rh > 100) return 100;
    return (int)(rh + 0.5);
}

typedef struct {
    float temp_f;
    float dewpoint_f;
    int humidity;
    int code;
    float wind_mph;
    int wind_dir_deg;   /* degrees 0-359, or -1 for variable (VRB) */
    char desc[24];
} current_t;

typedef struct {
    char date[12];
    char name[24];   /* NWS period name e.g. "Tonight", "Sunday" */
    int code;
    float max_f;
    float min_f;
    float precip_mm;
    float wind_mph;  /* NWS windSpeed parsed (high end of range if "X to Y mph") */
    char desc[48];
    char detailed[256]; /* NWS detailedForecast (full paragraph) */
} daily_t;

typedef struct {
    char date[12];
    char desc[32];
} severe_t;

static struct {
    bool online;
    bool have_current;
    current_t current;
    daily_t daily[WEATHER_FORECAST_DAYS];
    int daily_len;
    severe_t severe[WEATHER_SEVERE_MAX];
    int severe_len;
} s_weather;

static float c2f(float c) { return c * 9.f / 5.f + 32.f; }

static int http_event(esp_http_client_event_t *evt)
{
    char **buf = evt->user_data ? (char **)evt->user_data : NULL;
    if (!buf) return ESP_OK;

    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA:
        if (!esp_http_client_is_chunked_response(evt->client) && evt->data_len > 0) {
            size_t need = (*buf ? strlen(*buf) : 0) + (size_t)evt->data_len + 1;
            if (need > RESPONSE_BUF_SIZE) need = RESPONSE_BUF_SIZE;
            if (*buf == NULL) {
                *buf = malloc(RESPONSE_BUF_SIZE);
                if (!*buf) break;
                (*buf)[0] = '\0';
            }
            size_t cur = strlen(*buf);
            if (cur + (size_t)evt->data_len < RESPONSE_BUF_SIZE) {
                memcpy(*buf + cur, evt->data, (size_t)evt->data_len);
                (*buf)[cur + (size_t)evt->data_len] = '\0';
            }
        }
        break;
    default:
        break;
    }
    return ESP_OK;
}

typedef struct {
    char *buf;
    size_t cap;
} nws_buf_t;

static int nws_http_event(esp_http_client_event_t *evt)
{
    nws_buf_t *nb = evt->user_data ? (nws_buf_t *)evt->user_data : NULL;
    if (!nb || !nb->buf || nb->cap == 0) return ESP_OK;

    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA:
        if (!esp_http_client_is_chunked_response(evt->client) && evt->data_len > 0) {
            size_t cur = strlen(nb->buf);
            size_t add = (size_t)evt->data_len;
            if (cur + add + 1 > nb->cap) add = nb->cap - cur - 1;
            if (add > 0) {
                memcpy(nb->buf + cur, evt->data, add);
                nb->buf[cur + add] = '\0';
            }
        }
        break;
    default:
        break;
    }
    return ESP_OK;
}

/* Parse Aviation Weather METAR JSON array; first element used. temp/dewp C, wspd knots. */
static void parse_metar_json(const char *json_str)
{
    cJSON *root = cJSON_ParseWithLength(json_str, json_str ? (unsigned int)strlen(json_str) : 0);
    if (!root || !cJSON_IsArray(root)) {
        if (root) cJSON_Delete(root);
        return;
    }
    cJSON *ob = cJSON_GetArrayItem(root, 0);
    if (!ob || !cJSON_IsObject(ob)) {
        cJSON_Delete(root);
        return;
    }
    cJSON *t = cJSON_GetObjectItem(ob, "temp");
    if (!t || !cJSON_IsNumber(t)) {
        cJSON_Delete(root);
        return;
    }
    s_weather.have_current = true;
    {
        float temp_c = (float)t->valuedouble;
        s_weather.current.temp_f = c2f(temp_c);
    }
    {
        cJSON *d = cJSON_GetObjectItem(ob, "dewp");
        float dew_c = (d && cJSON_IsNumber(d)) ? (float)d->valuedouble : 0.f;
        s_weather.current.dewpoint_f = c2f(dew_c);
        s_weather.current.humidity = rh_from_temp_dew((float)t->valuedouble, dew_c);
    }
    {
        cJSON *cover = cJSON_GetObjectItem(ob, "cover");
        const char *cov = (cover && cJSON_IsString(cover)) ? cover->valuestring : NULL;
        s_weather.current.code = metar_cover_to_code(cov);
        snprintf(s_weather.current.desc, sizeof(s_weather.current.desc), "%s", metar_cover_desc(cov));
    }
    {
        cJSON *wspd = cJSON_GetObjectItem(ob, "wspd");
        float knots = (wspd && cJSON_IsNumber(wspd)) ? (float)wspd->valuedouble : 0.f;
        s_weather.current.wind_mph = knots * 1.15078f; /* knots -> mph */
    }
    {
        cJSON *wdir = cJSON_GetObjectItem(ob, "wdir");
        if (wdir && cJSON_IsNumber(wdir)) {
            int d = (int)(wdir->valuedouble + 0.5);
            s_weather.current.wind_dir_deg = (d >= 0 && d <= 360) ? (d == 360 ? 0 : d) : -1;
        } else if (wdir && cJSON_IsString(wdir) && strcasecmp(wdir->valuestring, "VRB") == 0) {
            s_weather.current.wind_dir_deg = -1; /* variable */
        } else {
            s_weather.current.wind_dir_deg = -1;
        }
    }
    s_weather.daily_len = 0; /* METAR has no forecast */
    cJSON_Delete(root);
}

/* Parse NWS forecast response properties.periods[] into s_weather.daily[]. */
static void parse_nws_forecast(const char *json_str)
{
    cJSON *root = cJSON_ParseWithLength(json_str, json_str ? (unsigned int)strlen(json_str) : 0);
    if (!root || !cJSON_IsObject(root)) {
        if (root) cJSON_Delete(root);
        return;
    }
    cJSON *props = cJSON_GetObjectItem(root, "properties");
    cJSON *periods = props ? cJSON_GetObjectItem(props, "periods") : NULL;
    if (!periods || !cJSON_IsArray(periods)) {
        cJSON_Delete(root);
        return;
    }
    int n = cJSON_GetArraySize(periods);
    if (n > WEATHER_FORECAST_DAYS) n = WEATHER_FORECAST_DAYS;
    s_weather.daily_len = 0;
    for (int i = 0; i < n && s_weather.daily_len < WEATHER_FORECAST_DAYS; i++) {
        cJSON *p = cJSON_GetArrayItem(periods, i);
        if (!p || !cJSON_IsObject(p)) continue;
        daily_t *d = &s_weather.daily[s_weather.daily_len];
        memset(d, 0, sizeof(*d));
        d->wind_mph = -1.f;  /* unknown until parsed */
        cJSON *name = cJSON_GetObjectItem(p, "name");
        if (name && cJSON_IsString(name)) {
            strncpy(d->name, name->valuestring, sizeof(d->name) - 1);
            d->name[sizeof(d->name) - 1] = '\0';
        }
        cJSON *start = cJSON_GetObjectItem(p, "startTime");
        if (start && cJSON_IsString(start) && strlen(start->valuestring) >= 10) {
            strncpy(d->date, start->valuestring, 10);
            d->date[10] = '\0';
        }
        cJSON *temp = cJSON_GetObjectItem(p, "temperature");
        cJSON *unit = cJSON_GetObjectItem(p, "temperatureUnit");
        if (temp && cJSON_IsNumber(temp)) {
            float t = (float)temp->valuedouble;
            if (unit && cJSON_IsString(unit) && (unit->valuestring[0] == 'C' || unit->valuestring[0] == 'c'))
                t = c2f(t);
            d->max_f = t;
            d->min_f = t;
        }
        cJSON *short_f = cJSON_GetObjectItem(p, "shortForecast");
        if (short_f && cJSON_IsString(short_f)) {
            strncpy(d->desc, short_f->valuestring, sizeof(d->desc) - 1);
            d->desc[sizeof(d->desc) - 1] = '\0';
        }
        /* NWS windSpeed is e.g. "5 mph", "5 to 10 mph", or "Calm" - parse high value for fly-day check */
        cJSON *wind_node = cJSON_GetObjectItem(p, "windSpeed");
        if (wind_node && cJSON_IsString(wind_node)) {
            const char *ws = wind_node->valuestring;
            if (ws && (strstr(ws, "Calm") || strstr(ws, "calm")))
                d->wind_mph = 0.f;
            else {
                float v1 = -1.f, v2 = -1.f;
                if (ws && sscanf(ws, "%f to %f", &v1, &v2) == 2)
                    d->wind_mph = (v2 > v1) ? v2 : v1;
                else if (ws && sscanf(ws, "%f", &v1) == 1)
                    d->wind_mph = v1;
            }
        }
        cJSON *det = cJSON_GetObjectItem(p, "detailedForecast");
        if (det && cJSON_IsString(det)) {
            strncpy(d->detailed, det->valuestring, sizeof(d->detailed) - 1);
            d->detailed[sizeof(d->detailed) - 1] = '\0';
        }
        s_weather.daily_len++;
    }
    cJSON_Delete(root);
}

/* Parse NWS alerts GeoJSON features[].properties into s_weather.severe[] (e.g. fog, severe weather). */
static void parse_nws_alerts(const char *json_str)
{
    cJSON *root = cJSON_ParseWithLength(json_str, json_str ? (unsigned int)strlen(json_str) : 0);
    if (!root || !cJSON_IsObject(root)) {
        if (root) cJSON_Delete(root);
        return;
    }
    cJSON *features = cJSON_GetObjectItem(root, "features");
    if (!features || !cJSON_IsArray(features)) {
        cJSON_Delete(root);
        return;
    }
    int n = cJSON_GetArraySize(features);
    if (n > WEATHER_SEVERE_MAX) n = WEATHER_SEVERE_MAX;
    s_weather.severe_len = 0;
    for (int i = 0; i < n && s_weather.severe_len < WEATHER_SEVERE_MAX; i++) {
        cJSON *feat = cJSON_GetArrayItem(features, i);
        if (!feat || !cJSON_IsObject(feat)) continue;
        cJSON *props = cJSON_GetObjectItem(feat, "properties");
        if (!props || !cJSON_IsObject(props)) continue;
        /* effective or onset (ISO8601) -> date (first 10 chars YYYY-MM-DD) */
        cJSON *eff = cJSON_GetObjectItem(props, "effective");
        if (!eff || !cJSON_IsString(eff)) eff = cJSON_GetObjectItem(props, "onset");
        const char *date_str = (eff && cJSON_IsString(eff)) ? eff->valuestring : NULL;
        /* event (e.g. "Dense Fog Advisory") or headline -> desc (truncate to severe_t.desc size) */
        cJSON *ev = cJSON_GetObjectItem(props, "event");
        cJSON *head = cJSON_GetObjectItem(props, "headline");
        const char *desc_str = (ev && cJSON_IsString(ev)) ? ev->valuestring : (head && cJSON_IsString(head) ? head->valuestring : NULL);
        if (!desc_str) continue;
        severe_t *s = &s_weather.severe[s_weather.severe_len];
        if (date_str && strlen(date_str) >= 10) {
            strncpy(s->date, date_str, 10);
            s->date[10] = '\0';
        } else {
            s->date[0] = '\0';
        }
        strncpy(s->desc, desc_str, sizeof(s->desc) - 1);
        s->desc[sizeof(s->desc) - 1] = '\0';
        s_weather.severe_len++;
    }
    cJSON_Delete(root);
}

/* Parse "https://host/path" into host and path to avoid esp_http_client URL parsing bug
 * that produces a leading colon in the hostname (e.g. ":aviationweather.gov"). */
static bool parse_https_url(const char *url, char *host, size_t host_size, char *path, size_t path_size)
{
    if (!url || strncmp(url, "https://", 8) != 0 || !host || host_size < 2 || !path || path_size < 2)
        return false;
    const char *p = url + 8;
    const char *slash = strchr(p, '/');
    if (!slash) {
        strncpy(host, p, host_size - 1);
        host[host_size - 1] = '\0';
        path[0] = '/';
        path[1] = '\0';
        return true;
    }
    size_t host_len = (size_t)(slash - p);
    if (host_len >= host_size) return false;
    memcpy(host, p, host_len);
    host[host_len] = '\0';
    size_t path_len = strlen(slash);
    if (path_len >= path_size) path_len = path_size - 1;
    memcpy(path, slash, path_len + 1);
    return true;
}

/* Perform HTTP GET with User-Agent; response written into buf (pre-allocated, buf_size bytes). */
static bool http_get_with_ua(const char *url, const char *user_agent, char *buf, size_t buf_size)
{
    if (!buf || buf_size < 512) return false;
    buf[0] = '\0';
    nws_buf_t nb = { .buf = buf, .cap = buf_size };
    esp_http_client_config_t cfg = {
        .event_handler = nws_http_event,
        .user_data = &nb,
        .timeout_ms = 25000,  /* forecast response is large; allow time for read */
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 4096, /* limit TLS read buffer (helps avoid -0x7100 on large response) */
    };
    char host[128];
    char path[256];
    if (parse_https_url(url, host, sizeof(host), path, sizeof(path))) {
        cfg.host = host;
        cfg.path = path;
        cfg.port = 443;
        cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
    } else {
        cfg.url = url;
    }
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return false;
    if (user_agent)
        esp_http_client_set_header(client, "User-Agent", user_agent);
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        esp_http_client_cleanup(client);
    } else {
        /* Skip cleanup after TLS/read error: mbedtls state can be corrupted and cleanup
         * crashes in mbedtls_asn1_free_named_data_list_shallow (LoadProhibited). Leak this handle. */
        ESP_LOGW(TAG, "http_get err=0x%lx", (unsigned long)err);
        return false;
    }
    if (strlen(buf) == 0) return false;
    return true;
}

static void load_severe_from_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    size_t len = 0;
    if (nvs_get_blob(h, NVS_KEY_SEVERE, NULL, &len) == ESP_OK && len > 0 && len < 1024) {
        char *buf = malloc(len + 1);
        if (buf && nvs_get_blob(h, NVS_KEY_SEVERE, buf, &len) == ESP_OK) {
            buf[len] = '\0';
            cJSON *arr = cJSON_ParseWithLength(buf, (unsigned int)len);
            if (arr && cJSON_IsArray(arr)) {
                int n = cJSON_GetArraySize(arr);
                if (n > WEATHER_SEVERE_MAX) n = WEATHER_SEVERE_MAX;
                s_weather.severe_len = 0;
                for (int i = 0; i < n && s_weather.severe_len < WEATHER_SEVERE_MAX; i++) {
                    cJSON *o = cJSON_GetArrayItem(arr, i);
                    if (!o) continue;
                    cJSON *d = cJSON_GetObjectItem(o, "date");
                    cJSON *desc = cJSON_GetObjectItem(o, "desc");
                    if (d && cJSON_IsString(d) && desc && cJSON_IsString(desc)) {
                        strncpy(s_weather.severe[s_weather.severe_len].date, d->valuestring, sizeof(s_weather.severe[0].date) - 1);
                        s_weather.severe[s_weather.severe_len].date[sizeof(s_weather.severe[0].date) - 1] = '\0';
                        strncpy(s_weather.severe[s_weather.severe_len].desc, desc->valuestring, sizeof(s_weather.severe[0].desc) - 1);
                        s_weather.severe[s_weather.severe_len].desc[sizeof(s_weather.severe[0].desc) - 1] = '\0';
                        s_weather.severe_len++;
                    }
                }
            }
            if (arr) cJSON_Delete(arr);
        }
        free(buf);
    }
    nvs_close(h);
}

static void save_severe_to_nvs(void)
{
    if (s_weather.severe_len <= 0) return;
    cJSON *arr = cJSON_CreateArray();
    if (!arr) return;
    for (int i = 0; i < s_weather.severe_len; i++) {
        cJSON *o = cJSON_CreateObject();
        if (o) {
            cJSON_AddStringToObject(o, "date", s_weather.severe[i].date);
            cJSON_AddStringToObject(o, "desc", s_weather.severe[i].desc);
            cJSON_AddItemToArray(arr, o);
        }
    }
    char *printed = cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);
    if (!printed) return;
    size_t len = strlen(printed);
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, NVS_KEY_SEVERE, printed, len + 1);
        nvs_commit(h);
        nvs_close(h);
    }
    free(printed);
}

static void weather_refresh_task(void *pv)
{
    vTaskDelay(pdMS_TO_TICKS(3000)); /* brief delay for network init, then fetch so dashboard has data sooner */
    for (;;) {
        weather_fetch_refresh();
        vTaskDelay(pdMS_TO_TICKS(300000)); /* 5 min */
    }
}

void weather_fetch_init(void)
{
    memset(&s_weather, 0, sizeof(s_weather));
    load_severe_from_nvs();
    xTaskCreate(weather_refresh_task, "weather", 8192, NULL, 5, NULL);
}

/* Copy up to 4 uppercase letters for ICAO; fallback to DEFAULT_STATION if empty. */
static void get_metar_station(char *out, size_t out_size)
{
    bridge_settings_t st;
    settings_load(&st);
    const char *src = st.weather_metar_station;
    if (!src || !src[0]) {
        strncpy(out, DEFAULT_STATION, out_size - 1);
        out[out_size - 1] = '\0';
        return;
    }
    size_t i = 0;
    while (i < out_size - 1 && src[i] && i < 4) {
        char c = src[i];
        out[i] = (char)(isalpha((unsigned char)c) ? toupper((unsigned char)c) : c);
        i++;
        if (c == '\0') break;
    }
    out[i] = '\0';
    if (i == 0) {
        strncpy(out, DEFAULT_STATION, out_size - 1);
        out[out_size - 1] = '\0';
    }
}

int weather_forecast_count(void)
{
    return s_weather.daily_len;
}

void weather_fetch_refresh(void)
{
    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    if (free_internal < WEATHER_MIN_FREE_INTERNAL_HEAP) {
        ESP_LOGW(TAG, "Weather skipped (low memory: %u bytes free), retry in 5 min",
                 (unsigned)free_internal);
        return;
    }

    char station[8];
    get_metar_station(station, sizeof(station));
    char path[128];
    snprintf(path, sizeof(path), METAR_PATH_FMT, station);

    /* Build full URL and parse to host/path so esp_http_client never sees a URL string
     * (avoids bug that produces hostname ":aviationweather.gov" and getaddrinfo failure). */
    char metar_url[192];
    snprintf(metar_url, sizeof(metar_url), "https://%s%s", METAR_HOST, path);
    char host_buf[128];
    char path_buf[160];
    if (!parse_https_url(metar_url, host_buf, sizeof(host_buf), path_buf, sizeof(path_buf))) {
        strncpy(host_buf, METAR_HOST, sizeof(host_buf) - 1);
        host_buf[sizeof(host_buf) - 1] = '\0';
        strncpy(path_buf, path, sizeof(path_buf) - 1);
        path_buf[sizeof(path_buf) - 1] = '\0';
    }

    char *resp_buf = NULL;
    esp_http_client_config_t cfg = {
        .url = NULL,  /* use .host + .path only; avoid URL parsing bug that yields ":aviationweather.gov" */
        .host = host_buf,
        .path = path_buf,
        .port = 443,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .event_handler = http_event,
        .user_data = &resp_buf,
        .timeout_ms = 10000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        s_weather.online = false;
        return;
    }
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        esp_http_client_cleanup(client);
    }
    /* On error skip cleanup: TLS state can be corrupted and cleanup crashes in mbedtls (LoadProhibited). Leak this handle. */

    if (err != ESP_OK || !resp_buf) {
        s_weather.online = false;
        if (resp_buf) free(resp_buf);
        return;
    }
    s_weather.online = true;
    parse_metar_json(resp_buf);
    save_severe_to_nvs();
    free(resp_buf);

    /* Delay so TLS context is fully freed before NWS (reduces -0x7F00 / -0x7100) */
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* Skip NWS if internal heap dropped (e.g. camera/Video tab); avoids "Dynamic Impl: alloc failed" */
    if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < WEATHER_MIN_FREE_INTERNAL_HEAP) {
        ESP_LOGW(TAG, "NWS skipped (low memory), retry in 5 min");
    } else {
    /* NWS extended forecast for Joplin, MO – single request to gridpoints URL (no points request) */
    static char nws_buf[NWS_RESPONSE_BUF];
    nws_buf[0] = '\0';
    if (http_get_with_ua(NWS_FORECAST_URL, NWS_USER_AGENT, nws_buf, sizeof(nws_buf))) {
        parse_nws_forecast(nws_buf);
        ESP_LOGI(TAG, "NWS forecast ok, %d periods (Joplin MO)", s_weather.daily_len);
    } else {
        ESP_LOGW(TAG, "NWS forecast failed, retrying in 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
        nws_buf[0] = '\0';
        if (http_get_with_ua(NWS_FORECAST_URL, NWS_USER_AGENT, nws_buf, sizeof(nws_buf))) {
            parse_nws_forecast(nws_buf);
            ESP_LOGI(TAG, "NWS forecast ok (retry), %d periods", s_weather.daily_len);
        } else {
            ESP_LOGW(TAG, "NWS forecast retry failed");
        }
    }

    /* NWS active alerts (fog, severe weather, etc.) for Joplin – populate severe_cached for dashboard */
    static char alerts_buf[NWS_ALERTS_BUF];
    alerts_buf[0] = '\0';
    if (http_get_with_ua(NWS_ALERTS_URL, NWS_USER_AGENT, alerts_buf, sizeof(alerts_buf))) {
        parse_nws_alerts(alerts_buf);
        if (s_weather.severe_len > 0) {
            save_severe_to_nvs();
            ESP_LOGI(TAG, "NWS alerts ok, %d active (e.g. fog)", s_weather.severe_len);
        }
    }
    }
}

static void json_escape_str(const char *in, char *out, size_t out_size)
{
    size_t j = 0;
    for (; in && *in && j < out_size - 2; in++) {
        if (*in == '"' || *in == '\\') { out[j++] = '\\'; out[j++] = *in; }
        else if (*in >= 32 && *in < 127) out[j++] = *in;
    }
    out[j] = '\0';
}

int weather_get_api_json(char *buf, size_t size)
{
    int n = 0;
    char station[8];
    get_metar_station(station, sizeof(station));
    n += snprintf(buf + n, (size > (size_t)n) ? size - (size_t)n : 0, "{\"online\":%s,\"station\":\"%s\"",
        s_weather.online ? "true" : "false", station);
    if (s_weather.online && s_weather.have_current) {
        n += snprintf(buf + n, (size > (size_t)n) ? size - (size_t)n : 0,
            ",\"current\":{\"temp_f\":%.1f,\"dewpoint_f\":%.1f,\"humidity\":%d,\"code\":%d,\"wind_mph\":%.1f,\"wind_dir_deg\":%d,\"desc\":\"%s\"}",
            s_weather.current.temp_f, s_weather.current.dewpoint_f, s_weather.current.humidity, s_weather.current.code,
            s_weather.current.wind_mph, s_weather.current.wind_dir_deg, s_weather.current.desc);
    } else {
        n += snprintf(buf + n, (size > (size_t)n) ? size - (size_t)n : 0, ",\"current\":null");
    }
    n += snprintf(buf + n, (size > (size_t)n) ? size - (size_t)n : 0, ",\"forecast\":[");
    for (int i = 0; i < s_weather.daily_len; i++) {
        if (i) n += snprintf(buf + n, (size > (size_t)n) ? size - (size_t)n : 0, ",");
        char edesc[64];
        json_escape_str(s_weather.daily[i].desc, edesc, sizeof(edesc));
        char ename[32];
        json_escape_str(s_weather.daily[i].name, ename, sizeof(ename));
        n += snprintf(buf + n, (size > (size_t)n) ? size - (size_t)n : 0,
            "{\"date\":\"%s\",\"name\":\"%s\",\"code\":%d,\"max_f\":%.1f,\"min_f\":%.1f,\"precip_mm\":%.1f,\"wind_mph\":%.1f,\"desc\":\"%s\"}",
            s_weather.daily[i].date, ename, s_weather.daily[i].code, s_weather.daily[i].max_f, s_weather.daily[i].min_f,
            s_weather.daily[i].precip_mm, s_weather.daily[i].wind_mph, edesc);
    }
    n += snprintf(buf + n, (size > (size_t)n) ? size - (size_t)n : 0, "],\"severe_cached\":[");
    for (int i = 0; i < s_weather.severe_len; i++) {
        if (i) n += snprintf(buf + n, (size > (size_t)n) ? size - (size_t)n : 0, ",");
        char ed[64];
        json_escape_str(s_weather.severe[i].desc, ed, sizeof(ed));
        n += snprintf(buf + n, (size > (size_t)n) ? size - (size_t)n : 0, "{\"date\":\"%s\",\"desc\":\"%s\"}", s_weather.severe[i].date, ed);
    }
    n += snprintf(buf + n, (size > (size_t)n) ? size - (size_t)n : 0, "]}");
    if (n < 0 || (size_t)n >= size) return -1;
    return n;
}
