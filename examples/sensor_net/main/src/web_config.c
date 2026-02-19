/*
 * Copyright 2023 Morse Micro
 * SPDX-License-Identifier: Apache-2.0
 *
 * Web interface to view and change HaLow / 2.4GHz AP settings. Connect to the device's AP
 * and open http://192.168.4.1
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "mmwlan.h"
#include "mmosal.h"
#include "settings.h"
#include "sensor_gateway_http.h"

static const char *TAG = "web_config";

#define MAX_FORM_FIELD 64
#define WEB_CONFIG_STACK_SIZE 8192

/* HaLow scan results (kept in RAM). */
#define HALOW_SCAN_MAX_RESULTS 16
typedef struct {
    char ssid[MMWLAN_SSID_MAXLEN + 1];
    int16_t rssi;
    uint8_t op_bw_mhz;
} halow_scan_entry_t;

static halow_scan_entry_t s_scan_results[HALOW_SCAN_MAX_RESULTS];
static int s_scan_count = 0;
static struct mmosal_semb *s_scan_done = NULL;

// #region agent log
static void debug_log(const char *hypothesis_id, const char *location, const char *message, const char *data_json)
{
    FILE *f = fopen("/Users/Gavin/esp/mm-iot-esp32/examples/halow_good/.cursor/debug.log", "a");
    if (!f) return;
    fprintf(f,
            "{\"sessionId\":\"debug-session\",\"runId\":\"pre-fix\",\"hypothesisId\":\"%s\","
            "\"location\":\"%s\",\"message\":\"%s\",\"data\":%s,\"timestamp\":%lu}\n",
            hypothesis_id, location, message, data_json,
            (unsigned long)mmosal_get_time_ms());
    fclose(f);
}
// #endregion

/* Escape string for HTML attribute value (replace & " < >) */
static void html_escape(const char *in, char *out, size_t out_size)
{
    size_t n = 0;
    while (*in && n + 6 < out_size) {
        switch (*in) {
        case '&':  n += snprintf(out + n, out_size - n, "&amp;");  break;
        case '"':  n += snprintf(out + n, out_size - n, "&quot;"); break;
        case '<':  n += snprintf(out + n, out_size - n, "&lt;");   break;
        case '>':  n += snprintf(out + n, out_size - n, "&gt;");   break;
        default:   out[n++] = *in; break;
        }
        in++;
    }
    out[n] = '\0';
}

/* Escape string for JSON value (replace " and \\) */
static void json_escape(const char *in, char *out, size_t out_size)
{
    size_t j = 0;
    for (; in && *in && j < out_size - 2; in++) {
        if (*in == '"' || *in == '\\') { out[j++] = '\\'; out[j++] = *in; }
        else if (*in >= 32 && *in < 127) out[j++] = *in;
    }
    out[j] = '\0';
}

/* Parse one application/x-www-form-urlencoded field into buf, max len. Returns pointer past this field. */
static const char *parse_form_field(const char *form, const char *name, char *buf, size_t buf_len)
{
    size_t name_len = strlen(name);
    const char *p = form;
    while (*p) {
        if (strncmp(p, name, name_len) == 0 && p[name_len] == '=') {
            p += name_len + 1;
            size_t i = 0;
            while (*p && *p != '&' && i < buf_len - 1) {
                if (*p == '%' && p[1] && p[2]) {
                    int hi = p[1], lo = p[2];
                    buf[i++] = (char)((hi >= 'A' ? (hi & 0x1f) + 9 : hi - '0') * 16 +
                                   (lo >= 'A' ? (lo & 0x1f) + 9 : lo - '0'));
                    p += 3;
                } else if (*p == '+') {
                    buf[i++] = ' ';
                    p++;
                } else {
                    buf[i++] = *p++;
                }
            }
            buf[i] = '\0';
            return p;
        }
        while (*p && *p != '&') p++;
        if (*p == '&') p++;
    }
    buf[0] = '\0';
    return form;
}

static esp_err_t get_handler(httpd_req_t *req)
{
    bridge_settings_t s;
    settings_load(&s);

    // #region agent log
    debug_log("H1", "web_config.c:get_handler", "render_settings_page", "{\"theme\":\"cyberpunk\"}");
    // #endregion

    char h_ssid[SETTINGS_MAX_SSID * 6], h_pass[SETTINGS_MAX_PASS * 6];
    char bh_ssid[SETTINGS_MAX_SSID * 6], bh_pass[SETTINGS_MAX_PASS * 6];
    char a_ssid[SETTINGS_MAX_SSID * 6], a_pass[SETTINGS_MAX_PASS * 6];
    char h_country[SETTINGS_MAX_COUNTRY * 6];
    html_escape(s.halow_ssid, h_ssid, sizeof(h_ssid));
    html_escape(s.halow_pass, h_pass, sizeof(h_pass));
    html_escape(s.wifi_backhaul_ssid, bh_ssid, sizeof(bh_ssid));
    html_escape(s.wifi_backhaul_pass, bh_pass, sizeof(bh_pass));
    html_escape(s.ap_ssid, a_ssid, sizeof(a_ssid));
    html_escape(s.ap_pass, a_pass, sizeof(a_pass));
    html_escape(s.country, h_country, sizeof(h_country));
    const char *iperf_checked = s.iperf_server_enabled ? " checked" : "";
    const char *backhaul_halow_selected = (s.backhaul_mode == BACKHAUL_MODE_HALOW) ? " selected" : "";
    const char *backhaul_wifi_selected = (s.backhaul_mode == BACKHAUL_MODE_WIFI_2G) ? " selected" : "";

    const char *saved = "";
    if (httpd_req_get_url_query_str(req, NULL, 0) > 0) {
        char buf[16];
        if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK &&
            strstr(buf, "saved=1") != NULL) {
            saved = "<p class=\"msg\">Settings saved. Reboot to apply Wi-Fi changes.</p>";
        }
    }

    /* Static buffer avoids heap allocation and format-truncation warnings. */
    static char html_buf[14336];
    size_t html_len = sizeof(html_buf);
    int n = snprintf(html_buf, html_len,
        "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>HaLow Bridge Settings</title>"
        "<style>"
        "*{box-sizing:border-box;}"
        "body{"
        "--bg:#050b08;--text:#d6ffe8;--muted:#9affc5;--accent:#36ff7a;"
        "--border:#1e293b;--panel:#0d1411;--panel-alt:#0b120f;"
        "--accent-soft:rgba(54,255,122,0.1);--glow:rgba(54,255,122,0.35);"
        "font-family:'Consolas','Monaco','Courier New',monospace;"
        "background:var(--bg);color:var(--text);"
        "max-width:520px;margin:1.5em auto;padding:1.5em;min-height:100vh;}"
        "body::before{content:'';position:fixed;top:0;left:0;right:0;height:2px;"
        "background:linear-gradient(90deg,var(--accent),var(--accent),var(--accent));"
        "background-size:200%% 100%%;opacity:0.85;z-index:1;animation:scanline 6s linear infinite;}"
        "@keyframes scanline{0%%{background-position:0%% 50%%}100%%{background-position:200%% 50%%}}"
        "body[data-skin='modern']{"
        "--bg:#f4f6f8;--text:#0f172a;--muted:#475569;--accent:#0f172a;"
        "--border:#cbd5e1;--panel:#fff;--panel-alt:#eef2f7;"
        "--accent-soft:rgba(15,23,42,0.12);--glow:rgba(15,23,42,0.2);"
        "font-family:system-ui,-apple-system,sans-serif;}"
        "body[data-skin='modern']::before{display:none;}"
        "body[data-skin='modern'] .cyberpunk-art{display:none;}"
        "body[data-skin='pink']{"
        "--bg:#0d080b;--text:#fce7f3;--muted:#f9a8d4;--accent:#ec4899;"
        "--border:#3f1d2e;--panel:#1a0d14;--panel-alt:#140a10;"
        "--accent-soft:rgba(236,72,153,0.15);--glow:rgba(236,72,153,0.4);}"
        "body[data-skin='rwb']{"
        "--bg:#f8fafc;--text:#0f172a;--muted:#475569;--accent:#1e40af;"
        "--border:#cbd5e1;--panel:#fff;--panel-alt:#f1f5f9;"
        "--accent-soft:rgba(30,64,175,0.12);--glow:rgba(30,64,175,0.25);"
        "font-family:system-ui,-apple-system,sans-serif;}"
        "body[data-skin='rwb']::before{display:none;}"
        "body[data-skin='rwb'] .cyberpunk-art{display:none;}"
        "body[data-skin='blush']{"
        "--bg:#fdf8fa;--text:#374151;--muted:#9ca3af;--accent:#e8b4bc;"
        "--border:#e5e7eb;--panel:#fff;--panel-alt:#f9fafb;"
        "--accent-soft:rgba(232,180,188,0.2);--glow:rgba(232,180,188,0.35);"
        "font-family:system-ui,-apple-system,sans-serif;}"
        "body[data-skin='blush']::before{display:none;}"
        "body[data-skin='blush'] .cyberpunk-art{display:none;}"
        "h1{font-size:1.35em;font-weight:600;letter-spacing:0.08em;text-transform:uppercase;"
        "color:var(--accent);text-shadow:0 0 12px var(--glow);margin-bottom:1.2em;"
        "border-bottom:1px solid var(--accent-soft);padding-bottom:0.5em;}"
        ".subtitle{margin:-0.8em 0 1.2em;color:var(--muted);font-size:0.85em;letter-spacing:0.04em;}"
        ".nav{margin:-0.5em 0 1.2em;}"
        ".nav a{display:inline-block;padding:0.4em 0.75em;border:1px solid var(--accent);border-radius:4px;"
        "color:var(--accent);letter-spacing:0.05em;text-decoration:none;}"
        ".nav a:hover{box-shadow:0 0 14px var(--glow);}"
        "label{display:block;margin-top:1em;font-size:0.9em;color:var(--muted);letter-spacing:0.04em;}"
        "input,select{width:100%%;padding:0.6em 0.75em;background:var(--panel);border:1px solid var(--border);"
        "border-radius:4px;color:var(--text);font-family:inherit;font-size:0.95em;"
        "transition:border-color 0.2s,box-shadow 0.2s;}"
        "input:focus,select:focus{outline:none;border-color:var(--accent);"
        "box-shadow:0 0 0 2px var(--accent-soft),0 0 12px var(--accent-soft);}"
        "input[type=number]{width:5em;}"
        ".scan-list{margin-top:0.75em;display:grid;gap:6px;}"
        ".scan-item{display:flex;align-items:center;justify-content:space-between;padding:0.5em 0.6em;"
        "border:1px solid var(--border);border-radius:4px;background:var(--panel-alt);"
        "box-shadow:0 0 10px var(--accent-soft);}"
        ".scan-item button{margin:0;padding:0.25em 0.5em;font-size:0.85em;}"
        ".msg{background:linear-gradient(135deg,var(--accent-soft),rgba(0,0,0,0.08));"
        "border:1px solid var(--accent-soft);color:var(--accent);padding:0.6em 0.75em;"
        "border-radius:4px;font-size:0.9em;margin-bottom:0.5em;text-shadow:0 0 8px var(--glow);}"
        "button{background:var(--panel);color:var(--accent);border:1px solid var(--accent);"
        "padding:0.5em 1em;margin-top:1em;margin-right:0.5em;font-family:inherit;font-size:0.9em;"
        "letter-spacing:0.05em;cursor:pointer;border-radius:4px;"
        "transition:box-shadow 0.2s,background 0.2s;}"
        "button:hover{box-shadow:0 0 14px var(--glow),inset 0 0 14px var(--accent-soft);"
        "background:var(--accent-soft);}"
        "a{color:var(--accent);text-decoration:none;}"
        "a button{border-color:var(--accent);color:var(--accent);}"
        "a button:hover{box-shadow:0 0 14px var(--glow);}"
        ".theme-row{margin:0.8em 0;display:flex;align-items:center;gap:10px;flex-wrap:wrap;}"
        ".theme-row label{display:inline;margin:0;font-size:0.85em;}"
        ".theme-row select{width:auto;min-width:120px;}"
        "body[data-font-size='small']{font-size:87.5%%;}body[data-font-size='medium']{font-size:100%%;}body[data-font-size='large']{font-size:112.5%%;}body[data-font-size='xlarge']{font-size:125%%;}"
        ".cyberpunk-art{position:relative;margin:1em 0 1.2em;padding:14px;border:1px solid var(--border);"
        "border-radius:6px;background:linear-gradient(135deg,var(--accent-soft),rgba(0,0,0,0));overflow:hidden;}"
        ".cyberpunk-art::before{content:'';position:absolute;inset:0;"
        "background:repeating-linear-gradient(135deg,transparent 0,transparent 10px,var(--accent-soft) 10px,var(--accent-soft) 12px);}"
        ".cyberpunk-art::after{content:'';position:absolute;right:-30px;top:-30px;width:120px;height:120px;"
        "border:2px solid var(--accent);border-radius:50%%;box-shadow:0 0 18px var(--glow);}"
        ".cyberpunk-art .art-grid{position:absolute;inset:0;"
        "background:linear-gradient(transparent 75%%,var(--accent-soft) 75%%),"
        "linear-gradient(90deg,transparent 75%%,var(--accent-soft) 75%%);"
        "background-size:24px 24px;mix-blend-mode:screen;opacity:0.6;}"
        ".cyberpunk-art .art-title{position:relative;font-size:0.8em;letter-spacing:0.3em;"
        "text-transform:uppercase;color:var(--muted);text-shadow:0 0 10px var(--glow);}"
        "</style></head><body>"
        "<h1>HaLow Bridge Settings</h1>"
        "<p class=\"subtitle\">Version 1.0.1</p>"
        "<div class=\"nav\"><a href=\"/\">Back to Dashboard</a></div>"
        "<div class=\"theme-row\">"
        "<label for=\"skinSelect\">Theme</label>"
        "<select id=\"skinSelect\">"
        "<option value=\"cyberpunk\">Cyberpunk</option>"
        "<option value=\"modern\">Modern</option>"
        "<option value=\"pink\">Pink</option>"
        "<option value=\"rwb\">Red White Blue</option>"
        "<option value=\"blush\">Blush</option>"
        "</select>"
        "<label for=\"fontSizeSelect\" style=\"margin-left:1em\">Font size</label>"
        "<select id=\"fontSizeSelect\">"
        "<option value=\"small\">Small</option>"
        "<option value=\"medium\">Medium</option>"
        "<option value=\"large\">Large</option>"
        "<option value=\"xlarge\">X-Large</option>"
        "</select>"
        "</div>"
        "<div class=\"cyberpunk-art\" aria-hidden=\"true\"><div class=\"art-grid\"></div><div class=\"art-title\">SYSTEM CORE</div></div>"
        "%s"
        "<div style=\"margin-top:1em\">"
        "<button type=\"button\" id=\"btnScan\">Scan HaLow networks</button>"
        "<div id=\"scanStatus\" style=\"margin-top:0.5em;color:#9affc5\"></div>"
        "<div id=\"scanList\" class=\"scan-list\"></div>"
        "</div>"
        "<form method=\"post\" action=\"/save\">"
        "<label>Backhaul mode"
        "<select name=\"backhaul_mode\">"
        "<option value=\"0\"%s>HaLow (default)</option>"
        "<option value=\"1\"%s>2.4&nbsp;GHz Wi-Fi (disable HaLow)</option>"
        "</select></label>"
        "<div style=\"margin-top:0.35em;color:#9affc5;font-size:0.85em;\">"
        "Note: AP/ESP-NOW use the same channel as the 2.4&nbsp;GHz backhaul; sensors scan for the gateway channel."
        "</div>"
        "<label>HaLow AP (STA) SSID <input name=\"halow_ssid\" value=\"%s\" maxlength=\"%d\"></label>"
        "<label>HaLow passphrase <input type=\"password\" name=\"halow_pass\" value=\"%s\" maxlength=\"%d\"></label>"
        "<div id=\"wifiBhFields\" style=\"display:none\">"
        "<label>2.4&nbsp;GHz backhaul SSID <input name=\"wifi_bh_ssid\" value=\"%s\" maxlength=\"%d\"></label>"
        "<label>2.4&nbsp;GHz backhaul password <input type=\"password\" name=\"wifi_bh_pass\" value=\"%s\" maxlength=\"%d\"></label>"
        "</div>"
        "<label>2.4&nbsp;GHz AP SSID <input name=\"ap_ssid\" value=\"%s\" maxlength=\"%d\"></label>"
        "<label>2.4&nbsp;GHz AP password <input type=\"password\" name=\"ap_pass\" value=\"%s\" maxlength=\"%d\"></label>"
        "<label>2.4&nbsp;GHz TX power (dBm) <input type=\"number\" name=\"ap_tx_power\" value=\"%d\" min=\"2\" max=\"20\" size=\"3\"></label>"
        "<label><input type=\"checkbox\" name=\"iperf_server\" value=\"1\"%s> Enable iperf server</label>"
        "<label>Country code (e.g. US) <input name=\"country\" value=\"%s\" maxlength=\"%d\" size=\"4\"></label>"
        "<button type=\"submit\">Save</button>"
        "<button type=\"button\" id=\"btnSaveReboot\">Save &amp; Reboot</button>"
        "</form>"
        "<a href=\"/reboot\"><button type=\"button\">Reboot device</button></a>"
        "<script>"
        "var scanBtn=document.getElementById('btnScan');"
        "var scanStatus=document.getElementById('scanStatus');"
        "var scanList=document.getElementById('scanList');"
        "scanBtn.onclick=function(){scanStatus.textContent='Scanning...';scanList.innerHTML='';"
        "fetch('/api/halow/scan').then(function(r){return r.json();}).then(function(j){"
        "var list=j.results||[];if(list.length===0){scanStatus.textContent='No networks found.';return;}"
        "list.sort(function(a,b){return Number(b.rssi)-Number(a.rssi);});"
        "scanStatus.textContent='Select a network (strongest first):';"
        "scanList.innerHTML=list.map(function(n){"
        "return '<div class=\"scan-item\"><div>'+n.ssid+' <span style=\"color:#9affc5\">('+n.rssi+' dBm)</span></div>'"
        "+'<button type=\"button\" data-ssid=\"'+n.ssid.replace(/\"/g,'&quot;')+'\">Use</button></div>';}).join('');"
        "scanList.querySelectorAll('button').forEach(function(b){b.onclick=function(){"
        "document.querySelector('input[name=halow_ssid]').value=this.dataset.ssid;};});"
        "}).catch(function(){scanStatus.textContent='Scan failed.';});};"
        "document.getElementById('btnSaveReboot').onclick=function(){"
        "var form=document.querySelector('form');"
        "fetch('/save_reboot',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},"
        "body:new URLSearchParams(new FormData(form))}).then(function(){"
        "scanStatus.textContent='Saved. Rebooting...';});};"
        "var bhSelect=document.querySelector('select[name=backhaul_mode]');"
        "var bhFields=document.getElementById('wifiBhFields');"
        "function updateBhFields(){bhFields.style.display=(bhSelect&&bhSelect.value==='1')?'block':'none';}"
        "if(bhSelect){bhSelect.onchange=updateBhFields;updateBhFields();}"
        "fetch('/api/ui_skin').then(function(r){return r.json();}).then(function(j){"
        "if(j&&j.skin){document.body.setAttribute('data-skin',j.skin);"
        "var ss=document.getElementById('skinSelect');if(ss)ss.value=j.skin;}"
        "var fs=(j&&j.font_size)||'medium';document.body.setAttribute('data-font-size',fs);"
        "var fss=document.getElementById('fontSizeSelect');if(fss)fss.value=fs;"
        "}).catch(function(){});"
        "function saveUiSkin(){"
        "var skin=document.getElementById('skinSelect');skin=skin?skin.value:'cyberpunk';"
        "var fs=document.getElementById('fontSizeSelect');fs=fs?fs.value:'medium';"
        "document.body.setAttribute('data-skin',skin);document.body.setAttribute('data-font-size',fs);"
        "fetch('/api/ui_skin',{method:'POST',headers:{'Content-Type':'application/json'},"
        "body:JSON.stringify({skin:skin,font_size:fs})});}"
        "document.getElementById('skinSelect').onchange=saveUiSkin;"
        "document.getElementById('fontSizeSelect').onchange=saveUiSkin;"
        "</script>"
        "</body></html>",
        saved,
        backhaul_halow_selected,
        backhaul_wifi_selected,
        h_ssid, SETTINGS_MAX_SSID - 1,
        h_pass, SETTINGS_MAX_PASS - 1,
        bh_ssid, SETTINGS_MAX_SSID - 1,
        bh_pass, SETTINGS_MAX_PASS - 1,
        a_ssid, SETTINGS_MAX_SSID - 1,
        a_pass, SETTINGS_MAX_PASS - 1,
        (int)s.ap_tx_power_dbm,
        iperf_checked,
        h_country, SETTINGS_MAX_COUNTRY - 1
    );
    if (n < 0 || (size_t)n >= html_len) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Response too long");
        return ESP_FAIL;
    }

    // #region agent log
    {
        char data_buf[64];
        snprintf(data_buf, sizeof(data_buf), "{\"html_len\":%u}", (unsigned)n);
        debug_log("H1", "web_config.c:get_handler", "render_complete", data_buf);
    }
    // #endregion

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_buf, (ssize_t)n);
    return ESP_OK;
}

static bool save_settings_from_request(httpd_req_t *req, bool send_redirect)
{
    if (req->content_len <= 0 || req->content_len > 1024) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad request");
        return false;
    }
    size_t buf_size = (size_t)req->content_len + 1;
    /* Prefer PSRAM for form buffer to avoid fragmenting internal heap */
    char *buf = (char *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = (char *)malloc(buf_size);
    }
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return false;
    }
    int r = httpd_req_recv(req, buf, req->content_len);
    if (r <= 0) {
        free(buf);
        return false;
    }
    buf[r] = '\0';

    bridge_settings_t s;
    settings_load(&s);

    char tmp[MAX_FORM_FIELD];
    parse_form_field(buf, "backhaul_mode", tmp, sizeof(tmp));
    if (tmp[0]) {
        if (strcmp(tmp, "1") == 0) {
            s.backhaul_mode = BACKHAUL_MODE_WIFI_2G;
        } else {
            s.backhaul_mode = BACKHAUL_MODE_HALOW;
        }
    }
    parse_form_field(buf, "halow_ssid", tmp, sizeof(tmp));
    if (tmp[0]) strncpy(s.halow_ssid, tmp, SETTINGS_MAX_SSID - 1), s.halow_ssid[SETTINGS_MAX_SSID - 1] = '\0';
    parse_form_field(buf, "halow_pass", tmp, sizeof(tmp));
    if (tmp[0]) strncpy(s.halow_pass, tmp, SETTINGS_MAX_PASS - 1), s.halow_pass[SETTINGS_MAX_PASS - 1] = '\0';
    parse_form_field(buf, "wifi_bh_ssid", tmp, sizeof(tmp));
    if (tmp[0]) strncpy(s.wifi_backhaul_ssid, tmp, SETTINGS_MAX_SSID - 1), s.wifi_backhaul_ssid[SETTINGS_MAX_SSID - 1] = '\0';
    parse_form_field(buf, "wifi_bh_pass", tmp, sizeof(tmp));
    if (tmp[0]) strncpy(s.wifi_backhaul_pass, tmp, SETTINGS_MAX_PASS - 1), s.wifi_backhaul_pass[SETTINGS_MAX_PASS - 1] = '\0';
    parse_form_field(buf, "ap_ssid", tmp, sizeof(tmp));
    if (tmp[0]) strncpy(s.ap_ssid, tmp, SETTINGS_MAX_SSID - 1), s.ap_ssid[SETTINGS_MAX_SSID - 1] = '\0';
    parse_form_field(buf, "ap_pass", tmp, sizeof(tmp));
    if (tmp[0]) strncpy(s.ap_pass, tmp, SETTINGS_MAX_PASS - 1), s.ap_pass[SETTINGS_MAX_PASS - 1] = '\0';
    parse_form_field(buf, "ap_tx_power", tmp, sizeof(tmp));
    if (tmp[0]) {
        int v = atoi(tmp);
        if (v >= 2 && v <= 20) s.ap_tx_power_dbm = (int8_t)v;
    }
    s.iperf_server_enabled = false;
    parse_form_field(buf, "iperf_server", tmp, sizeof(tmp));
    if (tmp[0] && (strcmp(tmp, "1") == 0 || strcmp(tmp, "on") == 0)) {
        s.iperf_server_enabled = true;
    }
    parse_form_field(buf, "country", tmp, sizeof(tmp));
    if (tmp[0]) strncpy(s.country, tmp, SETTINGS_MAX_COUNTRY - 1), s.country[SETTINGS_MAX_COUNTRY - 1] = '\0';

    free(buf);

    if (!settings_save(&s)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Save failed");
        return false;
    }
    ESP_LOGI(TAG, "Settings saved via web");

    if (send_redirect) {
        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", "/settings?saved=1");
        httpd_resp_send(req, NULL, 0);
    }
    return true;
}

static esp_err_t save_post_handler(httpd_req_t *req)
{
    return save_settings_from_request(req, true) ? ESP_OK : ESP_FAIL;
}

static esp_err_t save_reboot_post_handler(httpd_req_t *req)
{
    if (!save_settings_from_request(req, false)) {
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", 10);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

static esp_err_t reboot_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "Rebooting...", 13);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

static void scan_rx_callback(const struct mmwlan_scan_result *result, void *arg)
{
    (void)arg;
    if (!result || result->ssid_len == 0) return;
    char ssid_buf[MMWLAN_SSID_MAXLEN + 1];
    size_t n = result->ssid_len;
    if (n > MMWLAN_SSID_MAXLEN) n = MMWLAN_SSID_MAXLEN;
    memcpy(ssid_buf, result->ssid, n);
    ssid_buf[n] = '\0';

    for (int i = 0; i < s_scan_count; i++) {
        if (strcmp(s_scan_results[i].ssid, ssid_buf) == 0) {
            if (result->rssi > s_scan_results[i].rssi) {
                s_scan_results[i].rssi = result->rssi;
                s_scan_results[i].op_bw_mhz = result->op_bw_mhz;
            }
            return;
        }
    }

    if (s_scan_count >= HALOW_SCAN_MAX_RESULTS) return;
    strncpy(s_scan_results[s_scan_count].ssid, ssid_buf, sizeof(s_scan_results[0].ssid) - 1);
    s_scan_results[s_scan_count].ssid[sizeof(s_scan_results[0].ssid) - 1] = '\0';
    s_scan_results[s_scan_count].rssi = result->rssi;
    s_scan_results[s_scan_count].op_bw_mhz = result->op_bw_mhz;
    s_scan_count++;
}

static void scan_complete_callback(enum mmwlan_scan_state state, void *arg)
{
    (void)state;
    (void)arg;
    if (s_scan_done) mmosal_semb_give(s_scan_done);
}

static esp_err_t handler_get_halow_scan(httpd_req_t *req)
{
    if (s_scan_done == NULL) {
        s_scan_done = mmosal_semb_create("scan_done");
    }
    s_scan_count = 0;
    memset(s_scan_results, 0, sizeof(s_scan_results));

    struct mmwlan_scan_req scan_req = MMWLAN_SCAN_REQ_INIT;
    scan_req.scan_rx_cb = scan_rx_callback;
    scan_req.scan_complete_cb = scan_complete_callback;

    enum mmwlan_status status = mmwlan_scan_request(&scan_req);
    if (status != MMWLAN_SUCCESS) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Scan start failed");
        return ESP_FAIL;
    }

    /* Wait up to 8 seconds for scan completion. */
    mmosal_semb_wait(s_scan_done, 8000);

    char buf[2048];
    int len = snprintf(buf, sizeof(buf), "{\"results\":[");
    for (int i = 0; i < s_scan_count && len < (int)sizeof(buf) - 96; i++) {
        char esc[MMWLAN_SSID_MAXLEN * 2];
        json_escape(s_scan_results[i].ssid, esc, sizeof(esc));
        len += snprintf(buf + len, sizeof(buf) - len,
                        "%s{\"ssid\":\"%s\",\"rssi\":%d,\"op_bw\":%u}",
                        i ? "," : "", esc, (int)s_scan_results[i].rssi,
                        (unsigned)s_scan_results[i].op_bw_mhz);
    }
    len += snprintf(buf + len, sizeof(buf) - len, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

static httpd_uri_t uri_get = {
    .uri       = "/settings",
    .method    = HTTP_GET,
    .handler   = get_handler,
};

static httpd_uri_t uri_save = {
    .uri       = "/save",
    .method    = HTTP_POST,
    .handler   = save_post_handler,
};

static httpd_uri_t uri_save_reboot = {
    .uri       = "/save_reboot",
    .method    = HTTP_POST,
    .handler   = save_reboot_post_handler,
};

static httpd_uri_t uri_reboot = {
    .uri       = "/reboot",
    .method    = HTTP_GET,
    .handler   = reboot_handler,
};

static httpd_uri_t uri_halow_scan = {
    .uri       = "/api/halow/scan",
    .method    = HTTP_GET,
    .handler   = handler_get_halow_scan,
};

httpd_handle_t start_web_config_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 40;   /* web_config 5 + sensor_gateway_http 28+ */
    cfg.max_open_sockets = 11;   /* 11 - 3 reserved = 8 client slots (dashboard + settings + API) */
    cfg.stack_size = WEB_CONFIG_STACK_SIZE;
    cfg.lru_purge_enable = true; /* Reclaim idle sockets so long requests don't starve others */
    cfg.recv_wait_timeout = 15;  /* Tolerate slow HaLow/STA link */
    cfg.send_wait_timeout = 15;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }
    httpd_register_uri_handler(server, &uri_get);
    httpd_register_uri_handler(server, &uri_save);
    httpd_register_uri_handler(server, &uri_save_reboot);
    httpd_register_uri_handler(server, &uri_reboot);
    httpd_register_uri_handler(server, &uri_halow_scan);
    sensor_gateway_http_register(server);

    ESP_LOGI(TAG, "Web config: http://192.168.4.1/settings");
    return server;
}
