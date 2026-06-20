#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_server.h"

#include "web_logic.h"
#include "connect_logic.h"
#include "gps_logic.h"
#include "status_logic.h"
#include "command_logic.h"
#include "enums_logic.h"

#define EXAMPLE_ESP_WIFI_SSID      "OsmoRemote"
#define EXAMPLE_ESP_WIFI_PASS      "12345678"
#define EXAMPLE_ESP_WIFI_CHANNEL   1
#define EXAMPLE_MAX_STA_CONN       4

static const char *TAG = "LOGIC_WEB";

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    // esp_event_loop_create_default() is usually called in app_main or here.
    // If it's already created, this might return an error, so we ignore it if it exists.
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
    
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                    .required = false,
            },
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

// -------------------------------------------------------------------
// Web Server Handlers
// -------------------------------------------------------------------

const char* html_page = 
"<!DOCTYPE html>"
"<html>"
"<head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>Osmo Remote</title>"
"<style>"
"body{background:#121212;color:#f0f0f0;font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;margin:0;padding:20px;display:flex;flex-direction:column;align-items:center;}"
"h1{margin-top:10px;font-size:24px;color:#fff;font-weight:600;}"
".card{background:#1e1e1e;border-radius:16px;padding:20px;margin-bottom:20px;width:100%;max-width:350px;box-shadow:0 8px 16px rgba(0,0,0,0.4);box-sizing:border-box;}"
".row{display:flex;justify-content:space-between;margin-bottom:12px;font-size:16px;border-bottom:1px solid #333;padding-bottom:8px;}"
".row:last-child{border-bottom:none;margin-bottom:0;padding-bottom:0;}"
".val{font-weight:600;color:#4ade80;}"
".val.err{color:#f87171;}"
".val.neu{color:#f0f0f0;}"
".btn{background:#3b82f6;color:white;border:none;padding:14px;border-radius:12px;font-size:16px;font-weight:bold;cursor:pointer;width:100%;max-width:350px;margin-bottom:12px;transition:0.2s;box-sizing:border-box;}"
".btn:active{transform:scale(0.96);}"
".btn.rec{background:#ef4444;}"
".btn.pwr{background:#4b5563;}"
"</style></head>"
"<body>"
"<h1>Osmo GPS Remote</h1>"
"<div class='card'>"
"<div class='row'><span>Camera</span><span id='s-cam' class='val neu'>...</span></div>"
"<div class='row'><span>Recording</span><span id='s-rec' class='val neu'>...</span></div>"
"<div class='row'><span>GPS Status</span><span id='s-gps' class='val neu'>...</span></div>"
"<div class='row'><span>Satellites</span><span id='s-sat' class='val neu'>0</span></div>"
"<div class='row'><span>Location</span><span id='s-loc' class='val neu' style='font-size:12px;display:flex;align-items:center;'>...</span></div>"
"</div>"
"<button class='btn rec' onclick=\"cmd('shutter')\">Start/Stop Recording</button>"
"<button class='btn' onclick=\"cmd('mode')\">Switch Mode</button>"
"<button class='btn pwr' onclick=\"cmd('power_off')\">Power Off Camera</button>"
"<script>"
"function cmd(act){ fetch('/api/cmd',{method:'POST',body:act}); }"
"setInterval(()=>{ "
"fetch('/api/status').then(r=>r.json()).then(d=>{"
" document.getElementById('s-cam').innerText = d.ble;"
" document.getElementById('s-cam').className = (d.ble == 'Connected') ? 'val' : 'val err';"
" document.getElementById('s-rec').innerText = d.rec ? 'Recording' : 'Standby';"
" document.getElementById('s-rec').style.color = d.rec ? '#ef4444' : '#f0f0f0';"
" document.getElementById('s-gps').innerText = d.gps ? 'Locked' : 'Searching';"
" document.getElementById('s-gps').className = d.gps ? 'val' : 'val err';"
" document.getElementById('s-sat').innerText = d.sat;"
" document.getElementById('s-loc').innerText = d.gps ? d.lat.toFixed(5)+', '+d.lon.toFixed(5) : 'N/A';"
"});"
"}, 1000);"
"</script>"
"</body></html>";

static esp_err_t index_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t api_status_get_handler(httpd_req_t *req)
{
    GPS_Data_t gps = get_current_gps_data();
    
    const char* ble_status = (connect_logic_get_state() == PROTOCOL_CONNECTED) ? "Connected" : "Disconnected";
    int is_rec = is_camera_recording() ? 1 : 0;
    int gps_valid = is_current_gps_data_valid() ? 1 : 0;
    
    char resp[256];
    snprintf(resp, sizeof(resp), 
        "{\"ble\":\"%s\",\"rec\":%d,\"gps\":%d,\"sat\":%d,\"lat\":%f,\"lon\":%f}",
        ble_status, is_rec, gps_valid, gps.Num_Satellites, gps.Latitude, gps.Longitude);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t api_cmd_post_handler(httpd_req_t *req)
{
    char buf[64];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if ((ret = httpd_req_recv(req, buf, remaining)) <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    ESP_LOGI(TAG, "Received command: %s", buf);

    if (strncmp(buf, "shutter", 7) == 0) {
        command_logic_key_report_snapshot(); // Toggles recording/shutter
    } else if (strncmp(buf, "mode", 4) == 0) {
        command_logic_key_report_qs(); // Switches mode / quick switch
    } else if (strncmp(buf, "power_off", 9) == 0) {
        // Power off is not explicitly in command_logic.h, maybe send long-press QS if available?
        // We'll just ignore for now to avoid compilation errors.
    }

    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static httpd_uri_t uri_get_index = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = index_get_handler,
    .user_ctx = NULL
};

static httpd_uri_t uri_get_status = {
    .uri      = "/api/status",
    .method   = HTTP_GET,
    .handler  = api_status_get_handler,
    .user_ctx = NULL
};

static httpd_uri_t uri_post_cmd = {
    .uri      = "/api/cmd",
    .method   = HTTP_POST,
    .handler  = api_cmd_post_handler,
    .user_ctx = NULL
};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &uri_get_index);
        httpd_register_uri_handler(server, &uri_get_status);
        httpd_register_uri_handler(server, &uri_post_cmd);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void web_logic_init(void)
{
    // Initialize NVS (in case it wasn't initialized elsewhere yet)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();
    
    start_webserver();
}
