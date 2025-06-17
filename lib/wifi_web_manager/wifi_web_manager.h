#ifndef WIFI_WEB_MGR_H
#define WIFI_WEB_MGR_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>          //https://github.com/me-no-dev/AsyncTCP
#include <ESPAsyncWebServer.h> //https://github.com/me-no-dev/ESPAsyncWebServer
#include <ArduinoJson.h>
#include <Preferences.h>

#define WIFI_WEB_MGR_LIB

class wifi_web_manager
{
public:
    static void init(AsyncWebServer &ws, Preferences *pref = (Preferences *)NULL, String wifi_creds_key = "", String url_prefix = "", const BaseType_t task_cpu = 0);

private:
    static const String _index_html;
    static const String _script_js;
    static String _url_prefix;
    wifi_web_manager();
    static String _wifi_creds_key;
    struct nvs_wifi_creds
    {
        String SSID = "";
        String username = "";
        String password = "";
        String cert = "";
        uint8_t auth = 0;
    };
    static BaseType_t _task_cpu;
    static Preferences *_pref;
    static nvs_wifi_creds _temp_creds;
    static JsonDocument _doc;
    static SemaphoreHandle_t _wifi_json_mutex;
    static SemaphoreHandle_t _wifi_accs_mutex;
    static void _reconnect_wifi(void *pvParameters);
    static void _refresh_wifi(void *pvParameters);
    static TaskHandle_t _reconnect_wifi_handle;
    static ArRequestHandlerFunction _handle_root(AsyncWebServerRequest *request);
    static ArRequestHandlerFunction _handle_js(AsyncWebServerRequest *request);
    static ArRequestHandlerFunction _handle_fetch(AsyncWebServerRequest *request);
    static ArRequestHandlerFunction _handle_connected(AsyncWebServerRequest *request);
    static ArRequestHandlerFunction _handle_connect(AsyncWebServerRequest *request);
    static void _wifi_connected(WiFiEvent_t event);
    static void _wifi_disconnected(WiFiEvent_t event);
};
#endif