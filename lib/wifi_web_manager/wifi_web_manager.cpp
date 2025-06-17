#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>          //https://github.com/me-no-dev/AsyncTCP
#include <ESPAsyncWebServer.h> //https://github.com/me-no-dev/ESPAsyncWebServer
#include <ArduinoJson.h>
#include <Preferences.h>
#include <wifi_web_manager.h>
#include <esp_task_wdt.h>

Preferences *wifi_web_manager::_pref;                           // For permanent key storage
String wifi_web_manager::_wifi_creds_key;                       // Name for key-value pair for Preferences
wifi_web_manager::nvs_wifi_creds wifi_web_manager::_temp_creds; // Temporary internal WiFi parameters (saved to Preferences, if successful)
// Const String for storing HTML code
const String wifi_web_manager::_index_html = "<!DOCTYPE html><html lang=\"en\"><head><title>WiFi login manager</title><meta charset=\"UTF-8\"><script defer=\"defer\" src=\"script.js\"></script></head><body><form action=\"connect\" method=\"post\" id=\"form\"><select name=\"ssid\" id=\"select\"><option value=\"\">--Please choose an option--</option></select><br><fieldset hidden disabled=\"disabled\" id=\"enterprise\"><legend>Enter username and password and optionally certificate (only PEAP and TTLS currently supported).</legend><label for=\"username\">Username:</label><input name=\"username\" id=\"username\" type=\"text\"><br><label for=\"password\">Password:</label><input name=\"password\" id=\"password\" type=\"password\"><br><label for=\"certificate\">Certificate:</label><textarea name=\"certificate\" id=\"certificate\"></textarea></fieldset><fieldset hidden disabled=\"disabled\" id=\"psk\"><legend>Enter password.</legend><label for=\"password\">Password:</label><input name=\"password\" id=\"password\" type=\"password\"></fieldset><fieldset hidden disabled=\"disabled\" id=\"open\"><p>Open network.</p></fieldset><input type=\"submit\"></form><p id=\"con_status\">WiFi Status: unknown.</p></body></html>";
// Const string for storing JS code
const String wifi_web_manager::_script_js = "var last=\"\";var failed=!1;function on_update(){if(last!=\"\"){document.getElementById(last).setAttribute(\"hidden\",\"hidden\");document.getElementById(last).setAttribute(\"disabled\",\"disabled\")}\nlet selected=document.getElementById(\"select\").selectedOptions.item(0);switch(selected.dataset.auth){case \"0\":last=\"open\";break;case \"1\":last=\"psk\";break;case \"2\":last=\"enterprise\";break;default:return}\ndocument.getElementById(last).removeAttribute(\"hidden\");document.getElementById(last).removeAttribute(\"disabled\")}\nfunction on_submit(){failed=!1}\nasync function refresh_wifi_list(){let url=\"fetch\";let activeTextarea=document.activeElement;try{const response=await fetch(url);if(!response.ok){throw new Error(`Response status: ${response.status}`)}\nconst json=await response.json();let options=\"<option value=\\\"\\\">--Please choose an option--<\\/option>\";if(!(json.hasOwnProperty(document.getElementById(\"select\").value))){options=\"<option selected value=\\\"\\\">--Please choose an option--<\\/option>\"}\nObject.entries(json).forEach(([key,value])=>{if(document.getElementById(\"select\").value===key){options+=\"<option selected data-auth=\\\"\"+value.toString()+\"\\\">\"+key.toString()+\"</option>\\n\"}else{options+=\"<option data-auth=\\\"\"+value.toString()+\"\\\">\"+key.toString()+\"</option>\\n\"}});document.getElementById(\"select\").innerHTML=options;on_update()}catch(error){console.error(error.message)}\nif(activeTextarea.id===\"username\"||activeTextarea.id===\"password\"||activeTextarea.id===\"certificate\"){activeTextarea.focus()}}\nasync function refresh_con_status(){url=\"connected\";try{const response=await fetch(url);if(!response.ok){throw new Error(`Response status: ${response.status}`)}\nconst json=await response.json();if(!failed){if(json.connected===0){document.getElementById(\"con_status\").innerHTML=\"WiFi Status: not connected.\"}else if(json.connected===1){document.getElementById(\"con_status\").innerHTML=\"WiFi Status: connected.\"}else if(json.connected===-1){document.getElementById(\"con_status\").innerHTML=\"WiFi Status: connection error.\";failed=!0}}}catch(error){console.error(error.message);if(error instanceof TypeError){document.getElementById(\"con_status\").innerHTML=\"WiFi Status: ESP32 disconnected.\"}}}\ndocument.getElementById(\"select\").addEventListener(\"change\",on_update);document.getElementById(\"form\").addEventListener(\"submit\",on_submit);refresh_wifi_list();setInterval(refresh_wifi_list,250);setInterval(refresh_con_status,250);";
String wifi_web_manager::_url_prefix;                                       // String for storing URL prefix (if it is desired for the page to be elsewhere)
JsonDocument wifi_web_manager::_doc;                                        // JSON doc for storing WiFi SSID-authentification pairs
SemaphoreHandle_t wifi_web_manager::_wifi_json_mutex;                       // Semaphore for JSON of WiFi stations
SemaphoreHandle_t wifi_web_manager::_wifi_accs_mutex;                       // Semaphore for WiFi object
TaskHandle_t wifi_web_manager::_reconnect_wifi_handle = (TaskHandle_t)NULL; // Task handle for automatic WiFi reconnect
BaseType_t wifi_web_manager::_task_cpu;                                     // Variable for CPU selection

/*
 * @brief
 * wifi_web_manager::init() must be called at the beginning and it has to be called after WiFi is started (in either STA, AP or STA/AP mode) and before AsyncWebServer.begin() - after wifi_web_manager::init() AsyncWebServer.begin() has to be called for starting WiFi Web Manager.
 * @param ws - required parameter to the async web server object
 * @param pref - optional pointer to Preferences object - without it permanent save won't work
 * @param wifi_creds_key - optional parameter to change the default Preferences key-value pair name for storing WiFi credentials, must have size <= 11!
 * @param url_prefix - optional parameter to change the default ("/") web path to other subdirectory ("/"+url_prefix)
 * @param task_cpu - optional parameter to specify ESP32 cpu core to run WiFi refresh task
 */
void wifi_web_manager::init(AsyncWebServer &ws, Preferences *pref, String wifi_creds_key, String url_prefix, const BaseType_t task_cpu)
{
    _task_cpu = task_cpu;
    _wifi_creds_key = wifi_creds_key;
    if (_wifi_creds_key == "" || wifi_creds_key.length() > 11) // Ensuring Preferences key has a value and is of proper length
    {
        _wifi_creds_key = "wifi_key";
    }

    _pref = pref;

    if (url_prefix == "") // Ensuring URL prefix has a value
    {
        _url_prefix = "/";
    }
    else // Or that the value begins and ends with "/"
    {
        String prepend = "", append = "";
        if (url_prefix.charAt(0) != '/')
        {
            prepend = "/";
        }
        if (url_prefix.charAt(url_prefix.length() - 1) != '/')
        {
            append = "/";
        }
        _url_prefix = prepend + url_prefix + append;
    }

    _wifi_json_mutex = xSemaphoreCreateMutex();
    _wifi_accs_mutex = xSemaphoreCreateMutex();
    // Assigning handlers for /, /scripts.js, /fetch, /connected, /connect
    ws.on((_url_prefix).c_str(), _handle_root);
    ws.on((_url_prefix + "script.js").c_str(), _handle_js);
    ws.on((_url_prefix + "fetch").c_str(), _handle_fetch);
    ws.on((_url_prefix + "connected").c_str(), _handle_connected);
    ws.on((_url_prefix + "connect").c_str(), _handle_connect);
    WiFi.onEvent(_wifi_connected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);                                   // Handler if WiFi connected
    WiFi.onEvent(_wifi_disconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);                             // Handler if WiFi disconnected to restart function
    xTaskCreatePinnedToCore(_reconnect_wifi, "wifi_recon", 20 * 1024, NULL, 1, &_reconnect_wifi_handle, _task_cpu); // Task for reconnecting WiFi
    xTaskCreatePinnedToCore(_refresh_wifi, "wifi_refr", 20 * 1024, NULL, 1, NULL, _task_cpu);                       // Task for refreshing WiFi list
}

void wifi_web_manager::_refresh_wifi(void *pvParameters)
{
    int16_t scanned = -2;
    while (1)
    {
        Serial.println("Refreshing wifi list.");
        Serial.println(WiFi.isConnected());
        xSemaphoreTake(_wifi_json_mutex, portMAX_DELAY); // Mutex for access to JSON list of WiFi stations
        String local = "";
        serializeJson(_doc, local);
        Serial.println("Networks JSON: "+local);
        xSemaphoreGive(_wifi_json_mutex);
        if ((WiFi.status() == WL_CONNECTION_LOST) || (WiFi.status() == WL_DISCONNECTED))
        {
            xSemaphoreTake(_wifi_accs_mutex, portMAX_DELAY);
            scanned = WiFi.scanNetworks();
            Serial.println("Scan complete.");
            Serial.println(scanned);
            if (scanned > 0)
            {
                xSemaphoreTake(_wifi_json_mutex, portMAX_DELAY);
                _doc.clear();
                for (int16_t i = 0; i < scanned; i++)
                {
                    uint8_t auth = 0;
                    wifi_auth_mode_t auth_type = WiFi.encryptionType(i); // Storing and simplifying whether it is ENTERPRISE (auth=2), HOME/PSK (auth=1) or OPEN (auth=0) network
                    if ((auth_type == WIFI_AUTH_ENTERPRISE) || (auth_type == WIFI_AUTH_WPA2_ENTERPRISE) || (auth_type == WIFI_AUTH_WPA3_ENT_192))
                    {
                        auth = 2;
                    }
                    else if ((auth_type == WIFI_AUTH_WAPI_PSK) || (auth_type == WIFI_AUTH_WEP) || (auth_type == WIFI_AUTH_WPA_WPA2_PSK) || (auth_type == WIFI_AUTH_WPA2_WPA3_PSK) || (auth_type == WIFI_AUTH_WPA3_PSK) || (auth_type == WIFI_AUTH_WPA2_PSK) || (auth_type == WIFI_AUTH_WPA_PSK))
                    {
                        auth = 1;
                    }
                    else
                    {
                        auth = 0;
                    }
                    _doc[WiFi.SSID(i)] = String(auth); // And storing as key-value pair (SSID-authentication method)
                }
                xSemaphoreGive(_wifi_json_mutex);
                Serial.println("Refreshing WiFi list.");
                WiFi.scanDelete(); // Clearing internally stored WiFi list
            }
            else if (scanned == 0) // If nothing found clear list
            {
                Serial.println("Refreshing WiFi list (empty).");
                xSemaphoreTake(_wifi_json_mutex, portMAX_DELAY);
                _doc.clear();
                xSemaphoreGive(_wifi_json_mutex);
                WiFi.scanDelete();
            }
            xSemaphoreGive(_wifi_accs_mutex);
        }
        else
        {
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void wifi_web_manager::_reconnect_wifi(void *pvParameters)
{
    while (1)
    {
        Serial.println("Looping WiFi task.");
        long last = millis();
        if (WiFi.isConnected()) // If connected -> kill itself
        {
            Serial.println("Connected, killing.");
            _reconnect_wifi_handle = (TaskHandle_t)NULL;
            vTaskDelete(NULL);
        }
        else
        {
            if ((_pref != (Preferences *)NULL) && (_pref->isKey(_wifi_creds_key.c_str()))) // If Preferences contain WiFi key -> try to connect to the WiFi
            {
                Serial.println("Key found, trying to reconnect!");
                _temp_creds.SSID = _pref->isKey((_wifi_creds_key + "SSID").c_str()) ? _pref->getString((_wifi_creds_key + "SSID").c_str()) : "NF";
                _temp_creds.username = _pref->isKey((_wifi_creds_key + "USRN").c_str()) ? _pref->getString((_wifi_creds_key + "USRN").c_str()) : "NF";
                _temp_creds.password = _pref->isKey((_wifi_creds_key + "PSSW").c_str()) ? _pref->getString((_wifi_creds_key + "PSSW").c_str()) : "NF";
                _temp_creds.cert = _pref->isKey((_wifi_creds_key + "CERT").c_str()) ? _pref->getString((_wifi_creds_key + "CERT").c_str()) : "NF";
                _temp_creds.auth = (uint8_t)_pref->isKey((_wifi_creds_key + "AUTH").c_str()) ? _pref->getULong64((_wifi_creds_key + "AUTH").c_str()) : 80;
                xSemaphoreTake(_wifi_accs_mutex, portMAX_DELAY);
                switch (_temp_creds.auth)
                {
                case 0:
                    WiFi.disconnect();
                    WiFi.begin(_temp_creds.SSID);
                    break;
                case 1:
                    WiFi.disconnect();
                    WiFi.begin(_temp_creds.SSID, _temp_creds.password);
                    break;
                case 2:
                    if (WiFi.begin(_temp_creds.SSID, WPA2_AUTH_TTLS, "", _temp_creds.username, _temp_creds.password, _temp_creds.cert) != WL_CONNECTED)
                    {
                        WiFi.begin(_temp_creds.SSID, WPA2_AUTH_PEAP, "", _temp_creds.username, _temp_creds.password, _temp_creds.cert);
                    }
                    break;
                }
                while ((WiFi.status() != WL_CONNECT_FAILED) || WiFi.status() != WL_CONNECTED)
                {
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }
                xSemaphoreGive(_wifi_accs_mutex);
            }
            else // If key not found
            {
                Serial.println("Key not found.");
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // Wait between attempts
    }
}

ArRequestHandlerFunction wifi_web_manager::_handle_root(AsyncWebServerRequest *request) // Handling request for "/"
{
    Serial.println("Handling root.");
    request->send(200, "text/html", _index_html);
    return 0;
};
ArRequestHandlerFunction wifi_web_manager::_handle_js(AsyncWebServerRequest *request) // Handling request for "/script.js"
{
    Serial.println("Handling js.");
    request->send(200, "text/javascript", _script_js);
    return 0;
}
ArRequestHandlerFunction wifi_web_manager::_handle_fetch(AsyncWebServerRequest *request) // Handling JS request for refreshed WiFi list
{
    Serial.println("Handling fetch.");
    request->send(200, "text/plain", "Okikoki");
    return 0;
    xSemaphoreTake(_wifi_json_mutex, portMAX_DELAY); // Mutex for access to JSON list of WiFi stations
    String local = "";
    serializeJson(_doc, local);
    Serial.println(local);
    xSemaphoreGive(_wifi_json_mutex);
    request->send(200, "application/json", local);
    return 0;
};
ArRequestHandlerFunction wifi_web_manager::_handle_connected(AsyncWebServerRequest *request) // Handling JS request for WiFi connection status
{
    Serial.println("Handling connection check.");
    int con = 0;
    if (WiFi.status() == WL_CONNECTED) // Simplifying to three states - connected (1), lost connection or disconnected (in general failed connection) (-1), or unknown (-1)
    {
        con = 1;
    }
    else if ((WiFi.status() == WL_CONNECTION_LOST) || (WiFi.status() == WL_DISCONNECTED))
    {
        con = 0;
    }
    else
    {
        con = -1;
    }
    String local = "{\"connected\": " + String(con) + "}";
    request->send(200, "application/json", local);
    return 0;
}
ArRequestHandlerFunction wifi_web_manager::_handle_connect(AsyncWebServerRequest *request) // Handling form submission/request for connection to WiFi with given credentials in POST body
{
    Serial.println("Handling connect command.");
    String SSID = request->arg("ssid");
    Serial.println(SSID);
    xSemaphoreTake(_wifi_accs_mutex, portMAX_DELAY);
    if (request->hasArg("password")) // Deciding hierarchically, if POST body has password (doesn't -> OPEN network), if POST body has username (doesn't -> HOME/PSK network), if PSK and USERNAME -> ENTERPRISE network, optionally if certificate -> ENTERPRISE with CERT
    {
        if (request->hasArg("username"))
        {
            String cert = "";
            _temp_creds.auth = 2;
            _temp_creds.SSID = SSID;
            _temp_creds.username = request->arg("username");
            _temp_creds.password = request->arg("password");
            if (request->hasArg("certificate"))
            {
                cert = request->arg("certificate");
                _temp_creds.cert = cert;
            }
            WiFi.disconnect();
            if (WiFi.begin(SSID, WPA2_AUTH_TTLS, "", request->arg("username"), request->arg("password"), cert) != WL_CONNECTED) // Trying EAP-TTLS, if failed trying EAP-PEAP
            {
                WiFi.begin(SSID, WPA2_AUTH_PEAP, "", request->arg("username"), request->arg("password"), cert);
            }
        }
        else
        {
            WiFi.disconnect();
            WiFi.begin(SSID, request->arg("password"));
            _temp_creds.auth = 1;
            _temp_creds.SSID = SSID;
            _temp_creds.password = request->arg("password");
        }
    }
    else
    {
        WiFi.disconnect();
        WiFi.begin(SSID);
        _temp_creds.auth = 0;
        _temp_creds.SSID = SSID;
    }
    xSemaphoreGive(_wifi_accs_mutex);
    request->redirect(_url_prefix); // Sending client back to "/"
    return 0;
};

void wifi_web_manager::_wifi_connected(WiFiEvent_t event) // Handling event onConnect (when connection to WiFi was successful) and saving credentials
{
    if (_pref != (Preferences *)NULL && WiFi.isConnected())
    {
        Serial.println("Writing keys");
        _pref->putUChar(_wifi_creds_key.c_str(), 1);
        _pref->putString((_wifi_creds_key + "SSID").c_str(), _temp_creds.SSID);
        _pref->putString((_wifi_creds_key + "USRN").c_str(), _temp_creds.username);
        _pref->putString((_wifi_creds_key + "PSSW").c_str(), _temp_creds.password);
        _pref->putString((_wifi_creds_key + "CERT").c_str(), _temp_creds.cert);
        _pref->putULong64((_wifi_creds_key + "AUTH").c_str(), (uint64_t)_temp_creds.auth);
    }
};

void wifi_web_manager::_wifi_disconnected(WiFiEvent_t event)
{
    // WiFi.setAutoReconnect(true);
    if (_reconnect_wifi_handle == (TaskHandle_t)NULL)
    {
        Serial.println("Disconnected.");
        xTaskCreatePinnedToCore(_reconnect_wifi, "wifi_recon", 20 * 1024, NULL, 1, &_reconnect_wifi_handle, _task_cpu); // Task for reconnecting WiFi
    }
}
