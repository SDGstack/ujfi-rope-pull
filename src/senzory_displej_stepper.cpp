// #define board_devkitv1 || board_firebeetle
#if !(defined board_firebeetle || defined board_devkitv1)
#error "No known board defined!"
#endif

#include <Arduino.h>
#include <fenv.h>
#include <math.h>
#include <assert.h>
#include <queue>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <iot_is.h>
#include <job_manager.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <TMCStepper.h>
#include <ESP_FlexyStepper.h>
#include <Servo.h>
#include <esp_task_wdt.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <web/WebWiFi.h>
#include <web/WebCTRL.h>

// Preferences object to store settings from web
Preferences pos_wifi_pref;

// WiFi Web Manager

// Definitions
AsyncWebServer ws(80);
String AP_SSID = "esp32_rot";
String AP_PASS = "";
char hostname[] = "esp32_rot";
// HTML and JS files are automatically converted into header file Strings - check include/web/WebWiFi.h
String temp_wifi_ssid = "";
String temp_wifi_pass = "";

// Callbacks
ArRequestHandlerFunction web_wifi_handle_connect(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_wifi_handle_rem_saved_wifi(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_wifi_handle_connected(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_wifi_handle_root(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_wifi_handle_js(AsyncWebServerRequest *request);
void wifi_simple_connected_event(WiFiEvent_t event);
void reconn_wifi_simple(void *params);

// Callback for light sleep
ArRequestHandlerFunction web_wifi_handle_light_sleep(AsyncWebServerRequest *request);

// WiFi Web Control

// HTML and JS files are automatically converted into header file Strings - check include/web/WebCTRL.h
const String web_ctrl_prefix = "/ctrl";

// Callbacks
ArRequestHandlerFunction web_ctrl_handle_root(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_js(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_fetch_angle(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_set_angle(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_by_angle(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_find_home(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_set_mm_per_rot(AsyncWebServerRequest *request);

// Servo
Servo servo_lock;
int pos = 0;

#ifdef board_firebeetle // FireBeetle
#define pin_servo D9
#elif defined board_devkitv1 // DEVKIT V1
#define pin_servo GPIO_NUM_14
#endif

const float servo_small_unlock_angle = 20;
const float servo_large_unlock_angle = 30;
const float servo_lock_angle = 2;

// MQTT definitions (on mqtt.smogrovic.com - auth username is device id)
const char mqtt_server_url[] = "mqtt.smogrovic.com";
const int  mqtt_server_port = 1883;
const char mqtt_server_dev_id[] = "wxe6BVrTWJ1Kuib0SigC2JRU";
const char mqtt_server_dev_ds18b20[] = "ds18b20";
const char mqtt_server_dev_bme680temp[] = "bme680_temp";
const char mqtt_server_dev_bme680press[] = "bme680_press";
const char mqtt_server_dev_bme680hum[] = "bme680_hum";
const char mqtt_server_dev_bme680res[] = "bme680_res";
const char mqtt_server_dev_stepper_angle[] = "stepper_angle";
const char mqtt_server_dev_command_rotate_angle[] = "rot_angle";
const char mqtt_server_dev_command_rotate_to_angle[] = "rot_to_angle";
const char mqtt_server_dev_command_find_home[] = "find_home";

// FreeRTOS definitions
static const BaseType_t cpu = 0;

// Stepper motor
#ifdef board_firebeetle // FireBeetle
#define pin_dir GPIO_NUM_25
#define pin_step GPIO_NUM_26
#define pin_mot_tx GPIO_NUM_27
#define pin_mot_rx GPIO_NUM_9
#define pin_en GPIO_NUM_10

#elif defined board_devkitv1 // DEVKIT V1
#define pin_dir GPIO_NUM_15
#define pin_step GPIO_NUM_2
#define pin_mot_tx GPIO_NUM_17
#define pin_mot_rx GPIO_NUM_16
#define pin_en GPIO_NUM_4

#endif

// Definitions for light sleep

// Light sleep microswitch
#define light_sleep_switch_polarity_off HIGH

#ifdef board_firebeetle // FireBeetle
#define pin_light_sleep_switch GPIO_NUM_23

#elif defined board_devkitv1 // DEVKIT V1
#error "No defined light sleep switch pin for this board!"
#endif


// Definitions for stepper

// Stepping
uint16_t microstepping = 16;
const int steps_per_rot_full_step = 400;
const float deg_per_teeth = 360.0f / 32;
int steps_per_rot = microstepping * steps_per_rot_full_step;
const char mm_per_rot_key[] = "mm_per_rot";

// Max current settings
const uint16_t RUN_CURRENT = 4000;
const float R_SENSE = 0.11f;

// Stepper controller controller object & serial port
const unsigned char TMC2209_ADDR = 0b11;
#define SERIAL_PORT Serial1
TMC2209Stepper stepper_driver = TMC2209Stepper(&SERIAL_PORT, R_SENSE, TMC2209_ADDR);

// Software stepper code
ESP_FlexyStepper stepper = ESP_FlexyStepper();

// Locking objects for mutual access
bool home_found = false;
SemaphoreHandle_t rotate_command_mutex;
TaskHandle_t rel_rot_task_handle = NULL;

// Upper Microswitch
#define high_switch_polarity_off HIGH

#ifdef board_firebeetle // FireBeetle
#define pin_high_switch GPIO_NUM_18

#elif defined board_devkitv1 // DEVKIT V1
#define pin_high_switch GPIO_NUM_13

#endif

// Lower Microswitch
#define low_switch_polarity_off HIGH

#ifdef board_firebeetle // FireBeetle
#define pin_low_switch GPIO_NUM_21

#elif defined board_devkitv1 // DEVKIT V1
#define pin_low_switch GPIO_NUM_12

#endif
double rotations_to_lowest = 0.0;


// Function for finding home position.
void find_home(void *params);

// MQTT callback functions
bool mqtt_callback_rotate_angle(const std::vector<double> &params);
bool mqtt_callback_rotate_to_angle(const std::vector<double> &params);
bool mqtt_callback_find_home(const std::vector<double> &params);

// MQTT task functions
void task_rotate_abs(void *params);
void task_rotate_rel(void *params);

// WiFi callback
void on_wifi_connected(WiFiEvent_t event);

void adjust_position_to_whole_teeth();

void setup()
{
    // Start the Serial Monitor
    Serial.begin(115200);

    // Servo
    pinMode(pin_servo, OUTPUT);
    servo_lock.attach(pin_servo, Servo::CHANNEL_NOT_ATTACHED, 0, 180, 500, 2500, 50);

    // Set STEP/DIR/EN pins OUTPUT, LOW
    pinMode(pin_step, OUTPUT);
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_en, OUTPUT);
    digitalWrite(pin_en, LOW);
    digitalWrite(pin_step, LOW);
    digitalWrite(pin_dir, HIGH);

    // Lock servo at the beginning
    servo_lock.write(servo_lock_angle);

    // Start stepper motor controller
    SERIAL_PORT.setPins(pin_mot_rx, pin_mot_tx);
    SERIAL_PORT.begin(115200);
    stepper_driver.begin();
    stepper_driver.toff(5);
    stepper_driver.rms_current(RUN_CURRENT);
    stepper_driver.microsteps(microstepping);
    stepper_driver.pwm_autoscale(true);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Start stepper motor software driver
    stepper.connectToPins(pin_step, pin_dir);
    stepper.setEnablePin(pin_en);
    stepper.setSpeedInStepsPerSecond(steps_per_rot / 2);
    stepper.setAccelerationInStepsPerSecondPerSecond(steps_per_rot * 3);
    stepper.setAccelerationInStepsPerSecondPerSecond(steps_per_rot * 3);
    stepper.setStepsPerRevolution(steps_per_rot);
    stepper.enableDriver();
    stepper.moveRelativeInRevolutions(deg_per_teeth / 360.0f);
    adjust_position_to_whole_teeth();
    stepper.disableDriver();

    // Start WiFi preferences database 
    pos_wifi_pref.begin("pos_wifi_pref", false);

    // Set switch pin according to polarity in off state
    pinMode(pin_high_switch, (high_switch_polarity_off==HIGH)? INPUT_PULLUP : INPUT_PULLDOWN);
    pinMode(pin_low_switch, (low_switch_polarity_off==HIGH)? INPUT_PULLUP : INPUT_PULLDOWN);
    pinMode(pin_light_sleep_switch, (light_sleep_switch_polarity_off==HIGH)? INPUT_PULLUP : INPUT_PULLDOWN);
    
    // WiFi hostname & mDNS
    WiFi.setHostname(hostname);
    MDNS.begin(hostname);

    // Enable WiFi
    WiFi.mode(WIFI_MODE_APSTA);
    WiFi.softAP(AP_SSID, AP_PASS);

    // WiFi AP IP
    Serial.print("AP: ");
    Serial.println(WiFi.softAPIP());

    // Assign web control interface callbacks
    ws.on((web_ctrl_prefix + "/").c_str(), web_ctrl_handle_root);
    ws.on((web_ctrl_prefix + "/script.js").c_str(), web_ctrl_handle_js);
    ws.on((web_ctrl_prefix + "/light_sleep").c_str(), web_wifi_handle_light_sleep);
    ws.on((web_ctrl_prefix + "/fetch_angle").c_str(), web_ctrl_handle_fetch_angle);
    ws.on("/set_angle", web_ctrl_handle_set_angle);
    ws.on("/by_angle", web_ctrl_handle_by_angle);
    ws.on("/find_home", web_ctrl_handle_find_home);
    ws.on("/set_mm_per_rot", web_ctrl_handle_set_mm_per_rot);
    ws.on((web_ctrl_prefix).c_str(), [](AsyncWebServerRequest *request)
        { request->redirect(web_ctrl_prefix + "/"); });
    // Assign WiFi web manager callbacks
    ws.on("/simple_connect", web_wifi_handle_connect);
    ws.on("/rem_saved_wifi", web_wifi_handle_rem_saved_wifi);
    ws.on("/simple_check_connect", web_wifi_handle_connected);
    ws.on("/", web_wifi_handle_root);
    ws.on("/script.js", web_wifi_handle_js);
    WiFi.onEvent(wifi_simple_connected_event, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
    
    // Start web server
    ws.begin();

    // Create rotate command mutex
    rotate_command_mutex = xSemaphoreCreateBinary();
    assert(rotate_command_mutex != NULL);
    xSemaphoreGive(rotate_command_mutex);

    // Register MQTT handlers
    job_manager.register_command(mqtt_server_dev_command_rotate_angle, mqtt_callback_rotate_angle);
    job_manager.register_command(mqtt_server_dev_command_rotate_to_angle, mqtt_callback_rotate_to_angle);
    job_manager.register_command(mqtt_server_dev_command_find_home, mqtt_callback_find_home);
    job_manager.init();
    
    // Attempt connection to SDG IS
    Serial.println("Trying to connect to MQTT server.");
    iotIs.connect(mqtt_server_dev_id, mqtt_server_url, mqtt_server_port);
    Serial.println("Connected.");
}

void loop()
{
    if (WiFi.isConnected())
    {
        Serial.println(WiFi.localIP().toString());
        Serial.println("WiFi connected, waiting.");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
    else if (pos_wifi_pref.isKey("wifi_ssid"))
    {
        String ssid = pos_wifi_pref.getString("wifi_ssid");
        String pass = pos_wifi_pref.getString("wifi_pass");
        WiFi.disconnect(false, true);
        Serial.println(ssid);
        Serial.println(pass);
        WiFi.begin(ssid.c_str(), pass.c_str());
        temp_wifi_ssid = ssid;
        temp_wifi_pass = pass;
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
    else
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

// Web WiFi manager

// Root/index request callback
ArRequestHandlerFunction web_wifi_handle_root(AsyncWebServerRequest *request)
{
    Serial.println("Simple WiFi: Handling HTML.");
    request->send(200, "text/html", WebWiFi_index_html);
    return 0;
}

// JS request callback
ArRequestHandlerFunction web_wifi_handle_js(AsyncWebServerRequest *request)
{
    Serial.println("Simple WiFi: Handling JS.");
    request->send(200, "text/javascript", WebWiFi_script_js);
    return 0;
}

// Light sleep task
void light_sleep_task(void *params){
    Serial.println("Entering light sleep.");
    gpio_wakeup_enable(pin_light_sleep_switch, (light_sleep_switch_polarity_off==HIGH)? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    stepper.disableDriver();
    vTaskDelay(500/portTICK_PERIOD_MS);
    esp_light_sleep_start();
    vTaskDelay(100/portTICK_PERIOD_MS);
    Serial.println("Exiting light sleep.");
    WiFi.softAP(AP_SSID, AP_PASS);
    vTaskDelete(NULL);
} 

// Light sleep callback
ArRequestHandlerFunction web_wifi_handle_light_sleep(AsyncWebServerRequest *request){
    xTaskCreatePinnedToCore(light_sleep_task, "light_sleep", 10 * 1024, NULL, configMAX_PRIORITIES-1, NULL, cpu);
    request->send(200);
    return 0;
}

// Attempt connecting to WiFi callback
ArRequestHandlerFunction web_wifi_handle_connect(AsyncWebServerRequest *request)
{
    String ssid = request->arg("ssid");
    String pass = "";
    if (request->hasArg("passw"))
    {
        pass = request->arg("passw");
    }
    WiFi.disconnect(false, true);
    Serial.println("Simple WiFi: Attempting WiFi connect.");
    Serial.println(ssid);
    Serial.println(pass);
    WiFi.begin(ssid.c_str(), pass.c_str());
    temp_wifi_ssid = ssid;
    temp_wifi_pass = pass;
    request->send(200);
    return 0;
}

// Remove saved WiFi config callback
ArRequestHandlerFunction web_wifi_handle_rem_saved_wifi(AsyncWebServerRequest *request)
{
    Serial.println("Simple WiFi: Removing saved WiFi.");
    if (pos_wifi_pref.isKey("wifi_ssid"))
    {
        pos_wifi_pref.remove("wifi_ssid");
        pos_wifi_pref.remove("wifi_pass");
    }
    WiFi.disconnect(false, true);
    request->send(200);
    return 0;
}

// Check for connection callback
ArRequestHandlerFunction web_wifi_handle_connected(AsyncWebServerRequest *request)
{
    Serial.println("Simple WiFi: fetching connected.");
    String wifi_stat_str = "";
    switch (WiFi.status())
    {
    case WL_CONNECTED:
        wifi_stat_str = "\"con_stat\":\"0\",";
        wifi_stat_str += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
        break;
    case WL_CONNECT_FAILED:
        wifi_stat_str = "\"con_stat\":\"1\",";
        wifi_stat_str += "\"ip\":\"0\",";
        break;
    case WL_DISCONNECTED:
        wifi_stat_str = "\"con_stat\":\"2\",";
        wifi_stat_str += "\"ip\":\"0\",";
        break;
    case WL_CONNECTION_LOST:
        wifi_stat_str = "\"con_stat\":\"2\",";
        wifi_stat_str += "\"ip\":\"0\",";
        break;
    }
    wifi_stat_str += "\"saved\":\"" + String(pos_wifi_pref.isKey("wifi_ssid") ? 1 : 0) + "\"";
    wifi_stat_str = "{" + wifi_stat_str + "}";
    request->send(200, "application/json", wifi_stat_str);
    return 0;
}

// Callback for when WiFi connection is successful
void wifi_simple_connected_event(WiFiEvent_t event)
{
    Serial.println("Simple WiFi: Saving usable WiFi.");
    Serial.println(temp_wifi_ssid);
    Serial.println(temp_wifi_pass);
    pos_wifi_pref.putString("wifi_ssid", temp_wifi_ssid);
    pos_wifi_pref.putString("wifi_pass", temp_wifi_pass);
}

// Web Control
// Root/index request callback 
ArRequestHandlerFunction web_ctrl_handle_root(AsyncWebServerRequest *request)
{
    Serial.println("Web control: Handling root.");
    request->send(200, "text/html", WebCTRL_index_html);
    return 0;
}

// JS request callback
ArRequestHandlerFunction web_ctrl_handle_js(AsyncWebServerRequest *request)
{
    Serial.println("Web control: Handling js.");
    request->send(200, "text/javascript", WebCTRL_script_js);
    return 0;
}

// Set mm per rotation callback
ArRequestHandlerFunction web_ctrl_handle_set_mm_per_rot(AsyncWebServerRequest *request)
{
    Serial.println("Web control: Handling set millimeters per degree.");
    pos_wifi_pref.putFloat(mm_per_rot_key, request->arg("mm_per_rot").toDouble());
    request->send(200, "text/plain", "okikoki");
    return 0;
}

// Find home callback
ArRequestHandlerFunction web_ctrl_handle_find_home(AsyncWebServerRequest *request)
{
    Serial.println("Web control: find home.");
    xTaskCreatePinnedToCore(find_home, "find_home_t", 10 * 1024, (void *)false, 1, NULL, cpu);
    request->redirect(web_ctrl_prefix);
    return 0;
}

// Angle value request callback 
ArRequestHandlerFunction web_ctrl_handle_fetch_angle(AsyncWebServerRequest *request)
{
    Serial.println("Web control: Handling fetch.");
    float angle = NAN;
    float mm_per_rot = NAN;
    if (pos_wifi_pref.isKey(mm_per_rot_key))
    {
        mm_per_rot = pos_wifi_pref.getFloat(mm_per_rot_key);
    }
    if (home_found)
    {
        // angle = fmodf(fmodf(-stepper.getCurrentPositionInRevolutions() * 360.0f, 360.0f) + 360.0f, 360.0f);
        angle = stepper.getCurrentPositionInRevolutions() * 360.0f;
    }
    String local = "{\"angle_val\": \"" + String(angle) + "\"," + "\"mm_per_rot\": " + "\"" + String(mm_per_rot) + "\"" + "}";
    request->send(200, "application/json", local);
    return 0;
}

// Rotate absolute callback
ArRequestHandlerFunction web_ctrl_handle_set_angle(AsyncWebServerRequest *request)
{
    Serial.println("Web control: Handling set angle.");
    String angle_str = request->arg("angle_val");
    Serial.println(angle_str);
    double *p = new double((angle_str.toDouble() + deg_per_teeth) / 360.0f);
    xTaskCreatePinnedToCore(task_rotate_abs, "abs_rot_t", 10 * 1024, (void *)p, 1, NULL, cpu);
    request->redirect(web_ctrl_prefix);
    return 0;
}

// Rotate relative callback
ArRequestHandlerFunction web_ctrl_handle_by_angle(AsyncWebServerRequest *request)
{
    Serial.println("Web control: Handling rotate by angle.");
    String angle_str = request->arg("angle_val");
    Serial.println(angle_str);
    double *p = new double((angle_str.toDouble() + deg_per_teeth) / 360.0f);
    xTaskCreatePinnedToCore(task_rotate_rel, "rel_rot_t", 10 * 1024, (void *)p, 1, &rel_rot_task_handle, cpu);
    request->redirect(web_ctrl_prefix);
    return 0;
}

// MQTT rotate by angle callback function
bool mqtt_callback_rotate_angle(const std::vector<double> &params)
{
    if (!params.empty())
    {
        Serial.println("MQTT rotate by angle.");
        double *p = new double(-(params[0] + deg_per_teeth) / 360.0f);
        xTaskCreatePinnedToCore(task_rotate_rel, "rel_rot_t", 10 * 1024, (void *)p, 1, &rel_rot_task_handle, cpu);
        return true;
    }
    return false;
}

// MQTT rotate to angle callback function
bool mqtt_callback_rotate_to_angle(const std::vector<double> &params)
{
    if (!params.empty())
    {
        Serial.println("MQTT rotate to angle.");
        double *p = new double(-(params[0] + deg_per_teeth) / 360.0f);
        xTaskCreatePinnedToCore(task_rotate_abs, "abs_rot_t", 10 * 1024, (void *)p, 1, NULL, cpu);
        return true;
    }
    return false;
}

// MQTT find home callback function
bool mqtt_callback_find_home(const std::vector<double> &params)
{
    Serial.println("MQTT find home.");
    xTaskCreatePinnedToCore(find_home, "find_home_t", 10 * 1024, (void *)false, 1, NULL, cpu);
    return true;
}

// Function for adjusting stepper software driver position to whole teeth
void adjust_position_to_whole_teeth()
{
    double loc_pos_rev = ((double)((long long)(stepper.getCurrentPositionInRevolutions() * 360.0f / deg_per_teeth)) + 1.0f) * deg_per_teeth / 360.0f;
    stepper.setCurrentPositionInRevolutions(loc_pos_rev);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

// Task function for relative rotation
void task_rotate_rel(void *params)
{
    Serial.println("Func to rel rot entered, waiting for mutex");
    // Set watchdog to 30s
    esp_task_wdt_init(30, false);

    double *angle_ptr = (double *)params;
    double angle = *angle_ptr;
    delete angle_ptr;
    xSemaphoreTake(rotate_command_mutex, portMAX_DELAY);
    Serial.println("Entry to rel task.");
    stepper.enableDriver();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    stepper.moveRelativeInRevolutions(-deg_per_teeth / 2 / 360.0f); // Move wheel so that lock won't get stuck
    vTaskDelay(500 / portTICK_PERIOD_MS);
    if (angle > 0)
    {
        servo_lock.write(servo_large_unlock_angle); // unlock servo
    }
    else
    {
        servo_lock.write(servo_small_unlock_angle); // unlock servo
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    stepper.moveRelativeInRevolutions(deg_per_teeth / 2 / 360.0f); // Move the wheel back
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.println("Servo unlocked.");
    Serial.println("Rotate by angle task.");
    if (home_found)
    { // Making sure angle is in allowed range (<0; rotations_to_lowest>)
        angle += stepper.getCurrentPositionInRevolutions();
        angle = min(max(angle, 1.0), rotations_to_lowest);
        stepper.moveToPositionInRevolutions(angle);
    }

    else
    {
        esp_task_wdt_init(60, false);
        float init = stepper.getCurrentPositionInRevolutions();
        stepper.setAccelerationInStepsPerSecondPerSecond(steps_per_rot * 100);
        stepper.setSpeedInStepsPerSecond(steps_per_rot*100);
        if (angle >= 0)
        {

            //stepper.startJogging(1);
            //while (((stepper.getCurrentPositionInRevolutions() - init) < angle) && (digitalRead(pin_low_switch) == low_switch_polarity_off))
            stepper.setTargetPositionRelativeInRevolutions(angle);
            while ((stepper.getDistanceToTargetSigned() != 0) && (digitalRead(pin_low_switch) == low_switch_polarity_off))
            {
                stepper.processMovement();
                //Serial.println(stepper.getCurrentPositionInRevolutions(), 10);
                //Serial.println("Processing movement.");
            }
            stepper.emergencyStop();
            if (digitalRead(pin_low_switch) != low_switch_polarity_off)
            {
                Serial.println("Low switch hit.");
                stepper.moveRelativeInRevolutions(-1);
                stepper.moveRelativeInRevolutions(-1);
            }
            else
            {
                Serial.println("Movement finished.");
            }
        }
        else
        {
            //stepper.startJogging(-1);
            //while (((stepper.getCurrentPositionInRevolutions() - init) > angle) && (digitalRead(pin_high_switch) == high_switch_polarity_off))
            stepper.setTargetPositionRelativeInRevolutions(angle);
            while ((stepper.getDistanceToTargetSigned() != 0) && (digitalRead(pin_high_switch) == high_switch_polarity_off))
            {
                stepper.processMovement();
                //Serial.println(stepper.getCurrentPositionInRevolutions(), 10);
                //Serial.println("Processing movement.");
            }
            stepper.emergencyStop();
            if (digitalRead(pin_high_switch) != high_switch_polarity_off)
            {
                Serial.println("High switch hit.");
                stepper.moveRelativeInRevolutions(1);
                stepper.moveRelativeInRevolutions(1);
            }
            else
            {
                Serial.println("Movement finished.");
            }
        }
    }
    stepper.setAccelerationInStepsPerSecondPerSecond(steps_per_rot * 3);
    stepper.setSpeedInStepsPerSecond(steps_per_rot);
    servo_lock.write(servo_lock_angle); // lock servo
    Serial.println("Servo locked.");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    stepper.moveRelativeInRevolutions(-deg_per_teeth / 2 / 360.0f); // Move wheel so that forward and backward movement don't affect it's position (if for example rotation angle = 0)
    stepper.moveRelativeInRevolutions(deg_per_teeth / 360.0f);      // Move the wheel so that it is stuck to lock
    adjust_position_to_whole_teeth();
    stepper.disableDriver();
    xSemaphoreGive(rotate_command_mutex);
    vTaskDelete(NULL);
}

// Task function for absolute rotation
void task_rotate_abs(void *params)
{
    // Set watchdog to 30s
    esp_task_wdt_init(30, false);

    double *angle_ptr = (double *)params;
    double angle = *angle_ptr;
    delete angle_ptr;
    xSemaphoreTake(rotate_command_mutex, portMAX_DELAY);
    Serial.println("Entry to abs task.");
    stepper.enableDriver();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    stepper.moveRelativeInRevolutions(-deg_per_teeth / 2 / 360.0f); // Move wheel so that lock won't get stuck
    vTaskDelay(500 / portTICK_PERIOD_MS);
    if ((angle - stepper.getCurrentPositionInRevolutions()) > 0)
    {
        servo_lock.write(servo_large_unlock_angle); // unlock servo by small angle
    }
    else
    {
        servo_lock.write(servo_small_unlock_angle); // unlock servo by large angle
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    stepper.moveRelativeInRevolutions(deg_per_teeth / 2 / 360.0f); // Move the wheel back
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.println("Servo unlocked.");
    if (home_found)
    { // Making sure angle is in allowed range (<0; rotations_to_lowest>)
        Serial.println("Rotate to angle task.");
        Serial.println(angle);
        angle = min(max(angle, 1.0), rotations_to_lowest);
        Serial.println(angle);
        stepper.moveToPositionInRevolutions(angle);
    }
    else
    {
        Serial.println("Rotate to angle task failed - zero position not set.");
    }
    servo_lock.write(servo_lock_angle); // lock servo
    Serial.println("Servo locked.");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    stepper.moveRelativeInRevolutions(-deg_per_teeth / 2 / 360.0f); // Move wheel so that forward and backward movement don't affect it's position (if for example rotation angle = 0)
    stepper.moveRelativeInRevolutions(deg_per_teeth / 360.0f);      // Move the wheel so that it is stuck to lock
    adjust_position_to_whole_teeth();
    stepper.disableDriver();
    xSemaphoreGive(rotate_command_mutex);
    vTaskDelete(NULL);
}

// Task to find home
void find_home(void *params)
{
    // Set watchdog to 30s
    esp_task_wdt_init(30, false);

    long pos = 0;
    xSemaphoreTake(rotate_command_mutex, portMAX_DELAY);
    home_found = false;
    stepper.enableDriver();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    stepper.moveRelativeInRevolutions(-deg_per_teeth / 2 / 360.0f); // Move wheel so that lock won't get stuck
    vTaskDelay(500 / portTICK_PERIOD_MS);
    servo_lock.write(servo_large_unlock_angle); // unlock servo
    vTaskDelay(500 / portTICK_PERIOD_MS);
    stepper.moveRelativeInRevolutions(deg_per_teeth / 2 / 360.0f); // Move the wheel back
    vTaskDelay(500 / portTICK_PERIOD_MS);

    stepper.setSpeedInStepsPerSecond(steps_per_rot * 100);
    stepper.setAccelerationInStepsPerSecondPerSecond(steps_per_rot * 100);

    while (digitalRead(pin_high_switch) == high_switch_polarity_off)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        Serial.println("Moving.");
        stepper.moveRelativeInSteps(-50);
        pos++;
        pos %= 360 * steps_per_rot;
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
    stepper.setCurrentPositionAsHomeAndStop();

    pos = 0;
    while (digitalRead(pin_low_switch) == low_switch_polarity_off)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        Serial.println("Moving.");
        stepper.moveRelativeInSteps(50);
        pos++;
        pos %= 360 * steps_per_rot;
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    stepper.setAccelerationInStepsPerSecondPerSecond(steps_per_rot * 3);
    stepper.setSpeedInStepsPerSecond(steps_per_rot);

    while (digitalRead(pin_low_switch) != low_switch_polarity_off)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        stepper.moveRelativeInSteps(-4);
    }

    rotations_to_lowest = max(stepper.getCurrentPositionInRevolutions() - 1.0f, 0.0f);

    home_found = true;

    servo_lock.write(servo_lock_angle); // lock servo
    vTaskDelay(500 / portTICK_PERIOD_MS);
    stepper.moveRelativeInRevolutions(deg_per_teeth / 360.0f);
    adjust_position_to_whole_teeth();
    stepper.disableDriver();
    xSemaphoreGive(rotate_command_mutex);
    vTaskDelete(NULL);
}