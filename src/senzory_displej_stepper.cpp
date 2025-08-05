// #define board_devkitv1 || board_firebeetle
#define suff_mem
#define simple_wifi
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
#ifdef suff_mem
#include <ESPAsyncWebServer.h>
#endif
#include <wifi_web_manager.h>
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

// WiFi Web Manager definitions
#ifdef suff_mem
AsyncWebServer ws(80);
String AP_SSID = "esp32_rot_control";
String AP_PASS = "";
char hostname[] = "esp32_rot";
#endif

// WiFi Web control
#ifdef suff_mem
const String ctrl_index_html = "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\"><title>Web Control</title><style>.abs_unit,.abs_val_input,.con_stat,.con_stat_text,.display_unit,.left_button_rel,.rel_unit,.rel_val_input,.right_button_rel,.set_mm_per_rot,.set_mm_per_rot_txt{display:inline}</style><script src=\"script.js\" defer=\"defer\"></script></head><body><p class=\"con_stat_text\">Connection status:</p><p class=\"con_stat\"></p><br><p class=\"display_unit\">Display unit:</p><select class=\"select_unit\"><option value=\"1\">Degrees</option><option value=\"2\">Rotations</option><option value=\"3\">Radians</option><option value=\"4\">Millimeters</option></select><p>Current millimeters per rotation:<span class=\"esp_val\">NaN</span>.</p><p class=\"set_mm_per_rot_txt\">Set millimeters per rotation: <input type=\"number\" class=\"set_mm_per_rot\"></p><button class=\"submit_set_mm_per_rot\">Set</button><div hidden=\"hidden\" class=\"home_found\"><p>Current position:<span class=\"angle_cur\">0</span><span class=\"unit\">deg</span>.</p><p>Absolute position:<span class=\"angle_abs\">0</span><span class=\"unit\">deg.</span></p><input type=\"number\" class=\"abs_val_input\" value=\"0\"><select class=\"abs_unit\"><option value=\"1\">Degrees</option><option value=\"2\">Rotations</option><option value=\"3\">Radians</option><option value=\"4\">Millimeters</option></select><button class=\"set_abs\">Set</button><br><button class=\"toggle_sign_abs\">Toggle absolute value direction</button><button class=\"abs_submit_button\">Move to target value.</button></div><p class=\"home_not_found\">Home not found, until zero position is set, absolute movement and value is not available.</p><p>Find home position:</p><button class=\"submit_find_home\">Find home</button><p>Relative change:<span class=\"angle_rel\">0</span><span class=\"unit\">deg</span>.</p><p class=\"display_unit\">Relative unit:</p><input type=\"number\" class=\"rel_val_input\" value=\"0\"><select class=\"rel_unit\"><option value=\"1\">Degrees</option><option value=\"2\">Rotations</option><option value=\"3\">Radians</option><option value=\"4\">Millimeters</option></select><button class=\"set_rel\">Set</button><br><button class=\"toggle_sign_rel\">Toggle relative value direction</button><button class=\"rel_submit_button\">Move by set value.</button></body></html>";
const String ctrl_script_js = "const elem_div_home_found=document.getElementsByClassName(\"home_found\")[0];const elem_p_home_not_found=document.getElementsByClassName(\"home_not_found\")[0];const elem_submit_abs=document.getElementsByClassName(\"abs_submit_button\")[0];const elem_abs_unit=document.getElementsByClassName(\"abs_unit\")[0];const elem_toggle_sign_abs=document.getElementsByClassName(\"toggle_sign_abs\")[0];const elem_angle_abs=document.getElementsByClassName(\"angle_abs\")[0];const elem_angle_abs_input=document.getElementsByClassName(\"abs_val_input\")[0];const elem_angle_abs_input_set=document.getElementsByClassName(\"set_abs\")[0];elem_angle_abs_input_set.addEventListener(\"click\",click_set_abs_input);elem_toggle_sign_abs.addEventListener(\"click\",click_toggle_sign_abs);elem_submit_abs.addEventListener(\"click\",()=>click_submit_abs(),!1);elem_abs_unit.addEventListener(\"change\",select_unit_abs);const elem_submit_rel=document.getElementsByClassName(\"rel_submit_button\")[0];const elem_toggle_sign_rel=document.getElementsByClassName(\"toggle_sign_rel\")[0];const elem_rel_unit=document.getElementsByClassName(\"rel_unit\")[0];const elem_angle_rel=document.getElementsByClassName(\"angle_rel\")[0];const elem_angle_rel_input=document.getElementsByClassName(\"rel_val_input\")[0];const elem_angle_rel_input_set=document.getElementsByClassName(\"set_rel\")[0];elem_angle_rel_input_set.addEventListener(\"click\",click_set_rel_input);elem_toggle_sign_rel.addEventListener(\"click\",click_toggle_sign_rel);elem_submit_rel.addEventListener(\"click\",()=>click_submit_rel(),!1);elem_rel_unit.addEventListener(\"change\",select_unit_rel);const elem_angle_cur=document.getElementsByClassName(\"angle_cur\")[0];const elem_find_home=document.getElementsByClassName(\"submit_find_home\")[0];elem_find_home.addEventListener(\"click\",()=>click_submit_find_home(),!1);const elem_con_stat=document.getElementsByClassName(\"con_stat\")[0];var disp_coef_deg_per_unit=1.0;const elem_select_unit=document.getElementsByClassName(\"select_unit\")[0];const elem_unit_text=document.getElementsByClassName(\"unit\");const elem_current_mm_per_rot=document.getElementsByClassName(\"esp_val\")[0];const elem_set_mm_per_rot=document.getElementsByClassName(\"set_mm_per_rot\")[0];const elem_submit_set_mm_per_rot=document.getElementsByClassName(\"submit_set_mm_per_rot\")[0];var current_mm_per_rot=NaN;elem_select_unit.addEventListener(\"change\",select_unit_change);elem_submit_set_mm_per_rot.addEventListener(\"click\",click_submit_mm_per_rot);var abs_angle=parseInt(elem_angle_cur.innerHTML)*disp_coef_deg_per_unit;var rel_angle=parseInt(elem_angle_cur.innerHTML)*disp_coef_deg_per_unit;function select_unit_abs(){if(isNaN(current_mm_per_rot)&&elem_abs_unit.value==\"4\"){elem_angle_abs_input_set.setAttribute(\"disabled\",\"true\")}else{elem_angle_abs_input_set.removeAttribute(\"disabled\")}}\nfunction select_unit_rel(){if(isNaN(current_mm_per_rot)&&elem_rel_unit.value==\"4\"){elem_angle_rel_input_set.setAttribute(\"disabled\",\"true\")}else{elem_angle_rel_input_set.removeAttribute(\"disabled\")}}\nfunction select_unit_change(){let unit_text=\"\";switch(elem_select_unit.value){case \"1\":disp_coef_deg_per_unit=1.0;unit_text=\"deg\";break;case \"2\":disp_coef_deg_per_unit=360.0;unit_text=\"rot\";break;case \"3\":disp_coef_deg_per_unit=360.0/(2.0*Math.PI);unit_text=\"rad\";break;case \"4\":disp_coef_deg_per_unit=360.0/current_mm_per_rot;unit_text=\"mm\";break}\nfor(var i=0;i<elem_unit_text.length;i++){elem_unit_text[i].innerHTML=unit_text}\nrefresh_unit_vals()}\nfunction refresh_unit_vals(){elem_angle_abs.innerHTML=(abs_angle/disp_coef_deg_per_unit).toFixed(2);elem_angle_rel.innerHTML=(rel_angle/disp_coef_deg_per_unit).toFixed(2)}\nfunction click_set_abs_input(){let local_unit_mult=disp_coef_deg_per_unit;if(isNaN(disp_coef_deg_per_unit)){return}\nswitch(elem_abs_unit.value){case \"1\":local_unit_mult=1.0;break;case \"2\":local_unit_mult=360.0;break;case \"3\":local_unit_mult=360.0/(2.0*Math.PI);break;case \"4\":if(isNaN(current_mm_per_rot)){return}\nlocal_unit_mult=360.0/current_mm_per_rot;break}\nabs_angle=parseInt(elem_angle_abs_input.value)*local_unit_mult;refresh_unit_vals()}\nfunction click_set_rel_input(){let local_unit_mult=disp_coef_deg_per_unit;if(isNaN(disp_coef_deg_per_unit)){return}\nswitch(elem_rel_unit.value){case \"1\":local_unit_mult=1.0;break;case \"2\":local_unit_mult=360.0;break;case \"3\":local_unit_mult=360.0/(2.0*Math.PI);break;case \"4\":if(isNaN(current_mm_per_rot)){return}\nlocal_unit_mult=360.0/current_mm_per_rot;break}\nrel_angle=parseInt(elem_angle_rel_input.value)*local_unit_mult;refresh_unit_vals()}\nfunction click_toggle_sign_abs(){abs_angle*=-1;refresh_unit_vals()}\nfunction click_toggle_sign_rel(){rel_angle*=-1;refresh_unit_vals()}\nasync function click_submit_mm_per_rot(){const response=await fetch(\"/set_mm_per_rot\",{method:\"POST\",headers:{\"Content-Type\":\"application/x-www-form-urlencoded\",},body:new URLSearchParams({mm_per_rot:elem_set_mm_per_rot.value}),})}\nasync function click_submit_find_home(){const response=await fetch(\"/find_home\")}\nasync function click_submit_rel(){const response=await fetch(\"/by_angle\",{method:\"POST\",headers:{\"Content-Type\":\"application/x-www-form-urlencoded\",},body:new URLSearchParams({angle_val:rel_angle}),})}\nasync function click_submit_abs(){const response=await fetch(\"/set_angle\",{method:\"POST\",headers:{\"Content-Type\":\"application/x-www-form-urlencoded\",},body:new URLSearchParams({angle_val:abs_angle}),})}\nasync function refresh_angle(){url=\"fetch_angle\";try{const response=await fetch(url);if(!response.ok){throw new Error(`Response status: ${response.status}`)}\nconst json=await response.json();let angle=parseFloat(json.angle_val);current_mm_per_rot=parseFloat(json.mm_per_rot);elem_current_mm_per_rot.innerHTML=current_mm_per_rot;if(!isNaN(angle)){elem_angle_cur.innerHTML=angle/disp_coef_deg_per_unit;elem_div_home_found.removeAttribute(\"hidden\");elem_p_home_not_found.setAttribute(\"hidden\",\"hidden\")}else{elem_p_home_not_found.removeAttribute(\"hidden\");elem_div_home_found.setAttribute(\"hidden\",\"hidden\")}\nelem_submit_set_mm_per_rot.removeAttribute(\"hidden\");elem_set_mm_per_rot.removeAttribute(\"disabled\");elem_con_stat.innerHTML=\"ESP32 connected\";select_unit_change()}catch(error){console.error(\"Angle fetch error: \"+error.message);elem_con_stat.innerHTML=\"ESP32 disconnected.\";current_mm_per_rot=NaN;elem_current_mm_per_rot.innerHTML=current_mm_per_rot;elem_submit_set_mm_per_rot.setAttribute(\"hidden\",\"hidden\");elem_set_mm_per_rot.setAttribute(\"disabled\",\"true\");select_unit_change()}}\nsetInterval(refresh_angle,250);console.log(\"Interval enabled.\")";
const String web_ctrl_prefix = "/ctrl";

#ifdef simple_wifi
const String simple_wifi_html = "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\"><title>WiFi Connection</title><script defer=\"defer\" src=\"simple_connect_js.js\"></script></head><body><p>Connection status:<span class=\"con_stat\">ESP32 disconnected</span>.</p><p class=\"ip_p\" hidden>IP:<span class=\"con_ip\"></span>.</p><form class=\"wifi_form\"><label>SSID: <input type=\"text\" name=\"ssid\"></label><br><label>Password: <input type=\"password\" name=\"passw\"></label><br><input class=\"wifi_submit_button\" type=\"button\" value=\"Connect\"></form><div class=\"saved_wifi_div\" hidden><p>There exists saved WiFi.</p><button class=\"rem_saved_wifi_button\">Remove saved WiFi & disconnect.</button></div></body></html>";
const String simple_wifi_js = "const elem_connected=document.getElementsByClassName(\"con_stat\")[0];const elem_con_ip=document.getElementsByClassName(\"con_ip\")[0];const elem_ip_p=document.getElementsByClassName(\"ip_p\")[0];const elem_wifi_form=document.getElementsByClassName(\"wifi_form\")[0];const elem_wifi_submit_button=document.getElementsByClassName(\"wifi_submit_button\")[0];const elem_saved_wifi_div=document.getElementsByClassName(\"saved_wifi_div\")[0];const elem_rem_saved_wifi_button=document.getElementsByClassName(\"rem_saved_wifi_button\")[0];elem_wifi_submit_button.addEventListener(\"click\",form_submit);elem_rem_saved_wifi_button.addEventListener(\"click\",rem_saved_wifi);async function rem_saved_wifi(){try{const response=await fetch(\"rem_saved_wifi\")\nif(!response.ok){throw new Error(`Response status: ${response.status}`)}}catch(error){console.error(error.message);elem_connected.innerHTML=\"ESP32 disconnected (or some other remove saved WiFi error)\"}}\nasync function form_submit(){try{const response=await fetch(\"simple_connect\",{method:\"POST\",headers:{\"Content-Type\":\"application/x-www-form-urlencoded\",},body:new URLSearchParams(new FormData(elem_wifi_form)),});if(!response.ok){throw new Error(`Response status: ${response.status}`)}}catch(error){console.error(error.message);elem_connected.innerHTML=\"ESP32 disconnected (or some other form submit error)\"}}\nasync function check_connect(){try{const response=await fetch(\"simple_check_connect\");if(!response.ok){throw new Error(`Response status: ${response.status}`)}\nlet json=await response.json();console.log(json);switch(json.con_stat){case \"0\":elem_connected.innerHTML=\"WiFi connected\";elem_con_ip.innerHTML=json.ip\nelem_ip_p.removeAttribute(\"hidden\");break;case \"1\":elem_connected.innerHTML=\"attempted WiFi connection failed\";elem_ip_p.setAttribute(\"hidden\",\"hidden\");break;case \"2\":elem_connected.innerHTML=\"WiFi disconnected\";elem_ip_p.setAttribute(\"hidden\",\"hidden\");break}\nif(json.saved==\"1\"){elem_saved_wifi_div.removeAttribute(\"hidden\")}else{elem_saved_wifi_div.setAttribute(\"hidden\",\"hidden\")}}catch(error){console.error(error.message);elem_connected.innerHTML=\"ESP32 disconnected\"}}\nsetInterval(check_connect,250);console.log(\"Interval enabled.\")";
#endif
#endif

// WiFi station defintions
#ifndef suff_mem
String STA_SSID = "testspot";
String STA_PASS = "12345678";
#endif

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
const int mqtt_server_port = 1883;
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
uint16_t microstepping = 16;
const int steps_per_rot_full_step = 400;
const float deg_per_teeth = 360.0f / 32;
int steps_per_rot = microstepping * steps_per_rot_full_step;
const uint16_t RUN_CURRENT = 4000;
const float R_SENSE = 0.11f;
const unsigned char TMC2209_ADDR = 0b11;
#define SERIAL_PORT Serial1
TMC2209Stepper stepper_driver = TMC2209Stepper(&SERIAL_PORT, R_SENSE, TMC2209_ADDR);
bool home_found = false;
ESP_FlexyStepper stepper = ESP_FlexyStepper();
Preferences pos_wifi_pref;
SemaphoreHandle_t rotate_command_mutex;
const char mm_per_rot_key[] = "mm_per_rot";

// Nextion display
#ifdef board_firebeetle // FireBeetle
#define pin_dis_tx GPIO_NUM_17
#define pin_dis_rx GPIO_NUM_16
#elif defined board_devkitv1 // DEVKIT V1
#define pin_dis_tx GPIO_NUM_18
#define pin_dis_rx GPIO_NUM_23
#endif
HardwareSerial hw_disp = HardwareSerial(2);
std::queue<String> commands_queue;
SemaphoreHandle_t commands_queue_mutex;
int man_ctrl_task_dir = 0;
TaskHandle_t man_ctrl_task_handle = NULL;
TaskHandle_t update_pos_task_handle = NULL;
float man_ctrl_multiple = 1.0;

// Nextion display buttons
#define POS0 23
#define POS1 6
#define POS2 7
#define POS3 8
#define POS4 9
#define POS5 10
#define POS6 11
#define SET1 13
#define SET2 14
#define SET3 15
#define SET4 16
#define SET5 17
#define SET6 18
#define LEFT 19
#define HOME 21
#define RIGHT 20
#define ANG_I 22
#define M_10X 24
#define M_01X 25
#define M_0X1 26
#define CLEAR 27

// Upper Microswitch
#ifdef board_firebeetle // FireBeetle
#define pin_high_switch GPIO_NUM_18
#elif defined board_devkitv1 // DEVKIT V1
#define pin_high_switch GPIO_NUM_13
#endif
#define high_switch_polarity_off HIGH

// Lower Microswitch
#ifdef board_firebeetle // FireBeetle
#define pin_low_switch GPIO_NUM_21
#elif defined board_devkitv1 // DEVKIT V1
#define pin_low_switch GPIO_NUM_12
#endif
#define low_switch_polarity_off HIGH
double rotations_to_lowest = 0.0;
TaskHandle_t rel_rot_task_handle = NULL;

// DS18B20
#ifdef board_firebeetle // FireBeetle
#define pin_temp_sens GPIO_NUM_14
#elif defined board_devkitv1 // DEVKIT V1
#define pin_temp_sens GPIO_NUM_27
#endif
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(pin_temp_sens);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

// BME680
#define SEALEVELPRESSURE_HPA (1013.25)
#define bme680_addr 0x77
Adafruit_BME680 bme;

// Wrapper function to init Nextion display
void nextion_init();

// Wrapper function to init BME680 and DS18B20 sensors
void bme680_ds18b20_init();

// Wrapper function to init rope pulling
void rope_pull_init();

// Helper function to send command to the display
void send_string(String data);

// Helper function to disable element on the screen
void disable_element(int num);

// Helper function to enable element on the screen
void enable_element(int num);

// Helper function to update text value of an object
void put_out(String var, String val);

// Function to return the current angle of the motor
long get_angle();

// Update the reading of current position
void update_pos(void *params);

// Helper function to re-enable previous object on the screen
void enable_previous();

// Helper function to change page, without interfering with enabled/disabled controls
void change_page(int page);

// Task function for sensor readings refresh
void print_sensors(void *params);

// Position (in steps) to angle conversion
long pos_to_angle(long pos);

// Task for manually rotating the stepper motor
void man_ctrl_task(void *params);

// Task for handling queued commands form the display
void uart_mon_handle_commands(void *params);

// Callback handler for reading and queueing commands/data from the display
void uart_mon_read(void);

// Function for finding home position.
void find_home(bool special);
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

// Web control angle callback
#ifdef suff_mem
ArRequestHandlerFunction web_ctrl_handle_root(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_js(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_fetch_angle(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_set_angle(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_by_angle(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_find_home(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_set_mm_per_rot(AsyncWebServerRequest *request);

#ifdef simple_wifi
ArRequestHandlerFunction web_ctrl_handle_connect_simple(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_rem_saved_wifi(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_connected_simple(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_simple_html(AsyncWebServerRequest *request);
ArRequestHandlerFunction web_ctrl_handle_simple_js(AsyncWebServerRequest *request);
void wifi_simple_connected_event(WiFiEvent_t event);
String temp_wifi_ssid = "";
String temp_wifi_pass = "";
void reconn_wifi_simple(void *params);
#endif

#endif

void adjust_position_to_whole_teeth();

void setup()
{
    // Start the Serial Monitor
    Serial.begin(115200);

    // Servo
    pinMode(pin_servo, OUTPUT);
    servo_lock.attach(pin_servo, Servo::CHANNEL_NOT_ATTACHED, 0, 180, 500, 2500, 50);

    // // Start the DS18B20 sensor
    // sensors.begin();

    // // Start BME680
    // bme.begin(bme680_addr);
    // // Set up oversampling and filter initialization
    // bme.setTemperatureOversampling(BME680_OS_8X);
    // bme.setHumidityOversampling(BME680_OS_2X);
    // bme.setPressureOversampling(BME680_OS_4X);
    // bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    // bme.setGasHeater(320, 150); // 320*C for 150 ms
    // bme.setODR(BME68X_ODR_NONE);

    // // Start Nextion display UART
    // hw_disp.setPins(pin_dis_rx, pin_dis_tx);
    // hw_disp.begin(115200);
    // send_string("rest");
    // hw_disp.onReceive(uart_mon_read);
    commands_queue_mutex = xSemaphoreCreateMutex();

    // Set STEP/DIR/EN pins OUTPUT, LOW
    pinMode(pin_step, OUTPUT);
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_en, OUTPUT);
    digitalWrite(pin_en, LOW);
    digitalWrite(pin_step, LOW);
    digitalWrite(pin_dir, HIGH);

    // Lock servo at the beginning
    servo_lock.write(servo_lock_angle);

    // Start stepper motor control
    SERIAL_PORT.setPins(pin_mot_rx, pin_mot_tx);
    SERIAL_PORT.begin(115200);
    stepper_driver.begin();
    stepper_driver.toff(5);
    stepper_driver.rms_current(RUN_CURRENT);
    stepper_driver.microsteps(microstepping);
    stepper_driver.pwm_autoscale(true);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Start stepper motor lib
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

    pos_wifi_pref.begin("pos_wifi_pref", false);

    // Set switch pin according to polarity in off state
    if (high_switch_polarity_off == HIGH)
    {
        pinMode(pin_high_switch, INPUT_PULLUP);
    }
    else if (high_switch_polarity_off == LOW)
    {
        pinMode(pin_high_switch, INPUT_PULLDOWN);
    }
    if (low_switch_polarity_off == HIGH)
    {
        pinMode(pin_low_switch, INPUT_PULLUP);
    }
    else if (low_switch_polarity_off == LOW)
    {
        pinMode(pin_low_switch, INPUT_PULLDOWN);
    }

// Enable WiFi
#ifndef suff_mem
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(STA_SSID, STA_PASS);
    Serial.print("\nConnecting to WiFi.");
    while (!WiFi.isConnected())
    {
        Serial.print("\nConnecting to WiFi.");
    }
    Serial.println();
    Serial.print("Connected: ");
    Serial.println(WiFi.localIP());
#endif
#ifdef suff_mem
    WiFi.mode(WIFI_MODE_APSTA);
    WiFi.softAP(AP_SSID, AP_PASS);
#endif

    WiFi.setHostname(hostname);
    MDNS.begin(hostname);

// Start WiFi Web Manager
#ifdef suff_mem
    // wifi_web_manager::init(ws, &pos_wifi_pref, "", "/", 1-cpu);
    Serial.print("AP: ");
    Serial.println(WiFi.softAPIP());
#endif

#ifdef suff_mem
    ws.on((web_ctrl_prefix + "/").c_str(), web_ctrl_handle_root);
    ws.on((web_ctrl_prefix + "/script.js").c_str(), web_ctrl_handle_js);
    ws.on((web_ctrl_prefix + "/fetch_angle").c_str(), web_ctrl_handle_fetch_angle);
    ws.on("/set_angle", web_ctrl_handle_set_angle);
    ws.on("/by_angle", web_ctrl_handle_by_angle);
    ws.on("/find_home", web_ctrl_handle_find_home);
    ws.on("/set_mm_per_rot", web_ctrl_handle_set_mm_per_rot);
    ws.on((web_ctrl_prefix).c_str(), [](AsyncWebServerRequest *request)
          { request->redirect(web_ctrl_prefix + "/"); });
#ifdef simple_wifi
    ws.on("/simple_connect", web_ctrl_handle_connect_simple);
    ws.on("/rem_saved_wifi", web_ctrl_handle_rem_saved_wifi);
    ws.on("/simple_check_connect", web_ctrl_handle_connected_simple);
    ws.on("/", web_ctrl_handle_simple_html);
    ws.on("/simple_connect_js.js", web_ctrl_handle_simple_js);
    WiFi.onEvent(wifi_simple_connected_event, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
    // xTaskCreatePinnedToCore(reconn_wifi_simple, "wifi_sim_rec", 15 * 1024, NULL, 1, NULL, 1 - cpu);
#endif
    ws.begin();
#endif

    // Create rotate command mutex
    rotate_command_mutex = xSemaphoreCreateBinary();
    assert(rotate_command_mutex != NULL);
    Serial.println(uxSemaphoreGetCount(rotate_command_mutex));
    xSemaphoreGive(rotate_command_mutex);
    Serial.println(uxSemaphoreGetCount(rotate_command_mutex));

    // Register MQTT handlers
    job_manager.register_command(mqtt_server_dev_command_rotate_angle, mqtt_callback_rotate_angle);
    job_manager.register_command(mqtt_server_dev_command_rotate_to_angle, mqtt_callback_rotate_to_angle);
    job_manager.register_command(mqtt_server_dev_command_find_home, mqtt_callback_find_home);
    job_manager.init();
    Serial.println("Trying to connect to MQTT server.");
    iotIs.connect(mqtt_server_dev_id, mqtt_server_url, mqtt_server_port);
    Serial.println("Connected.");

    // Set sensor task
    xTaskCreatePinnedToCore(print_sensors, "Print sensors", 15 * 1024, NULL, 1, NULL, 1 - cpu);

    // Set UART command task
    xTaskCreatePinnedToCore(uart_mon_handle_commands, "Handle UART cmd", 15 * 1024, NULL, 1, NULL, 1 - cpu);

    // Position update task
    xTaskCreatePinnedToCore(update_pos, "UPDATE", 15 * 1024, NULL, 1, &update_pos_task_handle, cpu);
    Serial.println("Initialised.");
}

// TREBA PREROBIT POS0-POS6!!!!!!
//
//
//
//
/////////////////////////////////

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
    // vTaskDelete(NULL);
}

#ifdef suff_mem

#ifdef simple_wifi

// void reconn_wifi_simple(void *params)
// {
//     while (1)
//     {
//         if (WiFi.isConnected())
//         {
//             Serial.println("WiFi connected, waiting.");
//             vTaskDelay(10000 / portTICK_PERIOD_MS);
//         }
//         else if (pos_wifi_pref.isKey("wifi_ssid"))
//         {
//             String ssid = pos_wifi_pref.getString("wifi_ssid");
//             String pass = pos_wifi_pref.getString("wifi_pass");
//             WiFi.disconnect(false, true);
//             WiFi.begin(ssid, pass);
//             vTaskDelay(500 / portTICK_PERIOD_MS);
//         }
//         else
//         {
//             vTaskDelay(10000 / portTICK_PERIOD_MS);
//         }
//     }
// }

ArRequestHandlerFunction web_ctrl_handle_connect_simple(AsyncWebServerRequest *request)
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

ArRequestHandlerFunction web_ctrl_handle_rem_saved_wifi(AsyncWebServerRequest *request)
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

ArRequestHandlerFunction web_ctrl_handle_connected_simple(AsyncWebServerRequest *request)
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

ArRequestHandlerFunction web_ctrl_handle_simple_html(AsyncWebServerRequest *request)
{
    Serial.println("Simple WiFi: Handling HTML.");
    request->send(200, "text/html", simple_wifi_html);
    return 0;
}

ArRequestHandlerFunction web_ctrl_handle_simple_js(AsyncWebServerRequest *request)
{
    Serial.println("Simple WiFi: Handling JS.");
    request->send(200, "text/javascript", simple_wifi_js);
    return 0;
}

void wifi_simple_connected_event(WiFiEvent_t event)
{
    Serial.println("Simple WiFi: Saving usable WiFi.");
    Serial.println(temp_wifi_ssid);
    Serial.println(temp_wifi_pass);
    pos_wifi_pref.putString("wifi_ssid", temp_wifi_ssid);
    pos_wifi_pref.putString("wifi_pass", temp_wifi_pass);
}

#endif

void adjust_position_to_whole_teeth()
{
    double loc_pos_rev = ((double)((long long)(stepper.getCurrentPositionInRevolutions() * 360.0f / deg_per_teeth)) + 1.0f) * deg_per_teeth / 360.0f;
    stepper.setCurrentPositionInRevolutions(loc_pos_rev);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

ArRequestHandlerFunction web_ctrl_handle_set_mm_per_rot(AsyncWebServerRequest *request)
{
    Serial.println("Web control: Handling set millimeters per degree.");
    pos_wifi_pref.putFloat(mm_per_rot_key, request->arg("mm_per_rot").toDouble());
    request->send(200, "text/plain", "okikoki");
    return 0;
}

ArRequestHandlerFunction web_ctrl_handle_find_home(AsyncWebServerRequest *request)
{
    Serial.println("Web control: find home.");
    xTaskCreatePinnedToCore(find_home, "find_home_t", 10 * 1024, (void *)false, 1, NULL, cpu);
    request->redirect(web_ctrl_prefix);
    return 0;
}

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

ArRequestHandlerFunction web_ctrl_handle_root(AsyncWebServerRequest *request)
{
    Serial.println("Web control: Handling root.");
    request->send(200, "text/html", ctrl_index_html);
    return 0;
}

ArRequestHandlerFunction web_ctrl_handle_js(AsyncWebServerRequest *request)
{
    Serial.println("Web control: Handling js.");
    request->send(200, "text/javascript", ctrl_script_js);
    return 0;
}

ArRequestHandlerFunction web_ctrl_handle_set_angle(AsyncWebServerRequest *request) // z nejakeho dovodu polovica?
{
    Serial.println("Web control: Handling set angle.");
    String angle_str = request->arg("angle_val");
    Serial.println(angle_str);
    double *p = new double((angle_str.toDouble() + deg_per_teeth) / 360.0f);
    xTaskCreatePinnedToCore(task_rotate_abs, "abs_rot_t", 10 * 1024, (void *)p, 1, NULL, cpu);
    request->redirect(web_ctrl_prefix);
    return 0;
}

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
#endif

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
        angle = stepper.getCurrentPositionInRevolutions() + angle;
        min(max(angle, 0.0), rotations_to_lowest);
        stepper.moveToPositionInRevolutions(angle);
    }

    else
    {
        esp_task_wdt_init(60, false);
        float init = stepper.getCurrentPositionInRevolutions();
        if (angle >= 0)
        {
            stepper.startJogging(1);
            while (((stepper.getCurrentPositionInRevolutions() - init) < angle) && (digitalRead(pin_low_switch) == low_switch_polarity_off))
            {
                stepper.processMovement();
                Serial.println("Processing movement.");
            }
            stepper.stopJogging();
            if (digitalRead(pin_low_switch) != low_switch_polarity_off)
            {
                Serial.println("Low switch hit.");
                stepper.moveRelativeInRevolutions(-1);
            }
            else
            {
                Serial.println("Movement finished.");
            }
        }
        else
        {
            stepper.startJogging(-1);
            while (((stepper.getCurrentPositionInRevolutions() - init) > angle) && (digitalRead(pin_high_switch) == high_switch_polarity_off))
            {
                stepper.processMovement();
                Serial.println("Processing movement.");
            }
            stepper.stopJogging();
            if (digitalRead(pin_high_switch) != high_switch_polarity_off)
            {
                Serial.println("High switch hit.");
                stepper.moveRelativeInRevolutions(1);
            }
            else
            {
                Serial.println("Movement finished.");
            }
        }
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
        angle = min(max(angle, 0.0), rotations_to_lowest);
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

// Helper function to send command to the display
void send_string(String data)
{
    hw_disp.write(data.c_str());
    hw_disp.write(0xff);
    hw_disp.write(0xff);
    hw_disp.write(0xff);
}

// Helper function to disable element on the screen
void disable_element(int num)
{
    send_string("vis " + String(num) + ",0");
}

// Helper function to enable element on the screen
void enable_element(int num)
{
    send_string("vis " + String(num) + ",1");
}

// Helper function to update text value of an object
void put_out(String var, String val)
{
    send_string(var + ".txt=\"" + val + "\"");
}

// Function to return the current angle of the motor
long get_angle()
{
    long temp = (long)(-stepper.getCurrentPositionInRevolutions() * 360.0f);
    temp = ((temp % 360) + 360) % 360;
    return temp;
}

// Update the reading of current position
void update_pos(void *params)
{
    while (1)
    {
        put_out("t5", String(get_angle()));
        vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}

// Helper function to re-enable previous object on the screen
void enable_previous()
{
    xSemaphoreTake(rotate_command_mutex, portMAX_DELAY);
    if (home_found)
    {
        enable_element(POS0);
        if (pos_wifi_pref.isKey("set1"))
        {
            enable_element(POS1);
        }
        if (pos_wifi_pref.isKey("set2"))
        {
            enable_element(POS2);
        }
        if (pos_wifi_pref.isKey("set3"))
        {
            enable_element(POS3);
        }
        if (pos_wifi_pref.isKey("set4"))
        {
            enable_element(POS4);
        }
        if (pos_wifi_pref.isKey("set5"))
        {
            enable_element(POS5);
        }
        if (pos_wifi_pref.isKey("set6"))
        {
            enable_element(POS6);
        }
    }
    xSemaphoreGive(rotate_command_mutex);
    enable_element(CLEAR);
    enable_element(SET1);
    enable_element(SET2);
    enable_element(SET3);
    enable_element(SET4);
    enable_element(SET5);
    enable_element(SET6);
    enable_element(ANG_I);
    enable_element(LEFT);
    enable_element(RIGHT);
    enable_element(M_10X);
    enable_element(M_01X);
    enable_element(M_0X1);
}

// Helper function to change page, without interfering with enabled/disabled controls
void change_page(int page)
{
    send_string("page " + String(page));
    if (page == 0)
    {
        enable_previous();
    }
}

// Task function for sensor readings refresh
void print_sensors(void *params)
{
    // Set watchdog to 30s
    esp_task_wdt_init(30, false);

    while (1)
    {
        sensors.requestTemperatures();
        bme.performReading();
        double temperatureC = sensors.getTempCByIndex(0);
        temperatureC = (temperatureC == -127.0) ? -300.0f : temperatureC;
        double temperatureBME = bme.temperature;
        double pressure = bme.pressure;
        double humidity = bme.humidity;
        double gas_resistance = bme.gas_resistance;
        temperatureBME = (pressure == 0.0) ? -300.0f : temperatureBME;
        humidity = (pressure == 0.0) ? -300.0f : humidity;
        gas_resistance = (pressure == 0.0) ? -300.0f : gas_resistance;
        pressure = (pressure == 0.0) ? -300.0f : pressure;
        double angle = get_angle();
        put_out("t0", "Temperature: " + String(temperatureBME));
        put_out("t1", "Pressure: " + String((long)pressure));
        put_out("t2", "Humidity: " + String(humidity));
        put_out("t3", "Gas resistance: " + String((long)gas_resistance));
        put_out("t4", "DS18B20: " + String(temperatureC));
        if (WiFi.isConnected())
        {
            iotIs.send_data(mqtt_server_dev_ds18b20, temperatureC);
            iotIs.send_data(mqtt_server_dev_bme680temp, temperatureBME);
            iotIs.send_data(mqtt_server_dev_bme680press, pressure);
            iotIs.send_data(mqtt_server_dev_bme680hum, humidity);
            iotIs.send_data(mqtt_server_dev_bme680res, gas_resistance);
            xSemaphoreTake(rotate_command_mutex, portMAX_DELAY);
            if (home_found)
            {
                iotIs.send_data(mqtt_server_dev_stepper_angle, angle);
                Serial.println(angle);
            }
            else
            {
                iotIs.send_data(mqtt_server_dev_stepper_angle, -300.0f);
            }
            xSemaphoreGive(rotate_command_mutex);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

// Position (in steps) to angle conversion
long pos_to_angle(long pos)
{
    long angle = 0;
    angle = (long)((float)pos / ((float)steps_per_rot) * 360.0f);
    angle = ((angle % 360) + 360) % 360;
    return angle;
}

// Function to find home (zero) position by microswitch
void find_home(bool special = false)
{
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
        if (special)
        {
            put_out("pol0", "Poloha: " + String(pos_to_angle(pos)));
        }
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
        if (special)
        {
            put_out("pol0", "Poloha: " + String(pos_to_angle(pos)));
        }
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
}

// Task to find home
void find_home(void *params)
{
    // Set watchdog to 30s
    esp_task_wdt_init(30, false);

    bool special = false;
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
        if (special)
        {
            put_out("pol0", "Poloha: " + String(pos_to_angle(pos)));
        }
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
        if (special)
        {
            put_out("pol0", "Poloha: " + String(pos_to_angle(pos)));
        }
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

// Task for manually rotating the stepper motor
void man_ctrl_task(void *params)
{
    // Set watchdog to 30s
    esp_task_wdt_init(30, false);

    while (1)
    {
        digitalWrite(pin_dir, (man_ctrl_task_dir == -1) ? HIGH : LOW);
        digitalWrite(pin_step, LOW);
        digitalWrite(pin_step, HIGH);
        delayMicroseconds(1);
        digitalWrite(pin_step, LOW);
        stepper.setCurrentPositionInSteps(stepper.getCurrentPositionInSteps() + 1 * man_ctrl_task_dir);
        delayMicroseconds((long)(5000.0 / (man_ctrl_multiple)));
    }
}

// Task for handling queued commands form the display
void uart_mon_handle_commands(void *params)
{
    // Set watchdog to 30s
    esp_task_wdt_init(30, false);

    while (1)
    {
        xSemaphoreTake(commands_queue_mutex, portMAX_DELAY);
        if (commands_queue.size() && commands_queue.front().endsWith("\xff\xff\xff"))
        {
            String cmd_in = commands_queue.front();
            commands_queue.pop();
            xSemaphoreGive(commands_queue_mutex);
            if (cmd_in[0] == 0x65)
            {
                if (cmd_in[1] == 0x0)
                {
                    if (cmd_in[3] == 0x00)
                    {
                        switch (cmd_in[2])
                        {
                        case CLEAR:
                            pos_wifi_pref.clear();
                            for (int i = 0; i < 6; i++)
                            {
                                disable_element(POS1 + i);
                            }
                            enable_previous();
                            break;
                        case HOME:
                            change_page(1);
                            find_home(1);
                            change_page(0);
                            put_out("t5", "0");
                            break;
                        case SET1:
                            pos_wifi_pref.putFloat("set1", stepper.getCurrentPositionInRevolutions() * 360.0f);
                            enable_element(POS1);
                            break;
                        case SET2:
                            pos_wifi_pref.putFloat("set2", stepper.getCurrentPositionInRevolutions() * 360.0f);
                            enable_element(POS2);
                            break;
                        case SET3:
                            pos_wifi_pref.putFloat("set3", stepper.getCurrentPositionInRevolutions() * 360.0f);
                            enable_element(POS3);
                            break;
                        case SET4:
                            pos_wifi_pref.putFloat("set4", stepper.getCurrentPositionInRevolutions() * 360.0f);
                            enable_element(POS4);
                            break;
                        case SET5:
                            pos_wifi_pref.putFloat("set5", stepper.getCurrentPositionInRevolutions() * 360.0f);
                            enable_element(POS5);
                            break;
                        case SET6:
                            pos_wifi_pref.putFloat("set6", stepper.getCurrentPositionInRevolutions() * 360.0f);
                            enable_element(POS6);
                            break;
                        // TREBA PREROBIT POS0-POS6!!!!!!
                        case POS0:
                            change_page(1);
                            put_out("pol0", "Ciel: Home");
                            xSemaphoreTake(rotate_command_mutex, portMAX_DELAY);
                            stepper.moveToPositionInSteps(0);
                            xSemaphoreGive(rotate_command_mutex);
                            change_page(0);
                            break;
                        case POS1:
                            change_page(1);
                            put_out("pol0", "Ciel: " + String(-pos_wifi_pref.getFloat("set1")));
                            xSemaphoreTake(rotate_command_mutex, portMAX_DELAY);
                            stepper.moveRelativeInRevolutions(pos_wifi_pref.getFloat("set1") / 360.0f);
                            xSemaphoreGive(rotate_command_mutex);
                            change_page(0);
                            put_out("t5", String(get_angle()));
                            break;
                        case POS2:
                            change_page(1);
                            put_out("pol0", "Ciel: " + String(-pos_wifi_pref.getFloat("set2")));
                            xSemaphoreTake(rotate_command_mutex, portMAX_DELAY);
                            stepper.moveRelativeInRevolutions(pos_wifi_pref.getFloat("set2") / 360.0f);
                            xSemaphoreGive(rotate_command_mutex);
                            change_page(0);
                            put_out("t5", String(get_angle()));
                            break;
                        case POS3:
                            change_page(1);
                            put_out("pol0", "Ciel: " + String(-pos_wifi_pref.getFloat("set3")));
                            xSemaphoreTake(rotate_command_mutex, portMAX_DELAY);
                            stepper.moveRelativeInRevolutions(pos_wifi_pref.getFloat("set3") / 360.0f);
                            xSemaphoreGive(rotate_command_mutex);
                            change_page(0);
                            put_out("t5", String(get_angle()));
                            break;
                        case POS4:
                            change_page(1);
                            put_out("pol0", "Ciel: " + String(-pos_wifi_pref.getFloat("set4")));
                            xSemaphoreTake(rotate_command_mutex, portMAX_DELAY);
                            stepper.moveRelativeInRevolutions(pos_wifi_pref.getFloat("set4") / 360.0f);
                            xSemaphoreGive(rotate_command_mutex);
                            change_page(0);
                            put_out("t5", String(get_angle()));
                            break;
                        case POS5:
                            change_page(1);
                            put_out("pol0", "Ciel: " + String(-pos_wifi_pref.getFloat("set5")));
                            xSemaphoreTake(rotate_command_mutex, portMAX_DELAY);
                            stepper.moveRelativeInRevolutions(pos_wifi_pref.getFloat("set5") / 360.0f);
                            xSemaphoreGive(rotate_command_mutex);
                            change_page(0);
                            put_out("t5", String(get_angle()));
                            break;
                        case POS6:
                            change_page(1);
                            put_out("pol0", "Ciel: " + String(-pos_wifi_pref.getFloat("set6")));
                            xSemaphoreTake(rotate_command_mutex, portMAX_DELAY);
                            stepper.moveRelativeInRevolutions(pos_wifi_pref.getFloat("set6") / 360.0f);
                            xSemaphoreGive(rotate_command_mutex);
                            change_page(0);
                            put_out("t5", String(get_angle()));
                            break;
                        case LEFT:
                            vTaskDelete(man_ctrl_task_handle);  // (CCW direction) Stop manual control task (fired when the manual control button is released)
                            servo_lock.write(servo_lock_angle); // lock servo
                            vTaskDelay(500 / portTICK_PERIOD_MS);
                            stepper.moveRelativeInRevolutions(deg_per_teeth / 360.0f);
                            adjust_position_to_whole_teeth();
                            stepper.disableDriver();
                            xSemaphoreGive(rotate_command_mutex);
                            man_ctrl_task_handle = NULL;
                            break;
                        case RIGHT:
                            vTaskDelete(man_ctrl_task_handle);  // (CW direction) Stop manual control task (fired when the manual control button is released)
                            servo_lock.write(servo_lock_angle); // lock servo
                            vTaskDelay(500 / portTICK_PERIOD_MS);
                            stepper.moveRelativeInRevolutions(deg_per_teeth / 360.0f);
                            adjust_position_to_whole_teeth();
                            stepper.disableDriver();
                            xSemaphoreGive(rotate_command_mutex);
                            man_ctrl_task_handle = NULL;
                            break;
                        case M_10X:
                            man_ctrl_multiple = 5.0f;
                            break;
                        case M_01X:
                            man_ctrl_multiple = 1.0f;
                            break;
                        case M_0X1:
                            man_ctrl_multiple = 0.2f;
                            break;
                        }
                    }
                    else if (cmd_in[3] == 0x01)
                    {
                        switch (cmd_in[2])
                        {
                        case LEFT:
                            man_ctrl_task_dir = 1;
                            xSemaphoreTake(rotate_command_mutex, portMAX_DELAY);
                            stepper.enableDriver();
                            vTaskDelay(500 / portTICK_PERIOD_MS);
                            stepper.moveRelativeInRevolutions(-deg_per_teeth / 2 / 360.0f); // Move wheel so that lock won't get stuck
                            vTaskDelay(500 / portTICK_PERIOD_MS);
                            servo_lock.write(servo_large_unlock_angle); // unlock servo
                            vTaskDelay(500 / portTICK_PERIOD_MS);
                            stepper.moveRelativeInRevolutions(deg_per_teeth / 2 / 360.0f); // Move the wheel back
                            vTaskDelay(500 / portTICK_PERIOD_MS);
                            xTaskCreatePinnedToCore(man_ctrl_task, "CCW", 50 * 1024, NULL, 1, &man_ctrl_task_handle, cpu); // (CCW direction) Run the manual control task forever (stopped by releasing the rotate button)
                            break;
                        case RIGHT:
                            man_ctrl_task_dir = -1;
                            xSemaphoreTake(rotate_command_mutex, portMAX_DELAY);
                            stepper.enableDriver();
                            vTaskDelay(500 / portTICK_PERIOD_MS);
                            stepper.moveRelativeInRevolutions(-deg_per_teeth / 2 / 360.0f); // Move wheel so that lock won't get stuck
                            vTaskDelay(500 / portTICK_PERIOD_MS);
                            servo_lock.write(servo_small_unlock_angle); // unlock servo
                            vTaskDelay(500 / portTICK_PERIOD_MS);
                            stepper.moveRelativeInRevolutions(deg_per_teeth / 2 / 360.0f); // Move the wheel back
                            vTaskDelay(500 / portTICK_PERIOD_MS);
                            xTaskCreatePinnedToCore(man_ctrl_task, "CW", 50 * 1024, NULL, 1, &man_ctrl_task_handle, cpu); // (CW direction) Run the manual control task forever (stopped by releasing the rotate button)
                            break;
                        }
                    }
                }
            }
        }
        else
        {
            xSemaphoreGive(commands_queue_mutex);
            vTaskDelay(80 / portTICK_PERIOD_MS);
        }
    }
}

// Task for reading and queueing commands/data from the display
void uart_mon_read(void)
{
    // Set watchdog to 30s
    esp_task_wdt_init(30, false);

    xSemaphoreTake(commands_queue_mutex, portMAX_DELAY);
    while (hw_disp.available())
    {
        char in_dat = (char)hw_disp.read();
        if ((commands_queue.size()) && !(commands_queue.back().endsWith("\xff\xff\xff")))
        {
            commands_queue.back() += in_dat;
        }
        else
        {
            commands_queue.push(String(in_dat));
        }
    }
    xSemaphoreGive(commands_queue_mutex);
}
