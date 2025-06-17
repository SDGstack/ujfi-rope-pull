#ifndef IOT_IS_H
#define IOT_IS_H

#include <mqtt_client.h>
#include <string>
#include "fbs/Job_generated.h"
#include "fbs/JobControl_generated.h"

class IoTIs
{
public:
    IoTIs();
    ~IoTIs();

    void connect(const std::string &accessToken, const std::string &mqttHost, int mqttPort);
    bool send_data(const std::string &tag, double value);
    bool send_data(const std::string &tag, double value, int64_t ts);
    bool update_job_status(JobFlatBuffers::JobT &job);

    using JobReceivedCallback = std::function<void(JobFlatBuffers::JobT &)>;
    using JobControlReceivedCallback = std::function<void(JobFlatBuffers::JobControlT &)>;

    void set_job_received_callback(JobReceivedCallback callback);
    void set_job_control_received_callback(JobControlReceivedCallback callback);

private:
    esp_mqtt_client_handle_t _mqttClient;
    std::string _accessToken;
    std::string _mqttHost;
    int _mqttPort;

    JobReceivedCallback _job_received_callback;
    JobControlReceivedCallback _job_control_received_callback;

    static void connect_task(void *pvParameters);
    static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

    void on_connected();
    void on_data_received(esp_mqtt_event_handle_t event);
    void process_received_job(esp_mqtt_event_handle_t event);
    void process_received_job_control(esp_mqtt_event_handle_t event);
    int64_t get_current_time();
};

extern IoTIs iotIs;
#endif
