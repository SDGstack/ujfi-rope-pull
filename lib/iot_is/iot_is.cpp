#define LOG_LOCAL_LEVEL ESP_LOG_INFO

#include "iot_is.h"
#include "esp_log.h"
#include "command_registry.h"
#include "fbs/DataPoint_generated.h"
#include "fbs/Job_generated.h"
#include "fbs/JobControl_generated.h"

#define TAG "IoTIs"

IoTIs::IoTIs()
    : _accessToken(""), _mqttHost(""), _mqttPort(0), _mqttClient(nullptr)
{
}

IoTIs::~IoTIs()
{
    if (_mqttClient != nullptr)
    {
        esp_mqtt_client_stop(_mqttClient);
        esp_mqtt_client_destroy(_mqttClient);
    }
}

void IoTIs::connect(const std::string &accessToken, const std::string &mqttHost, int mqttPort)
{
    _accessToken = accessToken;
    _mqttHost = mqttHost;
    _mqttPort = mqttPort;

    xTaskCreate(&IoTIs::connect_task, "connect_task", 4096, this, 5, NULL);
}

void IoTIs::connect_task(void *pvParameters)
{
    IoTIs *instance = static_cast<IoTIs *>(pvParameters);

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.host = instance->_mqttHost.c_str();
    mqtt_cfg.port = instance->_mqttPort;
    mqtt_cfg.username = instance->_accessToken.c_str();
    mqtt_cfg.buffer_size = 16384;

    instance->_mqttClient = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(instance->_mqttClient, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, instance);
    esp_mqtt_client_start(instance->_mqttClient);

    vTaskDelete(NULL);
}

bool IoTIs::send_data(const std::string &tag, double value)
{
    return send_data(tag, value, get_current_time());
}

bool IoTIs::send_data(const std::string &tag, double value, int64_t ts)
{
    if (_mqttClient == nullptr)
    {
        ESP_LOGE(TAG, "MQTT client not initialized");
        return false;
    }

    flatbuffers::FlatBufferBuilder builder;

    auto datapoint = DataPointFlatBuffers::CreateDataPoint(builder, builder.CreateString(tag), value, ts);

    builder.Finish(datapoint);

    // Enqueue the message for publishing to MQTT
    std::string topic = "devices/" + _accessToken + "/data";
    esp_mqtt_client_enqueue(_mqttClient, topic.c_str(), (const char *)builder.GetBufferPointer(), builder.GetSize(), 1, 0, true);
    return true;
}

void IoTIs::mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    IoTIs *instance = (IoTIs *)handler_args;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        instance->on_connected();
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_DATA:
        instance->on_data_received(event);
        break;
    default:
        break;
    }
}

void IoTIs::on_connected()
{
    // Subscribe to the job update topic
    esp_mqtt_client_subscribe(_mqttClient, ("devices/" + _accessToken + "/job").c_str(), 1);
    esp_mqtt_client_subscribe(_mqttClient, ("devices/" + _accessToken + "/job/control").c_str(), 2);
}

void IoTIs::on_data_received(esp_mqtt_event_handle_t event)
{
    std::string topic(event->topic, event->topic_len);
    if (topic == "devices/" + _accessToken + "/job")
    {
        process_received_job(event);
    }
    else if (topic == "devices/" + _accessToken + "/job/control")
    {
        process_received_job_control(event);
    }
}

void IoTIs::process_received_job(esp_mqtt_event_handle_t event)
{
    flatbuffers::Verifier verifier(reinterpret_cast<const uint8_t *>(event->data), event->data_len);
    if (!JobFlatBuffers::VerifyJobBuffer(verifier))
    {
        ESP_LOGE(TAG, "Invalid JobFbs buffer");
        return;
    }

    JobFlatBuffers::JobT jobFbs;
    JobFlatBuffers::GetJob(event->data)->UnPackTo(&jobFbs);

    ESP_LOGI(TAG, "Received job: %s with %d steps and %d cycles", jobFbs.name.c_str(), jobFbs.total_steps, jobFbs.total_cycles);
    
    if (_job_received_callback)
    {
        _job_received_callback(jobFbs);
    }
    else
    {
        ESP_LOGW(TAG, "No job received callback set");
    }
}

bool IoTIs::update_job_status(JobFlatBuffers::JobT &job)
{
    flatbuffers::FlatBufferBuilder builder(512);
    auto name = builder.CreateString(job.name);
    auto job_id = builder.CreateString(job.job_id);

    JobFlatBuffers::JobBuilder jobBuilder(builder);
    jobBuilder.add_name(name);
    jobBuilder.add_job_id(job_id);
    jobBuilder.add_status(static_cast<JobFlatBuffers::JobStatusEnum>(job.status));
    jobBuilder.add_current_step(job.current_step);
    jobBuilder.add_total_steps(job.total_steps);
    jobBuilder.add_current_cycle(job.current_cycle);
    jobBuilder.add_total_cycles(job.total_cycles);
    jobBuilder.add_paused(job.paused);
    jobBuilder.add_started_at(job.started_at);
    jobBuilder.add_finished_at(job.finished_at);

    auto jobOffset = jobBuilder.Finish();
    builder.Finish(jobOffset);

    if (_mqttClient == nullptr)
    {
        ESP_LOGE(TAG, "MQTT client not initialized");
        return false;
    }

    // Publish to MQTT
    std::string topic = "devices/" + _accessToken + "/job_from_device";
    int msg_id = esp_mqtt_client_enqueue(_mqttClient, topic.c_str(),
                                         reinterpret_cast<const char *>(builder.GetBufferPointer()),
                                         builder.GetSize(), 1, 0, true);

    return msg_id != -1;
}

void IoTIs::process_received_job_control(esp_mqtt_event_handle_t event)
{
    flatbuffers::Verifier verifier(reinterpret_cast<const uint8_t *>(event->data), event->data_len);
    if (!JobFlatBuffers::VerifyJobControlBuffer(verifier))
    {
        ESP_LOGE(TAG, "Invalid JobControl buffer");
        return;
    }
    JobFlatBuffers::JobControlT jobControl;
    JobFlatBuffers::GetJobControl(event->data)->UnPackTo(&jobControl);

    if (_job_control_received_callback)
    {
        _job_control_received_callback(jobControl);
    }
    else
    {
        ESP_LOGW(TAG, "No job control received callback set");
    }
}

void IoTIs::set_job_received_callback(JobReceivedCallback callback)
{
    _job_received_callback = std::move(callback);
}

void IoTIs::set_job_control_received_callback(JobControlReceivedCallback callback)
{
    _job_control_received_callback = std::move(callback);
}

int64_t IoTIs::get_current_time()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000LL + (int64_t)tv.tv_usec / 1000LL;
}

IoTIs iotIs;