#ifndef JOB_MANAGER_H
#define JOB_MANAGER_H

#include <functional>
#include "command_registry.h"
#include <unordered_map>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "fbs/Job_generated.h"
#include "fbs/JobControl_generated.h"
#include <esp_event.h>
#include "iot_is.h"

class JobManager
{
public:
    JobManager();
    void init();
    void register_command(const std::string &name, CommandFunction func);
    void register_command(const std::string &name, std::function<bool()> func);

    bool start_job(JobFlatBuffers::JobT &job);

    void pause_job(const std::string &job_id);
    bool resume_job(const std::string &job_id);
    void cancel_job(const std::string &job_id);
    bool skip_step(const std::string &job_id);
    bool skip_cycle(const std::string &job_id);

private:
    std::unordered_map<std::string, JobFlatBuffers::JobT> active_jobs;
    std::unordered_map<std::string, TaskHandle_t> job_task_handles;

    void execute_job_task(const std::string &job_id);
    bool execute_command(const std::string &job_id);

    void process_job_control(JobFlatBuffers::JobControlT *job_control);
    void on_job_control_received(JobFlatBuffers::JobControlT& job_control);

    void notify_job_update(JobFlatBuffers::JobT &job);
    int64_t get_current_time();
};

extern JobManager job_manager;

#endif
