#define LOG_LOCAL_LEVEL ESP_LOG_INFO

#include "job_manager.h"
#include "command_registry.h"
#include <iostream>
#include <esp_timer.h>
#include <esp_task.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include <map>
#include "iot_is.h"
#include "fbs/Job_generated.h"
#include <esp_event.h>

static const char *TAG = "JobManager";
static const int MAX_PARALLEL_JOBS = 10;

CommandRegistry command_registry;

JobManager::JobManager()
{
}

void JobManager::init()
{
    iotIs.set_job_received_callback([this](JobFlatBuffers::JobT &job)
                                    { this->start_job(job); });

    iotIs.set_job_control_received_callback([this](JobFlatBuffers::JobControlT &job_control)
                                            { this->on_job_control_received(job_control); });
}

void JobManager::on_job_control_received(JobFlatBuffers::JobControlT &job_control)
{
    JobFlatBuffers::JobControlT job_control_copy = job_control;
    process_job_control(&job_control_copy);
}

void JobManager::process_job_control(JobFlatBuffers::JobControlT *job_control)
{
    ESP_LOGI(TAG, "Received job control: %s", job_control->job_id.c_str());

    switch (job_control->control)
    {
    case JobFlatBuffers::JobControlEnum_JOB_PAUSE:
        pause_job(job_control->job_id);
        break;
    case JobFlatBuffers::JobControlEnum_JOB_RESUME:
        resume_job(job_control->job_id);
        break;
    case JobFlatBuffers::JobControlEnum_JOB_CANCEL:
        cancel_job(job_control->job_id);
        break;
    case JobFlatBuffers::JobControlEnum_JOB_SKIP_STEP:
        skip_step(job_control->job_id);
        break;
    case JobFlatBuffers::JobControlEnum_JOB_SKIP_CYCLE:
        skip_cycle(job_control->job_id);
        break;
    default:
        ESP_LOGW(TAG, "Unknown job control action: %d", job_control->control);
    }
}

bool JobManager::start_job(JobFlatBuffers::JobT &job)
{
    if (active_jobs.find(job.job_id) != active_jobs.end() || active_jobs.size() >= MAX_PARALLEL_JOBS)
    {
        ESP_LOGE(TAG, "Error: Job with ID %s is already in progress or max parallel jobs reached.", job.job_id.c_str());
        job.status = JobFlatBuffers::JobStatusEnum_JOB_REJECTED;
        job.finished_at = get_current_time();
        notify_job_update(job);
        return false;
    }

    ESP_LOGI(TAG, "Starting job: %s with %d steps and %d cycles", job.name.c_str(), job.total_steps, job.total_cycles);
    auto emplaced = active_jobs.emplace(job.job_id, std::move(job));
    auto &stored_job = emplaced.first->second;
    stored_job.status = JobFlatBuffers::JobStatusEnum_JOB_IN_PROGRESS;

    BaseType_t result = xTaskCreate(
        [](void *arg)
        {
            auto *params = static_cast<std::pair<JobManager *, std::string> *>(arg);
            params->first->execute_job_task(params->second);
            delete params;
        },
        ("JobTask_" + stored_job.job_id).c_str(),
        4096,
        new std::pair<JobManager *, std::string>(this, stored_job.job_id),
        5,
        &job_task_handles[stored_job.job_id]);

    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create task for job %s", stored_job.job_id.c_str());
        active_jobs.erase(stored_job.job_id);
        return false;
    }

    return true;
}

void JobManager::pause_job(const std::string &job_id)
{
    auto it = active_jobs.find(job_id);
    if (it != active_jobs.end() && it->second.status == JobFlatBuffers::JobStatusEnum_JOB_IN_PROGRESS)
    {
        it->second.status = JobFlatBuffers::JobStatusEnum_JOB_PAUSED;
        it->second.paused = true;
        if (job_task_handles[job_id] != nullptr)
        {
            vTaskSuspend(job_task_handles[job_id]);
        }
        notify_job_update(it->second);
        ESP_LOGI(TAG, "Job %s paused", job_id.c_str());
    }
    else
    {
        ESP_LOGW(TAG, "Cannot pause job: No matching active job found for ID %s", job_id.c_str());
    }
}

bool JobManager::resume_job(const std::string &job_id)
{
    auto it = active_jobs.find(job_id);
    if (it != active_jobs.end() && it->second.status == JobFlatBuffers::JobStatusEnum_JOB_PAUSED)
    {
        it->second.status = JobFlatBuffers::JobStatusEnum_JOB_IN_PROGRESS;
        it->second.paused = false;
        if (job_task_handles[job_id] != nullptr)
        {
            vTaskResume(job_task_handles[job_id]);
        }
        notify_job_update(it->second);
        ESP_LOGI(TAG, "Job %s resumed", job_id.c_str());
        return true;
    }
    ESP_LOGW(TAG, "Cannot resume job: No matching paused job found for ID %s", job_id.c_str());
    return false;
}

void JobManager::cancel_job(const std::string &job_id)
{
    auto it = active_jobs.find(job_id);
    if (it != active_jobs.end())
    {
        it->second.status = JobFlatBuffers::JobStatusEnum_JOB_CANCELED;
        it->second.finished_at = get_current_time();
        if (job_task_handles[job_id] != nullptr)
        {
            vTaskDelete(job_task_handles[job_id]);
            job_task_handles.erase(job_id);
        }
        notify_job_update(it->second);
        ESP_LOGI(TAG, "Job %s canceled", job_id.c_str());
        active_jobs.erase(job_id);
    }
    else
    {
        ESP_LOGW(TAG, "Cannot cancel job: No matching active job found for ID %s", job_id.c_str());
    }
}

// Skip the current step
bool JobManager::skip_step(const std::string &job_id)
{
    auto it = active_jobs.find(job_id);
    if (it != active_jobs.end() && it->second.status == JobFlatBuffers::JobStatusEnum_JOB_IN_PROGRESS)
    {
        if (it->second.current_step < it->second.total_steps)
        {
            it->second.current_step++;
            ESP_LOGI(TAG, "Skipped step %d in job %s", it->second.current_step - 1, it->second.name.c_str());
            notify_job_update(it->second);
            return true;
        }
        else
        {
            ESP_LOGW(TAG, "Cannot skip step: Already at the last step of the cycle for job %s", job_id.c_str());
        }
    }
    else
    {
        ESP_LOGW(TAG, "Cannot skip step: No matching active job found for ID %s", job_id.c_str());
    }
    return false;
}

// Skip the current cycle
bool JobManager::skip_cycle(const std::string &job_id)
{
    auto it = active_jobs.find(job_id);
    if (it != active_jobs.end() && it->second.status == JobFlatBuffers::JobStatusEnum_JOB_IN_PROGRESS)
    {
        if (it->second.current_cycle < it->second.total_cycles)
        {
            it->second.current_cycle++;
            it->second.current_step = 1;
            ESP_LOGI(TAG, "Skipped to cycle %d in job %s", it->second.current_cycle, it->second.name.c_str());
            notify_job_update(it->second);
            return true;
        }
        else
        {
            ESP_LOGW(TAG, "Cannot skip cycle: Already at the last cycle of job %s", job_id.c_str());
        }
    }
    else
    {
        ESP_LOGW(TAG, "Cannot skip cycle: No matching active job found for ID %s", job_id.c_str());
    }
    return false;
}

void JobManager::execute_job_task(const std::string &job_id)
{
    JobFlatBuffers::JobT &job = active_jobs[job_id];
    while (job.current_cycle <= job.total_cycles)
    {
        while (job.current_step <= job.total_steps)
        {
            if (!execute_command(job_id))
            {
                notify_job_update(job);
                return;
            }

            if (job.current_step == job.total_steps && job.current_cycle == job.total_cycles)
            {
                break;
            }
            job.current_step++;
            notify_job_update(job);
        }

        if (job.current_cycle < job.total_cycles)
        {
            job.current_cycle++;
            job.current_step = 1;
        }
        else
        {
            break;
        }
    }

    if (job.status == JobFlatBuffers::JobStatusEnum_JOB_IN_PROGRESS)
    {
        job.status = JobFlatBuffers::JobStatusEnum_JOB_SUCCEEDED;
        job.finished_at = get_current_time();
    }

    ESP_LOGI(TAG, "Job %s finished", job.name.c_str());
    notify_job_update(job);
    active_jobs.erase(job_id);

    if (job_task_handles.find(job_id) != job_task_handles.end())
    {
        job_task_handles.erase(job_id);
        vTaskDelete(job_task_handles[job_id]);
    }
}

// Update execute_command to use the job ID
bool JobManager::execute_command(const std::string &job_id)
{
    JobFlatBuffers::JobT &job = active_jobs[job_id];
    const JobFlatBuffers::CommandT &command = *job.commands[job.current_step - 1];
    CommandFunction func = command_registry.get_command(command.name);
    if (func)
    {
        bool success = func(command.params);
        if (!success)
        {
            job.status = JobFlatBuffers::JobStatusEnum_JOB_FAILED;
            job.finished_at = get_current_time();
            ESP_LOGE(TAG, "Command %s failed.", command.name.c_str());
            return false;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Command %s not found in the registry.", command.name.c_str());
        job.status = JobFlatBuffers::JobStatusEnum_JOB_FAILED;
        job.finished_at = get_current_time();
        return false;
    }

    return true;
}

void JobManager::register_command(const std::string &name, CommandFunction func)
{
    command_registry.register_command(name, func);
}

void JobManager::register_command(const std::string &name, std::function<bool()> func)
{
    command_registry.register_command(name, func);
}

int64_t JobManager::get_current_time()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000LL + (int64_t)tv.tv_usec / 1000LL;
}

void JobManager::notify_job_update(JobFlatBuffers::JobT &job)
{
    iotIs.update_job_status(job);
}

JobManager job_manager;