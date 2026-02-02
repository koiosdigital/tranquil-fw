#include "job_processor.h"
#include "job_queue.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <unordered_map>

static const char* TAG = "JobProcessor";

namespace jobs {

class JobProcessor::Impl {
public:
    // Worker task context (defined inside Impl to access private type)
    struct WorkerContext {
        Job job;
        IJobExecutor* executor;
        Impl* processor;
    };

    Config config;
    TaskHandle_t processorTask = nullptr;
    SemaphoreHandle_t triggerSem = nullptr;
    std::atomic<bool> running{false};
    std::atomic<size_t> activeJobs{0};

    std::unordered_map<JobType, std::unique_ptr<IJobExecutor>> executors;
    SemaphoreHandle_t executorMutex = nullptr;

    static void processorTaskFunction(void* param) {
        auto* impl = static_cast<Impl*>(param);
        ESP_LOGI(TAG, "Processor task started");

        while (impl->running) {
            // Wait for trigger or timeout
            xSemaphoreTake(impl->triggerSem, pdMS_TO_TICKS(impl->config.poll_interval_ms));

            if (!impl->running) break;

            impl->processJobs();
        }

        ESP_LOGI(TAG, "Processor task exiting");
        vTaskDelete(nullptr);
    }

    void processJobs() {
        // Process jobs while we have capacity
        while (running && activeJobs < config.max_concurrent) {
            auto job = JobQueue::instance().claimNextPendingJob();
            if (!job) break;  // No more pending jobs

            // Find executor for this job type
            IJobExecutor* executor = nullptr;
            if (xSemaphoreTake(executorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                auto it = executors.find(job->type);
                if (it != executors.end()) {
                    executor = it->second.get();
                }
                xSemaphoreGive(executorMutex);
            }

            if (!executor) {
                ESP_LOGE(TAG, "No executor for job type: %s", jobTypeToString(job->type));
                JobQueue::instance().markFailed(job->uuid, "No executor registered");
                continue;
            }

            // Spawn worker task
            auto* ctx = new WorkerContext{*job, executor, this};
            activeJobs++;

            if (xTaskCreate(workerTaskFunction, "job_worker",
                           config.worker_stack_size, ctx,
                           config.worker_priority, nullptr) != pdPASS) {
                ESP_LOGE(TAG, "Failed to create worker task for job: %s", job->uuid.c_str());
                activeJobs--;
                delete ctx;
                JobQueue::instance().markFailed(job->uuid, "Failed to create worker task");
            }
        }
    }

    static void workerTaskFunction(void* param) {
        auto* ctx = static_cast<WorkerContext*>(param);

        ESP_LOGI(TAG, "Worker started for job: %s (type=%s)",
            ctx->job.uuid.c_str(), jobTypeToString(ctx->job.type));

        // Execute the job
        JobResult result = ctx->executor->execute(ctx->job);

        // Update job status
        if (result.success) {
            JobQueue::instance().markCompleted(ctx->job.uuid);
            ESP_LOGI(TAG, "Job completed successfully: %s", ctx->job.uuid.c_str());
        } else {
            JobQueue::instance().markFailed(ctx->job.uuid, result.error);
            ESP_LOGW(TAG, "Job failed: %s - %s", ctx->job.uuid.c_str(), result.error.c_str());
        }

        // Decrement active count and trigger next job processing
        ctx->processor->activeJobs--;
        if (ctx->processor->triggerSem) {
            xSemaphoreGive(ctx->processor->triggerSem);
        }

        delete ctx;
        vTaskDelete(nullptr);
    }
};

JobProcessor::JobProcessor() : impl_(std::make_unique<Impl>()) {}
JobProcessor::~JobProcessor() { stop(); }

JobProcessor& JobProcessor::instance() {
    static JobProcessor inst;
    return inst;
}

esp_err_t JobProcessor::init(std::unordered_map<JobType, std::unique_ptr<IJobExecutor>> executors,
                              const Config& config) {
    if (impl_->running) return ESP_OK;

    // Store executors first (before any concurrency)
    impl_->executors = std::move(executors);
    impl_->config = config;

    // Create trigger semaphore
    impl_->triggerSem = xSemaphoreCreateBinary();
    if (!impl_->triggerSem) {
        ESP_LOGE(TAG, "Failed to create trigger semaphore");
        return ESP_FAIL;
    }

    // Create executor mutex
    impl_->executorMutex = xSemaphoreCreateMutex();
    if (!impl_->executorMutex) {
        ESP_LOGE(TAG, "Failed to create executor mutex");
        vSemaphoreDelete(impl_->triggerSem);
        impl_->triggerSem = nullptr;
        return ESP_FAIL;
    }

    impl_->running = true;

    // Create processor task
    if (xTaskCreate(Impl::processorTaskFunction, "job_proc",
                   4096, impl_.get(), 3, &impl_->processorTask) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create processor task");
        impl_->running = false;
        vSemaphoreDelete(impl_->executorMutex);
        impl_->executorMutex = nullptr;
        vSemaphoreDelete(impl_->triggerSem);
        impl_->triggerSem = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "JobProcessor started (max_concurrent=%zu, poll_interval=%lums)",
        config.max_concurrent, (unsigned long)config.poll_interval_ms);
    return ESP_OK;
}

void JobProcessor::stop() {
    if (!impl_->running) return;

    impl_->running = false;

    // Trigger the task to wake up and exit
    if (impl_->triggerSem) {
        xSemaphoreGive(impl_->triggerSem);
    }

    // Wait for task to exit
    vTaskDelay(pdMS_TO_TICKS(200));

    if (impl_->executorMutex) {
        vSemaphoreDelete(impl_->executorMutex);
        impl_->executorMutex = nullptr;
    }

    if (impl_->triggerSem) {
        vSemaphoreDelete(impl_->triggerSem);
        impl_->triggerSem = nullptr;
    }

    impl_->processorTask = nullptr;
    ESP_LOGI(TAG, "JobProcessor stopped");
}

bool JobProcessor::isRunning() const {
    return impl_->running;
}

size_t JobProcessor::activeJobCount() const {
    return impl_->activeJobs;
}

void JobProcessor::triggerProcessing() {
    if (impl_->triggerSem) {
        xSemaphoreGive(impl_->triggerSem);
    }
}

} // namespace jobs
