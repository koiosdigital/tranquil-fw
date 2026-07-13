#include "job_processor.h"
#include "job_queue.h"
#include "raii_utils.hpp"
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
    std::atomic<bool> processorExited{false};
    std::atomic<size_t> activeJobs{0};

    std::unordered_map<JobType, std::unique_ptr<IJobExecutor>> executors;
    SemaphoreHandle_t executorMutex = nullptr;

    // In-memory retry backoff (job schema unchanged): job_id -> earliest tick
    // at which the job may be retried. Fixed-size, spinlock-protected — read
    // from the processor task, written from worker tasks.
    struct BackoffEntry {
        uint32_t job_id = 0;        // 0 = empty slot (record IDs start at 1)
        TickType_t not_before = 0;
    };
    static constexpr size_t MAX_BACKOFF_ENTRIES = 8;
    BackoffEntry backoff[MAX_BACKOFF_ENTRIES] = {};
    portMUX_TYPE backoffMux = portMUX_INITIALIZER_UNLOCKED;

    // delay = 5s * 2^retry_count, capped at 60s
    static uint32_t backoffDelayMs(int retry_count) {
        if (retry_count < 0) retry_count = 0;
        if (retry_count > 4) retry_count = 4;
        uint32_t delay = 5000u << retry_count;
        return delay > 60000u ? 60000u : delay;
    }

    void setBackoff(uint32_t job_id, uint32_t delay_ms) {
        TickType_t due = xTaskGetTickCount() + pdMS_TO_TICKS(delay_ms);
        portENTER_CRITICAL(&backoffMux);
        // Reuse the job's slot, else an empty slot, else evict the entry due
        // soonest (the evicted job just retries a bit early — safe).
        BackoffEntry* slot = nullptr;
        for (auto& e : backoff) {
            if (e.job_id == job_id) { slot = &e; break; }
        }
        if (!slot) {
            for (auto& e : backoff) {
                if (e.job_id == 0) { slot = &e; break; }
            }
        }
        if (!slot) {
            slot = &backoff[0];
            for (auto& e : backoff) {
                if ((int32_t)(e.not_before - slot->not_before) < 0) slot = &e;
            }
        }
        slot->job_id = job_id;
        slot->not_before = due;
        portEXIT_CRITICAL(&backoffMux);
    }

    void clearBackoff(uint32_t job_id) {
        portENTER_CRITICAL(&backoffMux);
        for (auto& e : backoff) {
            if (e.job_id == job_id) {
                e.job_id = 0;
                e.not_before = 0;
            }
        }
        portEXIT_CRITICAL(&backoffMux);
    }

    // true when the job has no pending backoff or its backoff has elapsed
    bool backoffDue(uint32_t job_id) {
        bool due = true;
        TickType_t now = xTaskGetTickCount();
        portENTER_CRITICAL(&backoffMux);
        for (auto& e : backoff) {
            if (e.job_id == job_id) {
                due = (int32_t)(now - e.not_before) >= 0;
                break;
            }
        }
        portEXIT_CRITICAL(&backoffMux);
        return due;
    }

    static void processorTaskFunction(void* param) {
        auto* impl = static_cast<Impl*>(param);
        ESP_LOGI(TAG, "Processor task started (poll=%lums, max_concurrent=%zu)",
            (unsigned long)impl->config.poll_interval_ms, impl->config.max_concurrent);

        while (impl->running) {
            // Wait for trigger or timeout
            BaseType_t woken =
                xSemaphoreTake(impl->triggerSem, pdMS_TO_TICKS(impl->config.poll_interval_ms));

            if (!impl->running) break;

            impl->processJobs();

            // A finishing worker gives triggerSem BEFORE decrementing
            // activeJobs (stop() safety: it must still be counted while the
            // semaphore handle is used). So a wake can arrive while we still
            // look at-capacity and get swallowed; briefly wait for the
            // decrement and re-check instead of stalling a full poll cycle.
            if (woken == pdTRUE && impl->running &&
                impl->activeJobs >= impl->config.max_concurrent) {
                vTaskDelay(pdMS_TO_TICKS(50));
                if (impl->running) impl->processJobs();
            }
        }

        ESP_LOGD(TAG, "Processor task exiting");
        impl->processorExited = true;  // stop() waits on this before teardown
        vTaskDelete(nullptr);
    }

    void processJobs() {
        // Process jobs while we have capacity
        while (running && activeJobs < config.max_concurrent) {
            auto job = JobQueue::instance().claimNextPendingJob();
            if (!job) {
                // Nothing claimable. If jobs are pending anyway, something
                // is wrong (DB claim failure) — say so instead of silently
                // idling.
                size_t pending = JobQueue::instance().getPendingCount();
                if (pending > 0) {
                    ESP_LOGW(TAG, "%zu pending jobs but none claimable (active=%u)",
                        pending, (unsigned)activeJobs.load());
                }
                break;
            }

            // In-memory retry backoff: a claimed job whose backoff hasn't
            // elapsed goes back to Pending (without burning a retry) and we
            // wait for the next poll/trigger instead of hammering it.
            if (!backoffDue(job->id)) {
                ESP_LOGD(TAG, "Job %u in retry backoff, deferring", job->id);
                JobQueue::instance().releaseJob(job->id);
                break;
            }

            // Find executor for this job type
            IJobExecutor* executor = nullptr;
            bool lookup_ok = false;
            {
                raii::MutexGuard guard(executorMutex, pdMS_TO_TICKS(1000));
                if (guard) {
                    lookup_ok = true;
                    auto it = executors.find(job->type);
                    if (it != executors.end()) {
                        executor = it->second.get();
                    }
                }
            }

            if (!lookup_ok) {
                // Scheduling hiccup taking the mutex, not a job failure —
                // requeue without burning a retry and try again next pass.
                ESP_LOGW(TAG, "Executor mutex busy, releasing job %u back to pending", job->id);
                JobQueue::instance().releaseJob(job->id);
                break;
            }

            if (!executor) {
                // No executor will ever appear for this type — permanent.
                ESP_LOGE(TAG, "No executor for job type: %s", jobTypeToString(job->type));
                JobQueue::instance().markFailed(job->id, "No executor registered", true);
                continue;
            }

            // Spawn worker task
            auto* ctx = new WorkerContext{*job, executor, this};
            activeJobs++;

            if (xTaskCreate(workerTaskFunction, "job_worker",
                           config.worker_stack_size, ctx,
                           config.worker_priority, nullptr) != pdPASS) {
                ESP_LOGE(TAG, "Failed to create worker task for job ID: %u", job->id);
                activeJobs--;
                delete ctx;
                JobQueue::instance().markFailed(job->id, "Failed to create worker task");
            }
        }
    }

    static void workerTaskFunction(void* param) {
        auto* ctx = static_cast<WorkerContext*>(param);

        ESP_LOGI(TAG, "Worker started for job ID: %u (type=%s)",
            ctx->job.id, jobTypeToString(ctx->job.type));

        // Execute the job
        JobResult result = ctx->executor->execute(ctx->job);

        // Update job status. A job is FINAL when it completed, failed
        // permanently, or exhausted its retries — only then may the
        // completion callback fire (clients must not hear "failed" on
        // attempt 1 of 4).
        bool is_final = true;
        if (result.success) {
            JobQueue::instance().markCompleted(ctx->job.id);
            ctx->processor->clearBackoff(ctx->job.id);
            ESP_LOGI(TAG, "Job completed: %u (type=%s)",
                ctx->job.id, jobTypeToString(ctx->job.type));
        } else {
            JobQueue::instance().markFailed(ctx->job.id, result.error, result.permanent);
            is_final = result.permanent ||
                       (ctx->job.retry_count >= ctx->job.max_retries);
            if (is_final) {
                ctx->processor->clearBackoff(ctx->job.id);
                ESP_LOGW(TAG, "Job failed (final%s): %u - %s",
                    result.permanent ? ", permanent" : "",
                    ctx->job.id, result.error.c_str());
            } else {
                uint32_t delay_ms = backoffDelayMs(ctx->job.retry_count);
                ctx->processor->setBackoff(ctx->job.id, delay_ms);
                ESP_LOGW(TAG, "Job failed (retry %d/%d in %lums): %u - %s",
                    ctx->job.retry_count + 1, ctx->job.max_retries,
                    (unsigned long)delay_ms, ctx->job.id, result.error.c_str());
            }
        }

        if (is_final && ctx->processor->config.on_job_complete) {
            ctx->processor->config.on_job_complete(ctx->job, result);
        }

        // Trigger next job processing BEFORE decrementing activeJobs:
        // stop() waits for activeJobs == 0 before deleting triggerSem, so
        // the give must happen while this job is still counted — the
        // reverse order lets stop() delete the semaphore between our
        // decrement and give (use-after-free of the handle).
        if (ctx->processor->triggerSem) {
            xSemaphoreGive(ctx->processor->triggerSem);
        }
        ctx->processor->activeJobs--;

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

    ESP_LOGD(TAG, "JobProcessor started");
    return ESP_OK;
}

void JobProcessor::stop() {
    if (!impl_->running) return;

    impl_->running = false;

    // Trigger the task to wake up and exit
    if (impl_->triggerSem) {
        xSemaphoreGive(impl_->triggerSem);
    }

    // Join the processor task (it can be inside a long SD-card-backed
    // processJobs() pass) and wait for all worker tasks to finish — both
    // keep using triggerSem/executorMutex/executors until they exit, so
    // tearing those down on a fixed delay is a use-after-free.
    constexpr int MAX_WAIT_MS = 30000;
    int waited = 0;
    while ((!impl_->processorExited || impl_->activeJobs > 0) && waited < MAX_WAIT_MS) {
        vTaskDelay(pdMS_TO_TICKS(50));
        waited += 50;
    }

    if (!impl_->processorExited || impl_->activeJobs > 0) {
        // Tasks are still alive; deleting the semaphores they hold would
        // corrupt the heap. Leak them deliberately and bail.
        ESP_LOGE(TAG, "Tasks still active after %dms (%u jobs) - leaking sync objects",
                 MAX_WAIT_MS, (unsigned)impl_->activeJobs.load());
        impl_->processorTask = nullptr;
        return;
    }

    if (impl_->executorMutex) {
        vSemaphoreDelete(impl_->executorMutex);
        impl_->executorMutex = nullptr;
    }

    if (impl_->triggerSem) {
        vSemaphoreDelete(impl_->triggerSem);
        impl_->triggerSem = nullptr;
    }

    impl_->processorTask = nullptr;
    impl_->processorExited = false;
    ESP_LOGD(TAG, "JobProcessor stopped");
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
