#pragma once

#include "job_types.h"
#include <memory>
#include <atomic>
#include <functional>
#include <unordered_map>

namespace jobs {

/**
 * @brief Interface for job executors
 */
class IJobExecutor {
public:
    virtual ~IJobExecutor() = default;
    virtual JobResult execute(const Job& job) = 0;
    virtual JobType jobType() const = 0;
};

/**
 * @brief Configuration for JobProcessor
 */
struct JobProcessorConfig {
    size_t max_concurrent = 2;          // Max concurrent jobs
    uint32_t poll_interval_ms = 5000;   // Poll interval
    size_t worker_stack_size = 12288;   // Stack for worker tasks
    int worker_priority = 4;            // Task priority

    // Invoked from the worker task after a job's status has been persisted
    // (markCompleted/markFailed), and ONLY when the job is final: completed,
    // failed permanently, or failed with retries exhausted. Intermediate
    // failures that will be retried do not fire this. Keep it light: it runs
    // on the worker's stack and delays the next job slot until it returns.
    std::function<void(const Job&, const JobResult&)> on_job_complete;
};

/**
 * @brief Background job processor with concurrency limiting
 *
 * Runs as a periodic FreeRTOS task that:
 * - Polls for pending jobs every N seconds
 * - Limits concurrent job execution (default: 2)
 * - Spawns worker tasks for each job
 * - Handles retry logic on failure
 */
class JobProcessor {
public:
    static JobProcessor& instance();

    using Config = JobProcessorConfig;

    // Initialize with executors and start processing
    esp_err_t init(std::unordered_map<JobType, std::unique_ptr<IJobExecutor>> executors,
                   const Config& config = Config{});

    // Lifecycle
    void stop();
    bool isRunning() const;

    // Status
    size_t activeJobCount() const;

    // Manual trigger (for testing or immediate processing)
    void triggerProcessing();

private:
    JobProcessor();
    ~JobProcessor();
    JobProcessor(const JobProcessor&) = delete;
    JobProcessor& operator=(const JobProcessor&) = delete;

    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace jobs
