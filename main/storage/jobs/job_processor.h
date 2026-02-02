#pragma once

#include "job_types.h"
#include <memory>
#include <atomic>
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
