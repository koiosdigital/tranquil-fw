#pragma once

#include "job_types.h"
#include <optional>

namespace jobs {

/**
 * @brief Convenience wrapper for job queue operations
 *
 * Thin wrapper around ManifestDatabase job methods.
 * Provides a cleaner interface for job enqueueing and management.
 */
class JobQueue {
public:
    static JobQueue& instance();

    // Lifecycle
    esp_err_t initialize();
    void shutdown();
    bool isInitialized() const;

    // Job creation helpers
    esp_err_t enqueueConversion(const std::string& pattern_uuid,
                                 const ConversionJobData& data,
                                 int priority = 0);

    esp_err_t enqueueThumbnail(const std::string& pattern_uuid,
                                const ThumbnailJobData& data,
                                int priority = 0);

    esp_err_t enqueueDownload(const std::string& pattern_uuid,
                               const DownloadJobData& data,
                               int priority = 0);

    // Job retrieval (for processor)
    std::optional<Job> claimNextPendingJob();

    // Job status updates
    esp_err_t markCompleted(const std::string& job_uuid);
    esp_err_t markFailed(const std::string& job_uuid, const std::string& error);

    // Query
    std::optional<Job> getJob(const std::string& job_uuid);
    std::optional<Job> getJobByPattern(const std::string& pattern_uuid, JobType type);
    bool hasJob(const std::string& pattern_uuid, JobType type);
    size_t getPendingCount();
    size_t getInProgressCount();

    // Cleanup
    esp_err_t deleteJob(const std::string& job_uuid);
    esp_err_t cancelJobsForPattern(const std::string& pattern_uuid);

private:
    JobQueue() = default;
    ~JobQueue() = default;
    JobQueue(const JobQueue&) = delete;
    JobQueue& operator=(const JobQueue&) = delete;

    bool initialized_ = false;
};

} // namespace jobs
