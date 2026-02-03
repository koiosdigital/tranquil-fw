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

    // Job creation helpers (pattern_id is internal ID, 0 for download jobs without existing pattern)
    esp_err_t enqueueConversion(uint32_t pattern_id,
                                 const ConversionJobData& data,
                                 int priority = 0);

    esp_err_t enqueueThumbnail(uint32_t pattern_id,
                                const ThumbnailJobData& data,
                                int priority = 0);

    // Download jobs use external_uuid since pattern may not exist yet
    esp_err_t enqueueDownload(const std::string& pattern_external_uuid,
                               const DownloadJobData& data,
                               int priority = 0);

    // Job retrieval (for processor)
    std::optional<Job> claimNextPendingJob();

    // Job status updates (by internal ID)
    esp_err_t markCompleted(uint32_t job_id);
    esp_err_t markFailed(uint32_t job_id, const std::string& error);

    // Query
    std::optional<Job> getJob(uint32_t job_id);
    std::optional<Job> getJobByPatternId(uint32_t pattern_id, JobType type);
    std::optional<Job> getJobByPatternExternalUuid(const std::string& external_uuid, JobType type);
    bool hasJobForPatternId(uint32_t pattern_id, JobType type);
    bool hasJobForExternalUuid(const std::string& external_uuid, JobType type);
    size_t getPendingCount();
    size_t getInProgressCount();

    // Cleanup
    esp_err_t deleteJob(uint32_t job_id);
    esp_err_t cancelJobsForPatternId(uint32_t pattern_id);
    esp_err_t cancelJobsForExternalUuid(const std::string& external_uuid);

private:
    JobQueue() = default;
    ~JobQueue() = default;
    JobQueue(const JobQueue&) = delete;
    JobQueue& operator=(const JobQueue&) = delete;

    bool initialized_ = false;
};

} // namespace jobs
