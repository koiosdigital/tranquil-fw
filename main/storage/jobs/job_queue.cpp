#include "job_queue.h"
#include "ManifestDatabase.h"
#include "esp_log.h"
#include <sys/stat.h>

static const char* TAG = "JobQueue";

namespace jobs {

JobQueue& JobQueue::instance() {
    static JobQueue inst;
    return inst;
}

esp_err_t JobQueue::initialize() {
    if (initialized_) return ESP_OK;

    // Ensure previews directory exists for thumbnails
    struct stat st = {0};
    if (stat("/sd/previews", &st) == -1) {
        if (mkdir("/sd/previews", 0775) != 0) {
            ESP_LOGE(TAG, "Failed to create /sd/previews directory");
            return ESP_FAIL;
        }
        ESP_LOGD(TAG, "Created /sd/previews directory");
    }

    initialized_ = true;
    ESP_LOGD(TAG, "JobQueue initialized");
    return ESP_OK;
}

void JobQueue::shutdown() {
    initialized_ = false;
    ESP_LOGD(TAG, "JobQueue shutdown");
}

bool JobQueue::isInitialized() const {
    return initialized_;
}

esp_err_t JobQueue::enqueueConversion(uint32_t pattern_id,
                                       const ConversionJobData& data,
                                       int priority) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    // Check if job already exists for this pattern
    if (hasJobForPatternId(pattern_id, JobType::Conversion)) {
        ESP_LOGW(TAG, "Conversion job already exists for pattern ID: %u", pattern_id);
        return ESP_OK;
    }

    Job job;
    job.id = 0;  // Will be assigned by TQDB
    job.type = JobType::Conversion;
    job.pattern_id = pattern_id;
    job.status = JobStatus::Pending;
    job.priority = priority;
    job.retry_count = 0;
    job.max_retries = 3;
    job.created_at = ManifestDatabase::currentTimestamp();
    job.job_data = data.toJson();

    return ManifestDatabase::instance().enqueueJob(job);
}

esp_err_t JobQueue::enqueueThumbnail(uint32_t pattern_id,
                                      const ThumbnailJobData& data,
                                      int priority) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    // Check if job already exists for this pattern
    if (hasJobForPatternId(pattern_id, JobType::Thumbnail)) {
        ESP_LOGW(TAG, "Thumbnail job already exists for pattern ID: %u", pattern_id);
        return ESP_OK;
    }

    Job job;
    job.id = 0;  // Will be assigned by TQDB
    job.type = JobType::Thumbnail;
    job.pattern_id = pattern_id;
    job.status = JobStatus::Pending;
    job.priority = priority;
    job.retry_count = 0;
    job.max_retries = 3;
    job.created_at = ManifestDatabase::currentTimestamp();
    job.job_data = data.toJson();

    return ManifestDatabase::instance().enqueueJob(job);
}

esp_err_t JobQueue::enqueueDownload(const std::string& pattern_external_uuid,
                                     const DownloadJobData& data,
                                     int priority) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    // Check if job already exists for this external UUID
    if (hasJobForExternalUuid(pattern_external_uuid, JobType::Download)) {
        ESP_LOGW(TAG, "Download job already exists for pattern: %s", pattern_external_uuid.c_str());
        return ESP_OK;
    }

    Job job;
    job.id = 0;  // Will be assigned by TQDB
    job.type = JobType::Download;
    job.pattern_id = 0;  // Pattern doesn't exist yet
    job.pattern_external_uuid = pattern_external_uuid;
    job.status = JobStatus::Pending;
    job.priority = priority;
    job.retry_count = 0;
    job.max_retries = 3;
    job.created_at = ManifestDatabase::currentTimestamp();
    job.job_data = data.toJson();

    return ManifestDatabase::instance().enqueueJob(job);
}

std::optional<Job> JobQueue::claimNextPendingJob() {
    if (!initialized_) return std::nullopt;
    return ManifestDatabase::instance().claimNextPendingJob();
}

esp_err_t JobQueue::markCompleted(uint32_t job_id) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    return ManifestDatabase::instance().markJobCompleted(job_id);
}

esp_err_t JobQueue::markFailed(uint32_t job_id, const std::string& error) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    return ManifestDatabase::instance().markJobFailed(job_id, error);
}

std::optional<Job> JobQueue::getJob(uint32_t job_id) {
    if (!initialized_) return std::nullopt;
    return ManifestDatabase::instance().getJob(job_id);
}

std::optional<Job> JobQueue::getJobByPatternId(uint32_t pattern_id, JobType type) {
    if (!initialized_) return std::nullopt;
    return ManifestDatabase::instance().getJobByPatternId(pattern_id, type);
}

std::optional<Job> JobQueue::getJobByPatternExternalUuid(const std::string& external_uuid, JobType type) {
    if (!initialized_) return std::nullopt;
    return ManifestDatabase::instance().getJobByPatternExternalUuid(external_uuid, type);
}

bool JobQueue::hasJobForPatternId(uint32_t pattern_id, JobType type) {
    if (!initialized_) return false;
    return ManifestDatabase::instance().hasJobForPattern(pattern_id, type);
}

bool JobQueue::hasJobForExternalUuid(const std::string& external_uuid, JobType type) {
    if (!initialized_) return false;
    return ManifestDatabase::instance().hasJobForPatternExternalUuid(external_uuid, type);
}

size_t JobQueue::getPendingCount() {
    if (!initialized_) return 0;
    return ManifestDatabase::instance().getPendingJobCount();
}

size_t JobQueue::getInProgressCount() {
    if (!initialized_) return 0;
    return ManifestDatabase::instance().getInProgressJobCount();
}

esp_err_t JobQueue::deleteJob(uint32_t job_id) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    return ManifestDatabase::instance().deleteJob(job_id);
}

esp_err_t JobQueue::cancelJobsForPatternId(uint32_t pattern_id) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    return ManifestDatabase::instance().cancelJobsForPattern(pattern_id);
}

esp_err_t JobQueue::cancelJobsForExternalUuid(const std::string& external_uuid) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    // Look up pattern by external UUID to get internal ID
    auto pattern_id = ManifestDatabase::instance().findPatternIdByExternalUuid(external_uuid);
    if (!pattern_id) {
        return ESP_OK;  // No pattern with this UUID, nothing to cancel
    }
    return ManifestDatabase::instance().cancelJobsForPattern(*pattern_id);
}

} // namespace jobs
