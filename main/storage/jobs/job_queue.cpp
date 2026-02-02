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
        ESP_LOGI(TAG, "Created /sd/previews directory");
    }

    initialized_ = true;
    ESP_LOGI(TAG, "JobQueue initialized");
    return ESP_OK;
}

void JobQueue::shutdown() {
    initialized_ = false;
    ESP_LOGI(TAG, "JobQueue shutdown");
}

bool JobQueue::isInitialized() const {
    return initialized_;
}

esp_err_t JobQueue::enqueueConversion(const std::string& pattern_uuid,
                                       const ConversionJobData& data,
                                       int priority) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    // Check if job already exists
    if (hasJob(pattern_uuid, JobType::Conversion)) {
        ESP_LOGW(TAG, "Conversion job already exists for pattern: %s", pattern_uuid.c_str());
        return ESP_OK;
    }

    Job job;
    job.uuid = ManifestDatabase::generateUUID();
    job.type = JobType::Conversion;
    job.pattern_uuid = pattern_uuid;
    job.status = JobStatus::Pending;
    job.priority = priority;
    job.retry_count = 0;
    job.max_retries = 3;
    job.created_at = ManifestDatabase::currentTimestamp();
    job.job_data = data.toJson();

    return ManifestDatabase::instance().enqueueJob(job);
}

esp_err_t JobQueue::enqueueThumbnail(const std::string& pattern_uuid,
                                      const ThumbnailJobData& data,
                                      int priority) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    // Check if job already exists
    if (hasJob(pattern_uuid, JobType::Thumbnail)) {
        ESP_LOGW(TAG, "Thumbnail job already exists for pattern: %s", pattern_uuid.c_str());
        return ESP_OK;
    }

    Job job;
    job.uuid = ManifestDatabase::generateUUID();
    job.type = JobType::Thumbnail;
    job.pattern_uuid = pattern_uuid;
    job.status = JobStatus::Pending;
    job.priority = priority;
    job.retry_count = 0;
    job.max_retries = 3;
    job.created_at = ManifestDatabase::currentTimestamp();
    job.job_data = data.toJson();

    return ManifestDatabase::instance().enqueueJob(job);
}

esp_err_t JobQueue::enqueueDownload(const std::string& pattern_uuid,
                                     const DownloadJobData& data,
                                     int priority) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    // Check if job already exists
    if (hasJob(pattern_uuid, JobType::Download)) {
        ESP_LOGW(TAG, "Download job already exists for pattern: %s", pattern_uuid.c_str());
        return ESP_OK;
    }

    Job job;
    job.uuid = ManifestDatabase::generateUUID();
    job.type = JobType::Download;
    job.pattern_uuid = pattern_uuid;
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

esp_err_t JobQueue::markCompleted(const std::string& job_uuid) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    return ManifestDatabase::instance().markJobCompleted(job_uuid);
}

esp_err_t JobQueue::markFailed(const std::string& job_uuid, const std::string& error) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    return ManifestDatabase::instance().markJobFailed(job_uuid, error);
}

std::optional<Job> JobQueue::getJob(const std::string& job_uuid) {
    if (!initialized_) return std::nullopt;
    return ManifestDatabase::instance().getJob(job_uuid);
}

std::optional<Job> JobQueue::getJobByPattern(const std::string& pattern_uuid, JobType type) {
    if (!initialized_) return std::nullopt;
    return ManifestDatabase::instance().getJobByPattern(pattern_uuid, type);
}

bool JobQueue::hasJob(const std::string& pattern_uuid, JobType type) {
    if (!initialized_) return false;
    return ManifestDatabase::instance().hasJob(pattern_uuid, type);
}

size_t JobQueue::getPendingCount() {
    if (!initialized_) return 0;
    return ManifestDatabase::instance().getPendingJobCount();
}

size_t JobQueue::getInProgressCount() {
    if (!initialized_) return 0;
    return ManifestDatabase::instance().getInProgressJobCount();
}

esp_err_t JobQueue::deleteJob(const std::string& job_uuid) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    return ManifestDatabase::instance().deleteJob(job_uuid);
}

esp_err_t JobQueue::cancelJobsForPattern(const std::string& pattern_uuid) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    return ManifestDatabase::instance().cancelJobsForPattern(pattern_uuid);
}

} // namespace jobs
