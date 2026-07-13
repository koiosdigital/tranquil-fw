/**
 * @file manifest_job.cpp
 * @brief ManifestDatabase job queue operations
 */

#include "manifest_internal.h"
#include "esp_log.h"
#include <vector>

static const char* TAG = MANIFEST_TAG;

// ─────────────────────────────────────────────────────────────────────────────
// Storage converters
// ─────────────────────────────────────────────────────────────────────────────

cJSON* job_to_storage(const jobs::Job& j) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) return nullptr;

    cJSON_AddNumberToObject(obj, "id", j.id);
    cJSON_AddStringToObject(obj, "type", jobs::jobTypeToString(j.type));
    cJSON_AddNumberToObject(obj, "pattern_id", j.pattern_id);
    cJSON_AddStringToObject(obj, "pattern_external_uuid", j.pattern_external_uuid.c_str());
    cJSON_AddStringToObject(obj, "status", jobs::jobStatusToString(j.status));
    cJSON_AddNumberToObject(obj, "priority", j.priority);
    cJSON_AddNumberToObject(obj, "retry_count", j.retry_count);
    cJSON_AddNumberToObject(obj, "max_retries", j.max_retries);
    cJSON_AddStringToObject(obj, "created_at", j.created_at.c_str());
    cJSON_AddStringToObject(obj, "started_at", j.started_at.c_str());
    cJSON_AddStringToObject(obj, "completed_at", j.completed_at.c_str());
    cJSON_AddStringToObject(obj, "error_message", j.error_message.c_str());
    cJSON_AddStringToObject(obj, "job_data", j.job_data.c_str());
    return obj;
}

jobs::Job job_from_storage(const cJSON* obj) {
    jobs::Job j;
    j.type = jobs::JobType::Conversion;
    j.status = jobs::JobStatus::Pending;
    j.priority = 0;
    j.retry_count = 0;
    j.max_retries = 0;
    if (!obj) return j;

    j.id = static_cast<uint32_t>(storage_get_num(obj, "id", 0));
    j.type = jobs::jobTypeFromString(storage_get_str(obj, "type").c_str());
    j.pattern_id = static_cast<uint32_t>(storage_get_num(obj, "pattern_id", 0));
    j.pattern_external_uuid = storage_get_str(obj, "pattern_external_uuid");
    j.status = jobs::jobStatusFromString(storage_get_str(obj, "status").c_str());
    j.priority = static_cast<int>(storage_get_num(obj, "priority", 0));
    j.retry_count = static_cast<int>(storage_get_num(obj, "retry_count", 0));
    j.max_retries = static_cast<int>(storage_get_num(obj, "max_retries", 0));
    j.created_at = storage_get_str(obj, "created_at");
    j.started_at = storage_get_str(obj, "started_at");
    j.completed_at = storage_get_str(obj, "completed_at");
    j.error_message = storage_get_str(obj, "error_message");
    j.job_data = storage_get_str(obj, "job_data");
    return j;
}

// Serialize + update + save in one step (shared by all job mutators)
static esp_err_t store_job(manifest::JsonlTable& table, const jobs::Job& j) {
    cJSON* obj = job_to_storage(j);
    if (!obj) return ESP_ERR_NO_MEM;

    bool ok = table.update(j.id, obj);
    cJSON_Delete(obj);
    if (!ok) return ESP_FAIL;

    return table.save() ? ESP_OK : ESP_FAIL;
}

// ─────────────────────────────────────────────────────────────────────────────
// Job queue operations
// ─────────────────────────────────────────────────────────────────────────────

esp_err_t ManifestDatabase::enqueueJob(jobs::Job& job) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    if (job.created_at.empty()) job.created_at = currentTimestamp();

    cJSON* obj = job_to_storage(job);
    if (!obj) return ESP_ERR_NO_MEM;

    uint32_t id = impl_->jobs.add(obj);
    cJSON_Delete(obj);
    if (id == 0) {
        ESP_LOGE(TAG, "Failed to enqueue job");
        return ESP_ERR_NO_MEM;
    }
    job.id = id;
    if (!impl_->jobs.save()) return ESP_FAIL;

    ESP_LOGI(TAG, "Job enqueued: id=%lu", (unsigned long)job.id);
    return ESP_OK;
}

std::optional<jobs::Job> ManifestDatabase::claimNextPendingJob() {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return std::nullopt;

    // Find best pending job (highest priority, oldest created_at)
    struct FindCtx {
        bool found;
        jobs::Job bestJob;
    } ctx = { false, {} };

    impl_->jobs.foreach(
        [](uint32_t, const cJSON* obj, void* c) -> bool {
            auto* ctx = static_cast<FindCtx*>(c);
            jobs::Job j = job_from_storage(obj);
            if (j.status != jobs::JobStatus::Pending) return true;

            if (!ctx->found ||
                j.priority > ctx->bestJob.priority ||
                (j.priority == ctx->bestJob.priority && j.created_at < ctx->bestJob.created_at)) {
                ctx->bestJob = std::move(j);
                ctx->found = true;
            }
            return true;
        }, &ctx);

    if (!ctx.found) return std::nullopt;

    // Claim the job
    ctx.bestJob.status = jobs::JobStatus::InProgress;
    ctx.bestJob.started_at = currentTimestamp();

    if (store_job(impl_->jobs, ctx.bestJob) != ESP_OK) {
        return std::nullopt;
    }

    ESP_LOGI(TAG, "Job claimed: id=%lu", (unsigned long)ctx.bestJob.id);
    return ctx.bestJob;
}

size_t ManifestDatabase::recoverOrphanedJobs() {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return 0;

    // Purge finished rows first: Completed/Failed jobs are never read again
    // and would otherwise accumulate forever.
    struct RecoverCtx {
        size_t purged = 0;
        size_t recovered = 0;
    } ctx;

    impl_->jobs.modify(
        [](uint32_t id, cJSON** obj, void* c) -> manifest::JsonlTable::Action {
            auto* ctx = static_cast<RecoverCtx*>(c);
            jobs::Job j = job_from_storage(*obj);

            if (j.status == jobs::JobStatus::Completed ||
                j.status == jobs::JobStatus::Failed) {
                ctx->purged++;
                return manifest::JsonlTable::Action::Remove;
            }

            if (j.status == jobs::JobStatus::InProgress) {
                j.status = jobs::JobStatus::Pending;
                j.started_at.clear();
                cJSON* updated = job_to_storage(j);
                if (!updated) return manifest::JsonlTable::Action::Keep;
                cJSON_Delete(*obj);
                *obj = updated;
                ESP_LOGW(TAG, "Recovered orphaned job: id=%lu (type=%s, was InProgress)",
                    (unsigned long)id, jobs::jobTypeToString(j.type));
                ctx->recovered++;
                return manifest::JsonlTable::Action::Update;
            }

            return manifest::JsonlTable::Action::Keep;
        }, &ctx);

    if (ctx.purged > 0) {
        ESP_LOGI(TAG, "Purged %zu finished jobs at boot", ctx.purged);
    }
    if (ctx.purged > 0 || ctx.recovered > 0) {
        impl_->jobs.save();
    }
    return ctx.recovered;
}

std::optional<jobs::Job> ManifestDatabase::getJob(uint32_t id) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return std::nullopt;

    cJSON* obj = impl_->jobs.get(id);
    if (!obj) return std::nullopt;

    jobs::Job j = job_from_storage(obj);
    cJSON_Delete(obj);
    return j;
}

std::optional<jobs::Job> ManifestDatabase::getJobByPatternId(uint32_t pattern_id, jobs::JobType type) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return std::nullopt;

    struct FindCtx {
        uint32_t pattern_id;
        jobs::JobType type;
        jobs::Job result;
        bool found;
    } ctx = { pattern_id, type, {}, false };

    impl_->jobs.foreach(
        [](uint32_t, const cJSON* obj, void* c) -> bool {
            auto* ctx = static_cast<FindCtx*>(c);
            jobs::Job j = job_from_storage(obj);
            if (j.pattern_id == ctx->pattern_id && j.type == ctx->type &&
                (j.status == jobs::JobStatus::Pending || j.status == jobs::JobStatus::InProgress)) {
                ctx->result = std::move(j);
                ctx->found = true;
                return false;
            }
            return true;
        }, &ctx);

    if (ctx.found) return ctx.result;
    return std::nullopt;
}

std::optional<jobs::Job> ManifestDatabase::getJobByPatternExternalUuid(const std::string& external_uuid,
    jobs::JobType type) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized || external_uuid.empty()) return std::nullopt;

    struct FindCtx {
        const std::string* external_uuid;
        jobs::JobType type;
        jobs::Job result;
        bool found;
    } ctx = { &external_uuid, type, {}, false };

    impl_->jobs.foreach(
        [](uint32_t, const cJSON* obj, void* c) -> bool {
            auto* ctx = static_cast<FindCtx*>(c);
            jobs::Job j = job_from_storage(obj);
            if (j.pattern_external_uuid == *ctx->external_uuid && j.type == ctx->type &&
                (j.status == jobs::JobStatus::Pending || j.status == jobs::JobStatus::InProgress)) {
                ctx->result = std::move(j);
                ctx->found = true;
                return false;
            }
            return true;
        }, &ctx);

    if (ctx.found) return ctx.result;
    return std::nullopt;
}

bool ManifestDatabase::hasJobForPattern(uint32_t pattern_id, jobs::JobType type) {
    return getJobByPatternId(pattern_id, type).has_value();
}

bool ManifestDatabase::hasJobForPatternExternalUuid(const std::string& external_uuid, jobs::JobType type) {
    return getJobByPatternExternalUuid(external_uuid, type).has_value();
}

esp_err_t ManifestDatabase::markJobCompleted(uint32_t id) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    auto j = getJob(id);
    if (!j) return ESP_ERR_NOT_FOUND;

    j->status = jobs::JobStatus::Completed;
    j->completed_at = currentTimestamp();

    esp_err_t rc = store_job(impl_->jobs, *j);
    if (rc == ESP_OK) {
        ESP_LOGI(TAG, "Job completed: id=%lu", (unsigned long)id);
    }
    return rc;
}

esp_err_t ManifestDatabase::markJobFailed(uint32_t id, const std::string& error, bool permanent) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    auto j = getJob(id);
    if (!j) return ESP_ERR_NOT_FOUND;

    if (!permanent && j->retry_count < j->max_retries) {
        j->status = jobs::JobStatus::Pending;
        j->retry_count++;
        j->error_message = error;
        j->started_at.clear();
    }
    else {
        // Permanent failures skip retries: retrying can never succeed.
        j->status = jobs::JobStatus::Failed;
        j->error_message = error;
        j->completed_at = currentTimestamp();
    }

    return store_job(impl_->jobs, *j);
}

esp_err_t ManifestDatabase::releaseJob(uint32_t id) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    auto j = getJob(id);
    if (!j) return ESP_ERR_NOT_FOUND;

    // Back to Pending without touching retry_count: this is not a failure of
    // the job itself (e.g. backoff not yet due, transient scheduling hiccup).
    j->status = jobs::JobStatus::Pending;
    j->started_at.clear();

    return store_job(impl_->jobs, *j);
}

esp_err_t ManifestDatabase::deleteJob(uint32_t id) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    if (!impl_->jobs.remove(id)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (!impl_->jobs.save()) return ESP_FAIL;
    ESP_LOGI(TAG, "Job deleted: id=%lu", (unsigned long)id);
    return ESP_OK;
}

esp_err_t ManifestDatabase::cancelJobsForPattern(uint32_t pattern_id) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    struct FilterCtx {
        uint32_t pattern_id;
        int deleted;
    } ctx = { pattern_id, 0 };

    impl_->jobs.modify(
        [](uint32_t, cJSON** obj, void* c) -> manifest::JsonlTable::Action {
            auto* ctx = static_cast<FilterCtx*>(c);
            jobs::Job j = job_from_storage(*obj);
            if (j.pattern_id == ctx->pattern_id && j.status == jobs::JobStatus::Pending) {
                ctx->deleted++;
                return manifest::JsonlTable::Action::Remove;
            }
            return manifest::JsonlTable::Action::Keep;
        }, &ctx);

    if (ctx.deleted > 0) {
        if (!impl_->jobs.save()) return ESP_FAIL;
        ESP_LOGI(TAG, "Cancelled %d jobs for pattern id=%lu", ctx.deleted, (unsigned long)pattern_id);
    }
    return ESP_OK;
}

esp_err_t ManifestDatabase::cancelJobsForExternalUuid(const std::string& external_uuid) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;
    if (external_uuid.empty()) return ESP_OK;

    struct FilterCtx {
        const std::string* external_uuid;
        int deleted;
    } ctx = { &external_uuid, 0 };

    impl_->jobs.modify(
        [](uint32_t, cJSON** obj, void* c) -> manifest::JsonlTable::Action {
            auto* ctx = static_cast<FilterCtx*>(c);
            jobs::Job j = job_from_storage(*obj);
            if (j.pattern_external_uuid == *ctx->external_uuid &&
                j.status == jobs::JobStatus::Pending) {
                ctx->deleted++;
                return manifest::JsonlTable::Action::Remove;
            }
            return manifest::JsonlTable::Action::Keep;
        }, &ctx);

    if (ctx.deleted > 0) {
        if (!impl_->jobs.save()) return ESP_FAIL;
        ESP_LOGI(TAG, "Cancelled %d jobs for pattern uuid=%s",
            ctx.deleted, external_uuid.c_str());
    }
    return ESP_OK;
}

size_t ManifestDatabase::getPendingJobCount() {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return 0;

    size_t count = 0;
    impl_->jobs.foreach(
        [](uint32_t, const cJSON* obj, void* ctx) -> bool {
            jobs::Job j = job_from_storage(obj);
            if (j.status == jobs::JobStatus::Pending) {
                (*static_cast<size_t*>(ctx))++;
            }
            return true;
        }, &count);

    return count;
}

size_t ManifestDatabase::getInProgressJobCount() {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return 0;

    size_t count = 0;
    impl_->jobs.foreach(
        [](uint32_t, const cJSON* obj, void* ctx) -> bool {
            jobs::Job j = job_from_storage(obj);
            if (j.status == jobs::JobStatus::InProgress) {
                (*static_cast<size_t*>(ctx))++;
            }
            return true;
        }, &count);

    return count;
}
