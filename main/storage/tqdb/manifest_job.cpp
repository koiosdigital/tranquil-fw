/**
 * @file manifest_job.cpp
 * @brief ManifestDatabase job queue operations
 */

#include "manifest_internal.h"
#include "../jobs/job_types.h"
#include "esp_log.h"
#include <vector>

static const char* TAG = MANIFEST_TAG;

// ─────────────────────────────────────────────────────────────────────────────
// Job queue operations
// ─────────────────────────────────────────────────────────────────────────────

esp_err_t ManifestDatabase::enqueueJob(jobs::Job& job) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    if (job.created_at.empty()) job.created_at = currentTimestamp();

    tqdb_err_t err = tqdb_add(impl_->db, "Job", &job);
    if (err != TQDB_OK) {
        ESP_LOGE(TAG, "Failed to enqueue job: %d", err);
        return (err == TQDB_ERR_FULL) ? ESP_ERR_NO_MEM : ESP_FAIL;
    }

    ESP_LOGI(TAG, "Job enqueued: id=%lu", (unsigned long)job.id);
    return ESP_OK;
}

std::optional<jobs::Job> ManifestDatabase::claimNextPendingJob() {
    if (!impl_->initialized) return std::nullopt;

    // Find best pending job (highest priority, oldest created_at)
    struct FindCtx {
        jobs::Job* best;
        jobs::Job bestJob;
    } ctx = { nullptr, {} };

    tqdb_foreach(impl_->db, "Job",
        [](const void* entity, void* c) -> bool {
            auto* ctx = static_cast<FindCtx*>(c);
            const jobs::Job* j = static_cast<const jobs::Job*>(entity);
            if (j->status != jobs::JobStatus::Pending) return true;

            if (!ctx->best ||
                j->priority > ctx->bestJob.priority ||
                (j->priority == ctx->bestJob.priority && j->created_at < ctx->bestJob.created_at)) {
                ctx->bestJob = *j;
                ctx->best = &ctx->bestJob;
            }
            return true;
        }, &ctx);

    if (!ctx.best) return std::nullopt;

    // Claim the job
    ctx.bestJob.status = jobs::JobStatus::InProgress;
    ctx.bestJob.started_at = currentTimestamp();

    if (tqdb_update(impl_->db, "Job", ctx.bestJob.id, &ctx.bestJob) != TQDB_OK) {
        return std::nullopt;
    }

    ESP_LOGI(TAG, "Job claimed: id=%lu", (unsigned long)ctx.bestJob.id);
    return ctx.bestJob;
}

size_t ManifestDatabase::recoverOrphanedJobs() {
    if (!impl_->initialized) return 0;

    // Purge finished rows first: Completed/Failed jobs are never read again
    // and would otherwise accumulate forever.
    size_t purged = 0;
    tqdb_delete_where(impl_->db, "Job",
        [](const void* entity, void* c) -> bool {
            const jobs::Job* j = static_cast<const jobs::Job*>(entity);
            if (j->status == jobs::JobStatus::Completed ||
                j->status == jobs::JobStatus::Failed) {
                (*static_cast<size_t*>(c))++;
                return false;  // Delete this one
            }
            return true;  // Keep
        }, &purged);
    if (purged > 0) {
        ESP_LOGI(TAG, "Purged %zu finished jobs at boot", purged);
    }

    // Collect first, update after: mutating the table during foreach is
    // not safe.
    std::vector<jobs::Job> orphans;
    tqdb_foreach(impl_->db, "Job",
        [](const void* entity, void* c) -> bool {
            const jobs::Job* j = static_cast<const jobs::Job*>(entity);
            if (j->status == jobs::JobStatus::InProgress) {
                static_cast<std::vector<jobs::Job>*>(c)->push_back(*j);
            }
            return true;
        }, &orphans);

    size_t recovered = 0;
    for (auto& j : orphans) {
        j.status = jobs::JobStatus::Pending;
        j.started_at.clear();
        if (tqdb_update(impl_->db, "Job", j.id, &j) == TQDB_OK) {
            ESP_LOGW(TAG, "Recovered orphaned job: id=%lu (type=%s, was InProgress)",
                (unsigned long)j.id, jobs::jobTypeToString(j.type));
            recovered++;
        }
        else {
            ESP_LOGE(TAG, "Failed to recover orphaned job: id=%lu", (unsigned long)j.id);
        }
    }
    return recovered;
}

std::optional<jobs::Job> ManifestDatabase::getJob(uint32_t id) {
    if (!impl_->initialized) return std::nullopt;

    jobs::Job j;
    if (tqdb_get(impl_->db, "Job", id, &j) == TQDB_OK) {
        return j;
    }
    return std::nullopt;
}

std::optional<jobs::Job> ManifestDatabase::getJobByPatternId(uint32_t pattern_id, jobs::JobType type) {
    if (!impl_->initialized) return std::nullopt;

    struct FindCtx {
        uint32_t pattern_id;
        jobs::JobType type;
        jobs::Job result;
        bool found;
    } ctx = { pattern_id, type, {}, false };

    tqdb_foreach(impl_->db, "Job",
        [](const void* entity, void* c) -> bool {
            auto* ctx = static_cast<FindCtx*>(c);
            const jobs::Job* j = static_cast<const jobs::Job*>(entity);
            if (j->pattern_id == ctx->pattern_id && j->type == ctx->type &&
                (j->status == jobs::JobStatus::Pending || j->status == jobs::JobStatus::InProgress)) {
                ctx->result = *j;
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
    if (!impl_->initialized || external_uuid.empty()) return std::nullopt;

    struct FindCtx {
        const std::string* external_uuid;
        jobs::JobType type;
        jobs::Job result;
        bool found;
    } ctx = { &external_uuid, type, {}, false };

    tqdb_foreach(impl_->db, "Job",
        [](const void* entity, void* c) -> bool {
            auto* ctx = static_cast<FindCtx*>(c);
            const jobs::Job* j = static_cast<const jobs::Job*>(entity);
            if (j->pattern_external_uuid == *ctx->external_uuid && j->type == ctx->type &&
                (j->status == jobs::JobStatus::Pending || j->status == jobs::JobStatus::InProgress)) {
                ctx->result = *j;
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
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    jobs::Job j;
    if (tqdb_get(impl_->db, "Job", id, &j) != TQDB_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    j.status = jobs::JobStatus::Completed;
    j.completed_at = currentTimestamp();

    esp_err_t rc = (tqdb_update(impl_->db, "Job", id, &j) == TQDB_OK) ? ESP_OK : ESP_FAIL;
    if (rc == ESP_OK) {
        ESP_LOGI(TAG, "Job completed: id=%lu", (unsigned long)id);
    }
    return rc;
}

esp_err_t ManifestDatabase::markJobFailed(uint32_t id, const std::string& error, bool permanent) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    jobs::Job j;
    if (tqdb_get(impl_->db, "Job", id, &j) != TQDB_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    if (!permanent && j.retry_count < j.max_retries) {
        j.status = jobs::JobStatus::Pending;
        j.retry_count++;
        j.error_message = error;
        j.started_at.clear();
    }
    else {
        // Permanent failures skip retries: retrying can never succeed.
        j.status = jobs::JobStatus::Failed;
        j.error_message = error;
        j.completed_at = currentTimestamp();
    }

    return (tqdb_update(impl_->db, "Job", id, &j) == TQDB_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t ManifestDatabase::releaseJob(uint32_t id) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    jobs::Job j;
    if (tqdb_get(impl_->db, "Job", id, &j) != TQDB_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    // Back to Pending without touching retry_count: this is not a failure of
    // the job itself (e.g. backoff not yet due, transient scheduling hiccup).
    j.status = jobs::JobStatus::Pending;
    j.started_at.clear();

    return (tqdb_update(impl_->db, "Job", id, &j) == TQDB_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t ManifestDatabase::deleteJob(uint32_t id) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    if (!tqdb_exists(impl_->db, "Job", id)) {
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t rc = (tqdb_delete(impl_->db, "Job", id) == TQDB_OK) ? ESP_OK : ESP_FAIL;
    if (rc == ESP_OK) {
        ESP_LOGI(TAG, "Job deleted: id=%lu", (unsigned long)id);
    }
    return rc;
}

esp_err_t ManifestDatabase::cancelJobsForPattern(uint32_t pattern_id) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    struct FilterCtx {
        uint32_t pattern_id;
        int deleted;
    } ctx = { pattern_id, 0 };

    tqdb_delete_where(impl_->db, "Job",
        [](const void* entity, void* c) -> bool {
            auto* ctx = static_cast<FilterCtx*>(c);
            const jobs::Job* j = static_cast<const jobs::Job*>(entity);
            if (j->pattern_id == ctx->pattern_id && j->status == jobs::JobStatus::Pending) {
                ctx->deleted++;
                return false;  // Delete this one
            }
            return true;  // Keep
        }, &ctx);

    if (ctx.deleted > 0) {
        ESP_LOGI(TAG, "Cancelled %d jobs for pattern id=%lu", ctx.deleted, (unsigned long)pattern_id);
    }
    return ESP_OK;
}

esp_err_t ManifestDatabase::cancelJobsForExternalUuid(const std::string& external_uuid) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;
    if (external_uuid.empty()) return ESP_OK;

    struct FilterCtx {
        const std::string* external_uuid;
        int deleted;
    } ctx = { &external_uuid, 0 };

    tqdb_delete_where(impl_->db, "Job",
        [](const void* entity, void* c) -> bool {
            auto* ctx = static_cast<FilterCtx*>(c);
            const jobs::Job* j = static_cast<const jobs::Job*>(entity);
            if (j->pattern_external_uuid == *ctx->external_uuid &&
                j->status == jobs::JobStatus::Pending) {
                ctx->deleted++;
                return false;  // Delete this one
            }
            return true;  // Keep
        }, &ctx);

    if (ctx.deleted > 0) {
        ESP_LOGI(TAG, "Cancelled %d jobs for pattern uuid=%s",
            ctx.deleted, external_uuid.c_str());
    }
    return ESP_OK;
}

size_t ManifestDatabase::getPendingJobCount() {
    if (!impl_->initialized) return 0;

    size_t count = 0;
    tqdb_foreach(impl_->db, "Job",
        [](const void* entity, void* ctx) -> bool {
            if (static_cast<const jobs::Job*>(entity)->status == jobs::JobStatus::Pending) {
                (*static_cast<size_t*>(ctx))++;
            }
            return true;
        }, &count);

    return count;
}

size_t ManifestDatabase::getInProgressJobCount() {
    if (!impl_->initialized) return 0;

    size_t count = 0;
    tqdb_foreach(impl_->db, "Job",
        [](const void* entity, void* ctx) -> bool {
            if (static_cast<const jobs::Job*>(entity)->status == jobs::JobStatus::InProgress) {
                (*static_cast<size_t*>(ctx))++;
            }
            return true;
        }, &count);

    return count;
}
