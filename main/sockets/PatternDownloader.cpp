#include "PatternDownloader.h"
#include "messages.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <mbedtls/base64.h>

#include <unordered_map>

#include "drm/drm_license.h"
#include "drm/drm_purchase.h"
#include "storage/jobs/job_queue.h"
#include "storage/jobs/job_types.h"

static const char* TAG = "pattern_dl";

//------------------------------------------------------------------------------
// Mutex helpers
//------------------------------------------------------------------------------

class FreeRtosMutex {
public:
    FreeRtosMutex() : handle_(xSemaphoreCreateMutex()) {}
    ~FreeRtosMutex() { if (handle_) vSemaphoreDelete(handle_); }

    FreeRtosMutex(const FreeRtosMutex&) = delete;
    FreeRtosMutex& operator=(const FreeRtosMutex&) = delete;

    void lock() { if (handle_) xSemaphoreTake(handle_, portMAX_DELAY); }
    void unlock() { if (handle_) xSemaphoreGive(handle_); }
    bool isValid() const { return handle_ != nullptr; }

private:
    SemaphoreHandle_t handle_;
};

class MutexGuard {
public:
    explicit MutexGuard(FreeRtosMutex& m) : mutex_(m) { mutex_.lock(); }
    ~MutexGuard() { mutex_.unlock(); }
    MutexGuard(const MutexGuard&) = delete;
    MutexGuard& operator=(const MutexGuard&) = delete;
private:
    FreeRtosMutex& mutex_;
};

//------------------------------------------------------------------------------
// Implementation - thin wrapper around JobQueue
//------------------------------------------------------------------------------

class PatternDownloader::Impl {
public:
    esp_err_t init() {
        if (initialized_) return ESP_OK;
        if (!mutex_.isValid()) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_ERR_NO_MEM;
        }
        initialized_ = true;
        ESP_LOGI(TAG, "Pattern downloader initialized");
        return ESP_OK;
    }

    void shutdown() {
        initialized_ = false;
    }

    DownloadResult queueDownload(const Kd__V1__PatternDownloadResponse* response,
        DownloadCallback callback) {
        if (!initialized_) {
            return { DownloadStatus::InvalidResponse, "", "Downloader not initialized" };
        }

        if (!response || !response->pattern_uuid || !response->download_url) {
            return { DownloadStatus::InvalidResponse, "", "Invalid download response" };
        }

        std::string pattern_uuid = response->pattern_uuid;

        // Check if this is a purchased pattern (has receipt)
        bool has_receipt = response->purchase_receipt_payload.data &&
            response->purchase_receipt_payload.len > 0 &&
            response->purchase_receipt_signature.data &&
            response->purchase_receipt_signature.len > 0;

        // For subscription patterns, check license validity and limits
        // For purchased patterns, we can download regardless of subscription status
        if (!has_receipt) {
            if (!drm_license_is_valid()) {
                drm_license_status_t status = drm_license_get_status();
                return { DownloadStatus::LicenseInvalid, pattern_uuid,
                        "License invalid: " + std::to_string(static_cast<int>(status)) };
            }

            if (!drm_license_can_download()) {
                return { DownloadStatus::PatternLimitReached, pattern_uuid,
                        "Pattern download limit reached" };
            }
        }

        // Check if download job already exists
        if (jobs::JobQueue::instance().hasJob(pattern_uuid, jobs::JobType::Download)) {
            return { DownloadStatus::InProgress, pattern_uuid, "Already downloading" };
        }

        // Build job data from response
        jobs::DownloadJobData data;
        data.download_url = response->download_url;

        if (response->pattern) {
            data.pattern_name = response->pattern->name ? response->pattern->name : "";
            data.pattern_creator = response->pattern->creator ? response->pattern->creator : "";
            data.size_bytes = response->pattern->size_bytes;
            data.encrypted = response->pattern->encrypted;
            data.reversible = response->pattern->reversible;
            data.start_point = response->pattern->start_point;
            data.created_at = response->pattern->created_at ? response->pattern->created_at : "";
        }

        // Check for purchase receipt in response
        if (response->purchase_receipt_payload.data && response->purchase_receipt_payload.len > 0 &&
            response->purchase_receipt_signature.data && response->purchase_receipt_signature.len > 0) {

            ESP_LOGI(TAG, "Pattern %s includes purchase receipt", pattern_uuid.c_str());

            // Base64 encode the receipt payload
            size_t payload_b64_len = 0;
            mbedtls_base64_encode(nullptr, 0, &payload_b64_len,
                response->purchase_receipt_payload.data, response->purchase_receipt_payload.len);
            std::string payload_b64(payload_b64_len, '\0');
            mbedtls_base64_encode(reinterpret_cast<unsigned char*>(payload_b64.data()), payload_b64.size(), &payload_b64_len,
                response->purchase_receipt_payload.data, response->purchase_receipt_payload.len);
            payload_b64.resize(payload_b64_len);
            data.receipt_payload_b64 = std::move(payload_b64);

            // Base64 encode the receipt signature
            size_t sig_b64_len = 0;
            mbedtls_base64_encode(nullptr, 0, &sig_b64_len,
                response->purchase_receipt_signature.data, response->purchase_receipt_signature.len);
            std::string sig_b64(sig_b64_len, '\0');
            mbedtls_base64_encode(reinterpret_cast<unsigned char*>(sig_b64.data()), sig_b64.size(), &sig_b64_len,
                response->purchase_receipt_signature.data, response->purchase_receipt_signature.len);
            sig_b64.resize(sig_b64_len);
            data.receipt_signature_b64 = std::move(sig_b64);
        }

        // Store callback for completion notification
        if (callback) {
            MutexGuard lock(mutex_);
            pending_callbacks_[pattern_uuid] = std::move(callback);
        }

        // Enqueue the download job
        esp_err_t err = jobs::JobQueue::instance().enqueueDownload(pattern_uuid, data);
        if (err != ESP_OK) {
            MutexGuard lock(mutex_);
            pending_callbacks_.erase(pattern_uuid);
            return { DownloadStatus::FileError, pattern_uuid, "Failed to enqueue download" };
        }

        ESP_LOGI(TAG, "Queued download job for pattern: %s", pattern_uuid.c_str());
        return { DownloadStatus::Queued, pattern_uuid, "" };
    }

    DownloadResult queueDownloadByUuid(const std::string& pattern_uuid,
        DownloadCallback callback) {
        if (!initialized_) {
            return { DownloadStatus::InvalidResponse, pattern_uuid, "Downloader not initialized" };
        }

        // Check license first
        if (!drm_license_is_valid()) {
            return { DownloadStatus::LicenseInvalid, pattern_uuid, "License invalid" };
        }

        if (!drm_license_can_download()) {
            return { DownloadStatus::PatternLimitReached, pattern_uuid, "Pattern limit reached" };
        }

        // Store callback for when response arrives
        if (callback) {
            MutexGuard lock(mutex_);
            pending_callbacks_[pattern_uuid] = std::move(callback);
        }

        // Request download info from cloud
        cloud_msg_send_pattern_download_request(pattern_uuid.c_str());

        return { DownloadStatus::Queued, pattern_uuid, "Waiting for server response" };
    }

    bool cancelDownload(const std::string& pattern_uuid) {
        // Cancel via job queue
        esp_err_t err = jobs::JobQueue::instance().cancelJobsForPattern(pattern_uuid);

        // Also remove any pending callback
        MutexGuard lock(mutex_);
        pending_callbacks_.erase(pattern_uuid);

        return err == ESP_OK;
    }

    bool isDownloading(const std::string& pattern_uuid) const {
        return jobs::JobQueue::instance().hasJob(pattern_uuid, jobs::JobType::Download);
    }

private:
    bool initialized_ = false;
    mutable FreeRtosMutex mutex_;

    // Pending callbacks for completion notifications
    std::unordered_map<std::string, DownloadCallback> pending_callbacks_;
};

//------------------------------------------------------------------------------
// Public API
//------------------------------------------------------------------------------

PatternDownloader& PatternDownloader::instance() {
    static PatternDownloader instance;
    return instance;
}

PatternDownloader::PatternDownloader() : impl_(std::make_unique<Impl>()) {}
PatternDownloader::~PatternDownloader() = default;

esp_err_t PatternDownloader::init() {
    return impl_->init();
}

void PatternDownloader::shutdown() {
    impl_->shutdown();
}

DownloadResult PatternDownloader::queueDownload(
    const Kd__V1__PatternDownloadResponse* response,
    DownloadCallback callback) {
    return impl_->queueDownload(response, std::move(callback));
}

DownloadResult PatternDownloader::queueDownloadByUuid(
    const std::string& pattern_uuid,
    DownloadCallback callback) {
    return impl_->queueDownloadByUuid(pattern_uuid, std::move(callback));
}

bool PatternDownloader::cancelDownload(const std::string& pattern_uuid) {
    return impl_->cancelDownload(pattern_uuid);
}

bool PatternDownloader::isDownloading(const std::string& pattern_uuid) const {
    return impl_->isDownloading(pattern_uuid);
}

const char* downloadStatusToString(DownloadStatus status) {
    switch (status) {
    case DownloadStatus::Success: return "Success";
    case DownloadStatus::Queued: return "Queued";
    case DownloadStatus::InProgress: return "InProgress";
    case DownloadStatus::LicenseInvalid: return "LicenseInvalid";
    case DownloadStatus::PatternLimitReached: return "PatternLimitReached";
    case DownloadStatus::NetworkError: return "NetworkError";
    case DownloadStatus::FileError: return "FileError";
    case DownloadStatus::InvalidResponse: return "InvalidResponse";
    case DownloadStatus::DecryptionError: return "DecryptionError";
    case DownloadStatus::Cancelled: return "Cancelled";
    default: return "Unknown";
    }
}
