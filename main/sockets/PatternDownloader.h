#pragma once

#include <esp_err.h>
#include <kd/v1/tranquil.pb-c.h>

#include <memory>
#include <string>
#include <functional>

/**
 * @brief Pattern download status codes
 */
enum class DownloadStatus {
    Success,
    Queued,
    InProgress,
    LicenseInvalid,
    PatternLimitReached,
    NetworkError,
    FileError,
    InvalidResponse,
    DecryptionError,
    Cancelled,
};

/**
 * @brief Download result with status and optional error message
 */
struct DownloadResult {
    DownloadStatus status;
    std::string pattern_uuid;
    std::string error;

    bool success() const { return status == DownloadStatus::Success; }
};

/**
 * @brief Callback type for download completion
 */
using DownloadCallback = std::function<void(const DownloadResult&)>;

/**
 * @brief Pattern downloader - thin wrapper around job queue
 *
 * Validates requests and enqueues download jobs to the job queue.
 * Actual downloads are performed by DownloadExecutor via JobProcessor.
 *
 * Usage:
 *   PatternDownloader::instance().queueDownload(response, callback);
 */
class PatternDownloader {
public:
    static PatternDownloader& instance();

    PatternDownloader(const PatternDownloader&) = delete;
    PatternDownloader& operator=(const PatternDownloader&) = delete;

    /**
     * @brief Initialize the downloader
     */
    esp_err_t init();

    /**
     * @brief Shutdown and cleanup
     */
    void shutdown();

    /**
     * @brief Queue a pattern for download
     *
     * Validates the response, checks license, and enqueues a download job.
     */
    DownloadResult queueDownload(const Kd__V1__PatternDownloadResponse* response,
        DownloadCallback callback = nullptr);

    /**
     * @brief Queue a pattern by UUID (requests info from cloud first)
     *
     * Sends a RequestPatternDownload message and queues when response arrives.
     */
    DownloadResult queueDownloadByUuid(const std::string& pattern_uuid,
        DownloadCallback callback = nullptr);

    /**
     * @brief Fire the completion callback registered for a pattern (if any)
     *
     * Called by the job-completion hook when a download job finishes.
     */
    void notifyDownloadComplete(const std::string& pattern_uuid,
        bool success, const std::string& error);

    /**
     * @brief Cancel a pending download job
     */
    bool cancelDownload(const std::string& pattern_uuid);

    /**
     * @brief Check if a pattern has a pending/active download job
     */
    bool isDownloading(const std::string& pattern_uuid) const;

private:
    PatternDownloader();
    ~PatternDownloader();

    class Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * @brief Convert DownloadStatus to string
 */
const char* downloadStatusToString(DownloadStatus status);
