// Download tracker - remembers which cloud pattern downloads are pending so
// a request that never gets a PatternDownloadResponse can be aged out.
//
// Touched from the httpd task (local RequestPatternDownload), the cloudlink
// task (download response / decline) and the job worker (completion), so
// every method takes the mutex.
#pragma once

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <map>
#include <string>
#include <cstdint>

struct PendingDownload {
    int64_t request_time_us;    // Time when request was made (for timeout)
};

class DownloadTracker {
public:
    static DownloadTracker& instance();

    // Register a pending download request
    // Called when forwarding a RequestPatternDownload to cloud
    void trackDownload(const std::string& pattern_uuid);

    // Remove a tracked download (e.g., download complete or cancelled)
    void removeDownload(const std::string& pattern_uuid);

    // Drop entries older than the pending timeout. Called from the 30 s
    // schedule tick (esp_timer task): gives up and retries next tick when
    // the lock is busy rather than blocking the timer task.
    void cleanupExpired();

    // Check if a download is being tracked
    bool isTracked(const std::string& pattern_uuid) const;

private:
    DownloadTracker();

    mutable SemaphoreHandle_t mutex_ = nullptr;

    // Map of pattern_uuid -> PendingDownload
    std::map<std::string, PendingDownload> pending_downloads_;

    // Timeout for pending downloads (5 minutes)
    static constexpr int64_t PENDING_TIMEOUT_US = 5 * 60 * 1000 * 1000LL;
};
