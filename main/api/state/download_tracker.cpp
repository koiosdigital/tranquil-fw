// Download tracker implementation
#include "download_tracker.h"
#include "raii_utils.hpp"

#include <esp_timer.h>
#include <esp_log.h>

static const char* TAG = "download_tracker";

DownloadTracker& DownloadTracker::instance() {
    static DownloadTracker instance;
    return instance;
}

DownloadTracker::DownloadTracker() : mutex_(xSemaphoreCreateMutex()) {
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create mutex");
    }
}

void DownloadTracker::trackDownload(const std::string& pattern_uuid) {
    if (pattern_uuid.empty()) return;

    raii::MutexGuard guard(mutex_, portMAX_DELAY);
    if (!guard) return;

    pending_downloads_[pattern_uuid] = PendingDownload{ esp_timer_get_time() };

    ESP_LOGI(TAG, "Tracking download for %s", pattern_uuid.c_str());
}

void DownloadTracker::removeDownload(const std::string& pattern_uuid) {
    raii::MutexGuard guard(mutex_, portMAX_DELAY);
    if (!guard) return;

    auto it = pending_downloads_.find(pattern_uuid);
    if (it != pending_downloads_.end()) {
        ESP_LOGD(TAG, "Removed tracking for %s", pattern_uuid.c_str());
        pending_downloads_.erase(it);
    }
}

void DownloadTracker::cleanupExpired() {
    raii::MutexGuard guard(mutex_, pdMS_TO_TICKS(50));
    if (!guard) return;

    int64_t now = esp_timer_get_time();

    for (auto it = pending_downloads_.begin(); it != pending_downloads_.end(); ) {
        if (now - it->second.request_time_us > PENDING_TIMEOUT_US) {
            ESP_LOGW(TAG, "Expiring stale download tracking: %s", it->first.c_str());
            it = pending_downloads_.erase(it);
        } else {
            ++it;
        }
    }
}

bool DownloadTracker::isTracked(const std::string& pattern_uuid) const {
    raii::MutexGuard guard(mutex_, portMAX_DELAY);
    if (!guard) return false;
    return pending_downloads_.find(pattern_uuid) != pending_downloads_.end();
}
