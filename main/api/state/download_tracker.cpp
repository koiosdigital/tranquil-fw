// Download tracker implementation
#include "download_tracker.h"
#include <esp_timer.h>
#include <esp_log.h>

static const char* TAG = "download_tracker";

DownloadTracker& DownloadTracker::instance() {
    static DownloadTracker instance;
    return instance;
}

void DownloadTracker::trackDownload(const std::string& pattern_uuid, int origin_socket_fd) {
    if (pattern_uuid.empty()) return;

    PendingDownload download = {
        .origin_socket_fd = origin_socket_fd,
        .request_time_us = esp_timer_get_time(),
        .pattern_uuid = pattern_uuid
    };

    pending_downloads_[pattern_uuid] = download;

    ESP_LOGI(TAG, "Tracking download for %s from socket %d",
        pattern_uuid.c_str(), origin_socket_fd);
}

int DownloadTracker::getOriginSocket(const std::string& pattern_uuid) {
    auto it = pending_downloads_.find(pattern_uuid);
    if (it == pending_downloads_.end()) {
        return -1;
    }

    int64_t now = esp_timer_get_time();
    if (now - it->second.request_time_us > PENDING_TIMEOUT_US) {
        ESP_LOGW(TAG, "Download %s expired", pattern_uuid.c_str());
        pending_downloads_.erase(it);
        return -1;
    }

    int socket_fd = it->second.origin_socket_fd;
    pending_downloads_.erase(it);

    ESP_LOGD(TAG, "Retrieved origin socket %d for download %s",
        socket_fd, pattern_uuid.c_str());

    return socket_fd;
}

int DownloadTracker::peekOriginSocket(const std::string& pattern_uuid) const {
    auto it = pending_downloads_.find(pattern_uuid);
    if (it == pending_downloads_.end()) {
        return -1;
    }

    int64_t now = esp_timer_get_time();
    if (now - it->second.request_time_us > PENDING_TIMEOUT_US) {
        return -1;
    }

    return it->second.origin_socket_fd;
}

void DownloadTracker::removeDownload(const std::string& pattern_uuid) {
    auto it = pending_downloads_.find(pattern_uuid);
    if (it != pending_downloads_.end()) {
        ESP_LOGD(TAG, "Removed tracking for %s", pattern_uuid.c_str());
        pending_downloads_.erase(it);
    }
}

void DownloadTracker::cleanupExpired() {
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
    return pending_downloads_.find(pattern_uuid) != pending_downloads_.end();
}
