// Download progress broadcaster implementation
#include "download_progress.h"
#include "websocket_server.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <kd/v1/tranquil.pb-c.h>

#include <cstdlib>
#include <vector>

static const char* TAG = "download_progress";

namespace {

// Callers span the httpd task and job worker tasks.
SemaphoreHandle_t s_mutex() {
    static SemaphoreHandle_t m = xSemaphoreCreateMutex();
    return m;
}

class Lock {
public:
    Lock() : ok_(xSemaphoreTake(s_mutex(), pdMS_TO_TICKS(500)) == pdTRUE) {}
    ~Lock() { if (ok_) xSemaphoreGive(s_mutex()); }
    explicit operator bool() const { return ok_; }
private:
    bool ok_;
};

}  // namespace

DownloadProgressBroadcaster& DownloadProgressBroadcaster::instance() {
    static DownloadProgressBroadcaster instance;
    return instance;
}

void DownloadProgressBroadcaster::track(const std::string& pattern_uuid) {
    set(pattern_uuid, 0, false);
}

void DownloadProgressBroadcaster::report(const std::string& pattern_uuid, uint8_t pct) {
    set(pattern_uuid, pct, false);
}

void DownloadProgressBroadcaster::reportIfTracked(const std::string& pattern_uuid, uint8_t pct) {
    set(pattern_uuid, pct, true);
}

void DownloadProgressBroadcaster::complete(const std::string& pattern_uuid) {
    if (pattern_uuid.empty()) return;
    {
        Lock lock;
        if (!lock) return;
        auto it = active_.find(pattern_uuid);
        if (it == active_.end()) return;
        it->second = 100;
    }
    broadcast();
    Lock lock;
    if (lock) active_.erase(pattern_uuid);
}

void DownloadProgressBroadcaster::drop(const std::string& pattern_uuid) {
    Lock lock;
    if (!lock) return;
    active_.erase(pattern_uuid);
}

void DownloadProgressBroadcaster::set(const std::string& pattern_uuid, uint8_t pct,
                                      bool only_if_tracked) {
    if (pattern_uuid.empty()) return;
    if (pct > 100) pct = 100;
    {
        Lock lock;
        if (!lock) return;
        auto it = active_.find(pattern_uuid);
        if (it == active_.end()) {
            if (only_if_tracked) return;
        } else if (it->second == pct) {
            return;  // No change — don't spam clients
        }
        active_[pattern_uuid] = pct;
    }
    broadcast();
}

void DownloadProgressBroadcaster::broadcast() {
    // Snapshot under the lock; pack and send outside it (websocket_broadcast
    // takes its own client mutex and does socket I/O).
    std::vector<std::pair<std::string, uint8_t>> snapshot;
    {
        Lock lock;
        if (!lock) return;
        snapshot.assign(active_.begin(), active_.end());
    }
    if (snapshot.empty()) return;

    std::vector<Kd__V1__PatternDownloadProgress> entries(snapshot.size());
    std::vector<Kd__V1__PatternDownloadProgress*> entry_ptrs(snapshot.size());
    for (size_t i = 0; i < snapshot.size(); i++) {
        kd__v1__pattern_download_progress__init(&entries[i]);
        entries[i].uuid = const_cast<char*>(snapshot[i].first.c_str());
        entries[i].progress_pct = snapshot[i].second;
        entry_ptrs[i] = &entries[i];
    }

    Kd__V1__PatternDownloadProgressReport report =
        KD__V1__PATTERN_DOWNLOAD_PROGRESS_REPORT__INIT;
    report.n_downloads = entry_ptrs.size();
    report.downloads = entry_ptrs.data();

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_PATTERN_DOWNLOAD_PROGRESS;
    msg.pattern_download_progress = &report;

    size_t len = kd__v1__tranquil_message__get_packed_size(&msg);
    auto* buf = static_cast<uint8_t*>(malloc(len));
    if (!buf) {
        ESP_LOGE(TAG, "Failed to alloc %zu bytes for progress report", len);
        return;
    }
    kd__v1__tranquil_message__pack(&msg, buf);
    websocket_broadcast(buf, len);
    free(buf);

    ESP_LOGD(TAG, "Broadcast progress for %zu download(s)", snapshot.size());
}
