// Download progress broadcaster implementation
#include "download_progress.h"
#include "websocket_server.h"
#include "response_router.h"  // fan progress out to the cloud too (gated on session)

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <kd/v1/tranquil.pb-c.h>

#include <cstdlib>
#include <vector>

static const char* TAG = "download_progress";

namespace {

// Callers span the httpd task, job worker tasks and the cloudlink task. The
// critical sections are tiny (map ops + a snapshot copy) and never do
// blocking I/O while held — socket sends happen outside the lock — so a
// blocking acquire cannot deadlock and never silently drops a progress tick.
// The esp_timer tick (expireStale) is the exception: it uses a short timeout.
SemaphoreHandle_t s_mutex() {
    static SemaphoreHandle_t m = xSemaphoreCreateMutex();
    return m;
}

class Lock {
public:
    explicit Lock(TickType_t timeout = portMAX_DELAY)
        : ok_(xSemaphoreTake(s_mutex(), timeout) == pdTRUE) {}
    ~Lock() { if (ok_) xSemaphoreGive(s_mutex()); }
    explicit operator bool() const { return ok_; }
private:
    bool ok_;
};

// Pack a single-entry report and send it to both transports.
void send_report(const std::vector<std::pair<std::string, uint8_t>>& snapshot,
                 bool failed, const char* error) {
    if (snapshot.empty()) return;

    std::vector<Kd__V1__PatternDownloadProgress> entries(snapshot.size());
    std::vector<Kd__V1__PatternDownloadProgress*> entry_ptrs(snapshot.size());
    for (size_t i = 0; i < snapshot.size(); i++) {
        kd__v1__pattern_download_progress__init(&entries[i]);
        entries[i].uuid = const_cast<char*>(snapshot[i].first.c_str());
        entries[i].progress_pct = snapshot[i].second;
        entries[i].failed = failed;
        if (failed && error) entries[i].error = const_cast<char*>(error);
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
    auto* buf = static_cast<uint8_t*>(heap_caps_malloc(len, MALLOC_CAP_SPIRAM));
    if (!buf) {
        ESP_LOGE(TAG, "Failed to alloc %zu bytes for progress report", len);
        return;
    }
    kd__v1__tranquil_message__pack(&msg, buf);

    // Local LAN clients...
    websocket_broadcast(buf, len);
    // ...and the cloud link, so an app connected off-LAN still sees progress
    // (proto: pattern_download_progress is "&&& sent to cloud"). No-op when the
    // cloud session is down.
    ResponseRouter::instance().forwardToCloud(buf, len);

    heap_caps_free(buf);
}

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
        it->second.pct = 100;
        it->second.updated_us = esp_timer_get_time();
    }
    broadcast();
    Lock lock;
    if (lock) active_.erase(pattern_uuid);
}

void DownloadProgressBroadcaster::fail(const std::string& pattern_uuid,
                                       const std::string& error) {
    if (pattern_uuid.empty()) return;

    uint8_t last_pct = 0;
    bool tracked = false;
    {
        Lock lock;
        if (!lock) return;
        auto it = active_.find(pattern_uuid);
        if (it != active_.end()) {
            last_pct = it->second.pct;
            tracked = true;
            active_.erase(it);
        }
    }
    // Only signal failure for downloads clients were actually tracking. Local
    // uploads aren't in this map; their failures ride the conversion/thumbnail
    // progress reports instead.
    if (!tracked) return;

    std::vector<std::pair<std::string, uint8_t>> one{ { pattern_uuid, last_pct } };
    send_report(one, /*failed=*/true, error.empty() ? "download failed" : error.c_str());
    ESP_LOGW(TAG, "Broadcast download failure for %s: %s",
             pattern_uuid.c_str(), error.c_str());
}

void DownloadProgressBroadcaster::drop(const std::string& pattern_uuid) {
    Lock lock;
    if (!lock) return;
    active_.erase(pattern_uuid);
}

void DownloadProgressBroadcaster::expireStale() {
    Lock lock(pdMS_TO_TICKS(50));
    if (!lock) return;
    pruneLocked(esp_timer_get_time());
}

size_t DownloadProgressBroadcaster::pruneLocked(int64_t now_us) {
    size_t dropped = 0;
    for (auto it = active_.begin(); it != active_.end(); ) {
        if (now_us - it->second.updated_us > STALE_US) {
            ESP_LOGW(TAG, "Dropping stale download %s (stuck at %u%%)",
                     it->first.c_str(), it->second.pct);
            it = active_.erase(it);
            dropped++;
        } else {
            ++it;
        }
    }
    return dropped;
}

void DownloadProgressBroadcaster::set(const std::string& pattern_uuid, uint8_t pct,
                                      bool only_if_tracked) {
    if (pattern_uuid.empty()) return;
    if (pct > 100) pct = 100;
    const int64_t now = esp_timer_get_time();
    {
        Lock lock;
        if (!lock) return;
        auto it = active_.find(pattern_uuid);
        if (it == active_.end()) {
            if (only_if_tracked) return;
            active_[pattern_uuid] = Entry{ pct, now };
        } else if (pct <= it->second.pct) {
            // Monotonic: never re-broadcast an unchanged value, and never let a
            // retry (which restarts the byte counter) drag the bar backwards.
            return;
        } else {
            it->second.pct = pct;
            it->second.updated_us = now;
        }
    }
    broadcast();
}

void DownloadProgressBroadcaster::broadcast() {
    // Snapshot under the lock; pack and send outside it (send_report does
    // socket I/O and takes the websocket client's own mutex).
    std::vector<std::pair<std::string, uint8_t>> snapshot;
    {
        Lock lock;
        if (!lock) return;
        pruneLocked(esp_timer_get_time());
        snapshot.reserve(active_.size());
        for (const auto& kv : active_) {
            snapshot.emplace_back(kv.first, kv.second.pct);
        }
    }
    if (snapshot.empty()) return;

    send_report(snapshot, /*failed=*/false, nullptr);
    ESP_LOGD(TAG, "Broadcast progress for %zu download(s)", snapshot.size());
}
