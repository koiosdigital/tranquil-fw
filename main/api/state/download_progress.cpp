// Download progress broadcaster implementation
#include "download_progress.h"
#include "websocket_server.h"
#include "messages.h"  // cloud_msg_queue_raw — fan progress out to the cloud too

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <kd/v1/tranquil.pb-c.h>

#include <cstdlib>
#include <vector>

static const char* TAG = "download_progress";

namespace {

// Callers span the httpd task and job worker tasks. The critical sections are
// tiny (map ops + a snapshot copy) and never do blocking I/O while held —
// socket sends happen outside the lock — so a blocking acquire cannot deadlock
// and, unlike the old 500 ms timeout, never silently drops a progress tick.
SemaphoreHandle_t s_mutex() {
    static SemaphoreHandle_t m = xSemaphoreCreateMutex();
    return m;
}

class Lock {
public:
    Lock() : ok_(xSemaphoreTake(s_mutex(), portMAX_DELAY) == pdTRUE) {}
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
    auto* buf = static_cast<uint8_t*>(malloc(len));
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
    cloud_msg_queue_raw(buf, len);

    free(buf);
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
        it->second = 100;
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
            last_pct = it->second;
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
            active_[pattern_uuid] = pct;
        } else if (pct <= it->second) {
            // Monotonic: never re-broadcast an unchanged value, and never let a
            // retry (which restarts the byte counter) drag the bar backwards.
            return;
        } else {
            it->second = pct;
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
        snapshot.assign(active_.begin(), active_.end());
    }
    if (snapshot.empty()) return;

    send_report(snapshot, /*failed=*/false, nullptr);
    ESP_LOGD(TAG, "Broadcast progress for %zu download(s)", snapshot.size());
}
