// Upload-pipeline progress broadcaster implementation.
#include "pipeline_progress.h"
#include "websocket_server.h"
#include "messages.h"  // cloud_msg_queue_raw
#include "sockets.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <kd/v1/tranquil.pb-c.h>

#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>
#include <string>

static const char* TAG = "pipeline_progress";

namespace {

// Per-uuid throttle. The converter reports every whole percent (two frames
// per percent once the download bar is included); each frame costs a
// per-client copy plus an httpd work-queue slot (12 deep, drops when full),
// and the frames most likely to be dropped were the terminal ones. Only let
// a non-terminal update through every >=5 points or >=250 ms.
struct Last {
    uint8_t pct;
    int64_t at_us;
};
std::map<std::string, Last> s_last;
std::mutex s_mutex;

bool should_send(const std::string& uuid, const char* stage, uint8_t pct) {
    const bool terminal = stage && (strcmp(stage, "complete") == 0 || strcmp(stage, "failed") == 0);
    const int64_t now = esp_timer_get_time();
    std::lock_guard<std::mutex> lock(s_mutex);
    if (terminal) {
        s_last.erase(uuid);
        return true;
    }
    auto it = s_last.find(uuid);
    if (it == s_last.end()) {
        s_last[uuid] = { pct, now };
        return true;
    }
    if (pct >= it->second.pct + 5 || now - it->second.at_us >= 250 * 1000) {
        it->second = { pct, now };
        return true;
    }
    return false;
}

// Pack a fully-built TranquilMessage and send it to both transports.
void send(const Kd__V1__TranquilMessage* msg) {
    size_t len = kd__v1__tranquil_message__get_packed_size(msg);
    auto* buf = static_cast<uint8_t*>(heap_caps_malloc(len, MALLOC_CAP_SPIRAM));
    if (!buf) {
        ESP_LOGE(TAG, "Failed to alloc %zu bytes", len);
        return;
    }
    kd__v1__tranquil_message__pack(msg, buf);
    websocket_broadcast(buf, len);   // local LAN clients
    // The cloud only mirrors store downloads (the `downloads` section);
    // upload post-processing is LAN-only, so don't spend a webhook on it.
    heap_caps_free(buf);
}

}  // namespace

namespace pipeline_progress {

void conversion(const std::string& uuid, const char* stage, uint8_t pct,
                const std::string& error) {
    if (uuid.empty()) return;
    if (pct > 100) pct = 100;
    if (!should_send(uuid, stage, pct)) return;

    Kd__V1__PatternConversionProgress entry = KD__V1__PATTERN_CONVERSION_PROGRESS__INIT;
    entry.uuid = const_cast<char*>(uuid.c_str());
    entry.stage = const_cast<char*>(stage ? stage : "");
    entry.progress_pct = pct;
    if (!error.empty()) entry.error = const_cast<char*>(error.c_str());

    Kd__V1__PatternConversionProgress* entries[1] = { &entry };
    Kd__V1__PatternConversionProgressReport report =
        KD__V1__PATTERN_CONVERSION_PROGRESS_REPORT__INIT;
    report.n_conversions = 1;
    report.conversions = entries;

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_PATTERN_CONVERSION_PROGRESS;
    msg.pattern_conversion_progress = &report;
    send(&msg);
}

void thumb(const std::string& uuid, const char* stage, uint8_t pct,
           const std::string& error) {
    if (uuid.empty()) return;
    if (pct > 100) pct = 100;
    if (!should_send("thumb:" + uuid, stage, pct)) return;

    Kd__V1__PatternThumbProgress entry = KD__V1__PATTERN_THUMB_PROGRESS__INIT;
    entry.uuid = const_cast<char*>(uuid.c_str());
    entry.stage = const_cast<char*>(stage ? stage : "");
    entry.progress_pct = pct;
    if (!error.empty()) entry.error = const_cast<char*>(error.c_str());

    Kd__V1__PatternThumbProgress* entries[1] = { &entry };
    Kd__V1__PatternThumbProgressReport report =
        KD__V1__PATTERN_THUMB_PROGRESS_REPORT__INIT;
    report.n_thumbnails = 1;
    report.thumbnails = entries;

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_PATTERN_THUMB_PROGRESS;
    msg.pattern_thumb_progress = &report;
    send(&msg);
}

}  // namespace pipeline_progress
