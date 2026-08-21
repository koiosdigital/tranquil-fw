// Upload-pipeline progress broadcaster implementation.
#include "pipeline_progress.h"
#include "websocket_server.h"
#include "messages.h"  // cloud_msg_queue_raw

#include <esp_log.h>
#include <kd/v1/tranquil.pb-c.h>

#include <cstdlib>

static const char* TAG = "pipeline_progress";

namespace {

// Pack a fully-built TranquilMessage and send it to both transports.
void send(const Kd__V1__TranquilMessage* msg) {
    size_t len = kd__v1__tranquil_message__get_packed_size(msg);
    auto* buf = static_cast<uint8_t*>(malloc(len));
    if (!buf) {
        ESP_LOGE(TAG, "Failed to alloc %zu bytes", len);
        return;
    }
    kd__v1__tranquil_message__pack(msg, buf);
    websocket_broadcast(buf, len);   // local LAN clients
    cloud_msg_queue_raw(buf, len);   // cloud (no-op when offline)
    free(buf);
}

}  // namespace

namespace pipeline_progress {

void conversion(const std::string& uuid, const char* stage, uint8_t pct,
                const std::string& error) {
    if (uuid.empty()) return;
    if (pct > 100) pct = 100;

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
