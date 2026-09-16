// Unified pattern handler implementation
#include "pattern_handler.h"
#include "ops/library_ops.h"
#include "download_tracker.h"
#include "download_progress.h"
#include "library_notifier.h"

#include <esp_log.h>
#include <cstring>

static const char* TAG = "pattern_handler";

namespace {

bool has(const char* s) { return s && s[0] != '\0'; }

}  // namespace

PatternHandler& PatternHandler::instance() {
    static PatternHandler instance;
    return instance;
}

HandleResult PatternHandler::handle(
    const Kd__V1__TranquilMessage* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    const char* rid = ctx.request_id;

    switch (msg->message_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_DOWNLOADED_PATTERNS_REQUEST:
            response = library_ops::build_pattern_list_message(rid);
            return HandleResult::ok(false, false);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_DELETE_PATTERN: {
            const auto* m = msg->delete_pattern;
            if (!m || !has(m->uuid)) {
                ESP_LOGW(TAG, "DeletePattern: missing uuid");
                response = makeCommandResult(false, "Missing uuid", rid, ESP_ERR_INVALID_ARG);
                return HandleResult::ok();
            }
            ESP_LOGI(TAG, "DeletePattern: %s", m->uuid);
            // Requester gets the CommandResult; everyone (LAN + cloud) gets
            // the updated list via the notifier (posted by library_ops).
            response = makeCommandResult(library_ops::delete_pattern(m->uuid), rid);
            return HandleResult::ok();
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_MODIFY_PATTERN: {
            const auto* m = msg->modify_pattern;
            if (!m || !has(m->uuid)) {
                ESP_LOGW(TAG, "ModifyPattern: missing uuid");
                response = makeCommandResult(false, "Missing uuid", rid, ESP_ERR_INVALID_ARG);
                return HandleResult::ok();
            }
            response = makeCommandResult(
                library_ops::rename_pattern(m->uuid, m->name ? m->name : ""), rid);
            return HandleResult::ok();
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_PATTERN_REQUEST: {
            const auto* m = msg->get_pattern_request;
            if (!m || !has(m->uuid)) {
                ESP_LOGW(TAG, "GetPatternRequest: missing uuid");
                response = makeCommandResult(false, "Missing uuid", rid, ESP_ERR_INVALID_ARG);
                return HandleResult::ok();
            }
            auto pattern = library_ops::get_pattern(m->uuid);
            if (!pattern) {
                ESP_LOGW(TAG, "GetPatternRequest: pattern not found: %s", m->uuid);
                response = makeCommandResult(false, "Pattern not found", rid, ESP_ERR_NOT_FOUND);
                return HandleResult::ok();
            }
            response = library_ops::build_pattern_info_message(*pattern, rid);
            ESP_LOGI(TAG, "GetPatternRequest: returning info for %s", m->uuid);
            return HandleResult::ok(false, false);
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_REQUEST_PATTERN_DOWNLOAD: {
            const auto* m = msg->request_pattern_download;
            if (!m || !has(m->pattern_uuid)) {
                ESP_LOGW(TAG, "RequestPatternDownload: missing pattern_uuid");
                response = makeCommandResult(false, "Missing pattern_uuid", rid, ESP_ERR_INVALID_ARG);
                return HandleResult::ok();
            }

            ESP_LOGI(TAG, "RequestPatternDownload: %s (from %s)",
                m->pattern_uuid, ctx.isLocal() ? "local" : "cloud");

            // Track the request so it can be aged out if the cloud never
            // answers, and start the progress bar at 0% (accepted, job not
            // started). Both sources forward to the cloud: a local request
            // needs the download URL; a cloud-initiated one is the designed
            // round-trip (app -> cloud command -> us -> forward back -> cloud
            // replies with a PatternDownloadResponse; see tranquil.proto).
            if (ctx.isLocal()) {
                DownloadTracker::instance().trackDownload(m->pattern_uuid);
            }
            DownloadProgressBroadcaster::instance().track(m->pattern_uuid);
            ESP_LOGI(TAG, "Forwarding pattern download request to cloud");
            return HandleResult::forwardToCloud();
        }

        default:
            return HandleResult::notHandled();
    }
}

bool PatternHandler::canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const {
    switch (msg_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_DOWNLOADED_PATTERNS_REQUEST:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_DELETE_PATTERN:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_MODIFY_PATTERN:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_PATTERN_REQUEST:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_REQUEST_PATTERN_DOWNLOAD:
            return true;
        default:
            return false;
    }
}

std::vector<Kd__V1__TranquilMessage__MessageCase> PatternHandler::supportedMessages() const {
    return {
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_DOWNLOADED_PATTERNS_REQUEST,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_DELETE_PATTERN,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_MODIFY_PATTERN,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_PATTERN_REQUEST,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_REQUEST_PATTERN_DOWNLOAD
    };
}

void PatternHandler::notifyPatternDownloadComplete(const std::string& pattern_uuid, bool success) {
    DownloadTracker::instance().removeDownload(pattern_uuid);

    if (!success) return;

    // Push the refreshed pattern list (debounced, LAN + cloud) so clients see
    // the new pattern without polling. A bulk playlist download coalesces
    // into one push instead of one per pattern.
    ESP_LOGI(TAG, "Pattern %s downloaded; scheduling list push", pattern_uuid.c_str());
    library_notifier::patterns_changed();
}
