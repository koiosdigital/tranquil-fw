// Unified pattern handler implementation
#include "pattern_handler.h"
#include "ManifestDatabase.h"
#include "download_tracker.h"
#include "websocket_server.h"

#include <esp_log.h>
#include <cstring>
#include <vector>

static const char* TAG = "pattern_handler";

PatternHandler& PatternHandler::instance() {
    static PatternHandler instance;
    return instance;
}

HandleResult PatternHandler::handle(
    const Kd__V1__TranquilMessage* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    switch (msg->message_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_DOWNLOADED_PATTERNS_REQUEST:
            return handleDownloadedPatternsRequest(response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_DELETE_PATTERN:
            return handleDeletePattern(msg->delete_pattern, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_MODIFY_PATTERN:
            return handleModifyPattern(msg->modify_pattern, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_PATTERN_REQUEST:
            return handleGetPatternRequest(msg->get_pattern_request, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_REQUEST_PATTERN_DOWNLOAD:
            return handleRequestPatternDownload(msg->request_pattern_download, ctx, response);

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

ResponseMessage PatternHandler::buildPatternListMessage() {
    auto patterns = ManifestDatabase::instance().getAllPatterns();

    // Local storage is sufficient: serialize() packs the message into its
    // own buffer before this scope ends. (This is also called from the job
    // worker task, so statics here would race with the dispatcher.)
    Kd__V1__DownloadedPatterns downloaded = KD__V1__DOWNLOADED_PATTERNS__INIT;
    std::vector<Kd__V1__PatternInfo*> pattern_ptrs;
    std::vector<Kd__V1__PatternInfo> pattern_infos;

    // Limit to 100 patterns to avoid excessive memory usage
    size_t count = std::min(patterns.size(), static_cast<size_t>(100));
    pattern_infos.resize(count);

    for (size_t i = 0; i < count; i++) {
        pattern_infos[i] = KD__V1__PATTERN_INFO__INIT;
        // Note: These pointers are valid while patterns vector is in scope
        // Use external_uuid as "uuid" for API compatibility
        pattern_infos[i].uuid = const_cast<char*>(patterns[i].external_uuid.c_str());
        pattern_infos[i].name = const_cast<char*>(patterns[i].name.c_str());
        pattern_infos[i].creator = const_cast<char*>(patterns[i].creator.c_str());
        pattern_infos[i].encrypted = patterns[i].encrypted;
        pattern_infos[i].size_bytes = patterns[i].size_bytes;
        pattern_infos[i].reversible = patterns[i].reversible;
        pattern_infos[i].start_point = patterns[i].start_point;
        pattern_infos[i].created_at = const_cast<char*>(patterns[i].created_at.c_str());
        pattern_infos[i].last_played_at = const_cast<char*>(patterns[i].last_played_at.c_str());
        pattern_ptrs.push_back(&pattern_infos[i]);
    }

    downloaded.n_patterns = pattern_ptrs.size();
    downloaded.patterns = pattern_ptrs.data();

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_DOWNLOADED_PATTERNS;
    resp.downloaded_patterns = &downloaded;

    ESP_LOGI(TAG, "DownloadedPatterns: returning %zu patterns", count);
    return serialize(&resp);
}

HandleResult PatternHandler::handleDownloadedPatternsRequest(ResponseMessage& response) {
    response = buildPatternListMessage();
    return HandleResult::ok(false, false);
}

void PatternHandler::notifyPatternDownloadComplete(const std::string& pattern_uuid, bool success) {
    DownloadTracker::instance().removeDownload(pattern_uuid);

    if (!success) return;

    // Push the refreshed pattern list so clients see the new pattern without
    // polling. The requester is included, so no separate unicast is needed.
    ResponseMessage msg = buildPatternListMessage();
    if (msg.valid()) {
        esp_err_t err = websocket_broadcast(msg.data(), msg.len());
        ESP_LOGI(TAG, "Broadcast pattern list after download of %s: %s",
            pattern_uuid.c_str(), esp_err_to_name(err));
    } else {
        ESP_LOGW(TAG, "Failed to build pattern list broadcast for %s",
            pattern_uuid.c_str());
    }
}

HandleResult PatternHandler::handleDeletePattern(
    const Kd__V1__DeletePattern* msg,
    ResponseMessage& response
) {
    if (!msg || !msg->uuid) {
        ESP_LOGW(TAG, "DeletePattern: missing uuid");
        response = makeCommandResult(false, "Missing uuid");
        return HandleResult::ok();
    }

    ESP_LOGI(TAG, "DeletePattern: %s", msg->uuid);

    // Find pattern by external UUID and delete by internal ID
    auto pattern = ManifestDatabase::instance().getPatternByExternalUuid(msg->uuid);
    if (!pattern) {
        ESP_LOGW(TAG, "DeletePattern: pattern not found: %s", msg->uuid);
        response = makeCommandResult(false, "Pattern not found");
        return HandleResult::ok();
    }

    esp_err_t ret = ManifestDatabase::instance().deletePattern(pattern->id);

    if (ret == ESP_OK) {
        response = makeCommandResult(true);
        // Broadcast pattern list change to local clients
        return HandleResult::ok(true, false);
    } else {
        response = makeCommandResult(false, "Failed to delete pattern");
        return HandleResult::ok();
    }
}

HandleResult PatternHandler::handleModifyPattern(
    const Kd__V1__ModifyPattern* msg,
    ResponseMessage& response
) {
    if (!msg || !msg->uuid) {
        ESP_LOGW(TAG, "ModifyPattern: missing uuid");
        response = makeCommandResult(false, "Missing uuid");
        return HandleResult::ok();
    }

    // Get existing pattern by external UUID
    auto pattern = ManifestDatabase::instance().getPatternByExternalUuid(msg->uuid);
    if (!pattern) {
        ESP_LOGW(TAG, "ModifyPattern: pattern not found: %s", msg->uuid);
        response = makeCommandResult(false, "Pattern not found");
        return HandleResult::ok();
    }

    // Update name if provided
    if (msg->name && strlen(msg->name) > 0) {
        pattern->name = msg->name;
        ESP_LOGI(TAG, "ModifyPattern: rename %s to '%s'", msg->uuid, msg->name);
    }

    // Save updated pattern using internal ID
    esp_err_t ret = ManifestDatabase::instance().updatePattern(pattern->id, *pattern);

    if (ret == ESP_OK) {
        response = makeCommandResult(true);
        return HandleResult::ok(true, false);
    } else {
        response = makeCommandResult(false, "Failed to update pattern");
        return HandleResult::ok();
    }
}

HandleResult PatternHandler::handleGetPatternRequest(
    const Kd__V1__GetPatternRequest* msg,
    ResponseMessage& response
) {
    if (!msg || !msg->uuid) {
        ESP_LOGW(TAG, "GetPatternRequest: missing uuid");
        response = makeCommandResult(false, "Missing uuid");
        return HandleResult::ok();
    }

    // Lookup by external UUID
    auto pattern = ManifestDatabase::instance().getPatternByExternalUuid(msg->uuid);
    if (!pattern) {
        ESP_LOGW(TAG, "GetPatternRequest: pattern not found: %s", msg->uuid);
        response = makeCommandResult(false, "Pattern not found");
        return HandleResult::ok();
    }

    // Build PatternInfo response. Locals, not statics: httpd and the
    // cloudlink task can run this concurrently, and shared static buffers
    // raced between get_packed_size() and pack().
    Kd__V1__PatternInfo info = KD__V1__PATTERN_INFO__INIT;
    char uuid_buf[64] = {0}, name_buf[128] = {0}, creator_buf[64] = {0};
    char created_buf[32] = {0}, played_buf[32] = {0};

    // Use external_uuid as "uuid" for API compatibility
    strncpy(uuid_buf, pattern->external_uuid.c_str(), sizeof(uuid_buf) - 1);
    strncpy(name_buf, pattern->name.c_str(), sizeof(name_buf) - 1);
    strncpy(creator_buf, pattern->creator.c_str(), sizeof(creator_buf) - 1);
    strncpy(created_buf, pattern->created_at.c_str(), sizeof(created_buf) - 1);
    strncpy(played_buf, pattern->last_played_at.c_str(), sizeof(played_buf) - 1);

    info.uuid = uuid_buf;
    info.name = name_buf;
    info.creator = creator_buf;
    info.encrypted = pattern->encrypted;
    info.size_bytes = pattern->size_bytes;
    info.reversible = pattern->reversible;
    info.start_point = pattern->start_point;
    info.created_at = created_buf;
    info.last_played_at = played_buf;

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_PATTERN_INFO;
    resp.pattern_info = &info;

    response = serialize(&resp);

    ESP_LOGI(TAG, "GetPatternRequest: returning info for %s", msg->uuid);
    return HandleResult::ok(false, false);
}

HandleResult PatternHandler::handleRequestPatternDownload(
    const Kd__V1__RequestPatternDownload* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    if (!msg || !msg->pattern_uuid) {
        ESP_LOGW(TAG, "RequestPatternDownload: missing pattern_uuid");
        response = makeCommandResult(false, "Missing pattern_uuid");
        return HandleResult::ok();
    }

    ESP_LOGI(TAG, "RequestPatternDownload: %s (from %s)",
        msg->pattern_uuid,
        ctx.isLocal() ? "local" : "cloud");

    // If request is from local client, forward to cloud
    if (ctx.isLocal()) {
        // Track the download so we can route progress back to this client
        DownloadTracker::instance().trackDownload(msg->pattern_uuid, ctx.socket_fd);

        ESP_LOGI(TAG, "Forwarding pattern download request to cloud (tracked socket %d)",
            ctx.socket_fd);
        return HandleResult::forwardToCloud();
    }

    // If request is from cloud (unlikely but handle it), forward back
    // This case shouldn't normally happen
    ESP_LOGW(TAG, "Pattern download request from cloud? Forwarding anyway");
    return HandleResult::forwardToCloud();
}
