// Unified pattern handler implementation
#include "pattern_handler.h"
#include "ManifestDatabase.h"
#include "download_tracker.h"

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

HandleResult PatternHandler::handleDownloadedPatternsRequest(ResponseMessage& response) {
    auto patterns = ManifestDatabase::instance().getAllPatterns();

    // Use static storage for the response (protobuf lifetime)
    static Kd__V1__DownloadedPatterns downloaded = KD__V1__DOWNLOADED_PATTERNS__INIT;
    static std::vector<Kd__V1__PatternInfo*> pattern_ptrs;
    static std::vector<Kd__V1__PatternInfo> pattern_infos;

    // Clear previous data
    pattern_ptrs.clear();
    pattern_infos.clear();

    // Limit to 100 patterns to avoid excessive memory usage
    size_t count = std::min(patterns.size(), static_cast<size_t>(100));
    pattern_infos.resize(count);

    for (size_t i = 0; i < count; i++) {
        pattern_infos[i] = KD__V1__PATTERN_INFO__INIT;
        // Note: These pointers are valid while patterns vector is in scope
        pattern_infos[i].uuid = const_cast<char*>(patterns[i].uuid.c_str());
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

    response = serialize(&resp);

    ESP_LOGI(TAG, "DownloadedPatterns: returning %zu patterns", count);
    return HandleResult::ok(false, false);
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
    esp_err_t ret = ManifestDatabase::instance().deletePattern(msg->uuid);

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

    // Get existing pattern
    auto pattern = ManifestDatabase::instance().getPattern(msg->uuid);
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

    // Save updated pattern
    esp_err_t ret = ManifestDatabase::instance().updatePattern(msg->uuid, *pattern);

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

    auto pattern = ManifestDatabase::instance().getPattern(msg->uuid);
    if (!pattern) {
        ESP_LOGW(TAG, "GetPatternRequest: pattern not found: %s", msg->uuid);
        response = makeCommandResult(false, "Pattern not found");
        return HandleResult::ok();
    }

    // Build PatternInfo response
    static Kd__V1__PatternInfo info = KD__V1__PATTERN_INFO__INIT;
    static char uuid_buf[64], name_buf[128], creator_buf[64];
    static char created_buf[32], played_buf[32];

    strncpy(uuid_buf, pattern->uuid.c_str(), sizeof(uuid_buf) - 1);
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
