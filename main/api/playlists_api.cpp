// Playlist REST API: a thin JSON codec over library_ops. Every mutation is
// pushed to WebSocket clients and the cloud by the ops layer.
#include "esp_http_server.h"
#include "cJSON.h"
#include "api_common.h"
#include "ops/library_ops.h"
#include <string.h>
#include <stdlib.h>
#include <string>
#include <vector>

using namespace api_common;

namespace {

// Playlist bodies carry whole pattern lists (up to ~100 uuids).
constexpr size_t kBodyLimit = 16384;

esp_err_t send_playlist(httpd_req_t* req, const Playlist& pl) {
    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    library_ops::playlist_to_json(&cb, pl);
    chunk_buf_finish(&cb);
    return ESP_OK;
}

const char* json_str(const cJSON* obj, const char* key) {
    const cJSON* v = cJSON_GetObjectItem(obj, key);
    return (v && cJSON_IsString(v)) ? v->valuestring : nullptr;
}

std::vector<std::string> string_array(const cJSON* array) {
    std::vector<std::string> out;
    const cJSON* item;
    cJSON_ArrayForEach(item, array) {
        if (cJSON_IsString(item)) out.emplace_back(item->valuestring);
    }
    return out;
}

// Lookup + 404 for the /api/playlists/{uuid} routes.
bool playlist_from_uri(httpd_req_t* req, std::string& uuid) {
    uuid = extract_uuid(req->uri, "/api/playlists/");
    if (uuid.empty() || !library_ops::get_playlist(uuid)) {
        httpd_resp_send_404(req);
        return false;
    }
    return true;
}

} // anonymous namespace

// GET /api/playlists - paginated list (streamed)
static esp_err_t playlists_list_handler(httpd_req_t* req) {
    int page, per_page;
    parse_pagination(req, page, per_page);

    auto result = ManifestDatabase::instance().getPlaylists(page, per_page);

    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);

    chunk_buf_printf(&cb,
        "{\"pagination\":{\"page\":%d,\"per_page\":%d,\"total_pages\":%d,\"total_items\":%d},\"playlists\":[",
        result.pagination.page, result.pagination.per_page,
        result.pagination.total_pages, result.pagination.total_items);

    bool first = true;
    for (const auto& playlist : result.items) {
        if (!first) chunk_buf_write(&cb, ",", 1);
        library_ops::playlist_to_json(&cb, playlist);
        first = false;
    }

    chunk_buf_write_str(&cb, "]}");
    chunk_buf_finish(&cb);
    return ESP_OK;
}

// GET /api/playlists/{uuid}
static esp_err_t playlists_detail_handler(httpd_req_t* req) {
    std::string uuid = extract_uuid(req->uri, "/api/playlists/");
    auto playlist = library_ops::get_playlist(uuid);
    if (!playlist) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    return send_playlist(req, *playlist);
}

// POST /api/playlists - create playlist
// Body: {"name","description","featured_pattern","pattern_uuids":[...],"uuid"}
static esp_err_t playlists_create_handler(httpd_req_t* req) {
    cJSON* json = parse_json_body(req, kBodyLimit);
    if (!json) return ESP_FAIL;

    library_ops::PlaylistDraft draft;
    if (const char* s = json_str(json, "uuid")) draft.uuid = s;
    if (const char* s = json_str(json, "name")) draft.name = s;
    if (const char* s = json_str(json, "description")) draft.description = s;
    if (const char* s = json_str(json, "featured_pattern")) draft.featured_pattern = s;
    // "pattern_uuids" per swagger; "patterns" kept for older payloads
    const cJSON* patterns = cJSON_GetObjectItem(json, "pattern_uuids");
    if (!patterns) patterns = cJSON_GetObjectItem(json, "patterns");
    if (patterns && cJSON_IsArray(patterns)) draft.pattern_uuids = string_array(patterns);
    cJSON_Delete(json);

    Playlist created;
    OpResult r = library_ops::create_playlist(draft, created);
    if (!r.ok()) return send_op_error(req, r);

    return send_playlist(req, created);
}

// POST /api/playlists/{uuid} - add/remove pattern
static esp_err_t playlists_modify_handler(httpd_req_t* req, const std::string& uuid) {
    cJSON* json = parse_json_body(req, kBodyLimit);
    if (!json) return ESP_FAIL;

    // "pattern_uuid" per swagger; "pattern" kept for older clients
    const char* pattern = json_str(json, "pattern_uuid");
    if (!pattern) pattern = json_str(json, "pattern");
    const char* action = json_str(json, "action");

    if (!pattern || !action) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing pattern_uuid or action");
        return ESP_FAIL;
    }

    Playlist updated;
    OpResult r;
    if (strcmp(action, "add") == 0) {
        r = library_ops::add_pattern_to_playlist(uuid, pattern, updated);
    } else if (strcmp(action, "remove") == 0 || strcmp(action, "delete") == 0) {
        r = library_ops::remove_pattern_from_playlist(uuid, pattern, updated);
    } else {
        r = OpResult::fail(ESP_ERR_INVALID_ARG, "Unknown action");
    }
    cJSON_Delete(json);

    if (!r.ok()) return send_op_error(req, r);
    return send_playlist(req, updated);
}

// POST /api/playlists/{uuid}/order - reorder playlist
static esp_err_t playlists_order_handler(httpd_req_t* req, const std::string& uuid) {
    cJSON* json = parse_json_body(req, kBodyLimit);
    if (!json) return ESP_FAIL;

    // Swagger shape is {"pattern_uuids":[...]}; a bare array is also accepted
    const cJSON* array = cJSON_IsObject(json) ? cJSON_GetObjectItem(json, "pattern_uuids") : json;
    if (!array || !cJSON_IsArray(array)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected pattern_uuids array");
        return ESP_FAIL;
    }

    std::vector<std::string> order = string_array(array);
    cJSON_Delete(json);

    Playlist updated;
    OpResult r = library_ops::reorder_playlist(uuid, order, updated);
    if (!r.ok()) return send_op_error(req, r);
    return send_playlist(req, updated);
}

// POST /api/playlists/{uuid}[/order] - router.
// ESP-IDF wildcards only match at the END of a template, so
// "/api/playlists/*/order" can never be registered/matched as its own URI —
// both modify and reorder share the "/api/playlists/*" template and are
// dispatched here by suffix.
static esp_err_t playlists_post_router(httpd_req_t* req) {
    std::string uuid;
    if (!playlist_from_uri(req, uuid)) return ESP_FAIL;

    const char* rest = req->uri + strlen("/api/playlists/") + uuid.size();
    if (strncmp(rest, "/order", 6) == 0) {
        return playlists_order_handler(req, uuid);
    }
    return playlists_modify_handler(req, uuid);
}

// PATCH /api/playlists/{uuid} - update metadata and/or replace pattern list
// Body (all optional): {"name","description","featured_pattern","pattern_uuids":[...]}
static esp_err_t playlists_patch_handler(httpd_req_t* req) {
    std::string uuid;
    if (!playlist_from_uri(req, uuid)) return ESP_FAIL;

    cJSON* json = parse_json_body(req, kBodyLimit);
    if (!json) return ESP_FAIL;

    library_ops::PlaylistPatch patch;
    if (const char* s = json_str(json, "name")) patch.name = std::string(s);
    if (const char* s = json_str(json, "description")) patch.description = std::string(s);
    if (const char* s = json_str(json, "featured_pattern")) patch.featured_pattern = std::string(s);
    const cJSON* pattern_uuids = cJSON_GetObjectItem(json, "pattern_uuids");
    if (pattern_uuids && cJSON_IsArray(pattern_uuids)) patch.pattern_uuids = string_array(pattern_uuids);
    cJSON_Delete(json);

    Playlist updated;
    OpResult r = library_ops::update_playlist(uuid, patch, updated);
    if (!r.ok()) return send_op_error(req, r);
    return send_playlist(req, updated);
}

// DELETE /api/playlists/{uuid}
static esp_err_t playlists_delete_handler(httpd_req_t* req) {
    std::string uuid = extract_uuid(req->uri, "/api/playlists/");
    if (uuid.empty()) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    OpResult r = library_ops::delete_playlist(uuid);
    if (!r.ok()) return send_op_error(req, r);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"deleted\"}", -1);
    return ESP_OK;
}

void playlists_api_register_handlers(httpd_handle_t server) {
    static httpd_uri_t playlists_list_uri = {
        .uri = "/api/playlists",
        .method = HTTP_GET,
        .handler = playlists_list_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &playlists_list_uri);

    static httpd_uri_t playlists_create_uri = {
        .uri = "/api/playlists",
        .method = HTTP_POST,
        .handler = playlists_create_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &playlists_create_uri);

    static httpd_uri_t playlists_detail_uri = {
        .uri = "/api/playlists/*",
        .method = HTTP_GET,
        .handler = playlists_detail_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &playlists_detail_uri);

    static httpd_uri_t playlists_post_uri = {
        .uri = "/api/playlists/*",
        .method = HTTP_POST,
        .handler = playlists_post_router,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &playlists_post_uri);

    static httpd_uri_t playlists_patch_uri = {
        .uri = "/api/playlists/*",
        .method = HTTP_PATCH,
        .handler = playlists_patch_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &playlists_patch_uri);

    static httpd_uri_t playlists_delete_uri = {
        .uri = "/api/playlists/*",
        .method = HTTP_DELETE,
        .handler = playlists_delete_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &playlists_delete_uri);
}
