#include "esp_http_server.h"
#include "cJSON.h"
#include "ManifestDatabase.h"
#include "api_common.h"
#include <string.h>
#include <stdlib.h>
#include <vector>

using namespace api_common;

namespace {

// Stream a playlist object. Pattern IDs are resolved to external UUIDs one at
// a time so the full list never has to sit in a JSON tree on the heap.
void write_playlist_json(ChunkBuf* cb, const Playlist& pl) {
    chunk_buf_write_str(cb, "{\"uuid\":");
    chunk_buf_write_json_string(cb, pl.external_uuid.c_str());
    chunk_buf_printf(cb, ",\"id\":%lu,\"name\":", (unsigned long)pl.id);
    chunk_buf_write_json_string(cb, pl.name.c_str());
    chunk_buf_write_str(cb, ",\"description\":");
    chunk_buf_write_json_string(cb, pl.description.c_str());

    chunk_buf_write_str(cb, ",\"featured_pattern\":");
    std::string featured;
    if (pl.featured_pattern_id != 0) {
        auto fp = ManifestDatabase::instance().getPattern(pl.featured_pattern_id);
        if (fp) featured = fp->external_uuid;
    }
    chunk_buf_write_json_string(cb, featured.c_str());

    chunk_buf_write_str(cb, ",\"date\":");
    chunk_buf_write_json_string(cb, pl.date.c_str());
    if (!pl.created_at.empty()) {
        chunk_buf_write_str(cb, ",\"created_at\":");
        chunk_buf_write_json_string(cb, pl.created_at.c_str());
    }
    if (!pl.updated_at.empty()) {
        chunk_buf_write_str(cb, ",\"updated_at\":");
        chunk_buf_write_json_string(cb, pl.updated_at.c_str());
    }

    chunk_buf_write_str(cb, ",\"pattern_uuids\":[");
    bool first = true;
    for (uint32_t pid : pl.pattern_ids) {
        auto p = ManifestDatabase::instance().getPattern(pid);
        if (!p) continue;
        if (!first) chunk_buf_write(cb, ",", 1);
        chunk_buf_write_json_string(cb, p->external_uuid.c_str());
        first = false;
    }
    chunk_buf_write_str(cb, "]}");
}

esp_err_t send_playlist(httpd_req_t* req, const Playlist& pl) {
    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    write_playlist_json(&cb, pl);
    chunk_buf_finish(&cb);
    return ESP_OK;
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
        write_playlist_json(&cb, playlist);
        first = false;
    }

    chunk_buf_write_str(&cb, "]}");
    chunk_buf_finish(&cb);
    return ESP_OK;
}

// GET /api/playlists/{uuid}
static esp_err_t playlists_detail_handler(httpd_req_t* req) {
    std::string uuid = extract_uuid(req->uri, "/api/playlists/");
    if (uuid.empty()) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    auto playlist = ManifestDatabase::instance().getPlaylistByExternalUuid(uuid);
    if (!playlist) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    return send_playlist(req, *playlist);
}

// POST /api/playlists - create playlist
static esp_err_t playlists_create_handler(httpd_req_t* req) {
    cJSON* json = parse_json_body(req, 4096);
    if (!json) return ESP_FAIL;

    Playlist playlist = ManifestDatabase::jsonToPlaylist(json);
    cJSON_Delete(json);

    // Generate external_uuid if not provided
    if (playlist.external_uuid.empty()) {
        playlist.external_uuid = ManifestDatabase::generateUUID();
    }
    if (playlist.created_at.empty()) {
        playlist.created_at = ManifestDatabase::currentTimestamp();
    }
    playlist.updated_at = playlist.created_at;

    if (ManifestDatabase::instance().addPlaylist(playlist) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    return send_playlist(req, playlist);
}

// POST /api/playlists/{uuid} - add/remove pattern
static esp_err_t playlists_modify_handler(httpd_req_t* req, const Playlist& playlist) {
    cJSON* json = parse_json_body(req, 4096);
    if (!json) return ESP_FAIL;

    // "pattern_uuid" per swagger; "pattern" kept for older clients
    cJSON* pattern_json = cJSON_GetObjectItem(json, "pattern_uuid");
    if (!pattern_json) pattern_json = cJSON_GetObjectItem(json, "pattern");
    cJSON* action = cJSON_GetObjectItem(json, "action");

    if (!pattern_json || !cJSON_IsString(pattern_json) || !action || !cJSON_IsString(action)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing pattern_uuid or action");
        return ESP_FAIL;
    }

    // Look up pattern by external UUID to get internal ID
    auto pattern_id = ManifestDatabase::instance().findPatternIdByExternalUuid(pattern_json->valuestring);
    if (!pattern_id) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Pattern not found");
        return ESP_FAIL;
    }

    esp_err_t result = ESP_FAIL;
    if (strcmp(action->valuestring, "add") == 0) {
        result = ManifestDatabase::instance().addPatternToPlaylist(playlist.id, *pattern_id);
    } else if (strcmp(action->valuestring, "remove") == 0 || strcmp(action->valuestring, "delete") == 0) {
        result = ManifestDatabase::instance().removePatternFromPlaylist(playlist.id, *pattern_id);
    }

    cJSON_Delete(json);

    if (result != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Re-fetch updated playlist
    auto updated = ManifestDatabase::instance().getPlaylist(playlist.id);
    if (!updated) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    return send_playlist(req, *updated);
}

// POST /api/playlists/{uuid}/order - reorder playlist
static esp_err_t playlists_order_handler(httpd_req_t* req, const Playlist& playlist) {
    cJSON* json = parse_json_body(req, 4096);
    if (!json) return ESP_FAIL;

    // Swagger shape is {"pattern_uuids":[...]}; a bare array is also accepted
    cJSON* array = cJSON_IsObject(json) ? cJSON_GetObjectItem(json, "pattern_uuids") : json;
    if (!array || !cJSON_IsArray(array)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected pattern_uuids array");
        return ESP_FAIL;
    }

    // Convert pattern external UUIDs to internal IDs
    std::vector<uint32_t> newOrder;
    cJSON* item;
    cJSON_ArrayForEach(item, array) {
        if (cJSON_IsString(item)) {
            auto pattern_id = ManifestDatabase::instance().findPatternIdByExternalUuid(item->valuestring);
            if (pattern_id) {
                newOrder.push_back(*pattern_id);
            }
        }
    }
    cJSON_Delete(json);

    if (ManifestDatabase::instance().reorderPlaylist(playlist.id, newOrder) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Re-fetch updated playlist
    auto updated = ManifestDatabase::instance().getPlaylist(playlist.id);
    if (!updated) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    return send_playlist(req, *updated);
}

// POST /api/playlists/{uuid}[/order] - router.
// ESP-IDF wildcards only match at the END of a template, so
// "/api/playlists/*/order" can never be registered/matched as its own URI —
// both modify and reorder share the "/api/playlists/*" template and are
// dispatched here by suffix.
static esp_err_t playlists_post_router(httpd_req_t* req) {
    std::string uuid = extract_uuid(req->uri, "/api/playlists/");
    if (uuid.empty()) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    auto playlist = ManifestDatabase::instance().getPlaylistByExternalUuid(uuid);
    if (!playlist) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    const char* rest = req->uri + strlen("/api/playlists/") + uuid.size();
    if (strncmp(rest, "/order", 6) == 0) {
        return playlists_order_handler(req, *playlist);
    }
    return playlists_modify_handler(req, *playlist);
}

// PATCH /api/playlists/{uuid} - update metadata and/or replace pattern list
// Body (all optional): {"name","description","featured_pattern","pattern_uuids":[...]}
static esp_err_t playlists_patch_handler(httpd_req_t* req) {
    std::string uuid = extract_uuid(req->uri, "/api/playlists/");
    if (uuid.empty()) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    auto existing = ManifestDatabase::instance().getPlaylistByExternalUuid(uuid);
    if (!existing) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    cJSON* json = parse_json_body(req, 4096);
    if (!json) return ESP_FAIL;

    Playlist updated = *existing;

    cJSON* name = cJSON_GetObjectItem(json, "name");
    if (name && cJSON_IsString(name) && strlen(name->valuestring) > 0) {
        updated.name = name->valuestring;
    }
    cJSON* description = cJSON_GetObjectItem(json, "description");
    if (description && cJSON_IsString(description)) {
        updated.description = description->valuestring;
    }
    cJSON* featured = cJSON_GetObjectItem(json, "featured_pattern");
    if (featured && cJSON_IsString(featured) && strlen(featured->valuestring) > 0) {
        auto pattern_id = ManifestDatabase::instance().findPatternIdByExternalUuid(featured->valuestring);
        if (pattern_id) updated.featured_pattern_id = *pattern_id;
    }

    // Replace pattern list if provided
    cJSON* pattern_uuids = cJSON_GetObjectItem(json, "pattern_uuids");
    if (pattern_uuids && cJSON_IsArray(pattern_uuids)) {
        updated.pattern_ids.clear();
        cJSON* item;
        cJSON_ArrayForEach(item, pattern_uuids) {
            if (cJSON_IsString(item)) {
                auto pattern_id = ManifestDatabase::instance().findPatternIdByExternalUuid(item->valuestring);
                if (pattern_id) updated.pattern_ids.push_back(*pattern_id);
            }
        }
    }
    cJSON_Delete(json);

    updated.updated_at = ManifestDatabase::currentTimestamp();

    if (ManifestDatabase::instance().updatePlaylist(existing->id, updated) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    auto refreshed = ManifestDatabase::instance().getPlaylist(existing->id);
    if (!refreshed) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    return send_playlist(req, *refreshed);
}

// DELETE /api/playlists/{uuid}
static esp_err_t playlists_delete_handler(httpd_req_t* req) {
    std::string uuid = extract_uuid(req->uri, "/api/playlists/");
    if (uuid.empty()) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    auto playlist = ManifestDatabase::instance().getPlaylistByExternalUuid(uuid);
    if (!playlist) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    if (ManifestDatabase::instance().deletePlaylist(playlist->id) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

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
