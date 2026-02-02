#include "esp_http_server.h"
#include "cJSON.h"
#include "ManifestDatabase.h"
#include <string.h>
#include <stdlib.h>
#include <vector>

namespace {

// Helper: create pagination JSON object
cJSON* createPaginationJson(const PaginationInfo& info) {
    cJSON* pagination = cJSON_CreateObject();
    cJSON_AddNumberToObject(pagination, "page", info.page);
    cJSON_AddNumberToObject(pagination, "per_page", info.per_page);
    cJSON_AddNumberToObject(pagination, "total_pages", info.total_pages);
    cJSON_AddNumberToObject(pagination, "total_items", info.total_items);
    return pagination;
}

// Helper: parse pagination query params
void parsePaginationParams(httpd_req_t* req, int& page, int& per_page) {
    page = 0;
    per_page = 20;

    char query[128] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char param[16] = {0};
        if (httpd_query_key_value(query, "page", param, sizeof(param)) == ESP_OK) {
            page = atoi(param);
            if (page < 0) page = 0;
        }
        if (httpd_query_key_value(query, "per_page", param, sizeof(param)) == ESP_OK) {
            per_page = atoi(param);
            if (per_page < 1) per_page = 1;
            if (per_page > 100) per_page = 100;
        }
    }
}

// Helper: extract UUID from URI like /api/playlists/{uuid} or /api/playlists/{uuid}/order
std::string extractUuid(const char* uri, const char* base) {
    if (strncmp(uri, base, strlen(base)) != 0) {
        return "";
    }
    const char* uuid_start = uri + strlen(base);
    const char* slash = strchr(uuid_start, '/');
    size_t uuid_len = slash ? static_cast<size_t>(slash - uuid_start) : strlen(uuid_start);
    if (uuid_len == 0 || uuid_len > 64) {
        return "";
    }
    return std::string(uuid_start, uuid_len);
}

} // anonymous namespace

// GET /api/playlists - paginated list
static esp_err_t playlists_list_handler(httpd_req_t* req) {
    int page, per_page;
    parsePaginationParams(req, page, per_page);

    auto result = ManifestDatabase::instance().getPlaylists(page, per_page);

    cJSON* response = cJSON_CreateObject();
    cJSON_AddItemToObject(response, "pagination", createPaginationJson(result.pagination));

    cJSON* playlistsArray = cJSON_CreateArray();
    for (const auto& playlist : result.items) {
        cJSON_AddItemToArray(playlistsArray, ManifestDatabase::playlistToJson(playlist));
    }
    cJSON_AddItemToObject(response, "playlists", playlistsArray);

    char* json_str = cJSON_PrintUnformatted(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(response);
    return ESP_OK;
}

// GET /api/playlists/{uuid}
static esp_err_t playlists_detail_handler(httpd_req_t* req) {
    std::string uuid = extractUuid(req->uri, "/api/playlists/");
    if (uuid.empty()) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    auto playlist = ManifestDatabase::instance().getPlaylist(uuid);
    if (!playlist) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    cJSON* json = ManifestDatabase::playlistToJson(*playlist);
    char* json_str = cJSON_PrintUnformatted(json);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

// POST /api/playlists - create playlist
static esp_err_t playlists_create_handler(httpd_req_t* req) {
    // Use stack buffer for JSON (cold path, small payloads)
    static constexpr size_t MAX_PLAYLIST_JSON = 4096;
    int len = req->content_len;
    if (len <= 0 || static_cast<size_t>(len) >= MAX_PLAYLIST_JSON) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid content length");
        return ESP_FAIL;
    }

    char buf[MAX_PLAYLIST_JSON];
    int received = httpd_req_recv(req, buf, len);
    if (received <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON* json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    Playlist playlist = ManifestDatabase::jsonToPlaylist(json);
    cJSON_Delete(json);

    // Generate UUID if not provided
    if (playlist.uuid.empty()) {
        playlist.uuid = ManifestDatabase::generateUUID();
    }
    if (playlist.created_at.empty()) {
        playlist.created_at = ManifestDatabase::currentTimestamp();
    }
    playlist.updated_at = playlist.created_at;

    if (ManifestDatabase::instance().addPlaylist(playlist) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON* response = ManifestDatabase::playlistToJson(playlist);
    char* response_str = cJSON_PrintUnformatted(response);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));

    free(response_str);
    cJSON_Delete(response);
    return ESP_OK;
}

// POST /api/playlists/{uuid} - add/remove pattern
static esp_err_t playlists_modify_handler(httpd_req_t* req) {
    std::string uuid = extractUuid(req->uri, "/api/playlists/");
    if (uuid.empty()) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    if (!ManifestDatabase::instance().playlistExists(uuid)) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Use stack buffer for JSON (cold path, small payloads)
    static constexpr size_t MAX_PLAYLIST_JSON = 4096;
    int len = req->content_len;
    if (len <= 0 || static_cast<size_t>(len) >= MAX_PLAYLIST_JSON) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request size");
        return ESP_FAIL;
    }

    char buf[MAX_PLAYLIST_JSON];
    int received = httpd_req_recv(req, buf, len);
    if (received <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON* json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON* pattern = cJSON_GetObjectItem(json, "pattern");
    cJSON* action = cJSON_GetObjectItem(json, "action");

    if (!pattern || !cJSON_IsString(pattern) || !action || !cJSON_IsString(action)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing pattern or action");
        return ESP_FAIL;
    }

    esp_err_t result = ESP_FAIL;
    if (strcmp(action->valuestring, "add") == 0) {
        result = ManifestDatabase::instance().addPatternToPlaylist(uuid, pattern->valuestring);
    } else if (strcmp(action->valuestring, "delete") == 0) {
        result = ManifestDatabase::instance().removePatternFromPlaylist(uuid, pattern->valuestring);
    }

    cJSON_Delete(json);

    if (result != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    auto updated = ManifestDatabase::instance().getPlaylist(uuid);
    if (!updated) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON* response = ManifestDatabase::playlistToJson(*updated);
    char* response_str = cJSON_PrintUnformatted(response);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));

    free(response_str);
    cJSON_Delete(response);
    return ESP_OK;
}

// POST /api/playlists/{uuid}/order - reorder playlist
static esp_err_t playlists_order_handler(httpd_req_t* req) {
    std::string uuid = extractUuid(req->uri, "/api/playlists/");
    if (uuid.empty()) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    if (!ManifestDatabase::instance().playlistExists(uuid)) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Use stack buffer for JSON (cold path, small payloads)
    static constexpr size_t MAX_PLAYLIST_JSON = 4096;
    int len = req->content_len;
    if (len <= 0 || static_cast<size_t>(len) >= MAX_PLAYLIST_JSON) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request size");
        return ESP_FAIL;
    }

    char buf[MAX_PLAYLIST_JSON];
    int received = httpd_req_recv(req, buf, len);
    if (received <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON* json = cJSON_Parse(buf);

    if (!json || !cJSON_IsArray(json)) {
        if (json) cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected array of UUIDs");
        return ESP_FAIL;
    }

    std::vector<std::string> newOrder;
    cJSON* item;
    cJSON_ArrayForEach(item, json) {
        if (cJSON_IsString(item)) {
            newOrder.push_back(item->valuestring);
        }
    }
    cJSON_Delete(json);

    if (ManifestDatabase::instance().reorderPlaylist(uuid, newOrder) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    auto updated = ManifestDatabase::instance().getPlaylist(uuid);
    if (!updated) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON* response = ManifestDatabase::playlistToJson(*updated);
    char* response_str = cJSON_PrintUnformatted(response);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));

    free(response_str);
    cJSON_Delete(response);
    return ESP_OK;
}

// DELETE /api/playlists/{uuid}
static esp_err_t playlists_delete_handler(httpd_req_t* req) {
    std::string uuid = extractUuid(req->uri, "/api/playlists/");
    if (uuid.empty()) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    if (!ManifestDatabase::instance().playlistExists(uuid)) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    if (ManifestDatabase::instance().deletePlaylist(uuid) != ESP_OK) {
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
    httpd_register_uri_handler(server, &playlists_list_uri);

    static httpd_uri_t playlists_create_uri = {
        .uri = "/api/playlists",
        .method = HTTP_POST,
        .handler = playlists_create_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &playlists_create_uri);

    static httpd_uri_t playlists_detail_uri = {
        .uri = "/api/playlists/*",
        .method = HTTP_GET,
        .handler = playlists_detail_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &playlists_detail_uri);

    static httpd_uri_t playlists_modify_uri = {
        .uri = "/api/playlists/*",
        .method = HTTP_POST,
        .handler = playlists_modify_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &playlists_modify_uri);

    static httpd_uri_t playlists_order_uri = {
        .uri = "/api/playlists/*/order",
        .method = HTTP_POST,
        .handler = playlists_order_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &playlists_order_uri);

    static httpd_uri_t playlists_delete_uri = {
        .uri = "/api/playlists/*",
        .method = HTTP_DELETE,
        .handler = playlists_delete_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &playlists_delete_uri);
}
