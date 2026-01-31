#include "patterns_api.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "ManifestDatabase.h"
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <vector>
#include <unistd.h>

namespace {

// Helper: get pattern file size on disk
size_t getPatternFileSize(const std::string& uuid) {
    char path[128];
    snprintf(path, sizeof(path), "/sd/patterns/%s.thr", uuid.c_str());
    struct stat st;
    if (stat(path, &st) == 0) {
        return st.st_size;
    }
    return 0;
}

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

} // anonymous namespace

// GET /api/patterns - paginated list
static esp_err_t patterns_list_handler(httpd_req_t* req) {
    int page, per_page;
    parsePaginationParams(req, page, per_page);

    auto result = ManifestDatabase::instance().getPatterns(page, per_page);

    cJSON* response = cJSON_CreateObject();
    cJSON_AddItemToObject(response, "pagination", createPaginationJson(result.pagination));

    cJSON* patternsArray = cJSON_CreateArray();
    for (auto& pattern : result.items) {
        // Update size from disk if not set
        if (pattern.size_bytes == 0) {
            pattern.size_bytes = getPatternFileSize(pattern.uuid);
        }
        cJSON_AddItemToArray(patternsArray, ManifestDatabase::patternToJson(pattern));
    }
    cJSON_AddItemToObject(response, "patterns", patternsArray);

    char* json_str = cJSON_PrintUnformatted(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(response);
    return ESP_OK;
}

// GET /api/patterns/{uuid} - detail
static esp_err_t patterns_detail_handler(httpd_req_t* req) {
    const char* uri = req->uri;
    const char* base = "/api/patterns/";

    if (strncmp(uri, base, strlen(base)) != 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    const char* uuid = uri + strlen(base);
    if (!uuid || strlen(uuid) == 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    auto pattern = ManifestDatabase::instance().getPattern(uuid);
    if (!pattern) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Update size from disk if not set
    if (pattern->size_bytes == 0) {
        pattern->size_bytes = getPatternFileSize(pattern->uuid);
    }

    cJSON* json = ManifestDatabase::patternToJson(*pattern);
    char* json_str = cJSON_PrintUnformatted(json);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

// POST /api/patterns - create pattern (upload)
static esp_err_t patterns_create_handler(httpd_req_t* req) {
    int len = req->content_len;
    if (len <= 0 || len > 1024 * 1024) {  // Max 1MB for metadata
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid content length");
        return ESP_FAIL;
    }

    char* buf = static_cast<char*>(malloc(len + 1));
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int received = httpd_req_recv(req, buf, len);
    if (received <= 0) {
        free(buf);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON* json = cJSON_Parse(buf);
    free(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    Pattern pattern = ManifestDatabase::jsonToPattern(json);
    cJSON_Delete(json);

    // Generate UUID if not provided
    if (pattern.uuid.empty()) {
        pattern.uuid = ManifestDatabase::generateUUID();
    }

    // Apply defaults for uploaded patterns
    if (pattern.creator.empty()) {
        pattern.creator = "Uploaded";
    }
    if (pattern.created_at.empty()) {
        pattern.created_at = ManifestDatabase::currentTimestamp();
    }

    // Get file size if pattern file exists
    pattern.size_bytes = getPatternFileSize(pattern.uuid);

    if (ManifestDatabase::instance().addPattern(pattern) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON* response = ManifestDatabase::patternToJson(pattern);
    char* response_str = cJSON_PrintUnformatted(response);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));

    free(response_str);
    cJSON_Delete(response);
    return ESP_OK;
}

// DELETE /api/patterns/{uuid}
static esp_err_t patterns_delete_handler(httpd_req_t* req) {
    const char* uri = req->uri;
    const char* base = "/api/patterns/";

    if (strncmp(uri, base, strlen(base)) != 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    const char* uuid = uri + strlen(base);
    if (!uuid || strlen(uuid) == 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    if (!ManifestDatabase::instance().patternExists(uuid)) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    if (ManifestDatabase::instance().deletePattern(uuid) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"deleted\"}", -1);
    return ESP_OK;
}

void patterns_api_register_handlers(httpd_handle_t server) {
    static httpd_uri_t patterns_list_uri = {
        .uri = "/api/patterns",
        .method = HTTP_GET,
        .handler = patterns_list_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &patterns_list_uri);

    static httpd_uri_t patterns_create_uri = {
        .uri = "/api/patterns",
        .method = HTTP_POST,
        .handler = patterns_create_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &patterns_create_uri);

    static httpd_uri_t patterns_detail_uri = {
        .uri = "/api/patterns/*",
        .method = HTTP_GET,
        .handler = patterns_detail_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &patterns_detail_uri);

    static httpd_uri_t patterns_delete_uri = {
        .uri = "/api/patterns/*",
        .method = HTTP_DELETE,
        .handler = patterns_delete_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &patterns_delete_uri);
}
