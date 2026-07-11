#include "license_api.h"
#include "drm/drm_license.h"
#include "api_common.h"
#include "cJSON.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "license_api";

// Print `response` (may be NULL under OOM), send it with the given status,
// and free everything. cJSON_PrintUnformatted can also fail — never hand
// its result to httpd_resp_sendstr unchecked (strlen(NULL) crashes).
static esp_err_t send_json_response(httpd_req_t* req, cJSON* response, const char* status) {
    char* json_str = response ? cJSON_PrintUnformatted(response) : nullptr;
    cJSON_Delete(response);
    if (!json_str) {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, status);
    httpd_resp_sendstr(req, json_str);
    free(json_str);
    return ESP_OK;
}

// GET /api/license/store-token
static esp_err_t handle_get_store_token(httpd_req_t* req) {
    cJSON* response = cJSON_CreateObject();

    if (!drm_license_is_valid()) {
        cJSON_AddBoolToObject(response, "success", false);
        cJSON_AddStringToObject(response, "error", "No valid license");
        return send_json_response(req, response, "403 Forbidden");
    }

    char token[DRM_STORE_TOKEN_MAX_SIZE + 1];
    size_t token_len = 0;

    esp_err_t ret = drm_license_get_store_token(token, &token_len);

    if (ret != ESP_OK || token_len == 0) {
        cJSON_AddBoolToObject(response, "success", false);
        cJSON_AddStringToObject(response, "error", "No store token in license");
        return send_json_response(req, response, "404 Not Found");
    }

    cJSON_AddBoolToObject(response, "success", true);
    cJSON_AddStringToObject(response, "store_token", token);

    ret = send_json_response(req, response, "200 OK");
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Store token retrieved via REST API (%zu bytes)", token_len);
    }
    return ret;
}

void license_api_register_handlers(httpd_handle_t server) {
    static httpd_uri_t store_token_uri = {
        .uri = "/api/license/store-token",
        .method = HTTP_GET,
        .handler = handle_get_store_token,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &store_token_uri);

    ESP_LOGI(TAG, "License API handlers registered");
}
