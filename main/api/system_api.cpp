// System REST API: a thin JSON codec over system_ops (device info, homing,
// reboot, factory reset).
#include "system_api.h"
#include "api_common.h"
#include "ops/system_ops.h"

#include <esp_log.h>

static const char* TAG = "system_api";

using namespace api_common;

// GET /api/system/info
static esp_err_t system_info_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    system_ops::info_to_json(&cb);
    chunk_buf_finish(&cb);
    return ESP_OK;
}

// POST /api/system/home - body optional: {"force_full_calibration": bool}
// Starts the homing run on its own task and returns immediately; poll
// GET /api/system/info (is_homing / is_homed) or watch the SystemInfo push.
static esp_err_t system_home_handler(httpd_req_t* req) {
    bool force_full = false;
    if (req->content_len > 0) {
        cJSON* json = parse_json_body(req, 256);
        if (!json) return ESP_FAIL;
        cJSON* force = cJSON_GetObjectItem(json, "force_full_calibration");
        force_full = force && cJSON_IsTrue(force);
        cJSON_Delete(json);
    }

    OpResult r = system_ops::home(force_full);
    httpd_resp_set_type(req, "application/json");
    if (r.err == ESP_ERR_INVALID_STATE) {
        // Documented as 409 (a run is already in progress).
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "{\"success\":false,\"error\":\"Homing already in progress\"}");
        return ESP_OK;
    }
    if (!r.ok()) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"success\":false,\"error\":\"Failed to start homing\"}");
        return ESP_OK;
    }

    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_sendstr(req, "{\"success\":true,\"started\":true}");
    return ESP_OK;
}

// POST /api/system/reboot - restart (settings and library are kept).
// Refused for browser origins (see api_common::reject_browser_origin).
static esp_err_t system_reboot_handler(httpd_req_t* req) {
    if (reject_browser_origin(req)) return ESP_FAIL;
    ESP_LOGW(TAG, "Reboot requested via REST");
    OpResult r = system_ops::reboot("REST");
    if (!r.ok()) return send_op_error(req, r);
    return send_ok(req);
}

// POST /api/system/factory-reset. LAN-only and refused for browser origins.
static esp_err_t system_factory_reset_handler(httpd_req_t* req) {
    if (reject_browser_origin(req)) return ESP_FAIL;
    ESP_LOGW(TAG, "Factory reset requested via REST");
    OpResult r = system_ops::factory_reset("REST");
    if (!r.ok()) return send_op_error(req, r);
    return send_ok(req);
}

void system_api_register_handlers(httpd_handle_t server) {
    static httpd_uri_t info_uri = {
        .uri = "/api/system/info",
        .method = HTTP_GET,
        .handler = system_info_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &info_uri);

    static httpd_uri_t home_uri = {
        .uri = "/api/system/home",
        .method = HTTP_POST,
        .handler = system_home_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &home_uri);

    static httpd_uri_t reboot_uri = {
        .uri = "/api/system/reboot",
        .method = HTTP_POST,
        .handler = system_reboot_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &reboot_uri);

    static httpd_uri_t factory_reset_uri = {
        .uri = "/api/system/factory-reset",
        .method = HTTP_POST,
        .handler = system_factory_reset_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &factory_reset_uri);
}
