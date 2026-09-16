// Schedule REST API: a thin JSON codec over schedule_ops. Local clients
// manage the schedule over HTTP; the protobuf Schedule message stays on the
// sockets for cloud sync and for notifying connected WS clients of changes
// (the library notifier pushes it after every set, whichever transport
// wrote it).
#include "schedule_api.h"
#include "api_common.h"
#include "ops/schedule_ops.h"

#include <esp_log.h>
#include <optional>
#include <vector>

static const char* TAG = "schedule_api";

using namespace api_common;

namespace {

esp_err_t send_schedule(httpd_req_t* req) {
    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    schedule_ops::to_json(&cb);
    chunk_buf_finish(&cb);
    return ESP_OK;
}

// GET /api/schedule
esp_err_t schedule_get_handler(httpd_req_t* req) {
    return send_schedule(req);
}

// PUT /api/schedule - replace the item list; quiet_hours replaced only when present
esp_err_t schedule_put_handler(httpd_req_t* req) {
    cJSON* json = parse_json_body(req, 16384);
    if (!json) return ESP_FAIL;

    const char* err = nullptr;
    std::vector<ScheduleItem> items;
    std::optional<QuietHours> quiet;
    const bool parsed = schedule_ops::schedule_from_json(json, items, quiet, &err);
    cJSON_Delete(json);
    if (!parsed) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, err ? err : "Malformed schedule");
        return ESP_FAIL;
    }

    OpResult r = schedule_ops::set_schedule(std::move(items), quiet);
    if (!r.ok()) return send_op_error(req, r);

    ESP_LOGI(TAG, "Schedule updated via REST");
    return send_schedule(req);
}

// POST /api/schedule/run - body {"action":{...},"force":bool}
// Runs the action now. 200 {"success":bool,"detail":string}; success is
// false with a detail when quiet hours block it or the action cannot start.
esp_err_t schedule_run_handler(httpd_req_t* req) {
    cJSON* json = parse_json_body(req, 2048);
    if (!json) return ESP_FAIL;

    cJSON* action_json = cJSON_GetObjectItem(json, "action");
    if (!action_json || !cJSON_IsObject(action_json)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing action");
        return ESP_FAIL;
    }
    const char* err = nullptr;
    ScheduleAction action;
    if (!schedule_ops::action_from_json(action_json, action, &err)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, err ? err : "Malformed action");
        return ESP_FAIL;
    }
    const cJSON* force_item = cJSON_GetObjectItem(json, "force");
    const bool force = force_item && cJSON_IsBool(force_item) && cJSON_IsTrue(force_item);
    cJSON_Delete(json);

    OpResult r = schedule_ops::run_action(action, force);

    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    chunk_buf_printf(&cb, "{\"success\":%s,\"detail\":", r.ok() ? "true" : "false");
    chunk_buf_write_json_string(&cb, r.ok() ? "" : r.detail);
    chunk_buf_write(&cb, "}", 1);
    chunk_buf_finish(&cb);
    return ESP_OK;
}

}  // namespace

void schedule_api_register_handlers(httpd_handle_t server) {
    static httpd_uri_t schedule_get_uri = {
        .uri = "/api/schedule",
        .method = HTTP_GET,
        .handler = schedule_get_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &schedule_get_uri);

    static httpd_uri_t schedule_put_uri = {
        .uri = "/api/schedule",
        .method = HTTP_PUT,
        .handler = schedule_put_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &schedule_put_uri);

    static httpd_uri_t schedule_run_uri = {
        .uri = "/api/schedule/run",
        .method = HTTP_POST,
        .handler = schedule_run_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &schedule_run_uri);
}
