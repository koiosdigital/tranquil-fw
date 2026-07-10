// Schedule REST API. Local clients manage the schedule over HTTP; the
// protobuf Schedule message stays on the sockets for cloud sync and for
// notifying connected WS clients of changes.
#include "schedule_api.h"
#include "api_common.h"
#include "schedule_manager.h"
#include "response_router.h"

#include <esp_log.h>
#include <vector>

static const char* TAG = "schedule_api";

using namespace api_common;

static void write_schedule_json(ChunkBuf* cb) {
    const auto& items = ScheduleManager::instance().getSchedule();
    chunk_buf_write_str(cb, "{\"items\":[");
    bool first = true;
    for (const auto& item : items) {
        if (!first) chunk_buf_write(cb, ",", 1);
        chunk_buf_printf(cb, "{\"days_of_week\":%lu,\"time_of_day\":%lu,\"action_type\":%d,\"uuid\":",
            (unsigned long)item.days_of_week,
            (unsigned long)item.time_of_day,
            (int)item.action_type);
        chunk_buf_write_json_string(cb, item.uuid.c_str());
        chunk_buf_write(cb, "}", 1);
        first = false;
    }
    chunk_buf_write_str(cb, "]}");
}

static esp_err_t send_schedule(httpd_req_t* req) {
    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    write_schedule_json(&cb);
    chunk_buf_finish(&cb);
    return ESP_OK;
}

// GET /api/schedule
static esp_err_t schedule_get_handler(httpd_req_t* req) {
    return send_schedule(req);
}

// Notify other consumers of a schedule change: broadcast the protobuf
// Schedule to local WS clients and push it to the cloud (mirrors what the
// WS handler did for a local SetSchedule).
static void notify_schedule_changed() {
    Kd__V1__TranquilSchedule schedule_proto;
    std::vector<Kd__V1__TranquilScheduleItem> items_storage;
    std::vector<Kd__V1__TranquilScheduleItem*> item_ptrs;
    std::vector<Kd__V1__ScheduleAction> actions_storage;

    ScheduleManager::instance().toProto(&schedule_proto, items_storage, item_ptrs, actions_storage);

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE;
    msg.schedule = &schedule_proto;

    size_t len = kd__v1__tranquil_message__get_packed_size(&msg);
    uint8_t* buf = static_cast<uint8_t*>(malloc(len));
    if (!buf) return;
    kd__v1__tranquil_message__pack(&msg, buf);

    ResponseMessage packed(buf, len);
    ResponseRouter::instance().broadcastToLocal(packed);
    ResponseRouter::instance().sendToCloud(packed);
}

// PUT /api/schedule - replace entire schedule
// Body: {"items":[{"days_of_week":127,"time_of_day":3600,"action_type":5,"uuid":"..."}]}
static esp_err_t schedule_put_handler(httpd_req_t* req) {
    cJSON* json = parse_json_body(req, 8192);
    if (!json) return ESP_FAIL;

    cJSON* items_json = cJSON_GetObjectItem(json, "items");
    if (!items_json || !cJSON_IsArray(items_json)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing items array");
        return ESP_FAIL;
    }

    std::vector<ScheduleItem> items;
    cJSON* item_json;
    cJSON_ArrayForEach(item_json, items_json) {
        if (!cJSON_IsObject(item_json)) continue;

        ScheduleItem item = {};
        cJSON* days = cJSON_GetObjectItem(item_json, "days_of_week");
        cJSON* time = cJSON_GetObjectItem(item_json, "time_of_day");
        cJSON* action = cJSON_GetObjectItem(item_json, "action_type");
        cJSON* uuid = cJSON_GetObjectItem(item_json, "uuid");

        if (!days || !cJSON_IsNumber(days) || !time || !cJSON_IsNumber(time) ||
            !action || !cJSON_IsNumber(action)) {
            cJSON_Delete(json);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                "Items need days_of_week, time_of_day, action_type");
            return ESP_FAIL;
        }

        item.days_of_week = (uint32_t)days->valueint & 0x7F;
        item.time_of_day = (uint32_t)time->valueint % 86400;
        item.action_type = (Kd__V1__ScheduleAction__ScheduleActionType)action->valueint;
        if (uuid && cJSON_IsString(uuid)) {
            item.uuid = uuid->valuestring;
        }
        items.push_back(item);
    }
    cJSON_Delete(json);

    esp_err_t ret = ScheduleManager::instance().setSchedule(items);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set schedule: %s", esp_err_to_name(ret));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save schedule");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Schedule updated via REST: %zu items", items.size());
    notify_schedule_changed();

    return send_schedule(req);
}

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
}
