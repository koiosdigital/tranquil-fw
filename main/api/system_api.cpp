// System REST API: device info, homing, reboot, factory reset.
#include "system_api.h"
#include "api_common.h"
#include "system_info.h"
#include "homing_service.h"
#include "SandTablePlayer.h"
#include "factory_reset.h"

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>

static const char* TAG = "system_api";

using namespace api_common;

// GET /api/system/info
static esp_err_t system_info_handler(httpd_req_t* req) {
    system_info::Snapshot s;
    system_info::snapshot(s);

    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    chunk_buf_write_str(&cb, "{\"firmware_version\":");
    chunk_buf_write_json_string(&cb, s.firmware_version);
    chunk_buf_write_str(&cb, ",\"hardware_model\":");
    chunk_buf_write_json_string(&cb, s.hardware_model);
    chunk_buf_write_str(&cb, ",\"device_id\":");
    chunk_buf_write_json_string(&cb, s.device_id);
    chunk_buf_write_str(&cb, ",\"hostname\":");
    chunk_buf_write_json_string(&cb, s.hostname);
    chunk_buf_write_str(&cb, ",\"ip_address\":");
    chunk_buf_write_json_string(&cb, s.ip_address);
    chunk_buf_printf(&cb, ",\"is_homed\":%s,\"is_homing\":%s,\"free_heap\":%lu,\"free_psram\":%lu,",
        s.is_homed ? "true" : "false", s.is_homing ? "true" : "false",
        (unsigned long)s.free_heap, (unsigned long)s.free_psram);
    chunk_buf_printf(&cb, "\"uptime_s\":%lu,\"wifi_rssi\":%ld,\"quiet_hours_active\":%s,\"timezone\":",
        (unsigned long)s.uptime_s, (long)s.wifi_rssi, s.quiet_hours_active ? "true" : "false");
    chunk_buf_write_json_string(&cb, s.timezone);
    chunk_buf_write(&cb, "}", 1);
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

    ESP_LOGI(TAG, "Home request (force_full=%d)", force_full);

    esp_err_t err = homing_service::start(force_full);
    httpd_resp_set_type(req, "application/json");
    if (err == ESP_ERR_INVALID_STATE) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "{\"success\":false,\"error\":\"Homing already in progress\"}");
        return ESP_OK;
    }
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"success\":false,\"error\":\"Failed to start homing\"}");
        return ESP_OK;
    }

    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_sendstr(req, "{\"success\":true,\"started\":true}");
    return ESP_OK;
}

static void restart_timer_callback(void* arg) {
    (void)arg;
    ESP_LOGW(TAG, "Restarting device...");
    esp_restart();
}

static void schedule_restart(uint32_t delay_ms) {
    esp_timer_handle_t restart_timer;
    esp_timer_create_args_t timer_args = {
        .callback = restart_timer_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "restart",
    };
    if (esp_timer_create(&timer_args, &restart_timer) == ESP_OK) {
        esp_timer_start_once(restart_timer, static_cast<uint64_t>(delay_ms) * 1000);
        ESP_LOGI(TAG, "Device will restart in %lums", static_cast<unsigned long>(delay_ms));
    }
    else {
        ESP_LOGW(TAG, "Failed to create restart timer, restarting immediately");
        esp_restart();
    }
}

// POST /api/system/reboot - restart (settings and library are kept)
static esp_err_t system_reboot_handler(httpd_req_t* req) {
    ESP_LOGW(TAG, "Reboot requested via REST");
    SandTablePlayer::stop();
    esp_err_t send_err = send_ok(req);
    schedule_restart(500);
    return send_err;
}

// POST /api/system/factory-reset
static esp_err_t system_factory_reset_handler(httpd_req_t* req) {
    ESP_LOGW(TAG, "Factory reset requested via REST");

    esp_err_t err = tranquil_factory_reset();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Factory reset failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Factory reset failed");
        return ESP_FAIL;
    }

    esp_err_t send_err = send_ok(req);
    schedule_restart(300);
    return send_err;
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
