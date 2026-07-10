// System REST API: device info, homing, factory reset.
#include "system_api.h"
#include "api_common.h"
#include "SandTablePlayer.h"
#include "factory_reset.h"

#include <esp_log.h>
#include <esp_app_desc.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <kd_common.h>

static const char* TAG = "system_api";

using namespace api_common;

// GET /api/system/info
static esp_err_t system_info_handler(httpd_req_t* req) {
    const esp_app_desc_t* app_desc = esp_app_get_description();
    const char* device_id = kd_common_get_device_name();
    const char* hostname = kd_common_get_wifi_hostname();

    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    chunk_buf_write_str(&cb, "{\"firmware_version\":");
    chunk_buf_write_json_string(&cb, app_desc->version);
    chunk_buf_write_str(&cb, ",\"hardware_model\":\"tranquil\",\"device_id\":");
    chunk_buf_write_json_string(&cb, device_id ? device_id : "");
    chunk_buf_write_str(&cb, ",\"hostname\":");
    chunk_buf_write_json_string(&cb, hostname ? hostname : "");
    chunk_buf_printf(&cb, ",\"is_homed\":%s,\"free_heap\":%lu}",
        SandTablePlayer::isHomed() ? "true" : "false",
        (unsigned long)esp_get_free_heap_size());
    chunk_buf_finish(&cb);
    return ESP_OK;
}

static const char* motion_error_to_string(sand_table::MotionError err) {
    switch (err) {
        case sand_table::MotionError::None: return "None";
        case sand_table::MotionError::NotHomed: return "Not homed";
        case sand_table::MotionError::OutOfBounds: return "Out of bounds";
        case sand_table::MotionError::QueueFull: return "Queue full";
        case sand_table::MotionError::QueueEmpty: return "Queue empty";
        case sand_table::MotionError::InvalidState: return "Invalid state";
        case sand_table::MotionError::HardwareFault: return "Hardware fault";
        case sand_table::MotionError::StallDetected: return "Stall detected";
        case sand_table::MotionError::Timeout: return "Timeout";
        case sand_table::MotionError::HomingFailed: return "Homing failed";
        case sand_table::MotionError::EmergencyStop: return "Emergency stop";
        default: return "Unknown error";
    }
}

// POST /api/system/home - body optional: {"force_full_calibration": bool}
// Runs the homing sequence synchronously (same semantics as the WS handler);
// progress/state updates reach clients via the player state broadcast.
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

    auto* motion_controller = SandTablePlayer::getMotionController();
    if (!motion_controller) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Motion controller not initialized");
        return ESP_FAIL;
    }

    auto result = motion_controller->home(force_full);
    if (result.is_err()) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_status(req, "500 Internal Server Error");
        ChunkBuf cb;
        chunk_buf_init(&cb, req);
        chunk_buf_write_str(&cb, "{\"success\":false,\"error\":");
        chunk_buf_write_json_string(&cb, motion_error_to_string(result.error()));
        chunk_buf_write_str(&cb, "}");
        chunk_buf_finish(&cb);
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true}");
    return ESP_OK;
}

static void restart_timer_callback(void* arg) {
    (void)arg;
    ESP_LOGW(TAG, "Restarting device after factory reset...");
    esp_restart();
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

    // Schedule restart after a short delay so the response gets out first
    esp_timer_handle_t restart_timer;
    esp_timer_create_args_t timer_args = {
        .callback = restart_timer_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "factory_reset_restart",
    };
    if (esp_timer_create(&timer_args, &restart_timer) == ESP_OK) {
        esp_timer_start_once(restart_timer, 300 * 1000);  // 300ms
        ESP_LOGI(TAG, "Device will restart in 300ms");
    } else {
        ESP_LOGW(TAG, "Failed to create restart timer, restarting immediately");
        esp_restart();
    }

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

    static httpd_uri_t factory_reset_uri = {
        .uri = "/api/system/factory-reset",
        .method = HTTP_POST,
        .handler = system_factory_reset_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &factory_reset_uri);
}
