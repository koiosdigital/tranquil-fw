// Motion/LED configuration + preset REST API (NVS-backed via ConfigManager).
#include "config_api.h"
#include "api_common.h"
#include "config_manager.h"
#include "motion_controller.h"
#include "SandTablePlayer.h"

#include <esp_log.h>
#include <strings.h>

static const char* TAG = "config_api";

using namespace api_common;

// LED strip enum <-> string mapping (mirrors the `led_config` console command).
static const char* led_ic_name(uint8_t v) {
    switch (v) { case 0: return "ws2812"; case 1: return "sk6812"; case 2: return "fw1906"; default: return "unknown"; }
}
static const char* led_format_name(uint8_t v) {
    switch (v) { case 3: return "rgb"; case 4: return "rgbw"; case 5: return "rgbcct"; default: return "unknown"; }
}
static const char* led_order_name(uint8_t v) {
    static const char* n[] = { "rgb", "rbg", "grb", "gbr", "brg", "bgr" };
    return v < 6 ? n[v] : "unknown";
}
static int led_ic_code(const char* s) {
    if (!strcasecmp(s, "ws2812")) return 0;
    if (!strcasecmp(s, "sk6812")) return 1;
    if (!strcasecmp(s, "fw1906")) return 2;
    return -1;
}
static int led_format_code(const char* s) {
    if (!strcasecmp(s, "rgb")) return 3;
    if (!strcasecmp(s, "rgbw")) return 4;
    if (!strcasecmp(s, "rgbcct")) return 5;
    return -1;
}
static int led_order_code(const char* s) {
    static const char* n[] = { "rgb", "rbg", "grb", "gbr", "brg", "bgr" };
    for (int i = 0; i < 6; ++i) if (!strcasecmp(s, n[i])) return i;
    return -1;
}

// GET /api/config
static esp_err_t config_get_handler(httpd_req_t* req) {
    auto& cfg = sand_table::ConfigManager::instance();
    const auto& motion = cfg.motion_config();
    const auto& led = cfg.led_config();
    const auto& cal = cfg.calibration();

    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);

    chunk_buf_printf(&cb,
        "{\"motion\":{\"steps_per_rev\":%lu,\"microsteps\":%u,"
        "\"rho_max_rpm\":%ld,",
        (unsigned long)motion.steps_per_rev, (unsigned)motion.microsteps,
        (long)motion.rho_max_rpm);
    chunk_buf_printf(&cb,
        "\"theta_current_ma\":%u,\"rho_current_ma\":%u,\"stallguard_threshold\":%u},",
        (unsigned)motion.theta_current_ma, (unsigned)motion.rho_current_ma,
        (unsigned)motion.stallguard_threshold);
    chunk_buf_printf(&cb,
        "\"led\":{\"has_leds\":%s,\"led_count\":%u,\"is_rgbw\":%s,"
        "\"ic_type\":\"%s\",\"format\":\"%s\",\"color_order\":\"%s\",\"white_swap\":%s},",
        led.has_leds ? "true" : "false", (unsigned)led.led_count,
        led.is_rgbw ? "true" : "false",
        led_ic_name(led.ic_type), led_format_name(led.format),
        led_order_name(led.color_order), led.white_swap ? "true" : "false");
    chunk_buf_printf(&cb,
        "\"calibration\":{\"theta_steps_per_rotation\":%ld,\"rho_max_steps\":%ld,"
        "\"is_valid\":%s,\"timestamp\":%lu},",
        (long)cal.theta_steps_per_rotation, (long)cal.rho_max_steps,
        cal.is_valid ? "true" : "false", (unsigned long)cal.timestamp);
    chunk_buf_write_str(&cb, "\"active_preset_id\":");
    chunk_buf_write_json_string(&cb, cfg.active_preset_id());
    chunk_buf_write(&cb, "}", 1);
    chunk_buf_finish(&cb);
    return ESP_OK;
}

// PATCH /api/config - partial update: {"motion":{...}, "led":{...}}
// Zero/absent fields are left unchanged (mirrors the previous WS semantics).
static esp_err_t config_patch_handler(httpd_req_t* req) {
    cJSON* json = parse_json_body(req, 2048);
    if (!json) return ESP_FAIL;

    auto& cfg = sand_table::ConfigManager::instance();
    bool changed = false;

    cJSON* motion_json = cJSON_GetObjectItem(json, "motion");
    if (motion_json && cJSON_IsObject(motion_json)) {
        sand_table::RuntimeMotionConfig motion = cfg.motion_config();

        auto get_num = [motion_json](const char* key, double def) -> double {
            cJSON* item = cJSON_GetObjectItem(motion_json, key);
            return (item && cJSON_IsNumber(item)) ? item->valuedouble : def;
        };

        double v;
        if ((v = get_num("steps_per_rev", 0)) > 0) motion.steps_per_rev = (uint32_t)v;
        if ((v = get_num("microsteps", 0)) > 0) motion.microsteps = (uint16_t)v;
        if ((v = get_num("rho_max_rpm", 0)) > 0) motion.rho_max_rpm = (int32_t)v;
        if ((v = get_num("theta_current_ma", 0)) > 0) motion.theta_current_ma = (uint16_t)v;
        if ((v = get_num("rho_current_ma", 0)) > 0) motion.rho_current_ma = (uint16_t)v;
        if ((v = get_num("stallguard_threshold", 0)) > 0) motion.stallguard_threshold = (uint8_t)v;

        esp_err_t ret = cfg.set_motion_config(motion);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set motion config: %s", esp_err_to_name(ret));
        } else {
            changed = true;
            // Push run/hold current to the TMC drivers now so the change
            // takes effect without a reboot. (Microstep resolution and the
            // step calibration are boot-time; RPM/StallGuard are read live by
            // motion/homing.)
            if (auto* mc = SandTablePlayer::getMotionController()) {
                mc->apply_motor_config();
            }
        }
    }

    cJSON* led_json = cJSON_GetObjectItem(json, "led");
    if (led_json && cJSON_IsObject(led_json)) {
        sand_table::RuntimeLEDConfig led = cfg.led_config();

        cJSON* has_leds = cJSON_GetObjectItem(led_json, "has_leds");
        if (has_leds && cJSON_IsBool(has_leds)) led.has_leds = cJSON_IsTrue(has_leds);
        cJSON* led_count = cJSON_GetObjectItem(led_json, "led_count");
        if (led_count && cJSON_IsNumber(led_count) && led_count->valueint > 0) {
            led.led_count = (uint16_t)led_count->valueint;
        }
        // Legacy is_rgbw first so an explicit `format` below always wins.
        cJSON* is_rgbw = cJSON_GetObjectItem(led_json, "is_rgbw");
        if (is_rgbw && cJSON_IsBool(is_rgbw)) led.format = cJSON_IsTrue(is_rgbw) ? 4 : 3;

        // Strip type/layout. ic_type/format/color_order accept either the
        // string name (e.g. "fw1906", "rgbcct", "grb") or the raw int code.
        cJSON* ic = cJSON_GetObjectItem(led_json, "ic_type");
        if (ic && cJSON_IsString(ic)) { int v = led_ic_code(ic->valuestring); if (v >= 0) led.ic_type = (uint8_t)v; }
        else if (ic && cJSON_IsNumber(ic) && ic->valueint >= 0 && ic->valueint <= 2) led.ic_type = (uint8_t)ic->valueint;

        cJSON* fmt = cJSON_GetObjectItem(led_json, "format");
        if (fmt && cJSON_IsString(fmt)) { int v = led_format_code(fmt->valuestring); if (v >= 0) led.format = (uint8_t)v; }
        else if (fmt && cJSON_IsNumber(fmt) && fmt->valueint >= 3 && fmt->valueint <= 5) led.format = (uint8_t)fmt->valueint;

        cJSON* order = cJSON_GetObjectItem(led_json, "color_order");
        if (order && cJSON_IsString(order)) { int v = led_order_code(order->valuestring); if (v >= 0) led.color_order = (uint8_t)v; }
        else if (order && cJSON_IsNumber(order) && order->valueint >= 0 && order->valueint <= 5) led.color_order = (uint8_t)order->valueint;

        cJSON* wswap = cJSON_GetObjectItem(led_json, "white_swap");
        if (wswap && cJSON_IsBool(wswap)) led.white_swap = cJSON_IsTrue(wswap);

        led.is_rgbw = (led.format == 4);  // keep legacy flag consistent

        esp_err_t ret = cfg.set_led_config(led);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set LED config: %s", esp_err_to_name(ret));
        } else {
            changed = true;
        }
    }

    cJSON_Delete(json);

    ESP_LOGI(TAG, "Config PATCH: changed=%d", changed);

    if (!changed) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No valid config in body");
        return ESP_FAIL;
    }

    // Return the updated config
    return config_get_handler(req);
}

// DELETE /api/config/calibration
static esp_err_t calibration_delete_handler(httpd_req_t* req) {
    esp_err_t ret = sand_table::ConfigManager::instance().clear_calibration();
    ESP_LOGI(TAG, "ClearCalibration: %s", esp_err_to_name(ret));

    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to clear calibration");
        return ESP_FAIL;
    }
    return send_ok(req);
}

// GET /api/presets
static esp_err_t presets_list_handler(httpd_req_t* req) {
    auto& cfg = sand_table::ConfigManager::instance();
    auto presets = cfg.list_presets();

    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    chunk_buf_write_str(&cb, "{\"presets\":[");
    bool first = true;
    for (const auto& p : presets) {
        if (!first) chunk_buf_write(&cb, ",", 1);
        chunk_buf_write_str(&cb, "{\"id\":");
        chunk_buf_write_json_string(&cb, p.id);
        chunk_buf_write_str(&cb, ",\"name\":");
        chunk_buf_write_json_string(&cb, p.name);
        chunk_buf_write_str(&cb, ",\"description\":");
        chunk_buf_write_json_string(&cb, p.description);
        chunk_buf_write(&cb, "}", 1);
        first = false;
    }
    chunk_buf_write_str(&cb, "],\"active_preset_id\":");
    chunk_buf_write_json_string(&cb, cfg.active_preset_id());
    chunk_buf_write(&cb, "}", 1);
    chunk_buf_finish(&cb);
    return ESP_OK;
}

// POST /api/presets/load - {"preset_id": "..."}
static esp_err_t presets_load_handler(httpd_req_t* req) {
    cJSON* json = parse_json_body(req, 256);
    if (!json) return ESP_FAIL;

    cJSON* preset_id = cJSON_GetObjectItem(json, "preset_id");
    if (!preset_id || !cJSON_IsString(preset_id) || strlen(preset_id->valuestring) == 0) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing preset_id");
        return ESP_FAIL;
    }

    esp_err_t ret = sand_table::ConfigManager::instance().load_preset(preset_id->valuestring);
    ESP_LOGI(TAG, "LoadPreset '%s': %s", preset_id->valuestring, esp_err_to_name(ret));
    cJSON_Delete(json);

    // A preset changes motor current (among other things) — push it to the
    // drivers immediately so it applies without a reboot.
    if (ret == ESP_OK) {
        if (auto* mc = SandTablePlayer::getMotionController()) {
            mc->apply_motor_config();
        }
    }

    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NOT_FOUND) {
            httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Preset not found");
        } else {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to load preset");
        }
        return ESP_FAIL;
    }

    // Return the updated config so the client sees the applied values
    return config_get_handler(req);
}

void config_api_register_handlers(httpd_handle_t server) {
    static httpd_uri_t config_get_uri = {
        .uri = "/api/config",
        .method = HTTP_GET,
        .handler = config_get_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &config_get_uri);

    static httpd_uri_t config_patch_uri = {
        .uri = "/api/config",
        .method = HTTP_PATCH,
        .handler = config_patch_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &config_patch_uri);

    static httpd_uri_t calibration_delete_uri = {
        .uri = "/api/config/calibration",
        .method = HTTP_DELETE,
        .handler = calibration_delete_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &calibration_delete_uri);

    static httpd_uri_t presets_list_uri = {
        .uri = "/api/presets",
        .method = HTTP_GET,
        .handler = presets_list_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &presets_list_uri);

    static httpd_uri_t presets_load_uri = {
        .uri = "/api/presets/load",
        .method = HTTP_POST,
        .handler = presets_load_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &presets_load_uri);
}
