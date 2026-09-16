// Motion/LED configuration + preset REST API (NVS-backed via ConfigManager).
//
// Every write applies live where the hardware allows it: motor currents and
// StallGuard go to the TMC drivers, motion shaping is read per segment, the
// LED strip is rebuilt by led_runtime. Only microsteps / steps_per_rev need a
// reboot + re-home (the calibration is expressed in those steps).
#include "config_api.h"
#include "api_common.h"
#include "config_manager.h"
#include "motion_controller.h"
#include "SandTablePlayer.h"
#include "led/led_runtime.h"

#include <esp_log.h>
#include <strings.h>
#include <cmath>
#include <cstdio>

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

static void write_config_json(ChunkBuf* cb, bool led_applied, bool reboot_required) {
    auto& cfg = sand_table::ConfigManager::instance();
    const auto& motion = cfg.motion_config();
    const auto& led = cfg.led_config();
    const auto& cal = cfg.calibration();

    chunk_buf_printf(cb,
        "{\"motion\":{\"steps_per_rev\":%lu,\"microsteps\":%u,"
        "\"rho_max_rpm\":%ld,",
        (unsigned long)motion.steps_per_rev, (unsigned)motion.microsteps,
        (long)motion.rho_max_rpm);
    chunk_buf_printf(cb,
        "\"theta_current_ma\":%u,\"rho_current_ma\":%u,\"stallguard_threshold\":%u,",
        (unsigned)motion.theta_current_ma, (unsigned)motion.rho_current_ma,
        (unsigned)motion.stallguard_threshold);
    chunk_buf_printf(cb,
        "\"accel_steps_s2\":%lu,\"junction_dv_steps_s\":%lu,",
        (unsigned long)motion.accel_steps_s2, (unsigned long)motion.junction_dv_steps_s);
    chunk_buf_printf(cb,
        "\"theta_max_rot_per_min\":%.2f,\"motor_idle_timeout_s\":%lu},",
        (double)motion.theta_max_rot_per_min, (unsigned long)motion.motor_idle_timeout_s);
    chunk_buf_printf(cb,
        "\"led\":{\"has_leds\":%s,\"led_count\":%u,\"is_rgbw\":%s,"
        "\"ic_type\":\"%s\",\"format\":\"%s\",\"color_order\":\"%s\",\"white_swap\":%s},",
        led.has_leds ? "true" : "false", (unsigned)led.led_count,
        led.is_rgbw ? "true" : "false",
        led_ic_name(led.ic_type), led_format_name(led.format),
        led_order_name(led.color_order), led.white_swap ? "true" : "false");
    chunk_buf_printf(cb,
        "\"calibration\":{\"theta_steps_per_rotation\":%ld,\"rho_max_steps\":%ld,"
        "\"is_valid\":%s,\"timestamp\":%lu},",
        (long)cal.theta_steps_per_rotation, (long)cal.rho_max_steps,
        cal.is_valid ? "true" : "false", (unsigned long)cal.timestamp);
    chunk_buf_printf(cb, "\"is_homed\":%s,\"led_applied\":%s,\"reboot_required\":%s,",
        SandTablePlayer::isHomed() ? "true" : "false",
        led_applied ? "true" : "false", reboot_required ? "true" : "false");
    chunk_buf_write_str(cb, "\"active_preset_id\":");
    chunk_buf_write_json_string(cb, cfg.active_preset_id());
    chunk_buf_write(cb, "}", 1);
}

static esp_err_t send_config(httpd_req_t* req, bool led_applied, bool reboot_required) {
    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    write_config_json(&cb, led_applied, reboot_required);
    chunk_buf_finish(&cb);
    return ESP_OK;
}

// GET /api/config
static esp_err_t config_get_handler(httpd_req_t* req) {
    return send_config(req, false, false);
}

// Read a numeric field if present. Presence (not "> 0") decides whether a
// field is applied, so 0 is a legal value where it means something
// (motor_idle_timeout_s = 0 -> never de-energize; stallguard 0). NaN / inf
// are treated as absent by json_number, so every cast below is on a finite
// value that already passed its range check.
static bool get_num(cJSON* obj, const char* key, double& out) {
    return json_number(obj, key, out);
}

// PATCH /api/config - partial update: {"motion":{...}, "led":{...}}
// Absent fields are left unchanged. Returns the updated config plus
// `led_applied` (strip rebuilt live) and `reboot_required` (microsteps /
// steps_per_rev changed; take effect after reboot + re-home).
static esp_err_t config_patch_handler(httpd_req_t* req) {
    cJSON* json = parse_json_body(req, 2048);
    if (!json) return ESP_FAIL;

    auto& cfg = sand_table::ConfigManager::instance();
    bool changed = false;
    bool led_applied = false;
    bool reboot_required = false;
    const char* bad_field = nullptr;

    cJSON* motion_json = cJSON_GetObjectItem(json, "motion");
    if (motion_json && cJSON_IsObject(motion_json)) {
        sand_table::RuntimeMotionConfig motion = cfg.motion_config();
        const sand_table::RuntimeMotionConfig before = motion;
        double v;

        if (get_num(motion_json, "steps_per_rev", v)) {
            if (v < 1 || v > 10000) bad_field = "steps_per_rev"; else motion.steps_per_rev = (uint32_t)v;
        }
        if (get_num(motion_json, "microsteps", v)) {
            const int m = (v >= 1 && v <= 256) ? (int)v : 0;
            if (m != 1 && m != 2 && m != 4 && m != 8 && m != 16 && m != 32 && m != 64 && m != 128 && m != 256) {
                bad_field = "microsteps";
            } else {
                motion.microsteps = (uint16_t)m;
            }
        }
        if (get_num(motion_json, "rho_max_rpm", v)) {
            if (v < 1 || v > 100) bad_field = "rho_max_rpm"; else motion.rho_max_rpm = (int32_t)v;
        }
        if (get_num(motion_json, "theta_current_ma", v)) {
            if (v < 100 || v > 1500) bad_field = "theta_current_ma"; else motion.theta_current_ma = (uint16_t)v;
        }
        if (get_num(motion_json, "rho_current_ma", v)) {
            if (v < 100 || v > 1500) bad_field = "rho_current_ma"; else motion.rho_current_ma = (uint16_t)v;
        }
        if (get_num(motion_json, "stallguard_threshold", v)) {
            if (v < 0 || v > 255) bad_field = "stallguard_threshold"; else motion.stallguard_threshold = (uint8_t)v;
        }
        if (get_num(motion_json, "accel_steps_s2", v)) {
            if (v < 500 || v > 100000) bad_field = "accel_steps_s2"; else motion.accel_steps_s2 = (uint32_t)v;
        }
        if (get_num(motion_json, "junction_dv_steps_s", v)) {
            if (v < 10 || v > 5000) bad_field = "junction_dv_steps_s"; else motion.junction_dv_steps_s = (uint32_t)v;
        }
        if (get_num(motion_json, "theta_max_rot_per_min", v)) {
            if (v < 0.5 || v > 60) bad_field = "theta_max_rot_per_min"; else motion.theta_max_rot_per_min = (float)v;
        }
        if (get_num(motion_json, "motor_idle_timeout_s", v)) {
            if (v < 0 || v > 86400) bad_field = "motor_idle_timeout_s"; else motion.motor_idle_timeout_s = (uint32_t)v;
        }

        if (bad_field) {
            cJSON_Delete(json);
            char msg[64];
            snprintf(msg, sizeof(msg), "Out of range: %s", bad_field);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg);
            return ESP_FAIL;
        }

        esp_err_t ret = cfg.set_motion_config(motion);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set motion config: %s", esp_err_to_name(ret));
        } else {
            changed = true;
            reboot_required = motion.steps_per_rev != before.steps_per_rev ||
                              motion.microsteps != before.microsteps;
            // Currents, StallGuard and the shaping knobs apply now.
            if (auto* mc = SandTablePlayer::getMotionController()) {
                mc->apply_motor_config();
                mc->apply_motion_tunables();
                if (motion.stallguard_threshold != before.stallguard_threshold) {
                    mc->apply_stallguard_config();
                }
            }
        }
    }

    cJSON* led_json = cJSON_GetObjectItem(json, "led");
    if (led_json && cJSON_IsObject(led_json)) {
        sand_table::RuntimeLEDConfig led = cfg.led_config();

        double v;
        cJSON* has_leds = cJSON_GetObjectItem(led_json, "has_leds");
        if (has_leds && cJSON_IsBool(has_leds)) led.has_leds = cJSON_IsTrue(has_leds);
        if (get_num(led_json, "led_count", v)) {
            if (v < 1 || v > 2000) bad_field = "led_count"; else led.led_count = (uint16_t)v;
        }
        // Legacy is_rgbw first so an explicit `format` below always wins.
        cJSON* is_rgbw = cJSON_GetObjectItem(led_json, "is_rgbw");
        if (is_rgbw && cJSON_IsBool(is_rgbw)) led.format = cJSON_IsTrue(is_rgbw) ? 4 : 3;

        // Strip type/layout by NAME ("fw1906", "rgbcct", "grb"). Numeric
        // codes are accepted only in the firmware's own numbering (format
        // 3/4/5), never the proto LEDFormat enum (1/2/3) - use strings.
        // Numeric codes are range-checked before the cast.
        cJSON* ic = cJSON_GetObjectItem(led_json, "ic_type");
        if (ic && cJSON_IsString(ic)) { int c = led_ic_code(ic->valuestring); if (c >= 0) led.ic_type = (uint8_t)c; else bad_field = "ic_type"; }
        else if (get_num(led_json, "ic_type", v)) { if (v < 0 || v > 2) bad_field = "ic_type"; else led.ic_type = (uint8_t)v; }

        cJSON* fmt = cJSON_GetObjectItem(led_json, "format");
        if (fmt && cJSON_IsString(fmt)) { int c = led_format_code(fmt->valuestring); if (c >= 0) led.format = (uint8_t)c; else bad_field = "format"; }
        else if (get_num(led_json, "format", v)) { if (v < 3 || v > 5) bad_field = "format"; else led.format = (uint8_t)v; }

        cJSON* order = cJSON_GetObjectItem(led_json, "color_order");
        if (order && cJSON_IsString(order)) { int c = led_order_code(order->valuestring); if (c >= 0) led.color_order = (uint8_t)c; else bad_field = "color_order"; }
        else if (get_num(led_json, "color_order", v)) { if (v < 0 || v > 5) bad_field = "color_order"; else led.color_order = (uint8_t)v; }

        if (bad_field) {
            cJSON_Delete(json);
            char msg[64];
            snprintf(msg, sizeof(msg), "Out of range: %s", bad_field);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg);
            return ESP_FAIL;
        }

        cJSON* wswap = cJSON_GetObjectItem(led_json, "white_swap");
        if (wswap && cJSON_IsBool(wswap)) led.white_swap = cJSON_IsTrue(wswap);

        led.is_rgbw = (led.format == 4);  // keep legacy flag consistent

        esp_err_t ret = cfg.set_led_config(led);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set LED config: %s", esp_err_to_name(ret));
        } else {
            changed = true;
            // Rebuild the strip now - no reboot.
            led_runtime::apply(led);
            led_applied = true;
        }
    }

    cJSON_Delete(json);

    ESP_LOGI(TAG, "Config PATCH: changed=%d led_applied=%d reboot_required=%d",
        changed, led_applied, reboot_required);

    if (!changed) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No valid config in body");
        return ESP_FAIL;
    }

    return send_config(req, led_applied, reboot_required);
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

    auto& cfg = sand_table::ConfigManager::instance();
    const sand_table::RuntimeMotionConfig motion_before = cfg.motion_config();

    esp_err_t ret = cfg.load_preset(preset_id->valuestring);
    ESP_LOGI(TAG, "LoadPreset '%s': %s", preset_id->valuestring, esp_err_to_name(ret));
    cJSON_Delete(json);

    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NOT_FOUND) {
            httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Preset not found");
        } else {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to load preset");
        }
        return ESP_FAIL;
    }

    // A preset changes motor current / shaping and the strip - push all of it
    // to the hardware immediately so it applies without a reboot.
    if (auto* mc = SandTablePlayer::getMotionController()) {
        mc->apply_motor_config();
        mc->apply_motion_tunables();
        mc->apply_stallguard_config();
    }
    led_runtime::apply(cfg.led_config());

    const auto& motion_after = cfg.motion_config();
    const bool reboot_required = motion_after.steps_per_rev != motion_before.steps_per_rev ||
                                 motion_after.microsteps != motion_before.microsteps;

    // Return the updated config so the client sees the applied values
    return send_config(req, true, reboot_required);
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
