// LED REST API: a thin JSON codec over led_ops / led_state. Channel state
// is only ever read from led_state snapshots (taken under the driver lock).
#include "led_api.h"
#include "api_common.h"
#include "led_state.h"
#include "ops/led_ops.h"
#include "config_manager.h"

#include <esp_log.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

static const char* TAG = "led_api";

using namespace api_common;

namespace {

const char* format_name(PixelFormat f) {
    switch (f) {
        case PixelFormat::RGB: return "RGB";
        case PixelFormat::RGBW: return "RGBW";
        case PixelFormat::RGBCCT: return "RGBCCT";
    }
    return "RGB";
}

// Parse "/api/led/channel/<index>" -> index, or -1.
int channel_index_from_uri(const char* uri) {
    const char* base = "/api/led/channel/";
    if (strncmp(uri, base, strlen(base)) != 0) return -1;
    const char* p = uri + strlen(base);
    if (*p < '0' || *p > '9') return -1;
    return atoi(p);
}

void write_channel_state(ChunkBuf* cb, const led_state::ChannelSnapshot& s) {
    const EffectConfig& eff = s.effect;

    chunk_buf_printf(cb, "{\"index\":%lu,\"effect_id\":", static_cast<unsigned long>(s.index));
    chunk_buf_write_json_string(cb, eff.effect.c_str());
    // Hex string per the documented LEDChannelState schema. On RGBW the
    // white channel is auto-derived, so 6-digit RGB is the full public state;
    // RGBCCT strips drive both whites explicitly and expose them as w/cw.
    chunk_buf_printf(cb, ",\"brightness\":%u,\"speed\":%u,\"on\":%s,\"color\":\"#%02x%02x%02x\"",
        eff.brightness, eff.speed, eff.enabled ? "true" : "false",
        eff.color.r, eff.color.g, eff.color.b);
    if (s.format == PixelFormat::RGBCCT) {
        chunk_buf_printf(cb, ",\"w\":%u,\"cw\":%u", eff.color.w, eff.color.cw);
    }
    chunk_buf_write(cb, "}", 1);
}

esp_err_t send_channel(httpd_req_t* req, uint32_t index) {
    led_state::ChannelSnapshot snap;
    if (!led_state::snapshot_channel(index, snap)) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Channel not found");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    write_channel_state(&cb, snap);
    chunk_buf_finish(&cb);
    return ESP_OK;
}

// GET /api/led/effects
esp_err_t effects_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "application/json");
    // Static list: let the app cache it.
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=3600");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    led_ops::effects_to_json(&cb);
    chunk_buf_finish(&cb);
    return ESP_OK;
}

// GET /api/led/config - strip hardware + live state of every channel
esp_err_t config_handler(httpd_req_t* req) {
    std::vector<led_state::ChannelSnapshot> snaps = led_state::snapshot_channels();
    const auto& hw = sand_table::ConfigManager::instance().led_config();

    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    chunk_buf_write_str(&cb, "{\"version\":");
    chunk_buf_write_json_string(&cb, getPixelDriverVersion());
    chunk_buf_printf(&cb, ",\"has_leds\":%s,\"channels\":[", snaps.empty() ? "false" : "true");
    bool first = true;
    for (const auto& s : snaps) {
        if (!first) chunk_buf_write(&cb, ",", 1);
        first = false;
        chunk_buf_printf(&cb, "{\"index\":%lu,\"num_leds\":%u,\"type\":\"%s\",\"state\":",
            static_cast<unsigned long>(s.index), s.pixel_count, format_name(s.format));
        write_channel_state(&cb, s);
        chunk_buf_write(&cb, "}", 1);
    }
    // Configured (NVS) hardware, even when the strip is currently disabled,
    // so the settings page can show/edit it.
    chunk_buf_printf(&cb, "],\"hardware\":{\"has_leds\":%s,\"led_count\":%u,\"ic_type\":%u,"
        "\"format\":%u,\"color_order\":%u,\"white_swap\":%s}}",
        hw.has_leds ? "true" : "false", hw.led_count, hw.ic_type, hw.format,
        hw.color_order, hw.white_swap ? "true" : "false");
    chunk_buf_finish(&cb);
    return ESP_OK;
}

// GET /api/led/channel/{index}
esp_err_t channel_get_handler(httpd_req_t* req) {
    int index = channel_index_from_uri(req->uri);
    if (index < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid channel index");
        return ESP_FAIL;
    }
    return send_channel(req, static_cast<uint32_t>(index));
}

// POST /api/led/channel/{index} - partial update
esp_err_t channel_post_handler(httpd_req_t* req) {
    int index = channel_index_from_uri(req->uri);
    if (index < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid channel index");
        return ESP_FAIL;
    }
    led_state::ChannelSnapshot current;
    if (!led_state::snapshot_channel(static_cast<uint32_t>(index), current)) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Channel not found");
        return ESP_FAIL;
    }

    cJSON* json = parse_json_body(req, 1024);
    if (!json) return ESP_FAIL;

    // Range clamping (brightness 0-255, speed 1-10) happens in led_state.
    led_state::ChannelPatch patch;
    PixelColor color = current.effect.color;
    bool color_changed = false;
    double v;

    cJSON* effect_id = cJSON_GetObjectItem(json, "effect_id");
    if (effect_id && cJSON_IsString(effect_id) && effect_id->valuestring[0]) {
        patch.effect_id = std::string(effect_id->valuestring);
    }
    if (json_number(json, "brightness", v)) patch.brightness = static_cast<int>(v < 0 ? 0 : (v > 255 ? 255 : v));
    if (json_number(json, "speed", v)) patch.speed = static_cast<int>(v < 0 ? 0 : (v > 10 ? 10 : v));
    cJSON* on = cJSON_GetObjectItem(json, "on");
    if (on && cJSON_IsBool(on)) patch.enabled = cJSON_IsTrue(on);

    cJSON* c = cJSON_GetObjectItem(json, "color");
    if (c && cJSON_IsString(c)) {
        color_changed = parse_hex_color(c->valuestring, color);
    }
    else if (c && cJSON_IsObject(c)) {
        // Legacy object form {r,g,b,w}
        if (json_number(c, "r", v)) { color.r = static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v)); color_changed = true; }
        if (json_number(c, "g", v)) { color.g = static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v)); color_changed = true; }
        if (json_number(c, "b", v)) { color.b = static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v)); color_changed = true; }
        if (json_number(c, "w", v)) { color.w = static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v)); color_changed = true; }
    }
    // RGBCCT white channels: top-level "w" (warm) / "cw" (cool), 0-255.
    if (json_number(json, "w", v)) { color.w = static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v)); color_changed = true; }
    if (json_number(json, "cw", v)) { color.cw = static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v)); color_changed = true; }
    if (color_changed) patch.color = color;

    cJSON_Delete(json);

    OpResult r = led_ops::set_channel(static_cast<uint32_t>(index), patch);
    if (!r.ok()) return send_op_error(req, r);
    ESP_LOGD(TAG, "Channel %d updated", index);
    return send_channel(req, static_cast<uint32_t>(index));
}

}  // namespace

void led_api_register_handlers(httpd_handle_t server) {
    static httpd_uri_t effects_uri = {
        .uri = "/api/led/effects", .method = HTTP_GET, .handler = effects_handler, .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &effects_uri);

    static httpd_uri_t config_uri = {
        .uri = "/api/led/config", .method = HTTP_GET, .handler = config_handler, .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &config_uri);

    static httpd_uri_t channel_get_uri = {
        .uri = "/api/led/channel/*", .method = HTTP_GET, .handler = channel_get_handler, .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &channel_get_uri);

    static httpd_uri_t channel_post_uri = {
        .uri = "/api/led/channel/*", .method = HTTP_POST, .handler = channel_post_handler, .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &channel_post_uri);
}
