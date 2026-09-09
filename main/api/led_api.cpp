// LED REST API implementation
#include "led_api.h"
#include "api_common.h"
#include "led_state.h"
#include "config_manager.h"
#include "kd_pixdriver.h"
#include "pixel_effects.h"

#include <esp_log.h>
#include <cstdio>
#include <cstring>

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

void write_channel_state(ChunkBuf* cb, uint32_t index) {
    auto id = led_state::channel_id(index);
    const PixelChannel* ch = id ? PixelDriver::getChannel(*id) : nullptr;
    if (!ch) {
        chunk_buf_write_str(cb, "null");
        return;
    }
    const EffectConfig& eff = ch->getEffectConfig();
    const PixelFormat fmt = ch->getConfig().format;

    chunk_buf_printf(cb, "{\"index\":%lu,\"effect_id\":", static_cast<unsigned long>(index));
    chunk_buf_write_json_string(cb, eff.effect.c_str());
    // Hex string per the documented LEDChannelState schema. On RGBW the
    // white channel is auto-derived, so 6-digit RGB is the full public state;
    // RGBCCT strips drive both whites explicitly and expose them as w/cw.
    chunk_buf_printf(cb, ",\"brightness\":%u,\"speed\":%u,\"on\":%s,\"color\":\"#%02x%02x%02x\"",
        eff.brightness, eff.speed, eff.enabled ? "true" : "false",
        eff.color.r, eff.color.g, eff.color.b);
    if (fmt == PixelFormat::RGBCCT) {
        chunk_buf_printf(cb, ",\"w\":%u,\"cw\":%u", eff.color.w, eff.color.cw);
    }
    chunk_buf_write(cb, "}", 1);
}

// GET /api/led/effects
esp_err_t effects_handler(httpd_req_t* req) {
    PixelEffectEngine* engine = PixelDriver::getEffectEngine();
    httpd_resp_set_type(req, "application/json");
    // Static list: let the app cache it.
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=3600");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    chunk_buf_write(&cb, "[", 1);
    if (engine) {
        bool first = true;
        for (const auto& eff : engine->getAllEffects()) {
            if (!first) chunk_buf_write(&cb, ",", 1);
            chunk_buf_write_str(&cb, "{\"id\":");
            chunk_buf_write_json_string(&cb, eff.id.c_str());
            chunk_buf_write_str(&cb, ",\"name\":");
            chunk_buf_write_json_string(&cb, eff.display_name.c_str());
            chunk_buf_write(&cb, "}", 1);
            first = false;
        }
    }
    chunk_buf_write(&cb, "]", 1);
    chunk_buf_finish(&cb);
    return ESP_OK;
}

// GET /api/led/config - strip hardware + live state of every channel
esp_err_t config_handler(httpd_req_t* req) {
    std::vector<int32_t> ids = PixelDriver::getChannelIds();
    const auto& hw = sand_table::ConfigManager::instance().led_config();

    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    chunk_buf_write_str(&cb, "{\"version\":");
    chunk_buf_write_json_string(&cb, getPixelDriverVersion());
    chunk_buf_printf(&cb, ",\"has_leds\":%s,\"channels\":[", ids.empty() ? "false" : "true");
    for (size_t i = 0; i < ids.size(); i++) {
        const PixelChannel* ch = PixelDriver::getChannel(ids[i]);
        if (!ch) continue;
        if (i) chunk_buf_write(&cb, ",", 1);
        const ChannelConfig& cfg = ch->getConfig();
        chunk_buf_printf(&cb, "{\"index\":%zu,\"num_leds\":%u,\"type\":\"%s\",\"state\":",
            i, cfg.pixel_count, format_name(cfg.format));
        write_channel_state(&cb, static_cast<uint32_t>(i));
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

esp_err_t send_channel(httpd_req_t* req, uint32_t index) {
    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    write_channel_state(&cb, index);
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
    if (!led_state::channel_id(static_cast<uint32_t>(index))) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Channel not found");
        return ESP_FAIL;
    }
    return send_channel(req, static_cast<uint32_t>(index));
}

bool parse_hex_color(const char* s, PixelColor& out) {
    if (*s == '#') s++;
    unsigned r = 0, g = 0, b = 0, w = 0;
    const size_t len = strlen(s);
    if (len == 6 && sscanf(s, "%02x%02x%02x", &r, &g, &b) == 3) {
        out.r = static_cast<uint8_t>(r);
        out.g = static_cast<uint8_t>(g);
        out.b = static_cast<uint8_t>(b);
        return true;
    }
    if (len == 8 && sscanf(s, "%02x%02x%02x%02x", &r, &g, &b, &w) == 4) {
        out.r = static_cast<uint8_t>(r);
        out.g = static_cast<uint8_t>(g);
        out.b = static_cast<uint8_t>(b);
        out.w = static_cast<uint8_t>(w);
        return true;
    }
    return false;
}

// POST /api/led/channel/{index} - partial update
esp_err_t channel_post_handler(httpd_req_t* req) {
    int index = channel_index_from_uri(req->uri);
    if (index < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid channel index");
        return ESP_FAIL;
    }
    auto id = led_state::channel_id(static_cast<uint32_t>(index));
    const PixelChannel* ch = id ? PixelDriver::getChannel(*id) : nullptr;
    if (!ch) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Channel not found");
        return ESP_FAIL;
    }

    cJSON* json = parse_json_body(req, 1024);
    if (!json) return ESP_FAIL;

    led_state::ChannelPatch patch;
    PixelColor color = ch->getEffectConfig().color;
    bool color_changed = false;

    cJSON* effect_id = cJSON_GetObjectItem(json, "effect_id");
    if (effect_id && cJSON_IsString(effect_id) && effect_id->valuestring[0]) {
        patch.effect_id = std::string(effect_id->valuestring);
    }
    cJSON* brightness = cJSON_GetObjectItem(json, "brightness");
    if (brightness && cJSON_IsNumber(brightness)) {
        int v = brightness->valueint;
        patch.brightness = static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
    }
    cJSON* speed = cJSON_GetObjectItem(json, "speed");
    if (speed && cJSON_IsNumber(speed)) {
        int v = speed->valueint;
        patch.speed = static_cast<uint8_t>(v < 1 ? 1 : (v > 10 ? 10 : v));
    }
    cJSON* on = cJSON_GetObjectItem(json, "on");
    if (on && cJSON_IsBool(on)) patch.enabled = cJSON_IsTrue(on);

    cJSON* c = cJSON_GetObjectItem(json, "color");
    if (c && cJSON_IsString(c)) {
        color_changed = parse_hex_color(c->valuestring, color);
    }
    else if (c && cJSON_IsObject(c)) {
        // Legacy object form {r,g,b,w}
        cJSON* r = cJSON_GetObjectItem(c, "r");
        cJSON* g = cJSON_GetObjectItem(c, "g");
        cJSON* b = cJSON_GetObjectItem(c, "b");
        cJSON* w = cJSON_GetObjectItem(c, "w");
        if (cJSON_IsNumber(r)) { color.r = static_cast<uint8_t>(r->valueint); color_changed = true; }
        if (cJSON_IsNumber(g)) { color.g = static_cast<uint8_t>(g->valueint); color_changed = true; }
        if (cJSON_IsNumber(b)) { color.b = static_cast<uint8_t>(b->valueint); color_changed = true; }
        if (cJSON_IsNumber(w)) { color.w = static_cast<uint8_t>(w->valueint); color_changed = true; }
    }
    // RGBCCT white channels: top-level "w" (warm) / "cw" (cool), 0-255.
    cJSON* w_top = cJSON_GetObjectItem(json, "w");
    if (w_top && cJSON_IsNumber(w_top)) { color.w = static_cast<uint8_t>(w_top->valueint); color_changed = true; }
    cJSON* cw_top = cJSON_GetObjectItem(json, "cw");
    if (cw_top && cJSON_IsNumber(cw_top)) { color.cw = static_cast<uint8_t>(cw_top->valueint); color_changed = true; }
    if (color_changed) patch.color = color;

    cJSON_Delete(json);

    if (!led_state::apply_channel_patch(static_cast<uint32_t>(index), patch)) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Channel not found");
        return ESP_FAIL;
    }
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
