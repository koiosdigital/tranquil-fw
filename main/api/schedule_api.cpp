// Schedule REST API. Local clients manage the schedule over HTTP; the
// protobuf Schedule message stays on the sockets for cloud sync and for
// notifying connected WS clients of changes (the library notifier pushes it
// after every set, whichever transport wrote it).
#include "schedule_api.h"
#include "api_common.h"
#include "schedule_manager.h"

#include <esp_log.h>
#include <algorithm>
#include <cstring>
#include <optional>
#include <vector>

static const char* TAG = "schedule_api";

using namespace api_common;

namespace {

bool parse_hex_color(const char* s, PixelColor& out) {
    if (*s == '#') s++;
    unsigned r = 0, g = 0, b = 0, w = 0;
    const size_t len = strlen(s);
    if (len == 6 && sscanf(s, "%02x%02x%02x", &r, &g, &b) == 3) {
        out.r = r; out.g = g; out.b = b;
        return true;
    }
    if (len == 8 && sscanf(s, "%02x%02x%02x%02x", &r, &g, &b, &w) == 4) {
        out.r = r; out.g = g; out.b = b; out.w = w;
        return true;
    }
    return false;
}

void write_action(ChunkBuf* cb, const ScheduleAction& a) {
    chunk_buf_printf(cb, "{\"type\":%d,\"uuid\":", (int)a.type);
    chunk_buf_write_json_string(cb, a.uuid.c_str());
    chunk_buf_printf(cb, ",\"shuffle\":%s,\"loop\":%s,\"feed_rate\":%.2f",
        a.shuffle ? "true" : "false", a.loop ? "true" : "false", (double)a.feed_rate);
    if (a.type == KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_SET_LED_STATE) {
        chunk_buf_write_str(cb, ",\"led\":{");
        bool first = true;
        auto sep = [&]() { if (!first) chunk_buf_write(cb, ",", 1); first = false; };
        if (a.led.enabled) { sep(); chunk_buf_printf(cb, "\"on\":%s", *a.led.enabled ? "true" : "false"); }
        if (a.led.effect_id) { sep(); chunk_buf_write_str(cb, "\"effect_id\":"); chunk_buf_write_json_string(cb, a.led.effect_id->c_str()); }
        if (a.led.brightness) { sep(); chunk_buf_printf(cb, "\"brightness\":%u", *a.led.brightness); }
        if (a.led.speed) { sep(); chunk_buf_printf(cb, "\"speed\":%u", *a.led.speed); }
        if (a.led.color) {
            sep();
            chunk_buf_printf(cb, "\"color\":\"#%02x%02x%02x\",\"w\":%u,\"cw\":%u",
                a.led.color->r, a.led.color->g, a.led.color->b, a.led.color->w, a.led.color->cw);
        }
        chunk_buf_write(cb, "}", 1);
    }
    chunk_buf_write(cb, "}", 1);
}

void write_schedule_json(ChunkBuf* cb) {
    auto& mgr = ScheduleManager::instance();
    const auto items = mgr.getSchedule();
    const auto quiet = mgr.getQuietHours();

    chunk_buf_write_str(cb, "{\"items\":[");
    bool first = true;
    for (const auto& item : items) {
        if (!first) chunk_buf_write(cb, ",", 1);
        first = false;
        chunk_buf_printf(cb, "{\"id\":%lu,\"name\":", (unsigned long)item.id);
        chunk_buf_write_json_string(cb, item.name.c_str());
        chunk_buf_printf(cb, ",\"days_of_week\":%lu,\"time_of_day\":%lu,\"enabled\":%s,\"obey_quiet_hours\":%s,\"action\":",
            (unsigned long)item.days_of_week,
            (unsigned long)item.time_of_day,
            item.enabled ? "true" : "false",
            item.obey_quiet_hours ? "true" : "false");
        write_action(cb, item.action);
        chunk_buf_write(cb, "}", 1);
    }

    chunk_buf_write_str(cb, "],\"quiet_hours\":{\"windows\":[");
    first = true;
    for (const auto& w : quiet.windows) {
        if (!first) chunk_buf_write(cb, ",", 1);
        first = false;
        chunk_buf_printf(cb, "{\"day_mask\":%lu,\"start_min\":%u,\"end_min\":%u,\"enabled\":%s}",
            (unsigned long)w.day_mask, w.start_min, w.end_min, w.enabled ? "true" : "false");
    }
    chunk_buf_printf(cb, "],\"stop_playback\":%s,\"lights_off\":%s},\"quiet_now\":%s}",
        quiet.stop_playback ? "true" : "false",
        quiet.lights_off ? "true" : "false",
        mgr.isQuietNow() ? "true" : "false");
}

esp_err_t send_schedule(httpd_req_t* req) {
    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    write_schedule_json(&cb);
    chunk_buf_finish(&cb);
    return ESP_OK;
}

bool json_bool(const cJSON* obj, const char* key, bool fallback) {
    const cJSON* v = cJSON_GetObjectItem(obj, key);
    if (v && cJSON_IsBool(v)) return cJSON_IsTrue(v);
    return fallback;
}

// Parses {"type":..,"uuid":..,"shuffle":..,"loop":..,"feed_rate":..,
//         "led":{"on","effect_id","brightness","speed","color","w","cw"}}
bool parse_action(const cJSON* json, ScheduleAction& out, const char** err) {
    const cJSON* type = cJSON_GetObjectItem(json, "type");
    if (!type || !cJSON_IsNumber(type)) { *err = "action.type is required"; return false; }
    out.type = (ScheduleActionType)type->valueint;

    const cJSON* uuid = cJSON_GetObjectItem(json, "uuid");
    if (uuid && cJSON_IsString(uuid)) out.uuid = uuid->valuestring;
    out.shuffle = json_bool(json, "shuffle", false);
    out.loop = json_bool(json, "loop", false);
    const cJSON* feed = cJSON_GetObjectItem(json, "feed_rate");
    if (feed && cJSON_IsNumber(feed)) out.feed_rate = (float)feed->valuedouble;

    const cJSON* led = cJSON_GetObjectItem(json, "led");
    if (led && cJSON_IsObject(led)) {
        const cJSON* on = cJSON_GetObjectItem(led, "on");
        if (on && cJSON_IsBool(on)) out.led.enabled = cJSON_IsTrue(on);
        const cJSON* effect = cJSON_GetObjectItem(led, "effect_id");
        if (effect && cJSON_IsString(effect) && effect->valuestring[0]) out.led.effect_id = std::string(effect->valuestring);
        const cJSON* bright = cJSON_GetObjectItem(led, "brightness");
        if (bright && cJSON_IsNumber(bright)) out.led.brightness = (uint8_t)std::clamp(bright->valueint, 0, 255);
        const cJSON* speed = cJSON_GetObjectItem(led, "speed");
        if (speed && cJSON_IsNumber(speed)) out.led.speed = (uint8_t)std::clamp(speed->valueint, 1, 10);

        PixelColor color;
        bool color_changed = false;
        const cJSON* c = cJSON_GetObjectItem(led, "color");
        if (c && cJSON_IsString(c)) {
            color_changed = parse_hex_color(c->valuestring, color);
        } else if (c && cJSON_IsObject(c)) {
            const cJSON* r = cJSON_GetObjectItem(c, "r");
            const cJSON* g = cJSON_GetObjectItem(c, "g");
            const cJSON* b = cJSON_GetObjectItem(c, "b");
            const cJSON* w = cJSON_GetObjectItem(c, "w");
            const cJSON* cw = cJSON_GetObjectItem(c, "cw");
            if (cJSON_IsNumber(r)) { color.r = (uint8_t)r->valueint; color_changed = true; }
            if (cJSON_IsNumber(g)) { color.g = (uint8_t)g->valueint; color_changed = true; }
            if (cJSON_IsNumber(b)) { color.b = (uint8_t)b->valueint; color_changed = true; }
            if (cJSON_IsNumber(w)) { color.w = (uint8_t)w->valueint; color_changed = true; }
            if (cJSON_IsNumber(cw)) { color.cw = (uint8_t)cw->valueint; color_changed = true; }
        }
        const cJSON* w_top = cJSON_GetObjectItem(led, "w");
        if (w_top && cJSON_IsNumber(w_top)) { color.w = (uint8_t)w_top->valueint; color_changed = true; }
        const cJSON* cw_top = cJSON_GetObjectItem(led, "cw");
        if (cw_top && cJSON_IsNumber(cw_top)) { color.cw = (uint8_t)cw_top->valueint; color_changed = true; }
        if (color_changed) out.led.color = color;
    }
    return true;
}

bool parse_item(const cJSON* item_json, ScheduleItem& item, const char** err) {
    const cJSON* days = cJSON_GetObjectItem(item_json, "days_of_week");
    const cJSON* time = cJSON_GetObjectItem(item_json, "time_of_day");
    if (!days || !cJSON_IsNumber(days) || !time || !cJSON_IsNumber(time)) {
        *err = "Items need days_of_week and time_of_day";
        return false;
    }
    item.days_of_week = (uint32_t)days->valueint;
    item.time_of_day = (uint32_t)time->valueint;

    const cJSON* id = cJSON_GetObjectItem(item_json, "id");
    if (id && cJSON_IsNumber(id)) item.id = (uint32_t)id->valuedouble;
    const cJSON* name = cJSON_GetObjectItem(item_json, "name");
    if (name && cJSON_IsString(name)) item.name = name->valuestring;
    item.enabled = json_bool(item_json, "enabled", true);
    item.obey_quiet_hours = json_bool(item_json, "obey_quiet_hours", true);

    const cJSON* action = cJSON_GetObjectItem(item_json, "action");
    if (action && cJSON_IsObject(action)) {
        return parse_action(action, item.action, err);
    }
    // Legacy flat form: {"action_type": n, "uuid": "..."}
    const cJSON* action_type = cJSON_GetObjectItem(item_json, "action_type");
    if (!action_type || !cJSON_IsNumber(action_type)) {
        *err = "Items need an action";
        return false;
    }
    item.action.type = (ScheduleActionType)action_type->valueint;
    const cJSON* uuid = cJSON_GetObjectItem(item_json, "uuid");
    if (uuid && cJSON_IsString(uuid)) item.action.uuid = uuid->valuestring;
    if (item.action.type == KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_PLAY_PLAYLIST) {
        item.action.loop = true;
    }
    return true;
}

bool parse_quiet(const cJSON* q, QuietHours& out, const char** err) {
    out.stop_playback = json_bool(q, "stop_playback", false);
    out.lights_off = json_bool(q, "lights_off", false);
    const cJSON* windows = cJSON_GetObjectItem(q, "windows");
    if (windows && !cJSON_IsArray(windows)) { *err = "quiet_hours.windows must be an array"; return false; }
    const cJSON* w;
    cJSON_ArrayForEach(w, windows) {
        if (!cJSON_IsObject(w)) continue;
        const cJSON* mask = cJSON_GetObjectItem(w, "day_mask");
        const cJSON* start = cJSON_GetObjectItem(w, "start_min");
        const cJSON* end = cJSON_GetObjectItem(w, "end_min");
        if (!mask || !cJSON_IsNumber(mask) || !start || !cJSON_IsNumber(start) || !end || !cJSON_IsNumber(end)) {
            *err = "Quiet windows need day_mask, start_min, end_min";
            return false;
        }
        QuietWindow win;
        win.day_mask = (uint32_t)mask->valueint;
        win.start_min = (uint16_t)(start->valueint % 1440);
        win.end_min = (uint16_t)(end->valueint % 1440);
        win.enabled = json_bool(w, "enabled", true);
        out.windows.push_back(win);
        if (out.windows.size() > ScheduleManager::kMaxQuietWindows) {
            *err = "Too many quiet windows";
            return false;
        }
    }
    return true;
}

// GET /api/schedule
esp_err_t schedule_get_handler(httpd_req_t* req) {
    return send_schedule(req);
}

// PUT /api/schedule - replace the item list; quiet_hours replaced only when present
esp_err_t schedule_put_handler(httpd_req_t* req) {
    cJSON* json = parse_json_body(req, 16384);
    if (!json) return ESP_FAIL;

    cJSON* items_json = cJSON_GetObjectItem(json, "items");
    if (!items_json || !cJSON_IsArray(items_json)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing items array");
        return ESP_FAIL;
    }

    const char* err = nullptr;
    std::vector<ScheduleItem> items;
    cJSON* item_json;
    cJSON_ArrayForEach(item_json, items_json) {
        if (!cJSON_IsObject(item_json)) continue;
        ScheduleItem item;
        if (!parse_item(item_json, item, &err)) {
            cJSON_Delete(json);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, err);
            return ESP_FAIL;
        }
        items.push_back(std::move(item));
        if (items.size() > ScheduleManager::kMaxItems) {
            cJSON_Delete(json);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Too many schedule items (max 32)");
            return ESP_FAIL;
        }
    }

    std::optional<QuietHours> quiet;
    cJSON* quiet_json = cJSON_GetObjectItem(json, "quiet_hours");
    if (quiet_json && cJSON_IsObject(quiet_json)) {
        QuietHours q;
        if (!parse_quiet(quiet_json, q, &err)) {
            cJSON_Delete(json);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, err);
            return ESP_FAIL;
        }
        quiet = std::move(q);
    }
    cJSON_Delete(json);

    esp_err_t ret = ScheduleManager::instance().setSchedule(std::move(items), quiet);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set schedule: %s", esp_err_to_name(ret));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save schedule");
        return ESP_FAIL;
    }

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
    if (!parse_action(action_json, action, &err)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, err);
        return ESP_FAIL;
    }
    const bool force = json_bool(json, "force", false);
    cJSON_Delete(json);

    auto res = ScheduleManager::instance().runAction(action, force);
    ESP_LOGI(TAG, "Run action %s (force=%d): %s", ScheduleManager::actionName(action.type), force,
        res.ok ? "ok" : (res.detail ? res.detail : "failed"));

    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    chunk_buf_printf(&cb, "{\"success\":%s,\"detail\":", res.ok ? "true" : "false");
    chunk_buf_write_json_string(&cb, res.detail ? res.detail : "");
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
