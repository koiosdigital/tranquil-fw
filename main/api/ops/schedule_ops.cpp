// Schedule operations implementation
#include "schedule_ops.h"

#include <esp_log.h>
#include <cmath>
#include <cstring>

static const char* TAG = "schedule_ops";

using namespace api_common;

namespace schedule_ops {

namespace {

constexpr auto kTypeMin      = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_LED_OFF;
constexpr auto kTypeMax      = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_STOP;
constexpr auto kTypePlaylist = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_PLAY_PLAYLIST;
constexpr auto kTypeLedState = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_SET_LED_STATE;

bool json_bool(const cJSON* obj, const char* key, bool fallback) {
    const cJSON* v = cJSON_GetObjectItem(obj, key);
    if (v && cJSON_IsBool(v)) return cJSON_IsTrue(v);
    return fallback;
}

// Reads obj[key] as a finite number within [lo, hi]. `present` reports
// whether the key was there at all; a present-but-invalid value fails with
// *err = what.
bool read_num(const cJSON* obj, const char* key, double lo, double hi,
              double& out, bool& present, const char** err, const char* what) {
    present = false;
    const cJSON* v = cJSON_GetObjectItem(obj, key);
    if (!v) return true;
    if (!cJSON_IsNumber(v) || !std::isfinite(v->valuedouble) ||
        v->valuedouble < lo || v->valuedouble > hi) {
        *err = what;
        return false;
    }
    present = true;
    out = v->valuedouble;
    return true;
}

bool read_color_component(const cJSON* obj, const char* key, uint8_t& out, bool& changed, const char** err) {
    double v;
    bool present;
    if (!read_num(obj, key, 0, 255, v, present, err, "led color components must be 0-255")) return false;
    if (present) {
        out = static_cast<uint8_t>(v);
        changed = true;
    }
    return true;
}

bool parse_led(const cJSON* led, ScheduleLedState& out, const char** err) {
    const cJSON* on = cJSON_GetObjectItem(led, "on");
    if (on && cJSON_IsBool(on)) out.enabled = cJSON_IsTrue(on);

    const cJSON* effect = cJSON_GetObjectItem(led, "effect_id");
    if (effect && cJSON_IsString(effect) && effect->valuestring[0]) {
        out.effect_id = std::string(effect->valuestring);
    }

    double v;
    bool present;
    if (!read_num(led, "brightness", 0, 255, v, present, err, "led.brightness must be 0-255")) return false;
    if (present) out.brightness = static_cast<uint8_t>(v);
    if (!read_num(led, "speed", 1, 10, v, present, err, "led.speed must be 1-10")) return false;
    if (present) out.speed = static_cast<uint8_t>(v);

    PixelColor color;
    bool color_changed = false;
    const cJSON* c = cJSON_GetObjectItem(led, "color");
    if (c && cJSON_IsString(c)) {
        if (!parse_hex_color(c->valuestring, color)) {
            *err = "led.color must be #rrggbb or #rrggbbww";
            return false;
        }
        color_changed = true;
    } else if (c && cJSON_IsObject(c)) {
        if (!read_color_component(c, "r", color.r, color_changed, err)) return false;
        if (!read_color_component(c, "g", color.g, color_changed, err)) return false;
        if (!read_color_component(c, "b", color.b, color_changed, err)) return false;
        if (!read_color_component(c, "w", color.w, color_changed, err)) return false;
        if (!read_color_component(c, "cw", color.cw, color_changed, err)) return false;
    }
    if (!read_color_component(led, "w", color.w, color_changed, err)) return false;
    if (!read_color_component(led, "cw", color.cw, color_changed, err)) return false;
    if (color_changed) out.color = color;
    return true;
}

bool parse_item(const cJSON* item_json, ScheduleItem& item, const char** err) {
    double v;
    bool present;

    if (!read_num(item_json, "days_of_week", 0, 127, v, present, err, "days_of_week must be 0-127")) return false;
    if (!present) { *err = "Items need days_of_week and time_of_day"; return false; }
    item.days_of_week = static_cast<uint32_t>(v);

    if (!read_num(item_json, "time_of_day", 0, 86399, v, present, err, "time_of_day must be 0-86399")) return false;
    if (!present) { *err = "Items need days_of_week and time_of_day"; return false; }
    item.time_of_day = static_cast<uint32_t>(v);

    if (!read_num(item_json, "id", 0, 4294967295.0, v, present, err, "id out of range")) return false;
    if (present) item.id = static_cast<uint32_t>(v);

    const cJSON* name = cJSON_GetObjectItem(item_json, "name");
    if (name && cJSON_IsString(name)) item.name = name->valuestring;
    item.enabled = json_bool(item_json, "enabled", true);
    item.obey_quiet_hours = json_bool(item_json, "obey_quiet_hours", true);

    const cJSON* action = cJSON_GetObjectItem(item_json, "action");
    if (action && cJSON_IsObject(action)) {
        return action_from_json(action, item.action, err);
    }

    // Legacy flat form: {"action_type": n, "uuid": "..."}
    if (!read_num(item_json, "action_type", kTypeMin, kTypeMax, v, present, err, "action_type out of range")) return false;
    if (!present) { *err = "Items need an action"; return false; }
    item.action.type = static_cast<ScheduleActionType>(static_cast<int>(v));
    const cJSON* uuid = cJSON_GetObjectItem(item_json, "uuid");
    if (uuid && cJSON_IsString(uuid)) {
        if (strlen(uuid->valuestring) > kMaxUuidChars) { *err = "uuid too long"; return false; }
        item.action.uuid = uuid->valuestring;
    }
    if (item.action.type == kTypePlaylist) {
        item.action.loop = true;
    }
    return true;
}

bool parse_quiet(const cJSON* q, QuietHours& out, const char** err) {
    // "stop_playback" is the pre-rename key; the behaviour has been a pause since.
    out.pause_playback = json_bool(q, "pause_playback", json_bool(q, "stop_playback", false));
    out.lights_off = json_bool(q, "lights_off", false);
    const cJSON* windows = cJSON_GetObjectItem(q, "windows");
    if (windows && !cJSON_IsArray(windows)) { *err = "quiet_hours.windows must be an array"; return false; }
    const cJSON* w;
    cJSON_ArrayForEach(w, windows) {
        if (!cJSON_IsObject(w)) continue;
        double v;
        bool present;
        QuietWindow win;

        if (!read_num(w, "day_mask", 0, 127, v, present, err, "quiet window day_mask must be 0-127")) return false;
        if (!present) { *err = "Quiet windows need day_mask, start_min, end_min"; return false; }
        win.day_mask = static_cast<uint32_t>(v);

        if (!read_num(w, "start_min", 0, 1439, v, present, err, "quiet window start_min must be 0-1439")) return false;
        if (!present) { *err = "Quiet windows need day_mask, start_min, end_min"; return false; }
        win.start_min = static_cast<uint16_t>(v);

        if (!read_num(w, "end_min", 0, 1439, v, present, err, "quiet window end_min must be 0-1439")) return false;
        if (!present) { *err = "Quiet windows need day_mask, start_min, end_min"; return false; }
        win.end_min = static_cast<uint16_t>(v);

        win.enabled = json_bool(w, "enabled", true);
        out.windows.push_back(win);
        if (out.windows.size() > ScheduleManager::kMaxQuietWindows) {
            *err = "Too many quiet windows";
            return false;
        }
    }
    return true;
}

void write_action(ChunkBuf* cb, const ScheduleAction& a) {
    chunk_buf_printf(cb, "{\"type\":%d,\"uuid\":", (int)a.type);
    chunk_buf_write_json_string(cb, a.uuid.c_str());
    chunk_buf_printf(cb, ",\"shuffle\":%s,\"loop\":%s,\"feed_rate\":%.2f",
        a.shuffle ? "true" : "false", a.loop ? "true" : "false", (double)a.feed_rate);
    if (a.type == kTypeLedState) {
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

}  // namespace

OpResult set_schedule(std::vector<ScheduleItem> items,
                      const std::optional<QuietHours>& quiet,
                      int exclude_local_fd,
                      bool exclude_cloud) {
    esp_err_t err = ScheduleManager::instance().setSchedule(std::move(items), quiet, exclude_local_fd, exclude_cloud);
    if (err == ESP_OK) return OpResult::success();
    if (err == ESP_ERR_INVALID_SIZE) return OpResult::fail(err, "Too many schedule items or quiet windows");
    ESP_LOGE(TAG, "Failed to set schedule: %s", esp_err_to_name(err));
    return OpResult::fail(err, "Failed to save schedule");
}

OpResult run_action(const ScheduleAction& action, bool force) {
    ScheduleManager::RunResult res = ScheduleManager::instance().runAction(action, force);
    ESP_LOGI(TAG, "Run action %s (force=%d): %s", ScheduleManager::actionName(action.type), force,
        res.ok ? "ok" : (res.detail ? res.detail : "failed"));
    if (res.ok) return OpResult::success();
    return OpResult::fail(ESP_FAIL, res.detail ? res.detail : "Action failed");
}

void to_json(ChunkBuf* cb) {
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
    chunk_buf_printf(cb, "],\"pause_playback\":%s,\"lights_off\":%s},\"quiet_now\":%s}",
        quiet.pause_playback ? "true" : "false",
        quiet.lights_off ? "true" : "false",
        mgr.isQuietNow() ? "true" : "false");
}

// Parses {"type":..,"uuid":..,"shuffle":..,"loop":..,"feed_rate":..,
//         "led":{"on","effect_id","brightness","speed","color","w","cw"}}
bool action_from_json(const cJSON* json, ScheduleAction& out, const char** err) {
    if (!json || !cJSON_IsObject(json)) { *err = "action must be an object"; return false; }

    double v;
    bool present;
    if (!read_num(json, "type", kTypeMin, kTypeMax, v, present, err, "action.type out of range")) return false;
    if (!present) { *err = "action.type is required"; return false; }
    if (v != std::floor(v)) { *err = "action.type must be an integer"; return false; }
    out.type = static_cast<ScheduleActionType>(static_cast<int>(v));

    const cJSON* uuid = cJSON_GetObjectItem(json, "uuid");
    if (uuid && cJSON_IsString(uuid)) {
        if (strlen(uuid->valuestring) > kMaxUuidChars) { *err = "action.uuid too long"; return false; }
        out.uuid = uuid->valuestring;
    }
    out.shuffle = json_bool(json, "shuffle", false);
    out.loop = json_bool(json, "loop", false);

    if (!read_num(json, "feed_rate", 0, 600, v, present, err, "action.feed_rate out of range")) return false;
    if (present) out.feed_rate = static_cast<float>(v);

    const cJSON* led = cJSON_GetObjectItem(json, "led");
    if (led && cJSON_IsObject(led)) {
        return parse_led(led, out.led, err);
    }
    return true;
}

bool schedule_from_json(const cJSON* root, std::vector<ScheduleItem>& items,
                        std::optional<QuietHours>& quiet, const char** err) {
    const cJSON* items_json = cJSON_GetObjectItem(root, "items");
    if (!items_json || !cJSON_IsArray(items_json)) {
        *err = "Missing items array";
        return false;
    }

    const cJSON* item_json;
    cJSON_ArrayForEach(item_json, items_json) {
        if (!cJSON_IsObject(item_json)) continue;
        ScheduleItem item;
        if (!parse_item(item_json, item, err)) return false;
        items.push_back(std::move(item));
        if (items.size() > ScheduleManager::kMaxItems) {
            *err = "Too many schedule items (max 32)";
            return false;
        }
    }

    const cJSON* quiet_json = cJSON_GetObjectItem(root, "quiet_hours");
    if (quiet_json && cJSON_IsObject(quiet_json)) {
        QuietHours q;
        if (!parse_quiet(quiet_json, q, err)) return false;
        quiet = std::move(q);
    }
    return true;
}

}  // namespace schedule_ops
