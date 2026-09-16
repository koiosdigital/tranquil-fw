// Schedule manager implementation
#include "schedule_manager.h"
#include "led_state.h"
#include "library_notifier.h"
#include "download_progress.h"
#include "download_tracker.h"
#include <freertos/task.h>
#include "SandTablePlayer.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_random.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <time.h>
#include <sys/time.h>
#include <cstring>
#include <algorithm>

static const char* TAG = "schedule_manager";

namespace {

constexpr auto kTypeUnspecified = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_UNSPECIFIED;
constexpr auto kTypeLedOff      = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_LED_OFF;
constexpr auto kTypeLedOn       = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_LED_ON;
constexpr auto kTypeRandom      = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_PLAY_RANDOM_PATTERN;
constexpr auto kTypePlaylist    = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_PLAY_PLAYLIST;
constexpr auto kTypePattern     = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_PLAY_PATTERN;
constexpr auto kTypeLedState    = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_SET_LED_STATE;
constexpr auto kTypeSpeed       = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_SET_SPEED;
constexpr auto kTypeStop        = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_STOP;

// Blocking by default; the esp_timer tick passes a short timeout and checks
// operator bool.
struct Lock {
    SemaphoreHandle_t m;
    bool held;
    explicit Lock(SemaphoreHandle_t mutex, TickType_t timeout = portMAX_DELAY)
        : m(mutex), held(mutex != nullptr && xSemaphoreTake(mutex, timeout) == pdTRUE) {}
    ~Lock() { if (held) xSemaphoreGive(m); }
    explicit operator bool() const { return held; }
};

// ---------------------------------------------------------------------------
// NVS blob v2: explicitly packed, little-endian, length-prefixed strings.
// ---------------------------------------------------------------------------
constexpr uint8_t kBlobMagic0 = 'S';
constexpr uint8_t kBlobMagic1 = '2';
constexpr uint8_t kBlobVersion = 2;

constexpr uint8_t kItemEnabled = 1 << 0;
constexpr uint8_t kItemObeyQuiet = 1 << 1;
constexpr uint8_t kActShuffle = 1 << 0;
constexpr uint8_t kActLoop = 1 << 1;
constexpr uint8_t kLedHasEffect = 1 << 0;
constexpr uint8_t kLedHasBrightness = 1 << 1;
constexpr uint8_t kLedHasSpeed = 1 << 2;
constexpr uint8_t kLedHasEnabled = 1 << 3;
constexpr uint8_t kLedHasColor = 1 << 4;
constexpr uint8_t kQuietStop = 1 << 0;
constexpr uint8_t kQuietLights = 1 << 1;

struct Writer {
    std::vector<uint8_t> buf;
    void u8(uint8_t v) { buf.push_back(v); }
    void u16(uint16_t v) { u8(v & 0xFF); u8(v >> 8); }
    void u32(uint32_t v) { u16(v & 0xFFFF); u16(v >> 16); }
    void str(const std::string& s) {
        size_t n = std::min<size_t>(s.size(), 255);
        u8(static_cast<uint8_t>(n));
        buf.insert(buf.end(), s.begin(), s.begin() + n);
    }
};

struct Reader {
    const uint8_t* p;
    const uint8_t* end;
    bool ok = true;
    bool need(size_t n) { if (static_cast<size_t>(end - p) < n) { ok = false; return false; } return true; }
    uint8_t u8() { if (!need(1)) return 0; return *p++; }
    uint16_t u16() { uint16_t lo = u8(); uint16_t hi = u8(); return lo | (hi << 8); }
    uint32_t u32() { uint32_t lo = u16(); uint32_t hi = u16(); return lo | (hi << 16); }
    std::string str() {
        uint8_t n = u8();
        if (!need(n)) return {};
        std::string s(reinterpret_cast<const char*>(p), n);
        p += n;
        return s;
    }
};

// Same-day window is [start, end) on a listed day; a wrap-midnight window
// covers [start, 24:00) on a listed day and [00:00, end) on the day after.
bool minute_in_window(int today, uint32_t now_min, const QuietWindow& w) {
    if (!w.enabled) return false;
    const uint32_t start = w.start_min;
    const uint32_t end = w.end_min;
    if (start == end) return false;
    const bool today_listed = (w.day_mask & (1u << today)) != 0;
    if (start < end) {
        return today_listed && now_min >= start && now_min < end;
    }
    if (today_listed && now_min >= start) return true;
    const int prev = (today + 6) % 7;
    return (w.day_mask & (1u << prev)) && now_min < end;
}

void set_all_channels(const led_state::ChannelPatch& patch) {
    for (size_t i = 0; i < led_state::channel_count(); i++) {
        led_state::apply_channel_patch(static_cast<uint32_t>(i), patch);
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

ScheduleManager& ScheduleManager::instance() {
    static ScheduleManager instance;
    return instance;
}

esp_err_t ScheduleManager::init() {
    if (initialized_) return ESP_OK;

    if (!mutex_) mutex_ = xSemaphoreCreateMutex();

    // Load: v2 blob, else the pre-id v1 blob (migrated on first save).
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (ret == ESP_OK) {
        size_t size = 0;
        bool loaded = false;
        if (nvs_get_blob(handle, kKeySchedule, nullptr, &size) == ESP_OK && size >= 4) {
            std::vector<uint8_t> buf(size);
            if (nvs_get_blob(handle, kKeySchedule, buf.data(), &size) == ESP_OK) {
                Lock lock(mutex_);
                loaded = loadV2(buf.data(), size);
                if (!loaded) ESP_LOGW(TAG, "Schedule blob v2 is corrupt; ignoring");
            }
        }
        if (!loaded && nvs_get_blob(handle, kKeyLegacy, nullptr, &size) == ESP_OK && size >= 2) {
            std::vector<uint8_t> buf(size);
            if (nvs_get_blob(handle, kKeyLegacy, buf.data(), &size) == ESP_OK) {
                Lock lock(mutex_);
                if (loadLegacyV1(buf.data(), size) == ESP_OK) {
                    ESP_LOGI(TAG, "Migrated %zu legacy schedule items", schedule_.size());
                    if (saveLocked() == ESP_OK) {
                        nvs_erase_key(handle, kKeyLegacy);
                        nvs_commit(handle);
                    }
                }
            }
        }
        nvs_close(handle);
    } else if (ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
    }

    esp_timer_create_args_t timer_args = {
        .callback = timerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "schedule",
        .skip_unhandled_events = true,
    };

    ret = esp_timer_create(&timer_args, &timer_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_timer_start_periodic(timer_, CHECK_INTERVAL_US);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(ret));
        esp_timer_delete(timer_);
        timer_ = nullptr;
        return ret;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Schedule manager initialized: %zu items, %zu quiet windows",
        schedule_.size(), quiet_.windows.size());
    return ESP_OK;
}

void ScheduleManager::shutdown() {
    if (!initialized_) return;

    if (timer_) {
        esp_timer_stop(timer_);
        esp_timer_delete(timer_);
        timer_ = nullptr;
    }

    initialized_ = false;
    ESP_LOGI(TAG, "Schedule manager shutdown");
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

std::vector<ScheduleItem> ScheduleManager::getSchedule() const {
    Lock lock(mutex_);
    return schedule_;
}

QuietHours ScheduleManager::getQuietHours() const {
    Lock lock(mutex_);
    return quiet_;
}

bool ScheduleManager::isQuietNow() const {
    int day;
    uint32_t secs;
    time_t now;
    if (!getCurrentTime(day, secs, now)) return false;
    Lock lock(mutex_);
    return quietNowLocked(day, secs);
}

bool ScheduleManager::quietNowLocked(int day_of_week, uint32_t seconds_since_midnight) const {
    const uint32_t now_min = seconds_since_midnight / 60;
    for (const auto& w : quiet_.windows) {
        if (minute_in_window(day_of_week, now_min, w)) return true;
    }
    return false;
}

const char* ScheduleManager::actionName(ScheduleActionType type) {
    switch (type) {
        case kTypeLedOff: return "led_off";
        case kTypeLedOn: return "led_on";
        case kTypeRandom: return "play_random_pattern";
        case kTypePlaylist: return "play_playlist";
        case kTypePattern: return "play_pattern";
        case kTypeLedState: return "set_led_state";
        case kTypeSpeed: return "set_speed";
        case kTypeStop: return "stop";
        default: return "unspecified";
    }
}

// ---------------------------------------------------------------------------
// Mutation
// ---------------------------------------------------------------------------

void ScheduleManager::sanitize(ScheduleItem& item) {
    item.days_of_week &= DayOfWeek::ALL_DAYS;
    item.time_of_day %= 86400;
    if (item.name.size() > kMaxNameLen) item.name.resize(kMaxNameLen);
    if (item.action.uuid.size() > kMaxUuidLen) item.action.uuid.resize(kMaxUuidLen);
    if (item.action.led.effect_id && item.action.led.effect_id->size() > kMaxEffectLen) {
        item.action.led.effect_id->resize(kMaxEffectLen);
    }
    if (item.action.led.speed) item.action.led.speed = std::clamp<uint8_t>(*item.action.led.speed, 1, 10);
    if (item.action.type == kTypeSpeed) {
        item.action.feed_rate = std::clamp(item.action.feed_rate, kFeedRateMin, kFeedRateMax);
    }
}

void ScheduleManager::sanitize(QuietHours& quiet) {
    if (quiet.windows.size() > kMaxQuietWindows) quiet.windows.resize(kMaxQuietWindows);
    for (auto& w : quiet.windows) {
        w.day_mask &= DayOfWeek::ALL_DAYS;
        w.start_min %= 1440;
        w.end_min %= 1440;
    }
}

esp_err_t ScheduleManager::setSchedule(std::vector<ScheduleItem> items,
                                       const std::optional<QuietHours>& quiet,
                                       int exclude_local_fd,
                                       bool exclude_cloud) {
    if (items.size() > kMaxItems) return ESP_ERR_INVALID_SIZE;
    if (quiet && quiet->windows.size() > kMaxQuietWindows) return ESP_ERR_INVALID_SIZE;

    // Assign ids to new items and de-duplicate any the client repeated.
    std::vector<uint32_t> seen;
    for (auto& item : items) {
        sanitize(item);
        while (item.id == 0 || std::find(seen.begin(), seen.end(), item.id) != seen.end()) {
            item.id = esp_random();
        }
        seen.push_back(item.id);
    }

    std::sort(items.begin(), items.end(),
        [](const ScheduleItem& a, const ScheduleItem& b) {
            return a.time_of_day < b.time_of_day;
        });

    esp_err_t ret;
    size_t n_items, n_windows;
    {
        Lock lock(mutex_);
        schedule_ = std::move(items);
        if (quiet) {
            quiet_ = *quiet;
            sanitize(quiet_);
        }
        // Forget fire records for items that no longer exist.
        for (auto it = last_fired_.begin(); it != last_fired_.end();) {
            bool present = std::any_of(schedule_.begin(), schedule_.end(),
                [&](const ScheduleItem& s) { return s.id == it->first; });
            it = present ? std::next(it) : last_fired_.erase(it);
        }
        ret = saveLocked();
        n_items = schedule_.size();
        n_windows = quiet_.windows.size();
    }

    ESP_LOGI(TAG, "Schedule set: %zu items, quiet windows=%zu (%s)",
        n_items, n_windows, esp_err_to_name(ret));
    if (ret == ESP_OK) library_notifier::schedule_changed(exclude_local_fd, exclude_cloud);
    return ret;
}

esp_err_t ScheduleManager::clearSchedule() {
    return setSchedule({}, QuietHours{});
}

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------

esp_err_t ScheduleManager::saveLocked() {
    Writer w;
    w.u8(kBlobMagic0);
    w.u8(kBlobMagic1);
    w.u8(kBlobVersion);
    w.u8((quiet_.stop_playback ? kQuietStop : 0) | (quiet_.lights_off ? kQuietLights : 0));

    w.u8(static_cast<uint8_t>(quiet_.windows.size()));
    for (const auto& win : quiet_.windows) {
        w.u8(static_cast<uint8_t>(win.day_mask));
        w.u16(win.start_min);
        w.u16(win.end_min);
        w.u8(win.enabled ? 1 : 0);
    }

    w.u16(static_cast<uint16_t>(schedule_.size()));
    for (const auto& item : schedule_) {
        w.u32(item.id);
        w.u8(static_cast<uint8_t>(item.days_of_week));
        w.u32(item.time_of_day);
        w.u8((item.enabled ? kItemEnabled : 0) | (item.obey_quiet_hours ? kItemObeyQuiet : 0));
        w.str(item.name);

        const auto& a = item.action;
        w.u8(static_cast<uint8_t>(a.type));
        w.str(a.uuid);
        w.u16(static_cast<uint16_t>(std::clamp(a.feed_rate, 0.0f, 600.0f) * 100.0f));
        w.u8((a.shuffle ? kActShuffle : 0) | (a.loop ? kActLoop : 0));

        uint8_t pres = 0;
        if (a.led.effect_id) pres |= kLedHasEffect;
        if (a.led.brightness) pres |= kLedHasBrightness;
        if (a.led.speed) pres |= kLedHasSpeed;
        if (a.led.enabled) pres |= kLedHasEnabled;
        if (a.led.color) pres |= kLedHasColor;
        w.u8(pres);
        if (a.led.effect_id) w.str(*a.led.effect_id);
        if (a.led.brightness) w.u8(*a.led.brightness);
        if (a.led.speed) w.u8(*a.led.speed);
        if (a.led.enabled) w.u8(*a.led.enabled ? 1 : 0);
        if (a.led.color) {
            w.u8(a.led.color->r); w.u8(a.led.color->g); w.u8(a.led.color->b);
            w.u8(a.led.color->w); w.u8(a.led.color->cw);
        }
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = nvs_set_blob(handle, kKeySchedule, w.buf.data(), w.buf.size());
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Schedule saved (%zu items, %zu bytes)", schedule_.size(), w.buf.size());
    } else {
        ESP_LOGE(TAG, "Failed to save schedule: %s", esp_err_to_name(ret));
    }
    return ret;
}

bool ScheduleManager::loadV2(const uint8_t* data, size_t len) {
    Reader r{data, data + len};
    if (r.u8() != kBlobMagic0 || r.u8() != kBlobMagic1) return false;
    const uint8_t version = r.u8();
    if (version != kBlobVersion) return false;
    const uint8_t qflags = r.u8();

    QuietHours quiet;
    quiet.stop_playback = (qflags & kQuietStop) != 0;
    quiet.lights_off = (qflags & kQuietLights) != 0;
    const uint8_t n_windows = r.u8();
    for (uint8_t i = 0; i < n_windows && r.ok; i++) {
        QuietWindow w;
        w.day_mask = r.u8();
        w.start_min = r.u16();
        w.end_min = r.u16();
        w.enabled = r.u8() != 0;
        quiet.windows.push_back(w);
    }

    std::vector<ScheduleItem> items;
    const uint16_t count = r.u16();
    items.reserve(std::min<size_t>(count, kMaxItems));
    for (uint16_t i = 0; i < count && r.ok; i++) {
        ScheduleItem item;
        item.id = r.u32();
        item.days_of_week = r.u8();
        item.time_of_day = r.u32();
        const uint8_t flags = r.u8();
        item.enabled = (flags & kItemEnabled) != 0;
        item.obey_quiet_hours = (flags & kItemObeyQuiet) != 0;
        item.name = r.str();

        auto& a = item.action;
        a.type = static_cast<ScheduleActionType>(r.u8());
        a.uuid = r.str();
        a.feed_rate = static_cast<float>(r.u16()) / 100.0f;
        const uint8_t aflags = r.u8();
        a.shuffle = (aflags & kActShuffle) != 0;
        a.loop = (aflags & kActLoop) != 0;

        const uint8_t pres = r.u8();
        if (pres & kLedHasEffect) a.led.effect_id = r.str();
        if (pres & kLedHasBrightness) a.led.brightness = r.u8();
        if (pres & kLedHasSpeed) a.led.speed = r.u8();
        if (pres & kLedHasEnabled) a.led.enabled = r.u8() != 0;
        if (pres & kLedHasColor) {
            PixelColor c;
            c.r = r.u8(); c.g = r.u8(); c.b = r.u8(); c.w = r.u8(); c.cw = r.u8();
            a.led.color = c;
        }
        if (!r.ok) break;
        sanitize(item);
        items.push_back(std::move(item));
    }
    if (!r.ok) return false;

    sanitize(quiet);
    schedule_ = std::move(items);
    quiet_ = std::move(quiet);
    ESP_LOGI(TAG, "Schedule loaded (%zu items, %zu quiet windows)", schedule_.size(), quiet_.windows.size());
    return true;
}

// v1: [count:u16] then per item [days:u32][time:u32][action:u8][uuid_len:u8][uuid]
esp_err_t ScheduleManager::loadLegacyV1(const uint8_t* data, size_t len) {
    const uint8_t* ptr = data;
    const uint8_t* end = data + len;
    if (len < 2) return ESP_ERR_INVALID_SIZE;

    uint16_t count;
    memcpy(&count, ptr, 2); ptr += 2;

    std::vector<ScheduleItem> items;
    std::vector<uint32_t> seen;
    for (uint16_t i = 0; i < count && ptr < end; i++) {
        if (ptr + 10 > end) break;
        ScheduleItem item;
        memcpy(&item.days_of_week, ptr, 4); ptr += 4;
        memcpy(&item.time_of_day, ptr, 4); ptr += 4;
        item.action.type = static_cast<ScheduleActionType>(*ptr++);
        uint8_t uuid_len = *ptr++;
        if (ptr + uuid_len > end) break;
        item.action.uuid = std::string(reinterpret_cast<const char*>(ptr), uuid_len);
        ptr += uuid_len;
        // v1 playlists always looped.
        if (item.action.type == kTypePlaylist) item.action.loop = true;
        while (item.id == 0 || std::find(seen.begin(), seen.end(), item.id) != seen.end()) {
            item.id = esp_random();
        }
        seen.push_back(item.id);
        sanitize(item);
        items.push_back(std::move(item));
        if (items.size() >= kMaxItems) break;
    }
    schedule_ = std::move(items);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Timer / execution
// ---------------------------------------------------------------------------

void ScheduleManager::timerCallback(void* arg) {
    auto* self = static_cast<ScheduleManager*>(arg);
    self->checkAndExecute();
}

bool ScheduleManager::getCurrentTime(int& day_of_week, uint32_t& seconds_since_midnight, time_t& now) const {
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    // Check if time is set (year > 2020)
    if (timeinfo.tm_year < (2020 - 1900)) {
        return false;
    }

    day_of_week = timeinfo.tm_wday;  // 0 = Sunday
    seconds_since_midnight = timeinfo.tm_hour * 3600 + timeinfo.tm_min * 60 + timeinfo.tm_sec;
    return true;
}

void ScheduleManager::actionTask(void* arg) {
    auto* item = static_cast<ScheduleItem*>(arg);
    auto& mgr = ScheduleManager::instance();
    // Quiet hours were already evaluated on the tick; run unconditionally.
    RunResult res = mgr.runAction(item->action, true);
    ESP_LOGI(TAG, "Scheduled item %lu '%s' (%s): %s%s%s",
        static_cast<unsigned long>(item->id), item->name.c_str(),
        actionName(item->action.type), res.ok ? "ok" : "failed",
        res.detail ? " - " : "", res.detail ? res.detail : "");
    delete item;
    vTaskDelete(nullptr);
}

void ScheduleManager::spawnAction(const ScheduleItem& item) {
    auto* copy = new ScheduleItem(item);
    if (xTaskCreate(actionTask, "sched_act", 6144, copy, 4, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Failed to spawn schedule action task");
        delete copy;
    }
}

void ScheduleManager::quietEdgeTask(void* arg) {
    const bool entering = reinterpret_cast<uintptr_t>(arg) != 0;
    ScheduleManager::instance().onQuietEdge(entering);
    vTaskDelete(nullptr);
}

void ScheduleManager::onQuietEdge(bool entering) {
    QuietHours quiet = getQuietHours();
    if (entering) {
        ESP_LOGI(TAG, "Quiet hours started (stop=%d lights_off=%d)", quiet.stop_playback, quiet.lights_off);
        if (quiet.stop_playback) {
            auto state = SandTablePlayer::getPlaybackState();
            if (state == PlaybackState::PLAYING || state == PlaybackState::PAUSED) {
                SandTablePlayer::stop();
            }
        }
        if (quiet.lights_off) {
            std::vector<uint32_t> dimmed;
            const size_t n = led_state::channel_count();
            for (size_t i = 0; i < n; i++) {
                led_state::ChannelSnapshot snap;
                if (!led_state::snapshot_channel(static_cast<uint32_t>(i), snap) || !snap.effect.enabled) continue;
                led_state::ChannelPatch patch;
                patch.enabled = false;
                if (led_state::apply_channel_patch(static_cast<uint32_t>(i), patch)) {
                    dimmed.push_back(static_cast<uint32_t>(i));
                }
            }
            Lock lock(mutex_);
            quiet_dimmed_channels_ = std::move(dimmed);
        }
    } else {
        std::vector<uint32_t> dimmed;
        {
            Lock lock(mutex_);
            dimmed.swap(quiet_dimmed_channels_);
        }
        ESP_LOGI(TAG, "Quiet hours ended (restoring %zu LED channels)", dimmed.size());
        for (uint32_t index : dimmed) {
            led_state::ChannelPatch patch;
            patch.enabled = true;
            led_state::apply_channel_patch(index, patch);
        }
    }
    library_notifier::system_info_changed();
}

void ScheduleManager::checkAndExecute() {
    // Housekeeping that rides on the 30 s tick: age out downloads that never
    // finished. Both give up (and retry next tick) if their lock is busy.
    DownloadProgressBroadcaster::instance().expireStale();
    DownloadTracker::instance().cleanupExpired();

    int day_of_week;
    uint32_t current_time;
    time_t now;
    if (!getCurrentTime(day_of_week, current_time, now)) {
        return;  // Time not set
    }

    // esp_timer task: a blocking take here would stall every other timer
    // behind an httpd / cloudlink writer holding the lock. Try briefly and
    // skip the tick instead; the fire window is 30 s wide, so the next tick
    // still catches anything due.
    Lock lock(mutex_, TICK_LOCK_WAIT);
    if (!lock) {
        ESP_LOGW(TAG, "Schedule tick skipped: lock busy");
        return;
    }

    const bool quiet = quietNowLocked(day_of_week, current_time);
    if (quiet != quiet_active_) {
        quiet_active_ = quiet;
        if (xTaskCreate(quietEdgeTask, "sched_quiet", 6144,
                        reinterpret_cast<void*>(static_cast<uintptr_t>(quiet ? 1 : 0)), 4, nullptr) != pdPASS) {
            ESP_LOGE(TAG, "Failed to spawn quiet-hours task");
        }
    }

    if (schedule_.empty()) return;

    for (const auto& item : schedule_) {
        if (!item.enabled) continue;
        if (!item.matches_day(day_of_week)) continue;

        // Within the execution window [T, T + tick]
        int32_t diff = static_cast<int32_t>(current_time) - static_cast<int32_t>(item.time_of_day);
        if (diff < 0 || diff > FIRE_WINDOW_S) continue;

        // Never twice for the same window (per item, so items sharing a time all fire).
        auto fired = last_fired_.find(item.id);
        if (fired != last_fired_.end() && now - fired->second < REFIRE_GUARD_S) continue;
        last_fired_[item.id] = now;

        if (quiet && item.obey_quiet_hours) {
            ESP_LOGI(TAG, "Skipping item %lu '%s' (quiet hours)",
                static_cast<unsigned long>(item.id), item.name.c_str());
            continue;
        }

        ESP_LOGI(TAG, "Executing item %lu '%s': %s at %02u:%02u:%02u",
            static_cast<unsigned long>(item.id), item.name.c_str(),
            actionName(item.action.type),
            item.time_of_day / 3600,
            (item.time_of_day % 3600) / 60,
            item.time_of_day % 60);
        spawnAction(item);
    }
}

ScheduleManager::RunResult ScheduleManager::runAction(const ScheduleAction& action, bool force) {
    if (!force && isQuietNow()) {
        return {false, "Quiet hours active"};
    }

    switch (action.type) {
        case kTypeLedOff: {
            led_state::ChannelPatch patch;
            patch.enabled = false;
            set_all_channels(patch);
            return {true, nullptr};
        }

        case kTypeLedOn: {
            led_state::ChannelPatch patch;
            patch.enabled = true;
            set_all_channels(patch);
            return {true, nullptr};
        }

        case kTypeLedState: {
            if (action.led.empty()) return {false, "LED state has no fields"};
            led_state::ChannelPatch patch;
            patch.effect_id = action.led.effect_id;
            patch.brightness = action.led.brightness;
            patch.speed = action.led.speed;
            patch.enabled = action.led.enabled;
            patch.color = action.led.color;
            set_all_channels(patch);
            return {true, nullptr};
        }

        case kTypeSpeed: {
            if (action.feed_rate <= 0.0f) return {false, "feed_rate required"};
            SandTablePlayer::setFeedRate(std::clamp(action.feed_rate, kFeedRateMin, kFeedRateMax));
            return {true, nullptr};
        }

        case kTypeStop: {
            SandTablePlayer::stop();
            return {true, nullptr};
        }

        case kTypeRandom: {
            // loop = keep chaining random patterns (PlayMode::RANDOM_LOOP)
            esp_err_t err = SandTablePlayer::playRandom(action.loop);
            if (err == ESP_ERR_NOT_FOUND) return {false, "No playable patterns"};
            return {err == ESP_OK, err == ESP_OK ? nullptr : "Failed to start pattern"};
        }

        case kTypePlaylist: {
            if (action.uuid.empty()) return {false, "Playlist uuid required"};
            esp_err_t err = SandTablePlayer::playPlaylist(action.uuid.c_str(), action.shuffle, action.loop);
            return {err == ESP_OK, err == ESP_OK ? nullptr : "Failed to start playlist"};
        }

        case kTypePattern: {
            if (action.uuid.empty()) return {false, "Pattern uuid required"};
            esp_err_t err = SandTablePlayer::playPattern(action.uuid.c_str());
            return {err == ESP_OK, err == ESP_OK ? nullptr : "Failed to start pattern"};
        }

        default:
            return {false, "Unknown action"};
    }
}

// ---------------------------------------------------------------------------
// Protobuf conversion
// ---------------------------------------------------------------------------

ScheduleAction ScheduleManager::actionFromProto(const Kd__V1__ScheduleAction* p) {
    ScheduleAction a;
    if (!p) return a;
    a.type = p->type;
    if (p->uuid) a.uuid = p->uuid;
    a.feed_rate = p->feed_rate;
    a.shuffle = p->shuffle;
    a.loop = p->loop;
    if (p->led) {
        const auto* led = p->led;
        if (led->_effect_id_case == KD__V1__SCHEDULE_LED_STATE___EFFECT_ID_EFFECT_ID && led->effect_id) {
            a.led.effect_id = std::string(led->effect_id);
        }
        if (led->_brightness_case == KD__V1__SCHEDULE_LED_STATE___BRIGHTNESS_BRIGHTNESS) {
            a.led.brightness = static_cast<uint8_t>(std::min<uint32_t>(led->brightness, 255));
        }
        if (led->_speed_case == KD__V1__SCHEDULE_LED_STATE___SPEED_SPEED) {
            a.led.speed = static_cast<uint8_t>(std::clamp<uint32_t>(led->speed, 1, 10));
        }
        if (led->_enabled_case == KD__V1__SCHEDULE_LED_STATE___ENABLED_ENABLED) {
            a.led.enabled = led->enabled != 0;
        }
        if (led->color) {
            a.led.color = PixelColor(
                static_cast<uint8_t>(led->color->r), static_cast<uint8_t>(led->color->g),
                static_cast<uint8_t>(led->color->b), static_cast<uint8_t>(led->color->w),
                static_cast<uint8_t>(led->color->cw));
        }
    }
    return a;
}

ScheduleItem ScheduleManager::itemFromProto(const Kd__V1__TranquilScheduleItem* p) {
    ScheduleItem item;
    if (!p) return item;
    item.id = p->id;
    if (p->name) item.name = p->name;
    item.days_of_week = p->days_of_week;
    item.time_of_day = p->time_of_day;
    item.enabled = p->_enabled_case == KD__V1__TRANQUIL_SCHEDULE_ITEM___ENABLED_ENABLED ? (p->enabled != 0) : true;
    item.obey_quiet_hours = p->_obey_quiet_hours_case == KD__V1__TRANQUIL_SCHEDULE_ITEM___OBEY_QUIET_HOURS_OBEY_QUIET_HOURS
        ? (p->obey_quiet_hours != 0) : true;
    item.action = actionFromProto(p->action);
    return item;
}

std::vector<ScheduleItem> ScheduleManager::itemsFromProto(const Kd__V1__TranquilSchedule* proto) {
    std::vector<ScheduleItem> items;
    if (!proto) return items;
    items.reserve(proto->n_schedule_items);
    for (size_t i = 0; i < proto->n_schedule_items; i++) {
        if (!proto->schedule_items[i]) continue;
        items.push_back(itemFromProto(proto->schedule_items[i]));
    }
    return items;
}

std::optional<QuietHours> ScheduleManager::quietFromProto(const Kd__V1__TranquilSchedule* proto) {
    if (!proto || !proto->quiet_hours) return std::nullopt;
    QuietHours q;
    const auto* p = proto->quiet_hours;
    q.stop_playback = p->stop_playback != 0;
    q.lights_off = p->lights_off != 0;
    for (size_t i = 0; i < p->n_windows; i++) {
        const auto* w = p->windows[i];
        if (!w) continue;
        QuietWindow win;
        win.day_mask = w->day_mask;
        win.start_min = static_cast<uint16_t>(w->start_min % 1440);
        win.end_min = static_cast<uint16_t>(w->end_min % 1440);
        win.enabled = w->enabled != 0;
        q.windows.push_back(win);
    }
    return q;
}

void ScheduleManager::toProto(ScheduleProtoScratch& s) const {
    {
        Lock lock(mutex_);
        s.snapshot = schedule_;
        s.quiet_snapshot = quiet_;
    }
    const size_t n = s.snapshot.size();
    s.items.assign(n, KD__V1__TRANQUIL_SCHEDULE_ITEM__INIT);
    s.actions.assign(n, KD__V1__SCHEDULE_ACTION__INIT);
    s.leds.assign(n, KD__V1__SCHEDULE_LED_STATE__INIT);
    s.colors.assign(n, KD__V1__LEDCOLOR__INIT);
    s.item_ptrs.clear();
    s.item_ptrs.reserve(n);

    for (size_t i = 0; i < n; i++) {
        const ScheduleItem& src = s.snapshot[i];
        auto& item = s.items[i];
        item.id = src.id;
        item.name = const_cast<char*>(src.name.c_str());
        item.days_of_week = src.days_of_week;
        item.time_of_day = src.time_of_day;
        item._enabled_case = KD__V1__TRANQUIL_SCHEDULE_ITEM___ENABLED_ENABLED;
        item.enabled = src.enabled;
        item._obey_quiet_hours_case = KD__V1__TRANQUIL_SCHEDULE_ITEM___OBEY_QUIET_HOURS_OBEY_QUIET_HOURS;
        item.obey_quiet_hours = src.obey_quiet_hours;

        auto& act = s.actions[i];
        act.type = src.action.type;
        act.uuid = const_cast<char*>(src.action.uuid.c_str());
        act.feed_rate = src.action.feed_rate;
        act.shuffle = src.action.shuffle;
        act.loop = src.action.loop;
        if (src.action.type == kTypeLedState) {
            auto& led = s.leds[i];
            const auto& l = src.action.led;
            if (l.effect_id) {
                led._effect_id_case = KD__V1__SCHEDULE_LED_STATE___EFFECT_ID_EFFECT_ID;
                led.effect_id = const_cast<char*>(l.effect_id->c_str());
            }
            if (l.brightness) {
                led._brightness_case = KD__V1__SCHEDULE_LED_STATE___BRIGHTNESS_BRIGHTNESS;
                led.brightness = *l.brightness;
            }
            if (l.speed) {
                led._speed_case = KD__V1__SCHEDULE_LED_STATE___SPEED_SPEED;
                led.speed = *l.speed;
            }
            if (l.enabled) {
                led._enabled_case = KD__V1__SCHEDULE_LED_STATE___ENABLED_ENABLED;
                led.enabled = *l.enabled;
            }
            if (l.color) {
                auto& c = s.colors[i];
                c.r = l.color->r; c.g = l.color->g; c.b = l.color->b; c.w = l.color->w; c.cw = l.color->cw;
                led.color = &c;
            }
            act.led = &led;
        }
        item.action = &act;
        s.item_ptrs.push_back(&item);
    }

    const size_t nw = s.quiet_snapshot.windows.size();
    s.windows.assign(nw, KD__V1__TRANQUIL_QUIET_WINDOW__INIT);
    s.window_ptrs.clear();
    s.window_ptrs.reserve(nw);
    for (size_t i = 0; i < nw; i++) {
        const auto& w = s.quiet_snapshot.windows[i];
        s.windows[i].day_mask = w.day_mask;
        s.windows[i].start_min = w.start_min;
        s.windows[i].end_min = w.end_min;
        s.windows[i].enabled = w.enabled;
        s.window_ptrs.push_back(&s.windows[i]);
    }
    s.quiet = KD__V1__TRANQUIL_QUIET_HOURS__INIT;
    s.quiet.n_windows = nw;
    s.quiet.windows = s.window_ptrs.data();
    s.quiet.stop_playback = s.quiet_snapshot.stop_playback;
    s.quiet.lights_off = s.quiet_snapshot.lights_off;

    s.schedule = KD__V1__TRANQUIL_SCHEDULE__INIT;
    s.schedule.n_schedule_items = s.item_ptrs.size();
    s.schedule.schedule_items = s.item_ptrs.data();
    s.schedule.quiet_hours = &s.quiet;
}

ResponseMessage ScheduleManager::buildScheduleMessage(const char* request_id) const {
    ScheduleProtoScratch scratch;
    toProto(scratch);

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE;
    msg.schedule = &scratch.schedule;
    return pack_tranquil_message(&msg, request_id);
}
