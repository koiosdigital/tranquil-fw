// Schedule manager - stores and executes scheduled actions, and enforces
// quiet hours.
//
// Model: a flat list of items (day mask + time of day + action), replaced as
// a whole by whichever transport wrote last (REST, LAN WS, cloud). Items carry
// a stable id so clients can key, toggle and run them; ids of 0 are assigned
// here. Quiet hours are a small set of windows during which items that honour
// them are skipped; entering a window can also stop the table and turn the
// lights off.
#pragma once

#include <esp_err.h>
#include <esp_timer.h>
#include <kd/v1/tranquil.pb-c.h>
#include <vector>
#include <string>
#include <optional>
#include <map>
#include <cstdint>
#include <ctime>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "handler_base.h"
#include "kd_pixdriver.h"

// Day of week bitmask (bit 0 = Sunday, bit 6 = Saturday)
namespace DayOfWeek {
    constexpr uint32_t SUNDAY    = (1 << 0);
    constexpr uint32_t MONDAY    = (1 << 1);
    constexpr uint32_t TUESDAY   = (1 << 2);
    constexpr uint32_t WEDNESDAY = (1 << 3);
    constexpr uint32_t THURSDAY  = (1 << 4);
    constexpr uint32_t FRIDAY    = (1 << 5);
    constexpr uint32_t SATURDAY  = (1 << 6);
    constexpr uint32_t ALL_DAYS  = 0x7F;
}

using ScheduleActionType = Kd__V1__ScheduleAction__ScheduleActionType;

// Partial LED patch for SET_LED_STATE; nullopt = leave unchanged.
struct ScheduleLedState {
    std::optional<std::string> effect_id;
    std::optional<uint8_t> brightness;
    std::optional<uint8_t> speed;
    std::optional<bool> enabled;
    std::optional<PixelColor> color;

    bool empty() const {
        return !effect_id && !brightness && !speed && !enabled && !color;
    }
};

struct ScheduleAction {
    ScheduleActionType type = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_UNSPECIFIED;
    std::string uuid;           // PLAY_PLAYLIST / PLAY_PATTERN
    ScheduleLedState led;       // SET_LED_STATE
    float feed_rate = 0.0f;     // SET_SPEED (1.0-5.0)
    bool shuffle = false;       // PLAY_PLAYLIST
    bool loop = false;          // PLAY_PLAYLIST
};

struct ScheduleItem {
    uint32_t id = 0;            // stable; 0 = unassigned
    std::string name;
    uint32_t days_of_week = 0;  // Bitmask: bit 0=Sun, bit 6=Sat
    uint32_t time_of_day = 0;   // Seconds since local midnight (0-86399)
    bool enabled = true;
    bool obey_quiet_hours = true;
    ScheduleAction action;

    bool matches_day(int day) const {
        return (days_of_week & (1u << day)) != 0;
    }
};

// A window whose end is before its start wraps past midnight; day_mask names
// the day the window starts on.
struct QuietWindow {
    uint32_t day_mask = 0;      // bit 0=Sun .. bit 6=Sat
    uint16_t start_min = 0;     // minutes since local midnight
    uint16_t end_min = 0;
    bool enabled = true;
};

struct QuietHours {
    std::vector<QuietWindow> windows;
    bool pause_playback = false; // pause playback when a window starts, resume when it ends
    bool lights_off = false;    // LEDs off when a window starts, restored when it ends
};

// Owns every buffer a packed TranquilSchedule points into. Keep it alive
// until the message has been serialized.
struct ScheduleProtoScratch {
    Kd__V1__TranquilSchedule schedule;
    Kd__V1__TranquilQuietHours quiet;
    std::vector<ScheduleItem> snapshot;          // string storage
    QuietHours quiet_snapshot;
    std::vector<Kd__V1__TranquilScheduleItem> items;
    std::vector<Kd__V1__TranquilScheduleItem*> item_ptrs;
    std::vector<Kd__V1__ScheduleAction> actions;
    std::vector<Kd__V1__ScheduleLedState> leds;
    std::vector<Kd__V1__LEDColor> colors;
    std::vector<Kd__V1__TranquilQuietWindow> windows;
    std::vector<Kd__V1__TranquilQuietWindow*> window_ptrs;
};

class ScheduleManager {
public:
    static constexpr size_t kMaxItems = 32;
    static constexpr size_t kMaxQuietWindows = 4;
    static constexpr size_t kMaxNameLen = 32;
    static constexpr size_t kMaxUuidLen = 64;
    static constexpr size_t kMaxEffectLen = 32;
    static constexpr float kFeedRateMin = 1.0f;
    static constexpr float kFeedRateMax = 5.0f;

    struct RunResult {
        bool ok;
        const char* detail;
    };

    static ScheduleManager& instance();

    // Initialize - loads schedule from NVS and starts the scheduler timer
    esp_err_t init();
    void shutdown();

    // Copies: the live state is replaced from httpd / cloudlink tasks while
    // the timer task iterates it.
    std::vector<ScheduleItem> getSchedule() const;
    QuietHours getQuietHours() const;

    // True while a quiet-hours window is active (false until time is synced).
    bool isQuietNow() const;

    // Packed TranquilMessage{schedule} with the current items + quiet hours.
    // `request_id` is echoed on the envelope when replying to a request.
    ResponseMessage buildScheduleMessage(const char* request_id = nullptr) const;

    // Fill a TranquilSchedule from the live state into caller-owned scratch.
    void toProto(ScheduleProtoScratch& scratch) const;

    // Replace the item list (validates, clamps, assigns missing ids) and,
    // when given, the quiet-hours config. Persists and notifies listeners.
    // The requester (already answered with the stored schedule) can be left
    // out of the push: its local socket fd, or exclude_cloud when it was the
    // cloud. ESP_ERR_INVALID_SIZE when over the caps.
    esp_err_t setSchedule(std::vector<ScheduleItem> items,
                          const std::optional<QuietHours>& quiet = std::nullopt,
                          int exclude_local_fd = -1,
                          bool exclude_cloud = false);
    esp_err_t clearSchedule();

    // Execute an action now. Honours quiet hours unless `force`. Safe to
    // call from httpd / socket tasks (play() touches the SD card and motion).
    RunResult runAction(const ScheduleAction& action, bool force);

    // A client issued play / pause / stop. Whatever the quiet-hours edge
    // paused is now the user's business: the end of the window resumes only
    // a pause the device made itself and nobody touched since.
    void notePlaybackCommand();

    // Protobuf conversions
    static ScheduleAction actionFromProto(const Kd__V1__ScheduleAction* proto);
    static ScheduleItem itemFromProto(const Kd__V1__TranquilScheduleItem* proto);
    static std::vector<ScheduleItem> itemsFromProto(const Kd__V1__TranquilSchedule* proto);
    static std::optional<QuietHours> quietFromProto(const Kd__V1__TranquilSchedule* proto);

    static const char* actionName(ScheduleActionType type);

private:
    ScheduleManager() = default;

    static void timerCallback(void* arg);
    void checkAndExecute();

    // Off the esp_timer task: play() loads a file from SD and drains motion.
    void spawnAction(const ScheduleItem& item);
    static void actionTask(void* arg);
    static void quietEdgeTask(void* arg);
    void onQuietEdge(bool entering);

    bool getCurrentTime(int& day_of_week, uint32_t& seconds_since_midnight, time_t& now) const;
    bool quietNowLocked(int day_of_week, uint32_t seconds_since_midnight) const;
    static void sanitize(ScheduleItem& item);
    static void sanitize(QuietHours& quiet);

    esp_err_t saveLocked();
    esp_err_t loadLegacyV1(const uint8_t* data, size_t len);
    bool loadV2(const uint8_t* data, size_t len);

    std::vector<ScheduleItem> schedule_;
    QuietHours quiet_;
    mutable SemaphoreHandle_t mutex_ = nullptr;
    esp_timer_handle_t timer_ = nullptr;
    bool initialized_ = false;

    // Per-item last fire, so two items at the same time both fire and one
    // item never fires twice for the same tick window.
    std::map<uint32_t, time_t> last_fired_;

    // Quiet-hours edge tracking (timer task only)
    bool quiet_active_ = false;
    // Set when the window entry paused playback (guarded by mutex_).
    bool paused_by_quiet_ = false;
    // Channels we switched off on window entry, to restore on exit.
    std::vector<uint32_t> quiet_dimmed_channels_;

    static constexpr int64_t CHECK_INTERVAL_US = 30 * 1000 * 1000LL;
    static constexpr int32_t FIRE_WINDOW_S = 30;
    static constexpr time_t REFIRE_GUARD_S = 120;
    // The tick runs on the esp_timer task: never block behind an httpd /
    // cloudlink writer, skip the tick instead (the fire window is 30 s).
    static constexpr TickType_t TICK_LOCK_WAIT = pdMS_TO_TICKS(50);

    static constexpr const char* kNamespace = "tranquil_sched";
    static constexpr const char* kKeyLegacy = "schedule";   // v1 blob (pre-ids)
    static constexpr const char* kKeySchedule = "sched_v2";
};
