// Schedule manager - stores and executes scheduled actions
#pragma once

#include <esp_err.h>
#include <esp_timer.h>
#include <kd/v1/tranquil.pb-c.h>
#include <vector>
#include <string>
#include <cstdint>

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

struct ScheduleItem {
    uint32_t days_of_week;  // Bitmask: bit 0=Sun, bit 6=Sat
    uint32_t time_of_day;   // Seconds since local midnight (0-86399)
    Kd__V1__ScheduleAction__ScheduleActionType action_type;
    std::string uuid;       // For PLAY_PATTERN or PLAY_PLAYLIST

    bool matches_day(int day) const {
        return (days_of_week & (1 << day)) != 0;
    }
};

class ScheduleManager {
public:
    static ScheduleManager& instance();

    // Initialize - loads schedule from NVS and starts the scheduler task
    esp_err_t init();

    // Shutdown - stops the scheduler task
    void shutdown();

    // Get current schedule
    const std::vector<ScheduleItem>& getSchedule() const { return schedule_; }

    // Set entire schedule (replaces existing)
    esp_err_t setSchedule(const std::vector<ScheduleItem>& items);

    // Add a single schedule item
    esp_err_t addItem(const ScheduleItem& item);

    // Clear all schedule items
    esp_err_t clearSchedule();

    // Save schedule to NVS
    esp_err_t saveSchedule();

    // Load schedule from NVS
    esp_err_t loadSchedule();

    // Convert protobuf schedule to internal format
    static std::vector<ScheduleItem> fromProto(const Kd__V1__TranquilSchedule* proto);

    // Convert internal format to protobuf (caller must manage memory)
    void toProto(Kd__V1__TranquilSchedule* proto,
                 std::vector<Kd__V1__TranquilScheduleItem>& items_storage,
                 std::vector<Kd__V1__TranquilScheduleItem*>& item_ptrs,
                 std::vector<Kd__V1__ScheduleAction>& actions_storage) const;

private:
    ScheduleManager() = default;

    // Timer callback for checking schedule
    static void timerCallback(void* arg);

    // Check current time against schedule and execute if needed
    void checkAndExecute();

    // Execute a schedule action
    void executeAction(const ScheduleItem& item);

    // Get current local time components
    bool getCurrentTime(int& day_of_week, uint32_t& seconds_since_midnight);

    std::vector<ScheduleItem> schedule_;
    esp_timer_handle_t timer_ = nullptr;
    bool initialized_ = false;

    // Track last executed time to avoid re-executing same action
    int last_executed_day_ = -1;
    uint32_t last_executed_time_ = 0xFFFFFFFF;

    // Check interval (30 seconds)
    static constexpr int64_t CHECK_INTERVAL_US = 30 * 1000 * 1000LL;

    // NVS namespace and key
    static constexpr const char* kNamespace = "tranquil_sched";
    static constexpr const char* kKeySchedule = "schedule";
};
