// Schedule manager implementation
#include "schedule_manager.h"
#include "SandTablePlayer.h"
#include "ManifestDatabase.h"
#include "kd_pixdriver.h"

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

ScheduleManager& ScheduleManager::instance() {
    static ScheduleManager instance;
    return instance;
}

esp_err_t ScheduleManager::init() {
    if (initialized_) return ESP_OK;

    // Load saved schedule
    esp_err_t ret = loadSchedule();
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to load schedule: %s", esp_err_to_name(ret));
    }

    // Create periodic timer for schedule checking
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
    ESP_LOGI(TAG, "Schedule manager initialized with %zu items", schedule_.size());
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

esp_err_t ScheduleManager::setSchedule(const std::vector<ScheduleItem>& items) {
    schedule_ = items;

    // Sort by time for efficient checking
    std::sort(schedule_.begin(), schedule_.end(),
        [](const ScheduleItem& a, const ScheduleItem& b) {
            return a.time_of_day < b.time_of_day;
        });

    ESP_LOGI(TAG, "Schedule set with %zu items", schedule_.size());
    return saveSchedule();
}

esp_err_t ScheduleManager::addItem(const ScheduleItem& item) {
    schedule_.push_back(item);

    // Re-sort
    std::sort(schedule_.begin(), schedule_.end(),
        [](const ScheduleItem& a, const ScheduleItem& b) {
            return a.time_of_day < b.time_of_day;
        });

    return saveSchedule();
}

esp_err_t ScheduleManager::clearSchedule() {
    schedule_.clear();
    return saveSchedule();
}

esp_err_t ScheduleManager::saveSchedule() {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    // Serialize schedule to binary format:
    // [count:uint16][item1][item2]...
    // Each item: [days:uint32][time:uint32][action:uint8][uuid_len:uint8][uuid:char*]

    size_t total_size = 2;  // count
    for (const auto& item : schedule_) {
        total_size += 4 + 4 + 1 + 1 + item.uuid.size();
    }

    std::vector<uint8_t> buf(total_size);
    uint8_t* ptr = buf.data();

    uint16_t count = schedule_.size();
    memcpy(ptr, &count, 2); ptr += 2;

    for (const auto& item : schedule_) {
        memcpy(ptr, &item.days_of_week, 4); ptr += 4;
        memcpy(ptr, &item.time_of_day, 4); ptr += 4;
        *ptr++ = static_cast<uint8_t>(item.action_type);
        uint8_t uuid_len = item.uuid.size();
        *ptr++ = uuid_len;
        memcpy(ptr, item.uuid.c_str(), uuid_len); ptr += uuid_len;
    }

    ret = nvs_set_blob(handle, kKeySchedule, buf.data(), total_size);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Schedule saved (%zu items, %zu bytes)", schedule_.size(), total_size);
    } else {
        ESP_LOGE(TAG, "Failed to save schedule: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t ScheduleManager::loadSchedule() {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(kNamespace, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    size_t required_size = 0;
    ret = nvs_get_blob(handle, kKeySchedule, nullptr, &required_size);
    if (ret != ESP_OK || required_size < 2) {
        nvs_close(handle);
        return ret;
    }

    std::vector<uint8_t> buf(required_size);
    ret = nvs_get_blob(handle, kKeySchedule, buf.data(), &required_size);
    nvs_close(handle);

    if (ret != ESP_OK) {
        return ret;
    }

    // Parse binary format
    const uint8_t* ptr = buf.data();
    const uint8_t* end = ptr + required_size;

    uint16_t count;
    memcpy(&count, ptr, 2); ptr += 2;

    schedule_.clear();
    schedule_.reserve(count);

    for (uint16_t i = 0; i < count && ptr < end; i++) {
        ScheduleItem item;

        if (ptr + 10 > end) break;  // Minimum item size

        memcpy(&item.days_of_week, ptr, 4); ptr += 4;
        memcpy(&item.time_of_day, ptr, 4); ptr += 4;
        item.action_type = static_cast<Kd__V1__ScheduleAction__ScheduleActionType>(*ptr++);
        uint8_t uuid_len = *ptr++;

        if (ptr + uuid_len > end) break;

        item.uuid = std::string(reinterpret_cast<const char*>(ptr), uuid_len);
        ptr += uuid_len;

        schedule_.push_back(item);
    }

    ESP_LOGI(TAG, "Schedule loaded (%zu items)", schedule_.size());
    return ESP_OK;
}

void ScheduleManager::timerCallback(void* arg) {
    auto* self = static_cast<ScheduleManager*>(arg);
    self->checkAndExecute();
}

bool ScheduleManager::getCurrentTime(int& day_of_week, uint32_t& seconds_since_midnight) {
    time_t now;
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

void ScheduleManager::checkAndExecute() {
    if (schedule_.empty()) return;

    int day_of_week;
    uint32_t current_time;

    if (!getCurrentTime(day_of_week, current_time)) {
        // Time not set
        return;
    }

    // Find matching schedule items
    for (const auto& item : schedule_) {
        // Check if this item matches today
        if (!item.matches_day(day_of_week)) continue;

        // Check if we're within the execution window (±30 seconds of scheduled time)
        int32_t diff = static_cast<int32_t>(current_time) - static_cast<int32_t>(item.time_of_day);
        if (diff < 0 || diff > 30) continue;

        // Don't re-execute the same action
        if (day_of_week == last_executed_day_ && item.time_of_day == last_executed_time_) {
            continue;
        }

        ESP_LOGI(TAG, "Executing scheduled action: type=%d at %02u:%02u:%02u",
            item.action_type,
            item.time_of_day / 3600,
            (item.time_of_day % 3600) / 60,
            item.time_of_day % 60);

        executeAction(item);

        last_executed_day_ = day_of_week;
        last_executed_time_ = item.time_of_day;
    }
}

void ScheduleManager::executeAction(const ScheduleItem& item) {
    switch (item.action_type) {
        case KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_LED_OFF: {
            auto channel_ids = PixelDriver::getChannelIds();
            for (int32_t id : channel_ids) {
                PixelChannel* ch = PixelDriver::getChannel(id);
                if (ch) {
                    EffectConfig cfg = ch->getEffectConfig();
                    cfg.enabled = false;
                    ch->setEffect(cfg);
                }
            }
            ESP_LOGI(TAG, "Scheduled: LED off");
            break;
        }

        case KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_LED_ON: {
            auto channel_ids = PixelDriver::getChannelIds();
            for (int32_t id : channel_ids) {
                PixelChannel* ch = PixelDriver::getChannel(id);
                if (ch) {
                    EffectConfig cfg = ch->getEffectConfig();
                    cfg.enabled = true;
                    ch->setEffect(cfg);
                }
            }
            ESP_LOGI(TAG, "Scheduled: LED on");
            break;
        }

        case KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_PLAY_RANDOM_PATTERN: {
            auto patterns = ManifestDatabase::instance().getAllPatterns();
            if (!patterns.empty()) {
                size_t idx = esp_random() % patterns.size();
                // Use external_uuid for playback
                SandTablePlayer::playPattern(patterns[idx].external_uuid.c_str());
                ESP_LOGI(TAG, "Scheduled: Playing random pattern %s", patterns[idx].external_uuid.c_str());
            }
            break;
        }

        case KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_PLAY_PLAYLIST: {
            if (!item.uuid.empty()) {
                SandTablePlayer::playPlaylist(item.uuid.c_str(), false, true);
                ESP_LOGI(TAG, "Scheduled: Playing playlist %s", item.uuid.c_str());
            }
            break;
        }

        case KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_PLAY_PATTERN: {
            if (!item.uuid.empty()) {
                SandTablePlayer::playPattern(item.uuid.c_str());
                ESP_LOGI(TAG, "Scheduled: Playing pattern %s", item.uuid.c_str());
            }
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown schedule action type: %d", item.action_type);
            break;
    }
}

std::vector<ScheduleItem> ScheduleManager::fromProto(const Kd__V1__TranquilSchedule* proto) {
    std::vector<ScheduleItem> items;

    if (!proto) return items;

    items.reserve(proto->n_schedule_items);

    for (size_t i = 0; i < proto->n_schedule_items; i++) {
        const auto* proto_item = proto->schedule_items[i];
        if (!proto_item) continue;

        ScheduleItem item;
        item.days_of_week = proto_item->days_of_week;
        item.time_of_day = proto_item->time_of_day;

        if (proto_item->action) {
            item.action_type = proto_item->action->type;
            if (proto_item->action->uuid) {
                item.uuid = proto_item->action->uuid;
            }
        } else {
            item.action_type = KD__V1__SCHEDULE_ACTION__SCHEDULE_ACTION_TYPE__SCHEDULE_ACTION_TYPE_UNSPECIFIED;
        }

        items.push_back(item);
    }

    return items;
}

void ScheduleManager::toProto(
    Kd__V1__TranquilSchedule* proto,
    std::vector<Kd__V1__TranquilScheduleItem>& items_storage,
    std::vector<Kd__V1__TranquilScheduleItem*>& item_ptrs,
    std::vector<Kd__V1__ScheduleAction>& actions_storage
) const {
    items_storage.clear();
    item_ptrs.clear();
    actions_storage.clear();

    items_storage.resize(schedule_.size());
    actions_storage.resize(schedule_.size());
    item_ptrs.reserve(schedule_.size());

    for (size_t i = 0; i < schedule_.size(); i++) {
        items_storage[i] = KD__V1__TRANQUIL_SCHEDULE_ITEM__INIT;
        items_storage[i].days_of_week = schedule_[i].days_of_week;
        items_storage[i].time_of_day = schedule_[i].time_of_day;

        actions_storage[i] = KD__V1__SCHEDULE_ACTION__INIT;
        actions_storage[i].type = schedule_[i].action_type;
        actions_storage[i].uuid = const_cast<char*>(schedule_[i].uuid.c_str());

        items_storage[i].action = &actions_storage[i];
        item_ptrs.push_back(&items_storage[i]);
    }

    *proto = KD__V1__TRANQUIL_SCHEDULE__INIT;
    proto->n_schedule_items = item_ptrs.size();
    proto->schedule_items = item_ptrs.data();
}
