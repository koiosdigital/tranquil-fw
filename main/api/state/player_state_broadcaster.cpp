// Player state broadcaster implementation
#include "player_state_broadcaster.h"
#include "player_handler.h"
#include "response_router.h"
#include "SandTablePlayer.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <cstring>

static const char* TAG = "state_broadcaster";

PlayerStateBroadcaster& PlayerStateBroadcaster::instance() {
    static PlayerStateBroadcaster instance;
    return instance;
}

void PlayerStateBroadcaster::init() {
    if (initialized_) return;

    // Initialize cached state
    memset(&cached_state_, 0, sizeof(cached_state_));

    // Create periodic timer for state checking
    esp_timer_create_args_t timer_args = {
        .callback = timerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "player_state",
        .skip_unhandled_events = true,
    };

    esp_err_t err = esp_timer_create(&timer_args, &timer_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
        return;
    }

    err = esp_timer_start_periodic(timer_, CHECK_INTERVAL_US);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(err));
        esp_timer_delete(timer_);
        timer_ = nullptr;
        return;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Player state broadcaster initialized (poll=%lldms, local=%llds, cloud=%llds)",
        CHECK_INTERVAL_US / 1000, LOCAL_COOLDOWN_US / 1000000, CLOUD_COOLDOWN_US / 1000000);
}

void PlayerStateBroadcaster::shutdown() {
    if (!initialized_) return;

    if (timer_) {
        esp_timer_stop(timer_);
        esp_timer_delete(timer_);
        timer_ = nullptr;
    }

    initialized_ = false;
    ESP_LOGI(TAG, "Player state broadcaster shutdown");
}

void PlayerStateBroadcaster::forceImmediateBroadcast() {
    // Reset cooldowns to force immediate broadcast
    last_local_broadcast_us_ = 0;
    last_cloud_send_us_ = 0;
}

void PlayerStateBroadcaster::timerCallback(void* arg) {
    auto* self = static_cast<PlayerStateBroadcaster*>(arg);
    self->checkAndBroadcast();
}

void PlayerStateBroadcaster::checkAndBroadcast() {
    // Get current player state
    PlaybackStatus status = SandTablePlayer::getStatus();

    CachedState current = {};
    current.state = static_cast<int>(status.state);
    current.mode = static_cast<int>(status.mode);
    strncpy(current.pattern_uuid, status.current_pattern_uuid.c_str(), sizeof(current.pattern_uuid) - 1);
    strncpy(current.playlist_uuid, status.current_playlist_uuid.c_str(), sizeof(current.playlist_uuid) - 1);
    current.progress_percent = status.progress_percent;
    current.pattern_index = status.pattern_index;
    current.playlist_size = status.playlist_size;
    current.shuffle = status.is_shuffle;
    current.loop = status.is_loop;
    current.feed_rate = static_cast<float>(status.feed_rate);

    // Check if any changes
    if (current == cached_state_) {
        return;  // No changes at all
    }

    bool significant_change = current.hasSignificantChange(cached_state_);
    bool progress_only = current.hasProgressOnlyChange(cached_state_);

    int64_t now_us = esp_timer_get_time();

    // Local broadcast logic
    if (significant_change) {
        // Immediate broadcast for significant changes
        broadcastToLocal();
        last_local_broadcast_us_ = now_us;
    } else if (progress_only) {
        // Throttled broadcast for progress-only changes
        if (now_us - last_local_broadcast_us_ >= LOCAL_COOLDOWN_US) {
            broadcastToLocal();
            last_local_broadcast_us_ = now_us;
        }
    }

    // Cloud send logic
    if (significant_change) {
        // Immediate send for significant changes
        sendToCloud();
        last_cloud_send_us_ = now_us;
    } else if (progress_only) {
        // Throttled send for progress-only changes
        if (now_us - last_cloud_send_us_ >= CLOUD_COOLDOWN_US) {
            sendToCloud();
            last_cloud_send_us_ = now_us;
        }
    }

    // Update cached state
    cached_state_ = current;
}

void PlayerStateBroadcaster::broadcastToLocal() {
    ResponseMessage response = PlayerHandler::instance().buildPlayerStateResponse();
    if (response.valid()) {
        ResponseRouter::instance().broadcastToLocal(response);
        ESP_LOGD(TAG, "Broadcast PlayerState to local clients");
    }
}

void PlayerStateBroadcaster::sendToCloud() {
    ResponseMessage response = PlayerHandler::instance().buildPlayerStateResponse();
    if (response.valid()) {
        ResponseRouter::instance().forwardToCloud(response.data(), response.len());
        ESP_LOGD(TAG, "Sent PlayerState to cloud");
    }
}

// CachedState comparison operators
bool PlayerStateBroadcaster::CachedState::operator==(const CachedState& other) const {
    return state == other.state &&
           mode == other.mode &&
           strcmp(pattern_uuid, other.pattern_uuid) == 0 &&
           strcmp(playlist_uuid, other.playlist_uuid) == 0 &&
           progress_percent == other.progress_percent &&
           pattern_index == other.pattern_index &&
           playlist_size == other.playlist_size &&
           shuffle == other.shuffle &&
           loop == other.loop &&
           feed_rate == other.feed_rate;
}

bool PlayerStateBroadcaster::CachedState::hasSignificantChange(const CachedState& other) const {
    // State change, mode change, track change, playlist change, shuffle/loop change
    return state != other.state ||
           mode != other.mode ||
           strcmp(pattern_uuid, other.pattern_uuid) != 0 ||
           strcmp(playlist_uuid, other.playlist_uuid) != 0 ||
           pattern_index != other.pattern_index ||
           playlist_size != other.playlist_size ||
           shuffle != other.shuffle ||
           loop != other.loop ||
           feed_rate != other.feed_rate;
}

bool PlayerStateBroadcaster::CachedState::hasProgressOnlyChange(const CachedState& other) const {
    // Only progress changed (and nothing significant)
    return !hasSignificantChange(other) && progress_percent != other.progress_percent;
}
