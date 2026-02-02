// Player state broadcaster - rate-limited state updates to local/cloud clients
#pragma once

#include <esp_timer.h>
#include <cstdint>

class PlayerStateBroadcaster {
public:
    static PlayerStateBroadcaster& instance();

    // Initialize the broadcaster (start periodic timer)
    void init();

    // Shutdown the broadcaster
    void shutdown();

    // Force an immediate broadcast (e.g., after handling a state-changing command)
    void forceImmediateBroadcast();

private:
    PlayerStateBroadcaster() = default;

    // Timer callback for periodic state checking
    static void timerCallback(void* arg);

    // Check state and broadcast if needed
    void checkAndBroadcast();

    // Broadcast to local WebSocket clients
    void broadcastToLocal();

    // Send to cloud WebSocket
    void sendToCloud();

    // Configuration
    static constexpr int64_t CHECK_INTERVAL_US = 100 * 1000;    // 100ms poll interval
    static constexpr int64_t LOCAL_COOLDOWN_US = 1000 * 1000;   // 1s for local progress updates
    static constexpr int64_t CLOUD_COOLDOWN_US = 5000 * 1000;   // 5s for cloud progress updates

    // State tracking for change detection
    struct CachedState {
        int state;              // PlaybackState enum
        int mode;               // PlayMode enum
        char pattern_uuid[64];
        char playlist_uuid[64];
        int progress_percent;
        size_t pattern_index;
        size_t playlist_size;
        bool shuffle;
        bool loop;
        float feed_rate;

        bool operator==(const CachedState& other) const;
        bool hasSignificantChange(const CachedState& other) const;  // State/mode/track changed
        bool hasProgressOnlyChange(const CachedState& other) const; // Only progress changed
    };

    CachedState cached_state_ = {};
    esp_timer_handle_t timer_ = nullptr;
    int64_t last_local_broadcast_us_ = 0;
    int64_t last_cloud_send_us_ = 0;
    bool initialized_ = false;
};
