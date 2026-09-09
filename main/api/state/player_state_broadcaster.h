// Player state broadcaster - rate-limited state updates to local/cloud clients
#pragma once

#include <esp_timer.h>
#include <atomic>
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

    // Push the current state to the cloud right now, regardless of change
    // detection or cooldowns (used when the cloud session comes up so the
    // Worker's cache never starts empty).
    void pushToCloudNow();

    // A remote client is actively watching (GetPlayerState with
    // watch_seconds from the cloud). Progress-only updates go to the cloud
    // at the fast cadence until the window expires. Extending an active
    // window is cheap and idempotent.
    void noteCloudAttention(uint32_t seconds);

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

    // Current cloud cooldown for progress-only changes
    int64_t cloudCooldownUs(int64_t now_us) const;

    // Configuration
    static constexpr int64_t CHECK_INTERVAL_US = 100 * 1000;          // 100ms poll interval
    static constexpr int64_t LOCAL_COOLDOWN_US = 1000 * 1000;         // 1s for local progress updates
    // Cloud progress-only cadence. Every frame is a Worker invocation plus a
    // D1/KV write, so idle at 15 s and only go fast while someone is watching.
    static constexpr int64_t CLOUD_COOLDOWN_IDLE_US = 15000 * 1000;   // 15s when nobody is watching
    static constexpr int64_t CLOUD_COOLDOWN_WATCHED_US = 2000 * 1000; // 2s while a remote client watches
    static constexpr uint32_t MAX_ATTENTION_S = 10 * 60;              // cap a single watch request

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
    std::atomic<int64_t> cloud_attention_until_us_{ 0 };
    bool initialized_ = false;
};
