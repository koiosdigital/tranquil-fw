#pragma once
#include <vector>
#include <string>
#include "PatternPlayer.h"
#include "ManifestManager.h"
#include "esp_err.h"
#include "cJSON.h"

class PlaylistPlayer {
public:
    static esp_err_t initialize();
    static void shutdown();

    static esp_err_t playPlaylist(const char* playlist_uuid, bool shuffle);
    static esp_err_t stop(bool stop_pattern = true);
    static esp_err_t skip();
    static esp_err_t setShuffle(bool shuffle);
    static bool isShuffle();
    static cJSON* getStateJSON();
    static const char* getCurrentPlaylistUUID();
    static const char* getCurrentPatternUUID();
    static bool isPlaying();

private:
    static void serviceTaskWrapper(void* param);
    static void serviceTask();
    static void loadPlaylist(const char* playlist_uuid);
    static void startCurrentPattern();
    static void advanceToNextPattern();
    static void shufflePlaylist();

    static const char* TAG;
    static bool initialized_;
    static TaskHandle_t service_task_handle_;
    static SemaphoreHandle_t state_mutex_;

    static char current_playlist_uuid_[64];
    static std::vector<std::string> playlist_pattern_uuids_;
    static std::vector<size_t> playlist_order_;
    static size_t current_index_;
    static bool is_shuffle_;
    static bool is_playing_;
};
