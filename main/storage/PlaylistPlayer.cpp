#include "PlaylistPlayer.h"
#include "esp_log.h"
#include <algorithm>
#include <random>
#include <cstring>

const char* PlaylistPlayer::TAG = "PlaylistPlayer";
bool PlaylistPlayer::initialized_ = false;
TaskHandle_t PlaylistPlayer::service_task_handle_ = nullptr;
SemaphoreHandle_t PlaylistPlayer::state_mutex_ = nullptr;
char PlaylistPlayer::current_playlist_uuid_[64] = { 0 };
std::vector<std::string> PlaylistPlayer::playlist_pattern_uuids_;
std::vector<size_t> PlaylistPlayer::playlist_order_;
size_t PlaylistPlayer::current_index_ = 0;
bool PlaylistPlayer::is_shuffle_ = false;
bool PlaylistPlayer::is_playing_ = false;

esp_err_t PlaylistPlayer::initialize() {
    if (initialized_) return ESP_OK;
    state_mutex_ = xSemaphoreCreateMutex();
    if (!state_mutex_) return ESP_ERR_NO_MEM;
    if (xTaskCreate(serviceTaskWrapper, "playlist_player", 4096, nullptr, 5, &service_task_handle_) != pdPASS) {
        vSemaphoreDelete(state_mutex_);
        return ESP_ERR_NO_MEM;
    }
    initialized_ = true;
    ESP_LOGI(TAG, "PlaylistPlayer initialized");
    return ESP_OK;
}

void PlaylistPlayer::shutdown() {
    if (!initialized_) return;
    stop(true);
    if (service_task_handle_) {
        vTaskDelete(service_task_handle_);
        service_task_handle_ = nullptr;
    }
    if (state_mutex_) {
        vSemaphoreDelete(state_mutex_);
        state_mutex_ = nullptr;
    }
    initialized_ = false;
    ESP_LOGI(TAG, "PlaylistPlayer shutdown");
}

esp_err_t PlaylistPlayer::playPlaylist(const char* playlist_uuid, bool shuffle) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_ERR_TIMEOUT;
    stop(true);
    loadPlaylist(playlist_uuid);
    is_shuffle_ = shuffle;
    if (is_shuffle_) shufflePlaylist();
    current_index_ = 0;
    is_playing_ = true;
    strncpy(current_playlist_uuid_, playlist_uuid, sizeof(current_playlist_uuid_) - 1);
    current_playlist_uuid_[sizeof(current_playlist_uuid_) - 1] = '\0';
    startCurrentPattern();
    xSemaphoreGive(state_mutex_);
    ESP_LOGI(TAG, "Started playlist: %s (shuffle=%d)", playlist_uuid, (int)shuffle);
    return ESP_OK;
}

esp_err_t PlaylistPlayer::stop(bool stop_pattern) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_ERR_TIMEOUT;
    is_playing_ = false;
    current_index_ = 0;
    playlist_pattern_uuids_.clear();
    playlist_order_.clear();
    current_playlist_uuid_[0] = '\0';
    if (stop_pattern) PatternPlayer::stop();
    xSemaphoreGive(state_mutex_);
    ESP_LOGI(TAG, "Stopped playlist");
    return ESP_OK;
}

esp_err_t PlaylistPlayer::skip() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_ERR_TIMEOUT;
    if (!is_playing_ || playlist_pattern_uuids_.empty()) {
        xSemaphoreGive(state_mutex_);
        return ESP_ERR_INVALID_STATE;
    }
    PatternPlayer::stop();
    advanceToNextPattern();
    xSemaphoreGive(state_mutex_);
    ESP_LOGI(TAG, "Skipped to next pattern");
    return ESP_OK;
}

esp_err_t PlaylistPlayer::setShuffle(bool shuffle) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_ERR_TIMEOUT;
    is_shuffle_ = shuffle;
    xSemaphoreGive(state_mutex_);
    ESP_LOGI(TAG, "Set shuffle: %d", (int)shuffle);
    return ESP_OK;
}

bool PlaylistPlayer::isShuffle() {
    return is_shuffle_;
}

cJSON* PlaylistPlayer::getStateJSON() {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "playlist_uuid", current_playlist_uuid_);
    cJSON_AddBoolToObject(root, "is_shuffle", is_shuffle_);
    cJSON_AddBoolToObject(root, "is_playing", is_playing_);
    if (is_playing_ && current_index_ < playlist_order_.size()) {
        size_t idx = playlist_order_[current_index_];
        cJSON_AddStringToObject(root, "current_pattern_uuid", playlist_pattern_uuids_[idx].c_str());
    }
    else {
        cJSON_AddStringToObject(root, "current_pattern_uuid", "");
    }
    return root;
}

const char* PlaylistPlayer::getCurrentPlaylistUUID() {
    return current_playlist_uuid_;
}

const char* PlaylistPlayer::getCurrentPatternUUID() {
    if (is_playing_ && current_index_ < playlist_order_.size()) {
        size_t idx = playlist_order_[current_index_];
        return playlist_pattern_uuids_[idx].c_str();
    }
    return "";
}

bool PlaylistPlayer::isPlaying() {
    return is_playing_;
}

void PlaylistPlayer::serviceTaskWrapper(void* param) {
    serviceTask();
}

void PlaylistPlayer::serviceTask() {
    while (true) {
        if (is_playing_ && !playlist_pattern_uuids_.empty()) {
            if (PatternPlayer::getPlaybackState() == PlaybackState::STOPPED) {
                advanceToNextPattern();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void PlaylistPlayer::loadPlaylist(const char* playlist_uuid) {
    playlist_pattern_uuids_.clear();
    playlist_order_.clear();
    current_index_ = 0;
    // Assume ManifestManager::getPlaylist returns a Playlist* with a vector<string> pattern_uuids
    std::string uuid_str(playlist_uuid);
    Playlist* playlist = ManifestManager::getPlaylist(uuid_str);
    if (playlist) {
        playlist_pattern_uuids_ = playlist->patterns;
        playlist_order_.resize(playlist_pattern_uuids_.size());
        for (size_t i = 0; i < playlist_order_.size(); ++i) playlist_order_[i] = i;
    }
}

void PlaylistPlayer::startCurrentPattern() {
    if (!is_playing_ || playlist_pattern_uuids_.empty() || current_index_ >= playlist_order_.size()) return;
    size_t idx = playlist_order_[current_index_];
    PatternPlayer::playPattern(playlist_pattern_uuids_[idx].c_str());
}

void PlaylistPlayer::advanceToNextPattern() {
    if (playlist_pattern_uuids_.empty()) return;
    current_index_++;
    if (current_index_ >= playlist_order_.size()) {
        is_playing_ = false;
        PatternPlayer::stop();
        ESP_LOGI(TAG, "Playlist finished");
        return;
    }
    startCurrentPattern();
}

void PlaylistPlayer::shufflePlaylist() {
    if (playlist_pattern_uuids_.empty()) return;
    playlist_order_.resize(playlist_pattern_uuids_.size());
    for (size_t i = 0; i < playlist_order_.size(); ++i) playlist_order_[i] = i;
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(playlist_order_.begin(), playlist_order_.end(), g);
}
