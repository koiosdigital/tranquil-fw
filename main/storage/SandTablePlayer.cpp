#include "SandTablePlayer.h"
#include "drm/drm_license.h"
#include "drm/drm_purchase.h"
#include "raii_utils.hpp"
#include "esp_log.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <random>

// Static member definitions
const char* SandTablePlayer::TAG = "SandTablePlayer";

sand_table::MotionController* SandTablePlayer::motion_controller_ = nullptr;
bool SandTablePlayer::initialized_ = false;
TaskHandle_t SandTablePlayer::service_task_handle_ = nullptr;
SemaphoreHandle_t SandTablePlayer::state_mutex_ = nullptr;
SemaphoreHandle_t SandTablePlayer::file_mutex_ = nullptr;
std::atomic<bool> SandTablePlayer::shutdown_requested_{ false };
std::atomic<bool> SandTablePlayer::service_task_exited_{ false };

PlaybackState SandTablePlayer::playback_state_ = PlaybackState::STOPPED;
PlayMode SandTablePlayer::play_mode_ = PlayMode::SINGLE_PATTERN;

char SandTablePlayer::current_pattern_uuid_[MAX_UUID_LEN] = { 0 };
std::optional<Pattern> SandTablePlayer::current_pattern_ = std::nullopt;
std::unique_ptr<IPatternReader> SandTablePlayer::pattern_reader_;
size_t SandTablePlayer::current_line_index_ = 0;
size_t SandTablePlayer::total_lines_ = 0;
bool SandTablePlayer::file_loaded_ = false;

char SandTablePlayer::current_playlist_uuid_[MAX_UUID_LEN] = { 0 };
std::vector<std::string> SandTablePlayer::playlist_patterns_;
std::vector<size_t> SandTablePlayer::playlist_order_;
size_t SandTablePlayer::playlist_index_ = 0;
bool SandTablePlayer::is_shuffle_ = false;
bool SandTablePlayer::is_loop_ = false;

PatternPosition SandTablePlayer::pattern_position_;
std::atomic<double> SandTablePlayer::feed_rate_{ 5.0 };
std::atomic<bool> SandTablePlayer::linear_interpolation_{ false };

// =============================================================================
// Initialization / Shutdown
// =============================================================================

esp_err_t SandTablePlayer::initialize(sand_table::MotionController* controller) {
    if (initialized_) return ESP_OK;

    if (!controller) {
        ESP_LOGE(TAG, "Motion controller is null");
        return ESP_ERR_INVALID_ARG;
    }

    motion_controller_ = controller;

    // Create mutexes for thread safety
    state_mutex_ = xSemaphoreCreateMutex();
    file_mutex_ = xSemaphoreCreateMutex();
    if (!state_mutex_ || !file_mutex_) {
        ESP_LOGE(TAG, "Failed to create mutexes");
        if (state_mutex_) vSemaphoreDelete(state_mutex_);
        if (file_mutex_) vSemaphoreDelete(file_mutex_);
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(serviceTaskWrapper, "sand_player", SERVICE_TASK_STACK_SIZE,
        nullptr, SERVICE_TASK_PRIORITY, &service_task_handle_) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create service task");
        vSemaphoreDelete(state_mutex_);
        vSemaphoreDelete(file_mutex_);
        return ESP_ERR_NO_MEM;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "SandTablePlayer initialized");
    return ESP_OK;
}

void SandTablePlayer::shutdown() {
    if (!initialized_) return;

    ESP_LOGI(TAG, "Shutting down SandTablePlayer");

    stop();

    // Ask the service task to exit and wait for it to acknowledge —
    // vTaskDelete on a task that holds file_mutex_/state_mutex_ would
    // leave the mutex in an undefined state.
    if (service_task_handle_) {
        shutdown_requested_ = true;
        for (int i = 0; i < 100 && !service_task_exited_; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (!service_task_exited_) {
            ESP_LOGW(TAG, "Service task did not exit, force-deleting");
            vTaskDelete(service_task_handle_);
        }
        service_task_handle_ = nullptr;
    }

    // Release the pattern file BEFORE deleting the mutex it is guarded by —
    // with file_mutex_ already gone, unloadPatternFile() would no-op and
    // leak the file handle.
    unloadPatternFile();

    if (state_mutex_) {
        vSemaphoreDelete(state_mutex_);
        state_mutex_ = nullptr;
    }
    if (file_mutex_) {
        vSemaphoreDelete(file_mutex_);
        file_mutex_ = nullptr;
    }

    motion_controller_ = nullptr;
    initialized_ = false;
    shutdown_requested_ = false;
    service_task_exited_ = false;

    ESP_LOGI(TAG, "SandTablePlayer shutdown complete");
}

// =============================================================================
// Pattern Playback
// =============================================================================

esp_err_t SandTablePlayer::playPattern(const char* pattern_uuid) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (!motion_controller_->is_homed()) return ESP_ERR_INVALID_STATE;

    // Check authorization for encrypted patterns before acquiring mutex
    // pattern_uuid is the external_uuid, so use getPatternByExternalUuid
    auto pattern_info = ManifestDatabase::instance().getPatternByExternalUuid(pattern_uuid);
    if (pattern_info.has_value() && pattern_info->encrypted) {
        // Check purchase receipt OR subscription license
        if (!drm_purchase_is_valid(pattern_uuid) && !drm_license_is_valid()) {
            ESP_LOGW(TAG, "Cannot play encrypted pattern: no valid authorization");
            return ESP_ERR_NOT_ALLOWED;
        }
    }

    raii::MutexGuard guard(state_mutex_, pdMS_TO_TICKS(1000));
    if (!guard) {
        ESP_LOGE(TAG, "Failed to acquire state mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Stop current playback if any — and actually drain queued motion.
    // Setting the state alone left already-planned segments executing, so
    // the old pattern kept drawing under the new one.
    if (playback_state_ != PlaybackState::STOPPED) {
        playback_state_ = PlaybackState::STOPPED;
        motion_controller_->halt_and_drain();
    }

    // Load the pattern file
    esp_err_t ret = loadPatternFile(pattern_uuid);
    if (ret != ESP_OK) {
        return ret;
    }

    // Get pattern info from ManifestDatabase (pattern_uuid is external_uuid)
    current_pattern_ = ManifestDatabase::instance().getPatternByExternalUuid(pattern_uuid);
    if (!current_pattern_.has_value()) {
        ESP_LOGE(TAG, "Pattern not found in manifest: %s", pattern_uuid);
        unloadPatternFile();
        return ESP_ERR_NOT_FOUND;
    }

    strncpy(current_pattern_uuid_, pattern_uuid, MAX_UUID_LEN - 1);
    current_pattern_uuid_[MAX_UUID_LEN - 1] = '\0';

    // Reset motion progress counters for accurate progress tracking
    motion_controller_->reset_progress_counters();

    play_mode_ = PlayMode::SINGLE_PATTERN;
    playback_state_ = PlaybackState::PLAYING;
    motion_controller_->resume();  // Ensure not paused

    ESP_LOGI(TAG, "Started playing pattern: %s (%s)",
        current_pattern_->name.c_str(), pattern_uuid);
    return ESP_OK;
}

// =============================================================================
// Playlist Playback
// =============================================================================

esp_err_t SandTablePlayer::playPlaylist(const char* playlist_uuid, bool shuffle, bool loop) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (!motion_controller_->is_homed()) return ESP_ERR_INVALID_STATE;

    raii::MutexGuard guard(state_mutex_, pdMS_TO_TICKS(1000));
    if (!guard) {
        return ESP_ERR_TIMEOUT;
    }

    // Stop current playback (drain queued motion too — see playPattern)
    if (playback_state_ != PlaybackState::STOPPED) {
        playback_state_ = PlaybackState::STOPPED;
        motion_controller_->halt_and_drain();
        unloadPatternFile();
    }

    // Load the playlist
    loadPlaylist(playlist_uuid);
    if (playlist_patterns_.empty()) {
        ESP_LOGE(TAG, "Playlist is empty or not found: %s", playlist_uuid);
        return ESP_ERR_NOT_FOUND;
    }

    strncpy(current_playlist_uuid_, playlist_uuid, MAX_UUID_LEN - 1);
    current_playlist_uuid_[MAX_UUID_LEN - 1] = '\0';

    is_shuffle_ = shuffle;
    is_loop_ = loop;

    if (is_shuffle_) {
        shufflePlaylistOrder();
    }

    playlist_index_ = 0;
    play_mode_ = is_loop_ ? PlayMode::PLAYLIST_LOOP :
        (is_shuffle_ ? PlayMode::PLAYLIST_SHUFFLE : PlayMode::PLAYLIST);
    playback_state_ = PlaybackState::PLAYING;

    startCurrentPattern();

    ESP_LOGI(TAG, "Started playlist: %s (shuffle=%d, loop=%d)",
        playlist_uuid, (int)shuffle, (int)loop);
    return ESP_OK;
}

esp_err_t SandTablePlayer::playPlaylistFromPattern(const char* playlist_uuid, const char* pattern_uuid, bool shuffle, bool loop) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (!motion_controller_->is_homed()) return ESP_ERR_INVALID_STATE;

    raii::MutexGuard guard(state_mutex_, pdMS_TO_TICKS(1000));
    if (!guard) {
        return ESP_ERR_TIMEOUT;
    }

    // Stop current playback (drain queued motion too — see playPattern)
    if (playback_state_ != PlaybackState::STOPPED) {
        playback_state_ = PlaybackState::STOPPED;
        motion_controller_->halt_and_drain();
        unloadPatternFile();
    }

    // Load the playlist
    loadPlaylist(playlist_uuid);
    if (playlist_patterns_.empty()) {
        ESP_LOGE(TAG, "Playlist is empty or not found: %s", playlist_uuid);
        return ESP_ERR_NOT_FOUND;
    }

    strncpy(current_playlist_uuid_, playlist_uuid, MAX_UUID_LEN - 1);
    current_playlist_uuid_[MAX_UUID_LEN - 1] = '\0';

    is_shuffle_ = shuffle;
    is_loop_ = loop;

    // Find the pattern index in the original list
    size_t start_index = 0;
    bool found = false;
    for (size_t i = 0; i < playlist_patterns_.size(); ++i) {
        if (playlist_patterns_[i] == pattern_uuid) {
            start_index = i;
            found = true;
            break;
        }
    }

    if (!found) {
        ESP_LOGE(TAG, "Pattern %s not found in playlist %s", pattern_uuid, playlist_uuid);
        return ESP_ERR_NOT_FOUND;
    }

    // Even if shuffle is enabled, we start at the specified pattern
    // Build order starting from that pattern
    playlist_order_.clear();
    playlist_order_.push_back(start_index);

    if (is_shuffle_) {
        // Add remaining patterns in shuffled order
        std::vector<size_t> remaining;
        for (size_t i = 0; i < playlist_patterns_.size(); ++i) {
            if (i != start_index) {
                remaining.push_back(i);
            }
        }
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(remaining.begin(), remaining.end(), g);
        for (size_t idx : remaining) {
            playlist_order_.push_back(idx);
        }
    }
    else {
        // Sequential from start pattern
        for (size_t i = 1; i < playlist_patterns_.size(); ++i) {
            playlist_order_.push_back((start_index + i) % playlist_patterns_.size());
        }
    }

    playlist_index_ = 0;
    play_mode_ = is_loop_ ? PlayMode::PLAYLIST_LOOP :
        (is_shuffle_ ? PlayMode::PLAYLIST_SHUFFLE : PlayMode::PLAYLIST);
    playback_state_ = PlaybackState::PLAYING;

    startCurrentPattern();

    ESP_LOGI(TAG, "Started playlist: %s from pattern %s (shuffle=%d, loop=%d)",
        playlist_uuid, pattern_uuid, (int)shuffle, (int)loop);
    return ESP_OK;
}

// =============================================================================
// Playback Control
// =============================================================================

esp_err_t SandTablePlayer::pause() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    raii::MutexGuard guard(state_mutex_, pdMS_TO_TICKS(1000));
    if (!guard) {
        return ESP_ERR_TIMEOUT;
    }

    if (playback_state_ == PlaybackState::PLAYING) {
        motion_controller_->pause();
        playback_state_ = PlaybackState::PAUSED;
        ESP_LOGI(TAG, "Playback paused");
    }

    return ESP_OK;
}

esp_err_t SandTablePlayer::resume() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    raii::MutexGuard guard(state_mutex_, pdMS_TO_TICKS(1000));
    if (!guard) {
        return ESP_ERR_TIMEOUT;
    }

    if (playback_state_ == PlaybackState::PAUSED) {
        motion_controller_->resume();
        playback_state_ = PlaybackState::PLAYING;
        ESP_LOGI(TAG, "Playback resumed");
    }

    return ESP_OK;
}

esp_err_t SandTablePlayer::stop() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    raii::MutexGuard guard(state_mutex_, pdMS_TO_TICKS(1000));
    if (!guard) {
        return ESP_ERR_TIMEOUT;
    }

    playback_state_ = PlaybackState::STOPPED;
    play_mode_ = PlayMode::SINGLE_PATTERN;

    // Clear pattern state
    unloadPatternFile();
    memset(current_pattern_uuid_, 0, sizeof(current_pattern_uuid_));
    current_pattern_ = std::nullopt;

    // Clear playlist state
    memset(current_playlist_uuid_, 0, sizeof(current_playlist_uuid_));
    playlist_patterns_.clear();
    playlist_order_.clear();
    playlist_index_ = 0;
    is_shuffle_ = false;
    is_loop_ = false;

    guard.release();

    ESP_LOGI(TAG, "Playback stopped");

    // Normal stop: drain queued motion and resync the planner position.
    // The old emergency_stop()+clear pair raced the stepper task's 10ms
    // poll — the clear usually landed before the task saw the flag, so
    // queued segments survived the "stop" and kept drawing.
    motion_controller_->halt_and_drain();
    return ESP_OK;
}

esp_err_t SandTablePlayer::emergencyStop() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    // Immediate halt - don't wait for mutex, just stop motion
    motion_controller_->emergency_stop();

    // Now acquire mutex to update state
    {
        raii::MutexGuard guard(state_mutex_, pdMS_TO_TICKS(1000));
        if (guard) {
            playback_state_ = PlaybackState::STOPPED;
            play_mode_ = PlayMode::SINGLE_PATTERN;
            unloadPatternFile();
            memset(current_pattern_uuid_, 0, sizeof(current_pattern_uuid_));
            current_pattern_ = std::nullopt;
            memset(current_playlist_uuid_, 0, sizeof(current_playlist_uuid_));
            playlist_patterns_.clear();
            playlist_order_.clear();
            playlist_index_ = 0;
            is_shuffle_ = false;
            is_loop_ = false;
        }
    }

    // Drain deterministically before clearing: clearing the e-stop flag
    // immediately after setting it raced the stepper task's poll, leaving
    // queued segments to execute after the "stop".
    motion_controller_->halt_and_drain();
    motion_controller_->clear_emergency_stop();
    ESP_LOGW(TAG, "Emergency stop executed");
    return ESP_OK;
}

esp_err_t SandTablePlayer::skip() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    raii::MutexGuard guard(state_mutex_, pdMS_TO_TICKS(1000));
    if (!guard) {
        return ESP_ERR_TIMEOUT;
    }

    // Only works in playlist mode
    if (play_mode_ == PlayMode::SINGLE_PATTERN || playlist_patterns_.empty()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Stop current pattern
    unloadPatternFile();

    // Advance to next
    advanceToNextPattern();

    ESP_LOGI(TAG, "Skipped to next pattern");
    return ESP_OK;
}

// =============================================================================
// Configuration
// =============================================================================

void SandTablePlayer::setFeedRate(double feed_rate) {
    if (feed_rate < 1.0) feed_rate = 1.0;
    if (feed_rate > 20.0) feed_rate = 20.0;
    feed_rate_.store(feed_rate, std::memory_order_relaxed);
    ESP_LOGI(TAG, "Set feed rate: %.2f", feed_rate);
}

double SandTablePlayer::getFeedRate() {
    return feed_rate_.load(std::memory_order_relaxed);
}

void SandTablePlayer::setLinearInterpolation(bool enabled) {
    linear_interpolation_.store(enabled, std::memory_order_relaxed);
    ESP_LOGI(TAG, "Linear interpolation: %s", enabled ? "on" : "off");
}

bool SandTablePlayer::isLinearInterpolation() {
    return linear_interpolation_.load(std::memory_order_relaxed);
}

esp_err_t SandTablePlayer::setShuffle(bool shuffle) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    raii::MutexGuard guard(state_mutex_, pdMS_TO_TICKS(1000));
    if (!guard) {
        return ESP_ERR_TIMEOUT;
    }

    is_shuffle_ = shuffle;
    if (play_mode_ != PlayMode::SINGLE_PATTERN) {
        play_mode_ = is_loop_ ? PlayMode::PLAYLIST_LOOP :
            (is_shuffle_ ? PlayMode::PLAYLIST_SHUFFLE : PlayMode::PLAYLIST);
    }

    guard.release();
    ESP_LOGI(TAG, "Set shuffle: %d", (int)shuffle);
    return ESP_OK;
}

bool SandTablePlayer::isShuffle() {
    return is_shuffle_;
}

esp_err_t SandTablePlayer::setLoop(bool loop) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    raii::MutexGuard guard(state_mutex_, pdMS_TO_TICKS(1000));
    if (!guard) {
        return ESP_ERR_TIMEOUT;
    }

    is_loop_ = loop;
    if (play_mode_ != PlayMode::SINGLE_PATTERN) {
        play_mode_ = is_loop_ ? PlayMode::PLAYLIST_LOOP :
            (is_shuffle_ ? PlayMode::PLAYLIST_SHUFFLE : PlayMode::PLAYLIST);
    }

    guard.release();
    ESP_LOGI(TAG, "Set loop: %d", (int)loop);
    return ESP_OK;
}

bool SandTablePlayer::isLoop() {
    return is_loop_;
}

// =============================================================================
// Status
// =============================================================================

PlaybackState SandTablePlayer::getPlaybackState() {
    return playback_state_;
}

PlayMode SandTablePlayer::getPlayMode() {
    return play_mode_;
}

PlaybackStatus SandTablePlayer::getStatus() {
    PlaybackStatus status;
    // Reading the uuid buffers / playlist vector while another task mutates
    // them under state_mutex_ is a data race — snapshot under the mutex.
    raii::MutexGuard guard(state_mutex_, pdMS_TO_TICKS(100));
    status.state = playback_state_;
    status.mode = play_mode_;
    if (guard) {
        status.current_pattern_uuid = current_pattern_uuid_;
        status.current_playlist_uuid = current_playlist_uuid_;
        status.pattern_index = playlist_index_;
        status.playlist_size = playlist_patterns_.size();
    }
    else {
        status.pattern_index = 0;
        status.playlist_size = 0;
    }
    status.progress_percent = getTotalProgress();
    status.feed_rate = feed_rate_;
    status.is_shuffle = is_shuffle_;
    status.is_loop = is_loop_;
    return status;
}

cJSON* SandTablePlayer::getStateJSON() {
    cJSON* root = cJSON_CreateObject();
    if (!root) return nullptr;

    // Snapshot mutable state under the mutex (see getStatus).
    raii::MutexGuard guard(state_mutex_, pdMS_TO_TICKS(100));

    const char* state_str = "STOPPED";
    switch (playback_state_) {
    case PlaybackState::PLAYING: state_str = "PLAYING"; break;
    case PlaybackState::PAUSED: state_str = "PAUSED"; break;
    default: break;
    }
    cJSON_AddStringToObject(root, "playback_state", state_str);

    const char* mode_str = "SINGLE_PATTERN";
    switch (play_mode_) {
    case PlayMode::PLAYLIST: mode_str = "PLAYLIST"; break;
    case PlayMode::PLAYLIST_LOOP: mode_str = "PLAYLIST_LOOP"; break;
    case PlayMode::PLAYLIST_SHUFFLE: mode_str = "PLAYLIST_SHUFFLE"; break;
    default: break;
    }
    cJSON_AddStringToObject(root, "play_mode", mode_str);

    cJSON_AddStringToObject(root, "pattern_uuid", current_pattern_uuid_);
    cJSON_AddStringToObject(root, "playlist_uuid", current_playlist_uuid_);
    cJSON_AddNumberToObject(root, "progress", getTotalProgress());
    cJSON_AddNumberToObject(root, "pattern_index", static_cast<int>(playlist_index_));
    cJSON_AddNumberToObject(root, "playlist_size", static_cast<int>(playlist_patterns_.size()));
    cJSON_AddNumberToObject(root, "feed_rate", feed_rate_);
    cJSON_AddBoolToObject(root, "is_shuffle", is_shuffle_);
    cJSON_AddBoolToObject(root, "is_loop", is_loop_);

    return root;
}

int SandTablePlayer::getTotalProgress() {
    if (!file_loaded_ || total_lines_ == 0) return 0;

    // Line-based progress. The old steps_completed/steps_queued ratio only
    // measured the buffered window (queued runs at most ~100 segments ahead
    // of completed), so it read ~100% almost immediately regardless of how
    // far into the pattern playback actually was. Lines fed lead physical
    // motion by at most that same buffered window, so this is accurate to a
    // few percent and monotonic.
    int percent = static_cast<int>(
        (static_cast<double>(current_line_index_) / static_cast<double>(total_lines_)) * 100.0);
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    return percent;
}

bool SandTablePlayer::isHomed() {
    return motion_controller_ && motion_controller_->is_homed();
}

sand_table::MotionController* SandTablePlayer::getMotionController() {
    return motion_controller_;
}

const Pattern* SandTablePlayer::getCurrentPattern() {
    return current_pattern_.has_value() ? &current_pattern_.value() : nullptr;
}

const char* SandTablePlayer::getCurrentPatternUUID() {
    return current_pattern_uuid_;
}

const char* SandTablePlayer::getCurrentPlaylistUUID() {
    return current_playlist_uuid_;
}

bool SandTablePlayer::isPatternLoaded() {
    return file_loaded_;
}

// =============================================================================
// Service Task
// =============================================================================

void SandTablePlayer::serviceTaskWrapper(void* param) {
    (void)param;
    serviceTask();
}

void SandTablePlayer::serviceTask() {
    ESP_LOGI(TAG, "Service task started");

    while (!shutdown_requested_) {
        // Snapshot playback state under the mutex. The line-feeding path
        // below runs WITHOUT state_mutex_ held because processPatternLine's
        // error paths call stop(), which takes it (non-recursive).
        bool playing = false;
        {
            raii::MutexGuard guard(state_mutex_, pdMS_TO_TICKS(100));
            if (guard) {
                playing = (playback_state_ == PlaybackState::PLAYING && file_loaded_);
            }
        }

        if (playing) {
            if (hasMoreLines()) {
                // Pump as many lines as the motion pipeline will take this
                // tick. Feeding a single line per 10ms tick capped throughput
                // at 100 points/s and starved the queue on dense patterns
                // (motors idled between points). processPatternLine returns
                // false when the queue is full; move_to itself applies
                // backpressure when the velocity planner fills.
                int fed = 0;
                while (fed < kMaxLinesPerTick && hasMoreLines()) {
                    // Bail out promptly if an API thread stopped/paused us
                    // (unlocked read — just an exit hint, the next tick's
                    // locked snapshot is authoritative)
                    if (playback_state_ != PlaybackState::PLAYING) break;

                    PatternLine line = peekNextLine();
                    ESP_LOGI(TAG, "Feeding line %zu/%zu: theta=%.4f, rho=%.4f, first=%d",
                        current_line_index_ + 1, total_lines_, line.theta, line.rho, line.is_first_line);
                    if (line.is_valid) {
                        if (!processPatternLine(line)) {
                            break;  // queue full or motion refused - retry next tick
                        }
                        popLine();
                        fed++;
                    }
                    else {
                        ESP_LOGW(TAG, "Invalid line, skipping");
                        popLine();
                    }
                }
            }
            else {
                // All lines queued - wait for motion to complete before advancing.
                // planner_pending covers segments still in the velocity planner
                // that haven't reached the execution queue yet.
                auto progress = motion_controller_->get_motion_progress();
                if (!progress.is_executing && progress.segments_queued == 0 &&
                    progress.planner_pending == 0) {
                    // Take state_mutex_ before touching current_pattern_ /
                    // playlist vectors — an API thread's stop()/play*() mutates
                    // the same std::string/vector state under this mutex, and
                    // unlocked concurrent writes corrupt the heap. Re-check
                    // the state after acquiring: it may have changed.
                    raii::MutexGuard guard(state_mutex_, pdMS_TO_TICKS(1000));
                    if (guard && playback_state_ == PlaybackState::PLAYING &&
                        file_loaded_ && !hasMoreLines()) {
                        ESP_LOGI(TAG, "Pattern finished");

                        // Handle playlist mode
                        if (play_mode_ != PlayMode::SINGLE_PATTERN && !playlist_patterns_.empty()) {
                            advanceToNextPattern();
                        }
                        else {
                            playback_state_ = PlaybackState::STOPPED;
                        }
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SERVICE_TASK_DELAY_MS));
    }

    // Shutdown handshake — see SandTablePlayer::shutdown().
    ESP_LOGI(TAG, "Service task exiting");
    service_task_exited_ = true;
    vTaskDelete(nullptr);
}

// =============================================================================
// Pattern File Handling
// =============================================================================

esp_err_t SandTablePlayer::loadPatternFile(const char* pattern_uuid) {
    raii::MutexGuard guard(file_mutex_, pdMS_TO_TICKS(1000));
    if (!guard) {
        ESP_LOGE(TAG, "Failed to acquire file mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Unload any existing reader first (without mutex, we already hold it)
    if (pattern_reader_) {
        pattern_reader_->close();
        pattern_reader_.reset();
    }
    current_line_index_ = 0;
    total_lines_ = 0;
    file_loaded_ = false;

    // Get pattern metadata to determine if encrypted (pattern_uuid is external_uuid)
    auto pattern = ManifestDatabase::instance().getPatternByExternalUuid(pattern_uuid);
    bool is_encrypted = pattern.has_value() ? pattern->encrypted : false;

    // Create the appropriate reader via factory function
    pattern_reader_ = createPatternReader(is_encrypted);

    // Open the pattern by UUID (reader handles path internally)
    esp_err_t ret = pattern_reader_->open(pattern_uuid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open pattern: %s (err=%s)", pattern_uuid, esp_err_to_name(ret));
        pattern_reader_.reset();
        return ret;  // Propagates ESP_ERR_NOT_ALLOWED for license issues
    }

    total_lines_ = pattern_reader_->getTotalLines();
    current_line_index_ = 0;
    file_loaded_ = true;

    ESP_LOGI(TAG, "Loaded pattern: %s (%zu points, encrypted=%d)",
        pattern_uuid, total_lines_, is_encrypted);

    return ESP_OK;
}

void SandTablePlayer::unloadPatternFile() {
    raii::MutexGuard guard(file_mutex_, pdMS_TO_TICKS(1000));
    if (!guard) {
        ESP_LOGE(TAG, "Failed to acquire file mutex");
        return;
    }

    if (pattern_reader_) {
        pattern_reader_->close();
        pattern_reader_.reset();
    }
    current_line_index_ = 0;
    total_lines_ = 0;
    file_loaded_ = false;
}

// NOTE: peekNextLine/popLine/hasMoreLines must do the pattern_reader_ null
// check INSIDE file_mutex_ — an API thread's stop() -> unloadPatternFile()
// destroys the reader under that mutex, so an unlocked check (or an
// unlocked virtual call) races it: check-then-use becomes a null deref and
// a call on a destroyed object becomes a use-after-free.

PatternLine SandTablePlayer::peekNextLine() {
    PatternPoint point;
    bool is_first;
    {
        raii::MutexGuard guard(file_mutex_, pdMS_TO_TICKS(1000));
        if (!guard) {
            ESP_LOGE(TAG, "Failed to acquire file mutex");
            return PatternLine();
        }
        if (!file_loaded_ || !pattern_reader_) return PatternLine();

        point = pattern_reader_->peekNext();
        is_first = (current_line_index_ == 0);
    }

    if (!point.valid) {
        return PatternLine();
    }

    return PatternLine(point.theta, point.rho, is_first);
}

void SandTablePlayer::popLine() {
    raii::MutexGuard guard(file_mutex_, pdMS_TO_TICKS(1000));
    if (!guard) {
        ESP_LOGE(TAG, "Failed to acquire file mutex");
        return;
    }
    if (!file_loaded_ || !pattern_reader_) return;
    // Advance the reader position
    pattern_reader_->readNext();
    current_line_index_++;
}

bool SandTablePlayer::hasMoreLines() {
    raii::MutexGuard guard(file_mutex_, pdMS_TO_TICKS(1000));
    if (!guard) {
        return false;
    }
    return file_loaded_ && pattern_reader_ && pattern_reader_->hasMore();
}

// =============================================================================
// Playlist Management
// =============================================================================

void SandTablePlayer::loadPlaylist(const char* playlist_uuid) {
    playlist_patterns_.clear();
    playlist_order_.clear();
    playlist_index_ = 0;

    // playlist_uuid is external_uuid
    auto playlist = ManifestDatabase::instance().getPlaylistByExternalUuid(playlist_uuid);
    if (playlist.has_value()) {
        // Convert pattern IDs to external UUIDs for playback
        for (uint32_t pattern_id : playlist->pattern_ids) {
            auto pattern = ManifestDatabase::instance().getPattern(pattern_id);
            if (pattern) {
                playlist_patterns_.push_back(pattern->external_uuid);
            }
        }
        playlist_order_.resize(playlist_patterns_.size());
        for (size_t i = 0; i < playlist_order_.size(); ++i) {
            playlist_order_[i] = i;
        }
    }
}

void SandTablePlayer::startCurrentPattern() {
    if (playlist_patterns_.empty() || playlist_index_ >= playlist_order_.size()) {
        return;
    }

    size_t idx = playlist_order_[playlist_index_];
    const std::string& pattern_uuid = playlist_patterns_[idx];

    // Check authorization for encrypted patterns (pattern_uuid is external_uuid)
    auto pattern_info = ManifestDatabase::instance().getPatternByExternalUuid(pattern_uuid);
    if (pattern_info.has_value() && pattern_info->encrypted) {
        // Check purchase receipt OR subscription license
        if (!drm_purchase_is_valid(pattern_uuid.c_str()) && !drm_license_is_valid()) {
            ESP_LOGW(TAG, "Skipping encrypted pattern (no valid authorization): %s", pattern_uuid.c_str());
            advanceToNextPattern();
            return;
        }
    }

    // Load the pattern file
    esp_err_t ret = loadPatternFile(pattern_uuid.c_str());
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load pattern: %s", pattern_uuid.c_str());
        advanceToNextPattern();
        return;
    }

    // Get pattern info
    current_pattern_ = ManifestDatabase::instance().getPatternByExternalUuid(pattern_uuid);
    strncpy(current_pattern_uuid_, pattern_uuid.c_str(), MAX_UUID_LEN - 1);
    current_pattern_uuid_[MAX_UUID_LEN - 1] = '\0';

    // Reset motion progress counters for accurate progress tracking
    motion_controller_->reset_progress_counters();

    // Reset pattern position
    pattern_position_ = PatternPosition();

    ESP_LOGI(TAG, "Starting pattern %zu/%zu: %s",
        playlist_index_ + 1, playlist_patterns_.size(), pattern_uuid.c_str());
}

void SandTablePlayer::advanceToNextPattern() {
    if (playlist_patterns_.empty()) return;

    playlist_index_++;

    if (playlist_index_ >= playlist_order_.size()) {
        if (is_loop_) {
            // Loop: reshuffle if needed and restart
            playlist_index_ = 0;
            if (is_shuffle_) {
                shufflePlaylistOrder();
            }
            ESP_LOGI(TAG, "Playlist looping");
        }
        else {
            // End of playlist
            playback_state_ = PlaybackState::STOPPED;
            ESP_LOGI(TAG, "Playlist finished");
            return;
        }
    }

    startCurrentPattern();
}

void SandTablePlayer::shufflePlaylistOrder() {
    if (playlist_patterns_.empty()) return;

    playlist_order_.resize(playlist_patterns_.size());
    for (size_t i = 0; i < playlist_order_.size(); ++i) {
        playlist_order_[i] = i;
    }

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(playlist_order_.begin(), playlist_order_.end(), g);

    ESP_LOGI(TAG, "Shuffled playlist order");
}

// =============================================================================
// Motion Processing
// =============================================================================

bool SandTablePlayer::processPatternLine(const PatternLine& line) {
    if (!line.is_valid) return false;

    constexpr double kTwoPi = 2.0 * M_PI;

    if (line.is_first_line) {
        // First line: record starting position, no move needed.
        // Rebase the pattern's absolute theta so its first point lands
        // within half a turn of where the table already is (same angle
        // modulo 2π) — files can open at an arbitrarily wound-up theta,
        // and without this the first move unwinds all of it physically.
        const double current = motion_controller_->get_position().theta;
        const double adjusted = current + std::remainder(line.theta - current, kTwoPi);
        pattern_position_.theta_offset = line.theta - adjusted;
        pattern_position_.prev_theta = adjusted;
        pattern_position_.prev_rho = line.rho;
        return true;
    }

    // Check if motion controller can accept command
    auto status = motion_controller_->get_status();
    if (!status.is_homed) {
        ESP_LOGE(TAG, "Motion controller not homed, stopping");
        stop();
        return false;
    }
    if (status.queue_depth >= status.queue_capacity) {
        // Queue full, don't pop line yet - will retry
        return false;
    }

    const double theta = line.theta - pattern_position_.theta_offset;

    // Multi-rotation jumps between consecutive lines are played IN FULL -
    // every authored rotation is executed (patterns use these as sweeps /
    // deliberate erases) - but at a speed boost: they are transits, not
    // fine drawing, and at draw speed a many-rotation jump takes forever.
    float speed_multiplier = 1.0f;
    const double delta = theta - pattern_position_.prev_theta;
    if (std::fabs(delta) >= kTwoPi) {
        speed_multiplier = kFoldedMoveSpeedMultiplier;
        ESP_LOGI(TAG, "Multi-rotation jump at line %zu (%.1f rotations) - playing in full at %gx speed",
            current_line_index_ + 1, std::fabs(delta) / kTwoPi,
            static_cast<double>(speed_multiplier));
    }

    // Send move directly to motion controller
    // Motion system handles segmentation and velocity planning
    sendMoveCommand(theta, line.rho, speed_multiplier);

    pattern_position_.prev_theta = theta;
    pattern_position_.prev_rho = line.rho;
    return true;
}

void SandTablePlayer::sendMoveCommand(double theta_rad, double rho_normalized,
                                      float speed_multiplier) {
    // Don't normalize theta - pattern files use continuous rotation (can exceed 2π)
    // The motion controller uses radians (0-2π) and normalized rho (0-1) directly
    // No conversion needed - pattern files use the same coordinate system

    // Feed rate semantics: despite the historical "RPM" name, this is a
    // CONSTANT PATH SPEED in normalized table units per minute (1.0 = the
    // table radius). That is deliberate for a sand table — the ball moves at
    // uniform surface speed regardless of radius; angular speed rises as the
    // ball approaches the center.
    float feedrate_rpm = static_cast<float>(feed_rate_.load(std::memory_order_relaxed));
    if (feedrate_rpm <= 0) {
        feedrate_rpm = static_cast<float>(sand_table::MotionConfig::RHO_MAX_SPEED_RPM);
    }
    // Transit moves (folded rotations) run above the user's draw speed; the
    // profile generator clamps to hardware step-rate limits downstream.
    feedrate_rpm *= speed_multiplier;

    sand_table::PolarPosition target{ theta_rad, rho_normalized };
    auto result = linear_interpolation_.load(std::memory_order_relaxed)
        ? motion_controller_->move_linear(target, feedrate_rpm)
        : motion_controller_->move_to(target, feedrate_rpm);

    if (result.is_err()) {
        ESP_LOGW(TAG, "Failed to send move command: (%.4f rad, %.4f)",
            theta_rad, rho_normalized);
        if (result.error() != sand_table::MotionError::QueueFull) {
            ESP_LOGE(TAG, "Stopping due to motion error");
            stop();
        }
    }
}
