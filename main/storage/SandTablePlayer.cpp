#include "SandTablePlayer.h"
#include "esp_log.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <random>

// Static member definitions
const char* SandTablePlayer::TAG = "SandTablePlayer";
const char* SandTablePlayer::PATTERNS_PATH = "/sd/patterns";

sand_table::MotionController* SandTablePlayer::motion_controller_ = nullptr;
bool SandTablePlayer::initialized_ = false;
TaskHandle_t SandTablePlayer::service_task_handle_ = nullptr;
SemaphoreHandle_t SandTablePlayer::state_mutex_ = nullptr;
SemaphoreHandle_t SandTablePlayer::file_mutex_ = nullptr;

PlaybackState SandTablePlayer::playback_state_ = PlaybackState::STOPPED;
PlayMode SandTablePlayer::play_mode_ = PlayMode::SINGLE_PATTERN;

char SandTablePlayer::current_pattern_uuid_[MAX_UUID_LEN] = {0};
std::optional<Pattern> SandTablePlayer::current_pattern_ = std::nullopt;
FILE* SandTablePlayer::pattern_file_ = nullptr;
size_t SandTablePlayer::current_line_index_ = 0;
size_t SandTablePlayer::total_lines_ = 0;
bool SandTablePlayer::file_loaded_ = false;

char SandTablePlayer::current_playlist_uuid_[MAX_UUID_LEN] = {0};
std::vector<std::string> SandTablePlayer::playlist_patterns_;
std::vector<size_t> SandTablePlayer::playlist_order_;
size_t SandTablePlayer::playlist_index_ = 0;
bool SandTablePlayer::is_shuffle_ = false;
bool SandTablePlayer::is_loop_ = false;

PatternPosition SandTablePlayer::pattern_position_;
double SandTablePlayer::feed_rate_ = 5.0;

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

    // Create service task - DISABLED for memory profiling
    // TODO: Re-enable after memory optimization
    // if (xTaskCreate(serviceTaskWrapper, "sand_player", SERVICE_TASK_STACK_SIZE,
    //                 nullptr, SERVICE_TASK_PRIORITY, &service_task_handle_) != pdPASS) {
    //     ESP_LOGE(TAG, "Failed to create service task");
    //     vSemaphoreDelete(state_mutex_);
    //     vSemaphoreDelete(file_mutex_);
    //     return ESP_ERR_NO_MEM;
    // }
    ESP_LOGW(TAG, "sand_player task DISABLED for memory profiling");

    initialized_ = true;
    ESP_LOGI(TAG, "SandTablePlayer initialized");
    return ESP_OK;
}

void SandTablePlayer::shutdown() {
    if (!initialized_) return;

    ESP_LOGI(TAG, "Shutting down SandTablePlayer");

    stop();

    if (service_task_handle_) {
        vTaskDelete(service_task_handle_);
        service_task_handle_ = nullptr;
    }

    if (state_mutex_) {
        vSemaphoreDelete(state_mutex_);
        state_mutex_ = nullptr;
    }
    if (file_mutex_) {
        vSemaphoreDelete(file_mutex_);
        file_mutex_ = nullptr;
    }

    unloadPatternFile();
    motion_controller_ = nullptr;
    initialized_ = false;

    ESP_LOGI(TAG, "SandTablePlayer shutdown complete");
}

// =============================================================================
// Pattern Playback
// =============================================================================

esp_err_t SandTablePlayer::playPattern(const char* pattern_uuid) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (!motion_controller_->is_homed()) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire state mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Stop current playback if any
    if (playback_state_ != PlaybackState::STOPPED) {
        playback_state_ = PlaybackState::STOPPED;
    }

    // Load the pattern file
    esp_err_t ret = loadPatternFile(pattern_uuid);
    if (ret != ESP_OK) {
        xSemaphoreGive(state_mutex_);
        return ret;
    }

    // Get pattern info from ManifestDatabase
    current_pattern_ = ManifestDatabase::instance().getPattern(pattern_uuid);
    if (!current_pattern_.has_value()) {
        ESP_LOGE(TAG, "Pattern not found in manifest: %s", pattern_uuid);
        unloadPatternFile();
        xSemaphoreGive(state_mutex_);
        return ESP_ERR_NOT_FOUND;
    }

    strncpy(current_pattern_uuid_, pattern_uuid, MAX_UUID_LEN - 1);
    current_pattern_uuid_[MAX_UUID_LEN - 1] = '\0';

    // Reset motion progress counters for accurate progress tracking
    motion_controller_->reset_progress_counters();

    play_mode_ = PlayMode::SINGLE_PATTERN;
    playback_state_ = PlaybackState::PLAYING;
    motion_controller_->resume();  // Ensure not paused

    xSemaphoreGive(state_mutex_);

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

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Stop current playback
    if (playback_state_ != PlaybackState::STOPPED) {
        playback_state_ = PlaybackState::STOPPED;
        unloadPatternFile();
    }

    // Load the playlist
    loadPlaylist(playlist_uuid);
    if (playlist_patterns_.empty()) {
        ESP_LOGE(TAG, "Playlist is empty or not found: %s", playlist_uuid);
        xSemaphoreGive(state_mutex_);
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

    xSemaphoreGive(state_mutex_);

    ESP_LOGI(TAG, "Started playlist: %s (shuffle=%d, loop=%d)",
             playlist_uuid, (int)shuffle, (int)loop);
    return ESP_OK;
}

// =============================================================================
// Playback Control
// =============================================================================

esp_err_t SandTablePlayer::pause() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (playback_state_ == PlaybackState::PLAYING) {
        motion_controller_->pause();
        playback_state_ = PlaybackState::PAUSED;
        ESP_LOGI(TAG, "Playback paused");
    }

    xSemaphoreGive(state_mutex_);
    return ESP_OK;
}

esp_err_t SandTablePlayer::resume() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (playback_state_ == PlaybackState::PAUSED) {
        motion_controller_->resume();
        playback_state_ = PlaybackState::PLAYING;
        ESP_LOGI(TAG, "Playback resumed");
    }

    xSemaphoreGive(state_mutex_);
    return ESP_OK;
}

esp_err_t SandTablePlayer::stop() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
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

    xSemaphoreGive(state_mutex_);

    ESP_LOGI(TAG, "Playback stopped");

    motion_controller_->emergency_stop();
    motion_controller_->clear_emergency_stop();
    return ESP_OK;
}

esp_err_t SandTablePlayer::skip() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Only works in playlist mode
    if (play_mode_ == PlayMode::SINGLE_PATTERN || playlist_patterns_.empty()) {
        xSemaphoreGive(state_mutex_);
        return ESP_ERR_INVALID_STATE;
    }

    // Stop current pattern
    unloadPatternFile();

    // Advance to next
    advanceToNextPattern();

    xSemaphoreGive(state_mutex_);

    ESP_LOGI(TAG, "Skipped to next pattern");
    return ESP_OK;
}

// =============================================================================
// Configuration
// =============================================================================

void SandTablePlayer::setFeedRate(double feed_rate) {
    if (feed_rate < 1.0) feed_rate = 1.0;
    if (feed_rate > 20.0) feed_rate = 20.0;
    feed_rate_ = feed_rate;
    ESP_LOGI(TAG, "Set feed rate: %.2f", feed_rate_);
}

double SandTablePlayer::getFeedRate() {
    return feed_rate_;
}

esp_err_t SandTablePlayer::setShuffle(bool shuffle) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    is_shuffle_ = shuffle;
    if (play_mode_ != PlayMode::SINGLE_PATTERN) {
        play_mode_ = is_loop_ ? PlayMode::PLAYLIST_LOOP :
                     (is_shuffle_ ? PlayMode::PLAYLIST_SHUFFLE : PlayMode::PLAYLIST);
    }

    xSemaphoreGive(state_mutex_);
    ESP_LOGI(TAG, "Set shuffle: %d", (int)shuffle);
    return ESP_OK;
}

bool SandTablePlayer::isShuffle() {
    return is_shuffle_;
}

esp_err_t SandTablePlayer::setLoop(bool loop) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    is_loop_ = loop;
    if (play_mode_ != PlayMode::SINGLE_PATTERN) {
        play_mode_ = is_loop_ ? PlayMode::PLAYLIST_LOOP :
                     (is_shuffle_ ? PlayMode::PLAYLIST_SHUFFLE : PlayMode::PLAYLIST);
    }

    xSemaphoreGive(state_mutex_);
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
    status.state = playback_state_;
    status.mode = play_mode_;
    status.current_pattern_uuid = current_pattern_uuid_;
    status.current_playlist_uuid = current_playlist_uuid_;
    status.progress_percent = getTotalProgress();
    status.pattern_index = playlist_index_;
    status.playlist_size = playlist_patterns_.size();
    status.feed_rate = feed_rate_;
    status.is_shuffle = is_shuffle_;
    status.is_loop = is_loop_;
    return status;
}

cJSON* SandTablePlayer::getStateJSON() {
    cJSON* root = cJSON_CreateObject();

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

    // With direct moves to motion controller, progress is based on:
    // 1. How many lines have been queued (current_line_index_ / total_lines_)
    // 2. How much of the queued motion has executed (motion progress)

    // Motion execution progress is the primary indicator
    if (motion_controller_) {
        auto mp = motion_controller_->get_motion_progress();
        if (mp.steps_queued > 0) {
            double motion_fraction = static_cast<double>(mp.steps_completed) /
                                     static_cast<double>(mp.steps_queued);
            int percent = static_cast<int>(motion_fraction * 100.0);
            if (percent > 100) percent = 100;
            if (percent < 0) percent = 0;
            return percent;
        }
    }

    // Fallback: use line-based progress if no motion data
    int percent = static_cast<int>(
        (static_cast<double>(current_line_index_) / static_cast<double>(total_lines_)) * 100.0);
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    return percent;
}

bool SandTablePlayer::isHomed() {
    return motion_controller_ && motion_controller_->is_homed();
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

    while (true) {
        // Process pattern lines when playing
        if (playback_state_ == PlaybackState::PLAYING && file_loaded_) {

            if (hasMoreLines()) {
                PatternLine line = peekNextLine();
                if (line.is_valid) {
                    // processPatternLine returns false if queue is full (will retry)
                    if (processPatternLine(line)) {
                        popLine();
                    }
                } else {
                    ESP_LOGW(TAG, "Invalid line, skipping");
                    popLine();
                }
            } else {
                // All lines queued - wait for motion to complete before advancing
                auto progress = motion_controller_->get_motion_progress();
                if (!progress.is_executing && progress.segments_queued == 0) {
                    ESP_LOGI(TAG, "Pattern finished");

                    // Handle playlist mode
                    if (play_mode_ != PlayMode::SINGLE_PATTERN && !playlist_patterns_.empty()) {
                        advanceToNextPattern();
                    } else {
                        playback_state_ = PlaybackState::STOPPED;
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SERVICE_TASK_DELAY_MS));
    }
}

// =============================================================================
// Pattern File Handling
// =============================================================================

esp_err_t SandTablePlayer::loadPatternFile(const char* pattern_uuid) {
    if (xSemaphoreTake(file_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire file mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Unload any existing file first (without mutex, we already hold it)
    if (pattern_file_) {
        fclose(pattern_file_);
        pattern_file_ = nullptr;
    }
    current_line_index_ = 0;
    total_lines_ = 0;
    file_loaded_ = false;

    // Get pattern metadata to determine if encrypted
    auto pattern = ManifestDatabase::instance().getPattern(pattern_uuid);
    bool is_encrypted = pattern.has_value() ? pattern->encrypted : false;

    char file_path[256];
    getPatternFilePath(pattern_uuid, is_encrypted, file_path, sizeof(file_path));

    // TODO: For encrypted patterns, use EncryptedPatternReader instead
    if (is_encrypted) {
        ESP_LOGE(TAG, "Encrypted pattern playback not yet implemented: %s", pattern_uuid);
        xSemaphoreGive(file_mutex_);
        return ESP_ERR_NOT_SUPPORTED;
    }

    pattern_file_ = fopen(file_path, "r");
    if (!pattern_file_) {
        ESP_LOGE(TAG, "Failed to open pattern file: %s", file_path);
        xSemaphoreGive(file_mutex_);
        return ESP_ERR_NOT_FOUND;
    }

    // Count total lines
    char line_buffer[MAX_LINE_BUFFER_SIZE];
    while (fgets(line_buffer, sizeof(line_buffer), pattern_file_)) {
        total_lines_++;
    }
    rewind(pattern_file_);

    current_line_index_ = 0;
    file_loaded_ = true;

    ESP_LOGI(TAG, "Loaded pattern file: %s (%zu lines)", file_path, total_lines_);

    xSemaphoreGive(file_mutex_);
    return ESP_OK;
}

void SandTablePlayer::unloadPatternFile() {
    if (xSemaphoreTake(file_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire file mutex");
        return;
    }

    if (pattern_file_) {
        fclose(pattern_file_);
        pattern_file_ = nullptr;
    }
    current_line_index_ = 0;
    total_lines_ = 0;
    file_loaded_ = false;

    xSemaphoreGive(file_mutex_);
}

PatternLine SandTablePlayer::parsePatternLine(const char* line, size_t line_number) {
    if (!line || strlen(line) == 0) {
        return PatternLine();
    }

    // Parse space-separated theta and rho values
    char theta_str[64], rho_str[64];
    int parsed = sscanf(line, "%63s %63s", theta_str, rho_str);

    if (parsed != 2) {
        ESP_LOGW(TAG, "Failed to parse line %zu: %s", line_number, line);
        return PatternLine();
    }

    char* endptr;
    double theta = strtod(theta_str, &endptr);
    if (*endptr != '\0') {
        ESP_LOGW(TAG, "Failed to convert theta on line %zu: %s", line_number, theta_str);
        return PatternLine();
    }

    double rho = strtod(rho_str, &endptr);
    if (*endptr != '\0') {
        ESP_LOGW(TAG, "Failed to convert rho on line %zu: %s", line_number, rho_str);
        return PatternLine();
    }

    bool is_first = (line_number == 0);
    return PatternLine(theta, rho, is_first);
}

void SandTablePlayer::getPatternFilePath(const char* pattern_uuid, bool encrypted, char* file_path, size_t file_path_size) {
    // Unencrypted patterns use .thr extension, encrypted use .dat
    const char* extension = encrypted ? "dat" : "thr";
    snprintf(file_path, file_path_size, "%s/%s.%s", PATTERNS_PATH, pattern_uuid, extension);
}

PatternLine SandTablePlayer::peekNextLine() {
    if (!file_loaded_ || !pattern_file_) return PatternLine();

    if (xSemaphoreTake(file_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire file mutex");
        return PatternLine();
    }

    long prev_pos = ftell(pattern_file_);
    char line_buffer[MAX_LINE_BUFFER_SIZE];
    size_t line_num = 0;

    rewind(pattern_file_);
    while (line_num < current_line_index_ && fgets(line_buffer, sizeof(line_buffer), pattern_file_)) {
        line_num++;
    }

    if (!fgets(line_buffer, sizeof(line_buffer), pattern_file_)) {
        fseek(pattern_file_, prev_pos, SEEK_SET);
        xSemaphoreGive(file_mutex_);
        return PatternLine();
    }

    fseek(pattern_file_, prev_pos, SEEK_SET);
    xSemaphoreGive(file_mutex_);

    // Remove newline
    size_t len = strlen(line_buffer);
    while (len > 0 && (line_buffer[len - 1] == '\n' || line_buffer[len - 1] == '\r')) {
        line_buffer[--len] = '\0';
    }

    return parsePatternLine(line_buffer, current_line_index_);
}

void SandTablePlayer::popLine() {
    if (!file_loaded_) return;
    current_line_index_++;
}

bool SandTablePlayer::hasMoreLines() {
    return file_loaded_ && (current_line_index_ < total_lines_);
}

// =============================================================================
// Playlist Management
// =============================================================================

void SandTablePlayer::loadPlaylist(const char* playlist_uuid) {
    playlist_patterns_.clear();
    playlist_order_.clear();
    playlist_index_ = 0;

    auto playlist = ManifestDatabase::instance().getPlaylist(playlist_uuid);
    if (playlist.has_value()) {
        playlist_patterns_ = playlist->patterns;
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

    // Load the pattern file
    esp_err_t ret = loadPatternFile(pattern_uuid.c_str());
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load pattern: %s", pattern_uuid.c_str());
        advanceToNextPattern();
        return;
    }

    // Get pattern info
    current_pattern_ = ManifestDatabase::instance().getPattern(pattern_uuid);
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
        } else {
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

    if (line.is_first_line) {
        // First line: record starting position, no move needed
        pattern_position_.prev_theta = line.theta;
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

    // Send move directly to motion controller
    // Motion system handles segmentation and velocity planning
    sendMoveCommand(line.theta, line.rho);

    pattern_position_.prev_theta = line.theta;
    pattern_position_.prev_rho = line.rho;
    return true;
}

void SandTablePlayer::sendMoveCommand(double theta_rad, double rho_normalized) {
    // Don't normalize theta - pattern files use continuous rotation (can exceed 2π)
    // The motion controller uses radians (0-2π) and normalized rho (0-1) directly
    // No conversion needed - pattern files use the same coordinate system

    // Feed rate is in RPM
    float feedrate_rpm = static_cast<float>(feed_rate_);
    if (feedrate_rpm <= 0) {
        feedrate_rpm = static_cast<float>(sand_table::MotionConfig::RHO_MAX_SPEED_RPM);
    }

    sand_table::PolarPosition target{theta_rad, rho_normalized};
    auto result = motion_controller_->move_to(target, feedrate_rpm);

    if (result.is_err()) {
        ESP_LOGW(TAG, "Failed to send move command: (%.4f rad, %.4f)",
                 theta_rad, rho_normalized);
        if (result.error() != sand_table::MotionError::QueueFull) {
            ESP_LOGE(TAG, "Stopping due to motion error");
            stop();
        }
    }
}
