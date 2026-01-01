#include "PatternPlayer.h"
#include "RobotMotionAPI.h"
#include "esp_log.h"
#include "cJSON.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string> // Temporarily needed for ManifestManager compatibility

// Static member definitions
const char* PatternPlayer::TAG = "PatternPlayer";
const char* PatternPlayer::PATTERNS_PATH = "/sd/patterns";
bool PatternPlayer::initialized_ = false;
TaskHandle_t PatternPlayer::service_task_handle_ = nullptr;
SemaphoreHandle_t PatternPlayer::state_mutex_ = nullptr;
SemaphoreHandle_t PatternPlayer::file_mutex_ = nullptr;

PlaybackState PatternPlayer::playback_state_ = PlaybackState::STOPPED;
char PatternPlayer::current_pattern_uuid_[MAX_PATTERN_UUID_LEN] = { 0 };
Pattern* PatternPlayer::current_pattern_ = nullptr;

FILE* PatternPlayer::pattern_file_ = nullptr;
size_t PatternPlayer::current_line_index_ = 0;
size_t PatternPlayer::total_lines_ = 0;
bool PatternPlayer::file_loaded_ = false;

// Interpolation state
InterpolationState PatternPlayer::interpolation_state_;

double PatternPlayer::feed_rate_ = 5.0;

esp_err_t PatternPlayer::initialize() {
    if (initialized_) return ESP_OK;

    // Create mutex for thread safety
    state_mutex_ = xSemaphoreCreateMutex();
    file_mutex_ = xSemaphoreCreateMutex(); // Create file mutex
    if (!state_mutex_ || !file_mutex_) {
        ESP_LOGE(TAG, "Failed to create mutexes");
        if (state_mutex_) vSemaphoreDelete(state_mutex_);
        if (file_mutex_) vSemaphoreDelete(file_mutex_);
        return ESP_ERR_NO_MEM;
    }

    // Create service task
    if (xTaskCreate(serviceTaskWrapper, "pattern_player", 8192,
        nullptr, SERVICE_TASK_PRIORITY, &service_task_handle_) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create service task");
        vSemaphoreDelete(state_mutex_);
        vSemaphoreDelete(file_mutex_);
        return ESP_ERR_NO_MEM;
    }

    initialized_ = true;
    ESP_LOGD(TAG, "PatternPlayer initialized successfully");
    return ESP_OK;
}

void PatternPlayer::shutdown() {
    if (!initialized_) return;

    ESP_LOGD(TAG, "Shutting down PatternPlayer");

    // Stop playback
    stop();

    // Delete service task
    if (service_task_handle_) {
        vTaskDelete(service_task_handle_);
        service_task_handle_ = nullptr;
    }

    // Clean up mutexes
    if (state_mutex_) {
        vSemaphoreDelete(state_mutex_);
        state_mutex_ = nullptr;
    }
    if (file_mutex_) {
        vSemaphoreDelete(file_mutex_);
        file_mutex_ = nullptr;
    }

    // Unload any loaded pattern
    unloadPatternFile();

    initialized_ = false;
    ESP_LOGD(TAG, "PatternPlayer shutdown complete");
}

esp_err_t PatternPlayer::playPattern(const char* pattern_uuid) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!RobotMotionSystem::isHomed()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire state mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Stop current playback if any
    if (playback_state_ != PlaybackState::STOPPED) {
        stop();
    }

    // Load the pattern file
    esp_err_t ret = loadPatternFile(pattern_uuid);
    if (ret != ESP_OK) {
        xSemaphoreGive(state_mutex_);
        return ret;
    }

    // Get pattern info from ManifestManager (using temporary std::string for compatibility)
    std::string pattern_uuid_str(pattern_uuid);
    current_pattern_ = ManifestManager::getPattern(pattern_uuid_str);
    if (!current_pattern_) {
        ESP_LOGE(TAG, "Pattern not found in manifest: %s", pattern_uuid);
        unloadPatternFile();
        xSemaphoreGive(state_mutex_);
        return ESP_ERR_NOT_FOUND;
    }

    strncpy(current_pattern_uuid_, pattern_uuid, MAX_PATTERN_UUID_LEN - 1);
    current_pattern_uuid_[MAX_PATTERN_UUID_LEN - 1] = '\0';

    RobotMotionSystem::stopMotion(); // Ensure robot is stopped before starting playback
    playback_state_ = PlaybackState::PLAYING;
    RobotMotionSystem::pauseMotion(false); // Resume motion if paused

    xSemaphoreGive(state_mutex_);

    ESP_LOGI(TAG, "Started playing pattern: %s (%s)",
        current_pattern_->name.c_str(), pattern_uuid);
    return ESP_OK;
}

esp_err_t PatternPlayer::pause() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (playback_state_ == PlaybackState::PLAYING) {
        RobotMotionSystem::pauseMotion(true);
        playback_state_ = PlaybackState::PAUSED;
        ESP_LOGI(TAG, "Pattern playback paused");
    }

    xSemaphoreGive(state_mutex_);
    return ESP_OK;
}

esp_err_t PatternPlayer::resume() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (playback_state_ == PlaybackState::PAUSED) {
        RobotMotionSystem::pauseMotion(false);
        playback_state_ = PlaybackState::PLAYING;
        ESP_LOGI(TAG, "Pattern playback resumed");
    }

    xSemaphoreGive(state_mutex_);
    return ESP_OK;
}

esp_err_t PatternPlayer::stop() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    playback_state_ = PlaybackState::STOPPED;

    // Stop interpolation
    interpolation_state_.in_progress = false;
    interpolation_state_.is_interpolating = false;

    unloadPatternFile();
    memset(current_pattern_uuid_, 0, sizeof(current_pattern_uuid_));
    current_pattern_ = nullptr;

    xSemaphoreGive(state_mutex_);

    ESP_LOGI(TAG, "Pattern playback stopped");

    RobotMotionSystem::stopMotion();
    return ESP_OK;
}

Pattern* PatternPlayer::getCurrentPattern() {
    return current_pattern_;
}

PlaybackState PatternPlayer::getPlaybackState() {
    return playback_state_;
}

PatternLine PatternPlayer::peekNextLine() {
    if (!file_loaded_ || !pattern_file_) return PatternLine();

    if (xSemaphoreTake(file_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire file mutex in peekNextLine");
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

void PatternPlayer::popLine() {
    if (!file_loaded_) return;
    current_line_index_++;
    if (current_line_index_ >= total_lines_) {
        playback_state_ = PlaybackState::STOPPED;
    }
}

bool PatternPlayer::hasMoreLines() {
    return file_loaded_ && (current_line_index_ < total_lines_);
}

esp_err_t PatternPlayer::loadPatternFile(const char* pattern_uuid) {
    if (xSemaphoreTake(file_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire file mutex in loadPatternFile");
        return ESP_ERR_TIMEOUT;
    }

    unloadPatternFile();
    char file_path[256];
    getPatternFilePath(pattern_uuid, file_path, sizeof(file_path));
    pattern_file_ = fopen(file_path, "r");
    if (!pattern_file_) {
        ESP_LOGE(TAG, "Failed to open pattern file: %s", file_path);
        xSemaphoreGive(file_mutex_);
        return ESP_ERR_NOT_FOUND;
    }
    // Count total lines
    total_lines_ = 0;
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

void PatternPlayer::unloadPatternFile() {
    if (xSemaphoreTake(file_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire file mutex in unloadPatternFile");
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

void PatternPlayer::getPatternFilePath(const char* pattern_uuid, char* file_path, size_t file_path_size) {
    snprintf(file_path, file_path_size, "%s/%s.thr", PATTERNS_PATH, pattern_uuid);
}

PatternLine PatternPlayer::parsePatternLine(const char* line, size_t line_number) {
    if (!line || strlen(line) == 0) {
        return PatternLine(); // Invalid
    }

    // Parse space-separated theta and rho values using sscanf
    char theta_str[64], rho_str[64];
    int parsed = sscanf(line, "%63s %63s", theta_str, rho_str);

    if (parsed != 2) {
        ESP_LOGW(TAG, "Failed to parse line %zu: %s", line_number, line);
        return PatternLine(); // Invalid
    }

    // Convert strings to doubles using strtod (no exceptions)
    char* endptr;
    double theta = strtod(theta_str, &endptr);
    if (*endptr != '\0') {
        ESP_LOGW(TAG, "Failed to convert theta to number on line %zu: %s", line_number, theta_str);
        return PatternLine(); // Invalid
    }

    double rho = strtod(rho_str, &endptr);
    if (*endptr != '\0') {
        ESP_LOGW(TAG, "Failed to convert rho to number on line %zu: %s", line_number, rho_str);
        return PatternLine(); // Invalid
    }

    // First line is special (line 0)
    bool is_first = (line_number == 0);

    return PatternLine(theta, rho, is_first);
}

void PatternPlayer::serviceTaskWrapper(void* param) {
    serviceTask();
}

// Theta-rho processing methods (based on Arduino EvaluatorThetaRhoLine)
bool PatternPlayer::processPatternLine(const PatternLine& line) {
    if (!line.is_valid) return false;

    double new_theta = line.theta;
    double new_rho = line.rho;

    if (line.is_first_line) {
        // On the first line, just set the current theta/rho as the starting point
        interpolation_state_.theta_start_offset = 0.0;
        interpolation_state_.prev_theta = new_theta;
        interpolation_state_.prev_rho = new_rho;
        interpolation_state_.is_interpolating = false;
        interpolation_state_.in_progress = false;
        return true;
    }

    // Regular line (_THRLINEN_) - setup interpolation
    double delta_theta = new_theta - interpolation_state_.prev_theta;
    double abs_delta_theta = fabs(delta_theta);
    double adapted_step_angle = interpolation_state_.step_angle;

    double abs_new_rho = fabs(new_rho);
    double abs_prev_rho = fabs(interpolation_state_.prev_rho);
    double avg_rho = abs_new_rho > abs_prev_rho ? abs_new_rho : abs_prev_rho;
    if (avg_rho > 1) avg_rho = 1;

    double max_step_angle = interpolation_state_.step_angle * 16;
    if (max_step_angle > M_PI / 2) max_step_angle = M_PI / 2;
    double min_step_angle = interpolation_state_.step_angle / 4;

    if (avg_rho > RHO_AT_DEFAULT_STEP_ANGLE) {
        adapted_step_angle = ((avg_rho - RHO_AT_DEFAULT_STEP_ANGLE) / (1 - RHO_AT_DEFAULT_STEP_ANGLE)) *
            (min_step_angle - interpolation_state_.step_angle) + interpolation_state_.step_angle;
    }
    else {
        adapted_step_angle = (avg_rho / RHO_AT_DEFAULT_STEP_ANGLE) *
            (interpolation_state_.step_angle - max_step_angle) + max_step_angle;
    }

    interpolation_state_.theta_inc = delta_theta >= 0 ? adapted_step_angle : -adapted_step_angle;
    double delta_rho = new_rho - interpolation_state_.prev_rho;

    if (abs_delta_theta < adapted_step_angle) {
        interpolation_state_.theta_inc = delta_theta;
        interpolation_state_.interpolate_steps = 1;
        interpolation_state_.rho_inc = delta_rho;
    }
    else {
        interpolation_state_.interpolate_steps = (int)floor(abs_delta_theta / adapted_step_angle);
        if (interpolation_state_.interpolate_steps < 1) return true;
        interpolation_state_.rho_inc = delta_rho * adapted_step_angle / abs_delta_theta;
    }

    interpolation_state_.cur_theta = interpolation_state_.prev_theta;
    interpolation_state_.cur_rho = interpolation_state_.prev_rho;
    interpolation_state_.prev_theta = new_theta;
    interpolation_state_.prev_rho = new_rho;
    interpolation_state_.cur_step = 0;
    interpolation_state_.in_progress = true;
    interpolation_state_.is_interpolating = true;

    return true;
}

void PatternPlayer::calcXYPos(double theta, double rho, double& x, double& y) {
    x = sin(theta) * rho;
    y = cos(theta) * rho;
}

void PatternPlayer::serviceInterpolation() {
    if (!interpolation_state_.in_progress) {
        return;
    }
    if (!interpolation_state_.is_interpolating) {
        return;
    }

    // Process multiple steps if possible
    for (int i = 0; i < PROCESS_STEPS_PER_SERVICE; i++) {
        //ESP_LOGI(TAG, "serviceInterpolation: step %d/%d", interpolation_state_.cur_step, interpolation_state_.interpolate_steps);
        if (interpolation_state_.cur_step >= interpolation_state_.interpolate_steps) {
            //ESP_LOGI(TAG, "serviceInterpolation: finished interpolation");
            interpolation_state_.in_progress = false;
            return;
        }

        // Check if robot can accept command
        if (!RobotMotionSystem::canAcceptCommand()) {
            return;
        }

        // Step
        interpolation_state_.cur_step++;

        // Increment
        interpolation_state_.cur_theta += interpolation_state_.theta_inc;
        interpolation_state_.cur_rho += interpolation_state_.rho_inc;

        // Convert to polar coordinates for robot
        double robot_theta = interpolation_state_.cur_theta;
        double robot_rho = interpolation_state_.cur_rho;

        // Ensure theta is in range [0, 2*PI)
        while (robot_theta < 0) {
            robot_theta += 2 * M_PI;
        }
        while (robot_theta >= 2 * M_PI) {
            robot_theta -= 2 * M_PI;
        }

        esp_err_t ret = RobotMotionSystem::moveToPolar(robot_theta, robot_rho, feed_rate_);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send polar move command: %s (%.02f, %.02f)", esp_err_to_name(ret), robot_theta, robot_rho);
            if (ret != ESP_ERR_NO_MEM) {
                ESP_LOGE(TAG, "Stopping interpolation due to error: %s", esp_err_to_name(ret));
                stop();
            }
            return;
        }
    }
}

void PatternPlayer::serviceTask() {
    ESP_LOGI(TAG, "Service task started");

    while (true) {
        serviceInterpolation();

        if (playback_state_ == PlaybackState::PLAYING && file_loaded_ && !interpolation_state_.in_progress) {
            if (hasMoreLines()) {
                PatternLine line = peekNextLine();
                if (line.is_valid) {
                    if (processPatternLine(line)) {
                        popLine();
                    }
                }
                else {
                    ESP_LOGI(TAG, "serviceTask: invalid line, skipping");
                    popLine();
                }
            }
            else {
                ESP_LOGI(TAG, "serviceTask: no more lines");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SERVICE_TASK_DELAY_MS));
    }
}

cJSON* PatternPlayer::getStateJSON() {
    cJSON* root = cJSON_CreateObject();
    const char* state_str = "STOPPED";
    switch (playback_state_) {
    case PlaybackState::PLAYING: state_str = "PLAYING"; break;
    case PlaybackState::PAUSED: state_str = "PAUSED"; break;
    default: break;
    }
    cJSON_AddStringToObject(root, "playback_state", state_str);
    cJSON_AddStringToObject(root, "pattern_uuid", current_pattern_uuid_);
    cJSON_AddNumberToObject(root, "progress", getTotalProgress());
    return root;
}

void PatternPlayer::setFeedRate(double feed_rate) {
    if (feed_rate < 1.0) feed_rate = 1.0;
    if (feed_rate > 20.0) feed_rate = 20.0;
    feed_rate_ = feed_rate;
    ESP_LOGI(TAG, "Set feed rate: %.2f", feed_rate_);
}

double PatternPlayer::getFeedRate() {
    return feed_rate_;
}

int PatternPlayer::getTotalProgress() {
    if (!file_loaded_ || total_lines_ == 0) return 0;
    double line_progress = 0.0;
    if (interpolation_state_.in_progress && interpolation_state_.interpolate_steps > 0) {
        line_progress = (double)interpolation_state_.cur_step / (double)interpolation_state_.interpolate_steps;
        if (line_progress > 1.0) line_progress = 1.0;
        if (line_progress < 0.0) line_progress = 0.0;
    }
    double total = (double)current_line_index_ + line_progress;
    int percent = (int)((total / (double)total_lines_) * 100.0);
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    return percent;
}
