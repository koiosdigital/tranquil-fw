#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ManifestManager.h"
#include "motion_controller.h"
#include "cJSON.h"
#include <string>
#include <vector>

#define MAX_UUID_LEN 64
#define MAX_LINE_BUFFER_SIZE 512

enum class PlaybackState {
    STOPPED,
    PLAYING,
    PAUSED
};

enum class PlayMode {
    SINGLE_PATTERN,
    PLAYLIST,
    PLAYLIST_LOOP,
    PLAYLIST_SHUFFLE
};

struct PatternLine {
    double theta;        // Angle in radians (non-wrapped)
    double rho;          // Radius from 0 (center) to 1 (max radius)
    bool is_first_line;  // True if this is the first line
    bool is_valid;       // True if line was parsed successfully

    PatternLine() : theta(0.0), rho(0.0), is_first_line(false), is_valid(false) {}
    PatternLine(double t, double r, bool first = false)
        : theta(t), rho(r), is_first_line(first), is_valid(true) {}
};

struct InterpolationState {
    bool in_progress;
    bool is_interpolating;
    int cur_step;
    int interpolate_steps;
    double cur_theta;
    double cur_rho;
    double prev_theta;
    double prev_rho;
    double theta_inc;
    double rho_inc;
    double theta_start_offset;
    double step_angle;  // ~1 degree in radians

    InterpolationState() {
        in_progress = false;
        is_interpolating = false;
        cur_step = 0;
        interpolate_steps = 0;
        cur_theta = 0.0;
        cur_rho = 0.0;
        prev_theta = 0.0;
        prev_rho = 0.0;
        theta_inc = 0.0;
        rho_inc = 0.0;
        theta_start_offset = 0.0;
        step_angle = 0.0174533;  // ~1 degree in radians
    }
};

struct PlaybackStatus {
    PlaybackState state;
    PlayMode mode;
    std::string current_pattern_uuid;
    std::string current_playlist_uuid;
    int progress_percent;
    size_t pattern_index;
    size_t playlist_size;
    double feed_rate;
    bool is_shuffle;
    bool is_loop;
};

class SandTablePlayer {
public:
    // Initialization
    static esp_err_t initialize(sand_table::MotionController* controller);
    static void shutdown();

    // Pattern playback
    static esp_err_t playPattern(const char* pattern_uuid);

    // Playlist playback
    static esp_err_t playPlaylist(const char* playlist_uuid, bool shuffle = false, bool loop = false);

    // Playback control
    static esp_err_t pause();
    static esp_err_t resume();
    static esp_err_t stop();
    static esp_err_t skip();  // Playlist only: skip to next pattern

    // Configuration
    static void setFeedRate(double feed_rate);
    static double getFeedRate();
    static esp_err_t setShuffle(bool shuffle);
    static bool isShuffle();
    static esp_err_t setLoop(bool loop);
    static bool isLoop();

    // Status
    static PlaybackState getPlaybackState();
    static PlayMode getPlayMode();
    static PlaybackStatus getStatus();
    static cJSON* getStateJSON();
    static int getTotalProgress();

    // Pattern info
    static Pattern* getCurrentPattern();
    static const char* getCurrentPatternUUID();
    static const char* getCurrentPlaylistUUID();
    static bool isPatternLoaded();

private:
    SandTablePlayer() = delete;
    ~SandTablePlayer() = delete;
    SandTablePlayer(const SandTablePlayer&) = delete;
    SandTablePlayer& operator=(const SandTablePlayer&) = delete;

    // Service task
    static void serviceTaskWrapper(void* param);
    static void serviceTask();

    // Pattern file handling
    static esp_err_t loadPatternFile(const char* pattern_uuid);
    static void unloadPatternFile();
    static PatternLine parsePatternLine(const char* line, size_t line_number);
    static void getPatternFilePath(const char* pattern_uuid, char* file_path, size_t file_path_size);
    static PatternLine peekNextLine();
    static void popLine();
    static bool hasMoreLines();

    // Playlist management
    static void loadPlaylist(const char* playlist_uuid);
    static void startCurrentPattern();
    static void advanceToNextPattern();
    static void shufflePlaylistOrder();

    // Motion processing
    static bool processPatternLine(const PatternLine& line);
    static void serviceInterpolation();
    static void sendMoveCommand(double theta_rad, double rho_normalized);

    // State
    static sand_table::MotionController* motion_controller_;
    static bool initialized_;
    static TaskHandle_t service_task_handle_;
    static SemaphoreHandle_t state_mutex_;
    static SemaphoreHandle_t file_mutex_;

    // Playback state
    static PlaybackState playback_state_;
    static PlayMode play_mode_;

    // Pattern state
    static char current_pattern_uuid_[MAX_UUID_LEN];
    static Pattern* current_pattern_;
    static FILE* pattern_file_;
    static size_t current_line_index_;
    static size_t total_lines_;
    static bool file_loaded_;

    // Playlist state
    static char current_playlist_uuid_[MAX_UUID_LEN];
    static std::vector<std::string> playlist_patterns_;
    static std::vector<size_t> playlist_order_;
    static size_t playlist_index_;
    static bool is_shuffle_;
    static bool is_loop_;

    // Interpolation state
    static InterpolationState interpolation_state_;

    // Configuration
    static double feed_rate_;

    // Constants
    static const char* TAG;
    static const char* PATTERNS_PATH;
    static constexpr uint32_t SERVICE_TASK_DELAY_MS = 10;
    static constexpr size_t SERVICE_TASK_STACK_SIZE = 8192;
    static constexpr UBaseType_t SERVICE_TASK_PRIORITY = 5;
    static constexpr int PROCESS_STEPS_PER_SERVICE = 100;
    static constexpr double RHO_AT_DEFAULT_STEP_ANGLE = 0.3;
};
