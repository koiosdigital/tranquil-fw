#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ManifestManager.h"
#include <stdio.h>
#include <stdbool.h>

#define MAX_PATTERN_UUID_LEN 64
#define MAX_LINE_BUFFER_SIZE 512

// Pattern line structure for theta-rho lines
struct PatternLine {
    double theta;        // Angle in radians (non-wrapped)
    double rho;          // Radius from 0 (center) to 1 (max radius)
    bool is_first_line;  // True if this is the first line (_THRLINE0_)
    bool is_valid;       // True if line was parsed successfully

    PatternLine() : theta(0.0), rho(0.0), is_first_line(false), is_valid(false) {}
    PatternLine(double t, double r, bool first = false)
        : theta(t), rho(r), is_first_line(first), is_valid(true) {
    }
};

enum class PlaybackState {
    STOPPED,
    PLAYING,
    PAUSED
};

// Interpolation state for theta-rho processing (based on Arduino EvaluatorThetaRhoLine)
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

    // Configuration
    double step_angle;

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
        step_angle = 0.0174533; // ~1 degree in radians (DEFAULT_STEP_ANGLE)
    }
};

class PatternPlayer {
public:
    // Static initialization
    static esp_err_t initialize();
    static void shutdown();

    // Pattern playback control
    static esp_err_t playPattern(const char* pattern_uuid);
    static esp_err_t pause();
    static esp_err_t resume();
    static esp_err_t stop();

    // Pattern navigation
    static Pattern* getCurrentPattern();
    static PlaybackState getPlaybackState();

    // Line-by-line pattern access (streaming from file)
    static PatternLine peekNextLine();     // Get next line from file without advancing (streaming)
    static void popLine();                 // Advance to next line (call after processing peek)
    static bool hasMoreLines();            // Check if more lines available

    // Progress tracking
    static int getTotalProgress();       // 0-100 percent progress
    static size_t getTotalLines();         // Total lines in pattern

    // Pattern file management
    static bool isPatternLoaded();
    static esp_err_t loadPatternFile(const char* pattern_uuid);
    static void unloadPatternFile();

    // State reporting
    static cJSON* getStateJSON();
    static void setFeedRate(double feed_rate);
    static double getFeedRate();

private:
    // Prevent instantiation
    PatternPlayer() = delete;
    ~PatternPlayer() = delete;
    PatternPlayer(const PatternPlayer&) = delete;
    PatternPlayer& operator=(const PatternPlayer&) = delete;

    // Service task
    static void serviceTaskWrapper(void* param);
    static void serviceTask();

    // Theta-rho processing (based on Arduino EvaluatorThetaRhoLine)
    static bool processPatternLine(const PatternLine& line);
    static void calcXYPos(double theta, double rho, double& x, double& y);
    static void serviceInterpolation();

    // File handling
    static PatternLine parsePatternLine(const char* line, size_t line_number);
    static void getPatternFilePath(const char* pattern_uuid, char* file_path, size_t file_path_size);

    // Static members
    static bool initialized_;
    static TaskHandle_t service_task_handle_;
    static SemaphoreHandle_t state_mutex_;
    static SemaphoreHandle_t file_mutex_; // Add file_mutex_ as a static member of PatternPlayer

    // Playback state
    static PlaybackState playback_state_;
    static char current_pattern_uuid_[MAX_PATTERN_UUID_LEN];
    static Pattern* current_pattern_;

    // File handling
    static FILE* pattern_file_;
    static size_t current_line_index_;
    static size_t total_lines_;
    static bool file_loaded_;

    // Interpolation state
    static InterpolationState interpolation_state_;

    // Configuration
    static double feed_rate_;
    static const char* TAG;
    static const char* PATTERNS_PATH;
    static constexpr uint32_t SERVICE_TASK_DELAY_MS = 10;
    static constexpr size_t SERVICE_TASK_STACK_SIZE = 4096;
    static constexpr UBaseType_t SERVICE_TASK_PRIORITY = 5;
    static constexpr int PROCESS_STEPS_PER_SERVICE = 100;
    static constexpr double RHO_AT_DEFAULT_STEP_ANGLE = 0.3;
};
