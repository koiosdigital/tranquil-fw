#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ManifestDatabase.h"
#include "PatternReader.h"
#include "motion_controller.h"
#include "cJSON.h"
#include <string>
#include <vector>
#include <optional>
#include <memory>
#include <atomic>

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
    PLAYLIST_SHUFFLE,
    RANDOM_LOOP      // a random library pattern after each one finishes, until stopped
};

struct PatternLine {
    double theta;        // Angle in radians (non-wrapped)
    double rho;          // Radius from 0 (center) to 1 (max radius)
    bool is_first_line;  // True if this is the first line
    bool is_valid;       // True if line was parsed successfully

    PatternLine() : theta(0.0), rho(0.0), is_first_line(false), is_valid(false) {}
    PatternLine(double t, double r, bool first = false)
        : theta(t), rho(r), is_first_line(first), is_valid(true) {
    }
};

// Tracks previous position for pattern playback
struct PatternPosition {
    double prev_theta = 0.0;   // Last COMMANDED theta (after offset), radians
    double prev_rho = 0.0;
    // Constant per-pattern rebase set at the first line: shifts the file's
    // absolute theta so its first point lands within half a turn of the
    // table's current position (same angle modulo 2π) instead of physically
    // unwinding however far the file's theta had accumulated when authored.
    // In-pattern multi-rotation jumps are NOT folded - they play in full at
    // kFoldedMoveSpeedMultiplier (see processPatternLine).
    double theta_offset = 0.0;
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
    // False when the snapshot could not be taken under the state lock (a
    // play() is loading a file). Fields are then the LAST consistent
    // snapshot, never a half-filled one.
    bool valid = true;
};

class SandTablePlayer {
public:
    // Initialization
    static esp_err_t initialize(sand_table::MotionController* controller);
    static void shutdown();

    // Pattern playback
    static esp_err_t playPattern(const char* pattern_uuid);

    // Random playback: start a random playable library pattern now. With
    // loop, another random pattern follows each one (PlayMode::RANDOM_LOOP).
    static esp_err_t playRandom(bool loop);
    // Random-loop toggle. Enabling while something plays keeps it running and
    // chains random patterns after it (a playlist is abandoned); enabling
    // while stopped starts a random pattern. Disabling lets the current
    // pattern finish, then stops. playPattern/playPlaylist/stop also clear it.
    static esp_err_t setRandomLoop(bool enabled);
    static bool isRandomLoop();

    // Playlist playback
    static esp_err_t playPlaylist(const char* playlist_uuid, bool shuffle = false, bool loop = false);
    // Start playlist from a specific pattern (even if shuffled, starts at this pattern)
    static esp_err_t playPlaylistFromPattern(const char* playlist_uuid, const char* pattern_uuid, bool shuffle = false, bool loop = false);

    // Playback control
    static esp_err_t pause();
    static esp_err_t resume();
    static esp_err_t stop();
    static esp_err_t emergencyStop();  // Immediate halt without state cleanup
    static esp_err_t skip();      // Playlist / random loop: skip to next pattern
    static esp_err_t previous();  // Playlist only: return to previous pattern

    // Configuration
    static void setFeedRate(double feed_rate);
    static double getFeedRate();
    static esp_err_t setShuffle(bool shuffle);
    static bool isShuffle();
    static esp_err_t setLoop(bool loop);
    static bool isLoop();

    // Route pattern moves through move_linear() (Cartesian interpolation:
    // straight lines in XY) instead of move_to() (direct polar: arcs).
    // Default off - theta-rho files are authored for polar interpolation.
    static void setLinearInterpolation(bool enabled);
    static bool isLinearInterpolation();

    // Status
    static PlaybackState getPlaybackState();
    static PlayMode getPlayMode();
    static PlaybackStatus getStatus();
    static cJSON* getStateJSON();
    static int getTotalProgress();
    static bool isHomed();
    static sand_table::MotionController* getMotionController();

    // Pattern info
    static const char* getCurrentPatternUUID();
    static const char* getCurrentPlaylistUUID();
    static bool isPatternLoaded();

    // True when `uuid` is the pattern currently loaded by the player (playing,
    // paused, or finished-but-still-open). Callers that are about to delete
    // or replace that pattern's file must stop() first: FATFS has no file
    // locking, so unlinking under the open reader corrupts playback.
    static bool isPlayingPattern(const char* uuid);

    // Block until playback is STOPPED and the motion controller has drained
    // (bounded). For callers that must own the sequencer next (homing).
    static bool waitStopped(TickType_t timeout);

private:
    SandTablePlayer() = delete;
    ~SandTablePlayer() = delete;
    SandTablePlayer(const SandTablePlayer&) = delete;
    SandTablePlayer& operator=(const SandTablePlayer&) = delete;

    // Service task
    static void serviceTaskWrapper(void* param);
    static void serviceTask();

    // Pattern file handling. loadPatternFile() = prepareFile() +
    // installPreparedFile(); the split lets play*() do the expensive part
    // (encrypted open ~1.7 s of RSA, progress pre-scan of the whole file)
    // WITHOUT state_mutex_ held, so status/control calls never block on it.
    struct PreparedFile {
        std::unique_ptr<IPatternReader> reader;
        size_t total_lines = 0;
        double total_path_len = 0.0;
        bool is_encrypted = false;
    };
    static esp_err_t prepareFile(const char* pattern_uuid, PreparedFile& out);
    static void installPreparedFile(PreparedFile&& prepared);  // file_mutex_
    static esp_err_t loadPatternFile(const char* pattern_uuid);
    static void unloadPatternFile();
    static PatternLine peekNextLine();
    static void popLine();
    static bool hasMoreLines();

    // Random playback (state_mutex_ held)
    static bool pickRandomPattern(std::string& out_uuid);
    static void startRandomPattern();
    static void clearPlaylistState();

    // Playlist management
    static void loadPlaylist(const char* playlist_uuid);
    static void startCurrentPattern();
    static void advanceToNextPattern();
    static void goToPreviousPattern();
    static void shufflePlaylistOrder();

    // Motion processing
    static bool processPatternLine(const PatternLine& line);
    static void sendMoveCommand(double theta_rad, double rho_normalized,
        float speed_multiplier = 1.0f);

    // State
    static sand_table::MotionController* motion_controller_;
    static bool initialized_;
    static TaskHandle_t service_task_handle_;
    static SemaphoreHandle_t state_mutex_;
    static SemaphoreHandle_t file_mutex_;
    // Shutdown handshake: shutdown() sets shutdown_requested_ and waits for
    // the service task to acknowledge via service_task_exited_ before any
    // mutex is deleted (deleting a mutex a task holds is undefined).
    static std::atomic<bool> shutdown_requested_;
    static std::atomic<bool> service_task_exited_;

    // Playback state. Atomic: mutated under state_mutex_, but read without
    // it by the status getters and the service task's per-line exit hint
    // (relaxed loads there — the locked snapshot is authoritative).
    static std::atomic<PlaybackState> playback_state_;
    static std::atomic<PlayMode> play_mode_;

    // Pattern state
    static char current_pattern_uuid_[MAX_UUID_LEN];
    static std::optional<Pattern> current_pattern_;
    static std::unique_ptr<IPatternReader> pattern_reader_;
    static size_t current_line_index_;
    static size_t total_lines_;
    static bool file_loaded_;

    // Path-length-based progress. Line counting misrepresents progress when
    // lines differ wildly in drawn length (a single multi-rotation sweep line
    // is minutes of motion), so progress is measured as path length instead:
    // the file is pre-scanned once at load for the total (cached per UUID so
    // loop replays skip the scan) and popLine() accumulates the same
    // per-segment metric as points are fed. Mutated under file_mutex_; the
    // published percent is atomic so status readers never tear a double.
    static double total_path_len_;      // whole pattern, normalized units
    static double completed_path_len_;  // fed to the planner so far
    static double prev_point_theta_;    // last popped point (file space)
    static double prev_point_rho_;
    static bool have_prev_point_;
    static std::atomic<int> path_progress_percent_;  // -1: no path data, line fallback
    static char prescan_uuid_[MAX_UUID_LEN];
    static double prescan_total_;

    // Playlist state
    static char current_playlist_uuid_[MAX_UUID_LEN];
    static std::vector<std::string> playlist_patterns_;
    static std::vector<size_t> playlist_order_;
    static size_t playlist_index_;
    static std::atomic<bool> is_shuffle_;
    static std::atomic<bool> is_loop_;

    // Pattern position tracking
    static PatternPosition pattern_position_;

    // Configuration. Atomic: written from API threads, read by the service
    // task mid-feed without the state mutex (a plain double store/load can
    // tear on this target).
    static std::atomic<double> feed_rate_;
    static std::atomic<bool> linear_interpolation_;

    // Speed multiplier for moves that had full rotations folded out of them
    // (transit moves, not drawing) - hurry through, don't draw slowly.
    static constexpr float kFoldedMoveSpeedMultiplier = 3.0f;

    // Constants
    static const char* TAG;
    static constexpr uint32_t SERVICE_TASK_DELAY_MS = 10;
    // Upper bound on pattern lines fed per service tick — keeps one tick's
    // work bounded while still saturating the motion queue (the queue-full
    // return from processPatternLine is the real throttle).
    static constexpr int kMaxLinesPerTick = 64;
    // 12 KB: the service task nests pattern open (encrypted: RSA key
    // recovery + PSA cipher setup), the path pre-scan, manifest lookups
    // (cJSON + std::string) and float-heavy logging. 4 KB overflowed.
    static constexpr size_t SERVICE_TASK_STACK_SIZE = 12288;
    static constexpr UBaseType_t SERVICE_TASK_PRIORITY = 5;
    // Random pick: bounded retries when a drawn pattern turns out to be an
    // unauthorized encrypted one (its verdict is cached, so a retry is
    // cheap; the bound only guards a library with nothing playable).
    static constexpr int kMaxRandomPickAttempts = 64;
};
