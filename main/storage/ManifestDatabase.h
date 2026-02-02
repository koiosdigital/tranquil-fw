#pragma once

#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <cstdint>
#include "esp_err.h"
#include "cJSON.h"

// Forward declarations for job types
namespace jobs {
    enum class JobType : uint8_t;
    enum class JobStatus : uint8_t;
    struct Job;
}

// Pattern metadata
struct Pattern {
    std::string uuid;
    std::string name;
    std::string creator;
    std::string date;
    int popularity = 0;

    // Extended metadata
    bool reversible = false;
    int start_point = 0;      // 0=center, 1=edge
    bool encrypted = false;
    size_t size_bytes = 0;
    std::string created_at;
    std::string last_played_at;
    std::string downloaded_at;
};

// Playlist metadata
struct Playlist {
    std::string uuid;
    std::string name;
    std::string description;
    std::vector<std::string> patterns;  // Pattern UUIDs in order
    std::string featured_pattern;
    std::string date;
    std::string created_at;
    std::string updated_at;
};

// Pagination info returned with list queries
struct PaginationInfo {
    int page = 0;
    int per_page = 20;
    int total_pages = 0;
    int total_items = 0;
};

// Paginated result wrapper
template<typename T>
struct PaginatedResult {
    std::vector<T> items;
    PaginationInfo pagination;
};

// SQLite-backed manifest database
// Thread-safe singleton with RAII resource management
class ManifestDatabase {
public:
    static ManifestDatabase& instance();

    // Lifecycle
    esp_err_t initialize();
    void shutdown();
    bool isInitialized() const;

    // Pattern CRUD
    esp_err_t addPattern(const Pattern& pattern);
    esp_err_t updatePattern(const std::string& uuid, const Pattern& pattern);
    esp_err_t deletePattern(const std::string& uuid);

    std::vector<Pattern> getAllPatterns();
    PaginatedResult<Pattern> getPatterns(int page = 0, int per_page = 20);
    std::optional<Pattern> getPattern(const std::string& uuid);
    bool patternExists(const std::string& uuid);
    size_t getPatternCount();

    // Pattern-specific updates
    esp_err_t updateLastPlayed(const std::string& uuid);
    esp_err_t incrementPopularity(const std::string& uuid);

    // Playlist CRUD
    esp_err_t addPlaylist(const Playlist& playlist);
    esp_err_t updatePlaylist(const std::string& uuid, const Playlist& playlist);
    esp_err_t deletePlaylist(const std::string& uuid);

    std::vector<Playlist> getAllPlaylists();
    PaginatedResult<Playlist> getPlaylists(int page = 0, int per_page = 20);
    std::optional<Playlist> getPlaylist(const std::string& uuid);
    bool playlistExists(const std::string& uuid);
    size_t getPlaylistCount();

    // Playlist-pattern operations
    esp_err_t addPatternToPlaylist(const std::string& playlistUuid, const std::string& patternUuid);
    esp_err_t removePatternFromPlaylist(const std::string& playlistUuid, const std::string& patternUuid);
    esp_err_t setFeaturedPattern(const std::string& playlistUuid, const std::string& patternUuid);
    esp_err_t reorderPlaylist(const std::string& playlistUuid, const std::vector<std::string>& newOrder);

    // Get patterns for a playlist (full Pattern objects, ordered)
    std::vector<Pattern> getPlaylistPatterns(const std::string& playlistUuid);

    // UUID generation
    static std::string generateUUID();

    // JSON conversion (for HTTP API)
    static cJSON* patternToJson(const Pattern& pattern);
    static cJSON* playlistToJson(const Playlist& playlist);
    static Pattern jsonToPattern(const cJSON* json);
    static Playlist jsonToPlaylist(const cJSON* json);

    // Utility
    static std::string currentTimestamp();

    // Memory management
    void releaseMemory();   // Release SQLite page cache
    esp_err_t vacuum();     // Compact database file (reclaim deleted space)

    // Job queue operations
    esp_err_t enqueueJob(const jobs::Job& job);
    std::optional<jobs::Job> claimNextPendingJob();
    std::optional<jobs::Job> getJob(const std::string& uuid);
    std::optional<jobs::Job> getJobByPattern(const std::string& pattern_uuid, jobs::JobType type);
    bool hasJob(const std::string& pattern_uuid, jobs::JobType type);
    esp_err_t markJobCompleted(const std::string& uuid);
    esp_err_t markJobFailed(const std::string& uuid, const std::string& error);
    esp_err_t deleteJob(const std::string& uuid);
    esp_err_t cancelJobsForPattern(const std::string& pattern_uuid);
    size_t getPendingJobCount();
    size_t getInProgressJobCount();

private:
    ManifestDatabase();
    ~ManifestDatabase();
    ManifestDatabase(const ManifestDatabase&) = delete;
    ManifestDatabase& operator=(const ManifestDatabase&) = delete;

    // Pimpl - hides SQLite implementation
    class Impl;
    std::unique_ptr<Impl> impl_;
};

// Backward compatibility alias
using ManifestManager = ManifestDatabase;
