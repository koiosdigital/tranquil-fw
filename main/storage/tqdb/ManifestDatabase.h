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
    uint32_t id = 0;              // TQDB auto-increment primary key
    std::string external_uuid;    // Server UUID for linking to server-downloaded patterns
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

    // Purchase/ownership metadata (for encrypted patterns)
    bool purchased = false;       // True if owned via purchase receipt
    int64_t purchased_at = 0;     // Unix timestamp, 0 if not owned
    std::string receipt_id;       // Receipt ID for support, empty if not owned
};

// Playlist metadata
struct Playlist {
    uint32_t id = 0;                    // TQDB auto-increment primary key
    std::string external_uuid;          // Server UUID for linking to server-downloaded playlists
    std::string name;
    std::string description;
    std::vector<uint32_t> pattern_ids;  // Pattern IDs in order (internal references)
    uint32_t featured_pattern_id = 0;   // Featured pattern ID (0 = none)
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

// Binary file-backed manifest database (.tqdb format)
// Thread-safe singleton with RAII resource management
class ManifestDatabase {
public:
    static ManifestDatabase& instance();

    // Lifecycle
    esp_err_t initialize();
    void shutdown();
    bool isInitialized() const;

    // Pattern CRUD (by internal ID)
    esp_err_t addPattern(Pattern& pattern);  // ID assigned on success
    esp_err_t updatePattern(uint32_t id, const Pattern& pattern);
    esp_err_t deletePattern(uint32_t id);

    std::vector<Pattern> getAllPatterns();
    PaginatedResult<Pattern> getPatterns(int page = 0, int per_page = 20);
    std::optional<Pattern> getPattern(uint32_t id);
    bool patternExists(uint32_t id);
    size_t getPatternCount();
    size_t getSubscriptionPatternCount();  // Count patterns where purchased=false

    // Pattern lookup by external UUID (for API compatibility)
    std::optional<Pattern> getPatternByExternalUuid(const std::string& uuid);
    std::optional<uint32_t> findPatternIdByExternalUuid(const std::string& uuid);

    // Pattern-specific updates
    esp_err_t updateLastPlayed(uint32_t id);
    esp_err_t incrementPopularity(uint32_t id);

    // Playlist CRUD (by internal ID)
    esp_err_t addPlaylist(Playlist& playlist);  // ID assigned on success
    esp_err_t updatePlaylist(uint32_t id, const Playlist& playlist);
    esp_err_t deletePlaylist(uint32_t id);

    std::vector<Playlist> getAllPlaylists();
    PaginatedResult<Playlist> getPlaylists(int page = 0, int per_page = 20);
    std::optional<Playlist> getPlaylist(uint32_t id);
    bool playlistExists(uint32_t id);
    size_t getPlaylistCount();

    // Playlist lookup by external UUID (for API compatibility)
    std::optional<Playlist> getPlaylistByExternalUuid(const std::string& uuid);
    std::optional<uint32_t> findPlaylistIdByExternalUuid(const std::string& uuid);

    // Playlist-pattern operations (using internal IDs)
    esp_err_t addPatternToPlaylist(uint32_t playlistId, uint32_t patternId);
    esp_err_t removePatternFromPlaylist(uint32_t playlistId, uint32_t patternId);
    esp_err_t setFeaturedPattern(uint32_t playlistId, uint32_t patternId);
    esp_err_t reorderPlaylist(uint32_t playlistId, const std::vector<uint32_t>& newOrder);

    // Get patterns for a playlist (full Pattern objects, ordered)
    std::vector<Pattern> getPlaylistPatterns(uint32_t playlistId);

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
    void releaseMemory();   // No-op, kept for API compatibility
    esp_err_t vacuum();     // Compact database file (reclaim deleted space)

    // Job queue operations (using internal IDs)
    esp_err_t enqueueJob(jobs::Job& job);  // ID assigned on success
    std::optional<jobs::Job> claimNextPendingJob();
    // Re-queue jobs left InProgress by a reboot mid-execution and purge
    // finished (Completed/Failed) rows so the table doesn't grow forever.
    // Returns the number recovered. Call once at startup before processing
    // begins.
    size_t recoverOrphanedJobs();
    std::optional<jobs::Job> getJob(uint32_t id);
    std::optional<jobs::Job> getJobByPatternId(uint32_t pattern_id, jobs::JobType type);
    std::optional<jobs::Job> getJobByPatternExternalUuid(const std::string& external_uuid, jobs::JobType type);
    bool hasJobForPattern(uint32_t pattern_id, jobs::JobType type);
    bool hasJobForPatternExternalUuid(const std::string& external_uuid, jobs::JobType type);
    esp_err_t markJobCompleted(uint32_t id);
    // permanent=true sends the job straight to Failed (no retry).
    esp_err_t markJobFailed(uint32_t id, const std::string& error, bool permanent = false);
    // Put a claimed (InProgress) job back to Pending WITHOUT burning a retry.
    esp_err_t releaseJob(uint32_t id);
    esp_err_t deleteJob(uint32_t id);
    esp_err_t cancelJobsForPattern(uint32_t pattern_id);
    // Cancel Pending jobs matched by the external UUID stored on the job row
    // (download jobs are enqueued with pattern_id=0 before the pattern exists).
    esp_err_t cancelJobsForExternalUuid(const std::string& external_uuid);
    size_t getPendingJobCount();
    size_t getInProgressJobCount();

    // Internal access for JSON conversion (implementation detail)
    class Impl;
    Impl* getImpl() { return impl_.get(); }

private:
    ManifestDatabase();
    ~ManifestDatabase();
    ManifestDatabase(const ManifestDatabase&) = delete;
    ManifestDatabase& operator=(const ManifestDatabase&) = delete;

    std::unique_ptr<Impl> impl_;
};

// Backward compatibility alias
using ManifestManager = ManifestDatabase;
