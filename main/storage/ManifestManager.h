#pragma once

#include <string>
#include <vector>
#include "esp_err.h"
#include "cJSON.h"

// Pattern structure matching the TypeScript interface
struct Pattern {
    std::string uuid;
    std::string name;
    std::string date;
    int popularity;
    std::string creator;

    Pattern() : popularity(0) {}
    Pattern(const std::string& id, const std::string& n, const std::string& d,
        int pop, const std::string& c)
        : uuid(id), name(n), date(d), popularity(pop), creator(c) {
    }
};

// Playlist structure matching the TypeScript interface
struct Playlist {
    std::string uuid;
    std::string name;
    std::string description;
    std::vector<std::string> patterns;
    std::string featured_pattern;
    std::string date;

    Playlist() {}
    Playlist(const std::string& id, const std::string& n, const std::string& desc,
        const std::vector<std::string>& pats, const std::string& featured,
        const std::string& d)
        : uuid(id), name(n), description(desc), patterns(pats), featured_pattern(featured), date(d) {
    }
};

class ManifestManager {
private:
    static const char* MANIFEST_PATH;
    static const char* TAG;

    static cJSON* manifest_json;
    static cJSON* patterns_array;
    static cJSON* playlists_array;
    static bool initialized_;

    // Prevent instantiation
    ManifestManager() = delete;
    ~ManifestManager() = delete;
    ManifestManager(const ManifestManager&) = delete;
    ManifestManager& operator=(const ManifestManager&) = delete;

    // Helper functions
    static esp_err_t loadManifest();
    static esp_err_t saveManifest();
    static esp_err_t createEmptyManifest();
    static std::string generateUUID();

public:
    // Static initialization - calls sd_init and initializes manifest
    static esp_err_t initialize();
    static void shutdown();

    // Pattern CRUD operations
    static esp_err_t addPattern(const Pattern& pattern);
    static esp_err_t updatePattern(const std::string& uuid, const Pattern& pattern);
    static esp_err_t deletePattern(const std::string& uuid);
    static std::vector<Pattern> getAllPatterns();
    static Pattern* getPattern(const std::string& uuid);
    static bool patternExists(const std::string& uuid);

    // Playlist CRUD operations
    static esp_err_t addPlaylist(const Playlist& playlist);
    static esp_err_t updatePlaylist(const std::string& uuid, const Playlist& playlist);
    static esp_err_t deletePlaylist(const std::string& uuid);
    static std::vector<Playlist> getAllPlaylists();
    static Playlist* getPlaylist(const std::string& uuid);
    static bool playlistExists(const std::string& uuid);

    static cJSON* patternToJson(const Pattern& pattern);
    static cJSON* playlistToJson(const Playlist& playlist);
    static Pattern jsonToPattern(const cJSON* json);
    static Playlist jsonToPlaylist(const cJSON* json);

    // Utility functions
    static esp_err_t addPatternToPlaylist(const std::string& playlistUuid, const std::string& patternUuid);
    static esp_err_t removePatternFromPlaylist(const std::string& playlistUuid, const std::string& patternUuid);
    static esp_err_t setFeaturedPattern(const std::string& playlistUuid, const std::string& patternUuid);

    // Reorder patterns in a playlist
    static bool reorderPlaylist(const std::string& playlistUuid, const std::vector<std::string>& newOrder);

    // Statistics
    static size_t getPatternCount();
    static size_t getPlaylistCount();
};
