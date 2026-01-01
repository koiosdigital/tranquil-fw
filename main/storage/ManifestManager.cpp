#include "ManifestManager.h"
#include "sd.h"
#include "esp_log.h"
#include "esp_random.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <algorithm>
#include <sys/stat.h>
#include <errno.h>
#include <dirent.h>

// Static member definitions
const char* ManifestManager::MANIFEST_PATH = "/sd/manifest.json";
const char* ManifestManager::TAG = "ManifestManager";
cJSON* ManifestManager::manifest_json = nullptr;
cJSON* ManifestManager::patterns_array = nullptr;
cJSON* ManifestManager::playlists_array = nullptr;
bool ManifestManager::initialized_ = false;

esp_err_t ManifestManager::initialize() {
    if (initialized_) return ESP_OK;

    init_sd();

    // Ensure pattern directory exists
    struct stat st = { 0 };
    if (stat("/sd/patterns", &st) == -1) {
        if (mkdir("/sd/patterns", 0775) != 0) {
            ESP_LOGE(TAG, "Failed to create pattern directory: /sd/patterns (%d)", errno);
            return ESP_FAIL;
        }
    }

    //delete existing manifest if it exists
    //if (remove(MANIFEST_PATH) != 0) {
    //    ESP_LOGE(TAG, "Failed to delete existing manifest file");
    //}

    // Try to load existing manifest
    esp_err_t ret = loadManifest();
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "No existing manifest found, creating new one");
        ret = createEmptyManifest();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create empty manifest");
            return ret;
        }
        ret = saveManifest();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save initial manifest");
            return ret;
        }
    }

    initialized_ = true;
    ESP_LOGI(TAG, "initialized successfully");
    return ESP_OK;
}

void ManifestManager::shutdown() {
    if (!initialized_) return;

    if (manifest_json) {
        cJSON_Delete(manifest_json);
        manifest_json = nullptr;
        patterns_array = nullptr;
        playlists_array = nullptr;
    }

    initialized_ = false;
    ESP_LOGI(TAG, "ManifestManager shutdown");
}

esp_err_t ManifestManager::loadManifest() {
    FILE* file = fopen(MANIFEST_PATH, "r");
    if (!file) {
        ESP_LOGW(TAG, "Manifest file not found: %s", MANIFEST_PATH);
        return ESP_ERR_NOT_FOUND;
    }

    // Get file size
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (file_size <= 0) {
        ESP_LOGW(TAG, "Manifest file is empty");
        fclose(file);
        return ESP_ERR_INVALID_SIZE;
    }

    // Allocate buffer and read file content
    char* content = (char*)malloc(file_size + 1);
    if (!content) {
        ESP_LOGE(TAG, "Failed to allocate memory for file content");
        fclose(file);
        return ESP_ERR_NO_MEM;
    }

    size_t bytes_read = fread(content, 1, file_size, file);
    fclose(file);

    if (bytes_read != file_size) {
        ESP_LOGE(TAG, "Failed to read complete file");
        free(content);
        return ESP_ERR_INVALID_SIZE;
    }

    content[file_size] = '\0'; // Null terminate

    // Parse JSON
    manifest_json = cJSON_Parse(content);
    free(content);

    if (!manifest_json) {
        ESP_LOGE(TAG, "Failed to parse manifest JSON");
        return ESP_ERR_INVALID_ARG;
    }

    // Get arrays
    patterns_array = cJSON_GetObjectItem(manifest_json, "patterns");
    playlists_array = cJSON_GetObjectItem(manifest_json, "playlists");

    if (!patterns_array || !playlists_array) {
        ESP_LOGE(TAG, "Invalid manifest structure - missing patterns or playlists array");
        cJSON_Delete(manifest_json);
        manifest_json = nullptr;
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Manifest loaded successfully - %d patterns, %d playlists",
        cJSON_GetArraySize(patterns_array), cJSON_GetArraySize(playlists_array));

    return ESP_OK;
}

esp_err_t ManifestManager::saveManifest() {
    if (!manifest_json) {
        ESP_LOGE(TAG, "No manifest data to save");
        return ESP_ERR_INVALID_STATE;
    }

    char* json_string = cJSON_Print(manifest_json);
    if (!json_string) {
        ESP_LOGE(TAG, "Failed to convert manifest to JSON string");
        return ESP_ERR_NO_MEM;
    }

    FILE* file = fopen(MANIFEST_PATH, "w");
    if (file == nullptr) {
        ESP_LOGE(TAG, "Failed to open manifest file for writing: %s", MANIFEST_PATH);
        free(json_string);
        return ESP_ERR_NOT_FOUND;
    }

    size_t json_len = strlen(json_string);
    size_t bytes_written = fwrite(json_string, 1, json_len, file);
    fclose(file);
    free(json_string);

    if (bytes_written != json_len) {
        ESP_LOGE(TAG, "Failed to write complete manifest file");
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGD(TAG, "Manifest saved successfully");
    return ESP_OK;
}

esp_err_t ManifestManager::createEmptyManifest() {
    manifest_json = cJSON_CreateObject();
    if (!manifest_json) {
        ESP_LOGE(TAG, "Failed to create manifest JSON object");
        return ESP_ERR_NO_MEM;
    }

    patterns_array = cJSON_CreateArray();
    playlists_array = cJSON_CreateArray();

    if (!patterns_array || !playlists_array) {
        ESP_LOGE(TAG, "Failed to create arrays");
        cJSON_Delete(manifest_json);
        manifest_json = nullptr;
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddItemToObject(manifest_json, "patterns", patterns_array);
    cJSON_AddItemToObject(manifest_json, "playlists", playlists_array);

    ESP_LOGI(TAG, "Empty manifest created");
    return ESP_OK;
}

std::string ManifestManager::generateUUID() {
    uint32_t random1 = esp_random();
    uint32_t random2 = esp_random();
    uint32_t random3 = esp_random();
    uint32_t random4 = esp_random();

    char uuid_str[37]; // 36 characters + null terminator
    snprintf(uuid_str, sizeof(uuid_str), "%08x-%04x-%04x-%04x-%08x%04x",
        (unsigned int)random1,
        (unsigned int)((random2 >> 16) & 0xFFFF),
        (unsigned int)(random2 & 0xFFFF),
        (unsigned int)((random3 >> 16) & 0xFFFF),
        (unsigned int)random4,
        (unsigned int)(random3 & 0xFFFF));

    return std::string(uuid_str);
}

cJSON* ManifestManager::patternToJson(const Pattern& pattern) {
    cJSON* json = cJSON_CreateObject();
    if (!json) return nullptr;

    cJSON_AddStringToObject(json, "uuid", pattern.uuid.c_str());
    cJSON_AddStringToObject(json, "name", pattern.name.c_str());
    cJSON_AddStringToObject(json, "date", pattern.date.c_str());
    cJSON_AddNumberToObject(json, "popularity", pattern.popularity);
    cJSON_AddStringToObject(json, "creator", pattern.creator.c_str());

    return json;
}

cJSON* ManifestManager::playlistToJson(const Playlist& playlist) {
    cJSON* json = cJSON_CreateObject();
    if (!json) return nullptr;

    cJSON_AddStringToObject(json, "uuid", playlist.uuid.c_str());
    cJSON_AddStringToObject(json, "name", playlist.name.c_str());
    cJSON_AddStringToObject(json, "description", playlist.description.c_str());
    cJSON_AddStringToObject(json, "featured_pattern", playlist.featured_pattern.c_str());
    cJSON_AddStringToObject(json, "date", playlist.date.c_str());

    // Add patterns array
    cJSON* patterns_json = cJSON_CreateArray();
    for (const auto& pattern_uuid : playlist.patterns) {
        cJSON_AddItemToArray(patterns_json, cJSON_CreateString(pattern_uuid.c_str()));
    }
    cJSON_AddItemToObject(json, "patterns", patterns_json);

    return json;
}

Pattern ManifestManager::jsonToPattern(const cJSON* json) {
    Pattern pattern;

    const cJSON* uuid = cJSON_GetObjectItem(json, "uuid");
    const cJSON* name = cJSON_GetObjectItem(json, "name");
    const cJSON* date = cJSON_GetObjectItem(json, "date");
    const cJSON* popularity = cJSON_GetObjectItem(json, "popularity");
    const cJSON* creator = cJSON_GetObjectItem(json, "creator");

    if (cJSON_IsString(uuid)) pattern.uuid = uuid->valuestring;
    if (cJSON_IsString(name)) pattern.name = name->valuestring;
    if (cJSON_IsString(date)) pattern.date = date->valuestring;
    if (cJSON_IsNumber(popularity)) pattern.popularity = popularity->valueint;
    if (cJSON_IsString(creator)) pattern.creator = creator->valuestring;

    return pattern;
}

Playlist ManifestManager::jsonToPlaylist(const cJSON* json) {
    Playlist playlist;

    const cJSON* uuid = cJSON_GetObjectItem(json, "uuid");
    const cJSON* name = cJSON_GetObjectItem(json, "name");
    const cJSON* description = cJSON_GetObjectItem(json, "description");
    const cJSON* featured_pattern = cJSON_GetObjectItem(json, "featured_pattern");
    const cJSON* date = cJSON_GetObjectItem(json, "date");
    const cJSON* patterns = cJSON_GetObjectItem(json, "patterns");

    if (cJSON_IsString(uuid)) playlist.uuid = uuid->valuestring;
    if (cJSON_IsString(name)) playlist.name = name->valuestring;
    if (cJSON_IsString(description)) playlist.description = description->valuestring;
    if (cJSON_IsString(featured_pattern)) playlist.featured_pattern = featured_pattern->valuestring;
    if (cJSON_IsString(date)) playlist.date = date->valuestring;

    if (cJSON_IsArray(patterns)) {
        const cJSON* pattern_uuid = nullptr;
        cJSON_ArrayForEach(pattern_uuid, patterns) {
            if (cJSON_IsString(pattern_uuid)) {
                playlist.patterns.push_back(pattern_uuid->valuestring);
            }
        }
    }

    return playlist;
}

// Pattern CRUD operations
esp_err_t ManifestManager::addPattern(const Pattern& pattern) {
    if (!initialized_) {
        ESP_LOGE(TAG, "ManifestManager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!patterns_array) {
        ESP_LOGE(TAG, "Patterns array not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Check if pattern already exists
    if (patternExists(pattern.uuid)) {
        ESP_LOGW(TAG, "Pattern with UUID %s already exists", pattern.uuid.c_str());
        return ESP_ERR_INVALID_ARG;
    }

    cJSON* pattern_json = patternToJson(pattern);
    if (!pattern_json) {
        ESP_LOGE(TAG, "Failed to convert pattern to JSON");
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddItemToArray(patterns_array, pattern_json);

    esp_err_t ret = saveManifest();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save manifest after adding pattern");
        return ret;
    }

    ESP_LOGI(TAG, "Pattern added: %s (%s)", pattern.name.c_str(), pattern.uuid.c_str());
    return ESP_OK;
}

esp_err_t ManifestManager::updatePattern(const std::string& uuid, const Pattern& pattern) {
    if (!patterns_array) {
        ESP_LOGE(TAG, "Patterns array not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Find and update pattern
    int size = cJSON_GetArraySize(patterns_array);
    for (int i = 0; i < size; i++) {
        cJSON* item = cJSON_GetArrayItem(patterns_array, i);
        const cJSON* item_uuid = cJSON_GetObjectItem(item, "uuid");

        if (cJSON_IsString(item_uuid) && strcmp(item_uuid->valuestring, uuid.c_str()) == 0) {
            // Replace the item
            cJSON* new_pattern = patternToJson(pattern);
            if (!new_pattern) {
                ESP_LOGE(TAG, "Failed to convert pattern to JSON");
                return ESP_ERR_NO_MEM;
            }

            cJSON_ReplaceItemInArray(patterns_array, i, new_pattern);

            esp_err_t ret = saveManifest();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save manifest after updating pattern");
                return ret;
            }

            ESP_LOGI(TAG, "Pattern updated: %s", uuid.c_str());
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "Pattern not found: %s", uuid.c_str());
    return ESP_ERR_NOT_FOUND;
}

esp_err_t ManifestManager::deletePattern(const std::string& uuid) {
    if (!patterns_array) {
        ESP_LOGE(TAG, "Patterns array not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    //Remove pattern from all playlists it is in
    int playlist_size = cJSON_GetArraySize(playlists_array);
    for (int i = 0; i < playlist_size; i++) {
        cJSON* playlist_item = cJSON_GetArrayItem(playlists_array, i);
        cJSON* patterns = cJSON_GetObjectItem(playlist_item, "patterns");
        if (cJSON_IsArray(patterns)) {
            int patterns_count = cJSON_GetArraySize(patterns);
            for (int j = 0; j < patterns_count; j++) {
                cJSON* pattern_item = cJSON_GetArrayItem(patterns, j);
                if (cJSON_IsString(pattern_item) && strcmp(pattern_item->valuestring, uuid.c_str()) == 0) {
                    // Remove pattern from playlist
                    cJSON_DeleteItemFromArray(patterns, j);
                    ESP_LOGI(TAG, "Removed pattern %s from playlist %s", uuid.c_str(), cJSON_GetObjectItem(playlist_item, "uuid")->valuestring);
                    // Save manifest after each removal
                    esp_err_t ret = saveManifest();
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to save manifest after removing pattern from playlist");
                        return ret;
                    }
                }
            }
        }
    }

    // Find and delete pattern
    int size = cJSON_GetArraySize(patterns_array);
    for (int i = 0; i < size; i++) {
        cJSON* item = cJSON_GetArrayItem(patterns_array, i);
        const cJSON* item_uuid = cJSON_GetObjectItem(item, "uuid");

        if (cJSON_IsString(item_uuid) && uuid == item_uuid->valuestring) {
            cJSON_DeleteItemFromArray(patterns_array, i);

            esp_err_t ret = saveManifest();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save manifest after deleting pattern");
                return ret;
            }

            ESP_LOGI(TAG, "Pattern deleted: %s", uuid.c_str());
            return ESP_OK;
        }
    }

    //Delete pattern file from filesystem
    std::string pattern_file_path = "/sd/patterns/" + uuid + ".thr";
    if (remove(pattern_file_path.c_str()) != 0) {
        ESP_LOGE(TAG, "Failed to delete pattern file: %s (%d)", pattern_file_path.c_str(), errno);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "Pattern not found: %s", uuid.c_str());
    return ESP_ERR_NOT_FOUND;
}

std::vector<Pattern> ManifestManager::getAllPatterns() {
    std::vector<Pattern> patterns;

    if (!initialized_ || !patterns_array) {
        ESP_LOGW(TAG, "ManifestManager not initialized");
        return patterns;
    }

    int size = cJSON_GetArraySize(patterns_array);
    for (int i = 0; i < size; i++) {
        cJSON* item = cJSON_GetArrayItem(patterns_array, i);
        patterns.push_back(jsonToPattern(item));
    }

    return patterns;
}

Pattern* ManifestManager::getPattern(const std::string& uuid) {
    if (!patterns_array) {
        ESP_LOGW(TAG, "Patterns array not initialized");
        return nullptr;
    }

    int size = cJSON_GetArraySize(patterns_array);
    for (int i = 0; i < size; i++) {
        cJSON* item = cJSON_GetArrayItem(patterns_array, i);
        const cJSON* item_uuid = cJSON_GetObjectItem(item, "uuid");

        if (cJSON_IsString(item_uuid) && strcmp(item_uuid->valuestring, uuid.c_str()) == 0) {
            static Pattern pattern; // Static to keep alive after return
            pattern = jsonToPattern(item);
            return &pattern;
        }
    }

    return nullptr;
}

bool ManifestManager::patternExists(const std::string& uuid) {
    return getPattern(uuid) != nullptr;
}

// Playlist CRUD operations
esp_err_t ManifestManager::addPlaylist(const Playlist& playlist) {
    if (!playlists_array) {
        ESP_LOGE(TAG, "Playlists array not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Check if playlist already exists
    if (playlistExists(playlist.uuid)) {
        ESP_LOGW(TAG, "Playlist with UUID %s already exists", playlist.uuid.c_str());
        return ESP_ERR_INVALID_ARG;
    }

    cJSON* playlist_json = playlistToJson(playlist);
    if (!playlist_json) {
        ESP_LOGE(TAG, "Failed to convert playlist to JSON");
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddItemToArray(playlists_array, playlist_json);

    esp_err_t ret = saveManifest();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save manifest after adding playlist");
        return ret;
    }

    ESP_LOGI(TAG, "Playlist added: %s (%s)", playlist.name.c_str(), playlist.uuid.c_str());
    return ESP_OK;
}

esp_err_t ManifestManager::updatePlaylist(const std::string& uuid, const Playlist& playlist) {
    if (!playlists_array) {
        ESP_LOGE(TAG, "Playlists array not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Find and update playlist
    int size = cJSON_GetArraySize(playlists_array);
    for (int i = 0; i < size; i++) {
        cJSON* item = cJSON_GetArrayItem(playlists_array, i);
        const cJSON* item_uuid = cJSON_GetObjectItem(item, "uuid");

        if (cJSON_IsString(item_uuid) && strcmp(item_uuid->valuestring, uuid.c_str()) == 0) {
            // Replace the item
            cJSON* new_playlist = playlistToJson(playlist);
            if (!new_playlist) {
                ESP_LOGE(TAG, "Failed to convert playlist to JSON");
                return ESP_ERR_NO_MEM;
            }

            cJSON_ReplaceItemInArray(playlists_array, i, new_playlist);

            esp_err_t ret = saveManifest();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save manifest after updating playlist");
                return ret;
            }

            ESP_LOGI(TAG, "Playlist updated: %s", uuid.c_str());
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "Playlist not found: %s", uuid.c_str());
    return ESP_ERR_NOT_FOUND;
}

esp_err_t ManifestManager::deletePlaylist(const std::string& uuid) {
    if (!playlists_array) {
        ESP_LOGE(TAG, "Playlists array not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Find and delete playlist
    int size = cJSON_GetArraySize(playlists_array);
    for (int i = 0; i < size; i++) {
        cJSON* item = cJSON_GetArrayItem(playlists_array, i);
        const cJSON* item_uuid = cJSON_GetObjectItem(item, "uuid");

        if (cJSON_IsString(item_uuid) && strcmp(item_uuid->valuestring, uuid.c_str()) == 0) {
            cJSON_DeleteItemFromArray(playlists_array, i);

            esp_err_t ret = saveManifest();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save manifest after deleting playlist");
                return ret;
            }

            ESP_LOGI(TAG, "Playlist deleted: %s", uuid.c_str());
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "Playlist not found: %s", uuid.c_str());
    return ESP_ERR_NOT_FOUND;
}

std::vector<Playlist> ManifestManager::getAllPlaylists() {
    std::vector<Playlist> playlists;

    if (!playlists_array) {
        ESP_LOGW(TAG, "Playlists array not initialized");
        return playlists;
    }

    int size = cJSON_GetArraySize(playlists_array);
    for (int i = 0; i < size; i++) {
        cJSON* item = cJSON_GetArrayItem(playlists_array, i);
        playlists.push_back(jsonToPlaylist(item));
    }

    return playlists;
}

Playlist* ManifestManager::getPlaylist(const std::string& uuid) {
    if (!playlists_array) {
        ESP_LOGW(TAG, "Playlists array not initialized");
        return nullptr;
    }

    int size = cJSON_GetArraySize(playlists_array);
    for (int i = 0; i < size; i++) {
        cJSON* item = cJSON_GetArrayItem(playlists_array, i);
        const cJSON* item_uuid = cJSON_GetObjectItem(item, "uuid");

        if (cJSON_IsString(item_uuid) && strcmp(item_uuid->valuestring, uuid.c_str()) == 0) {
            static Playlist playlist; // Static to keep alive after return
            playlist = jsonToPlaylist(item);
            return &playlist;
        }
    }

    return nullptr;
}

bool ManifestManager::playlistExists(const std::string& uuid) {
    return getPlaylist(uuid) != nullptr;
}

// Utility functions
esp_err_t ManifestManager::addPatternToPlaylist(const std::string& playlistUuid, const std::string& patternUuid) {
    Playlist* playlist = getPlaylist(playlistUuid);
    if (!playlist) {
        ESP_LOGW(TAG, "Playlist not found: %s", playlistUuid.c_str());
        return ESP_ERR_NOT_FOUND;
    }

    // Check if pattern already in playlist
    auto it = std::find(playlist->patterns.begin(), playlist->patterns.end(), patternUuid);
    if (it != playlist->patterns.end()) {
        ESP_LOGW(TAG, "Pattern %s already in playlist %s", patternUuid.c_str(), playlistUuid.c_str());
        return ESP_ERR_INVALID_ARG;
    }

    playlist->patterns.push_back(patternUuid);
    return updatePlaylist(playlistUuid, *playlist);
}

esp_err_t ManifestManager::removePatternFromPlaylist(const std::string& playlistUuid, const std::string& patternUuid) {
    Playlist* playlist = getPlaylist(playlistUuid);
    if (!playlist) {
        ESP_LOGW(TAG, "Playlist not found: %s", playlistUuid.c_str());
        return ESP_ERR_NOT_FOUND;
    }

    auto it = std::find(playlist->patterns.begin(), playlist->patterns.end(), patternUuid);
    if (it == playlist->patterns.end()) {
        ESP_LOGW(TAG, "Pattern %s not found in playlist %s", patternUuid.c_str(), playlistUuid.c_str());
        return ESP_ERR_NOT_FOUND;
    }

    playlist->patterns.erase(it);

    // If this was the featured pattern, clear it
    if (playlist->featured_pattern == patternUuid) {
        playlist->featured_pattern.clear();
    }

    return updatePlaylist(playlistUuid, *playlist);
}

esp_err_t ManifestManager::setFeaturedPattern(const std::string& playlistUuid, const std::string& patternUuid) {
    Playlist* playlist = getPlaylist(playlistUuid);
    if (!playlist) {
        ESP_LOGW(TAG, "Playlist not found: %s", playlistUuid.c_str());
        return ESP_ERR_NOT_FOUND;
    }

    // Check if pattern is in the playlist
    auto it = std::find(playlist->patterns.begin(), playlist->patterns.end(), patternUuid);
    if (it == playlist->patterns.end()) {
        ESP_LOGW(TAG, "Pattern %s not found in playlist %s", patternUuid.c_str(), playlistUuid.c_str());
        return ESP_ERR_NOT_FOUND;
    }

    playlist->featured_pattern = patternUuid;
    return updatePlaylist(playlistUuid, *playlist);
}

bool ManifestManager::reorderPlaylist(const std::string& playlistUuid, const std::vector<std::string>& newOrder) {
    Playlist* playlist = getPlaylist(playlistUuid);
    if (!playlist) {
        ESP_LOGW(TAG, "Playlist not found: %s", playlistUuid.c_str());
        return false;
    }
    // Validate all UUIDs exist in the playlist
    for (const auto& uuid : newOrder) {
        if (std::find(playlist->patterns.begin(), playlist->patterns.end(), uuid) == playlist->patterns.end()) {
            ESP_LOGW(TAG, "Pattern %s not found in playlist %s", uuid.c_str(), playlistUuid.c_str());
            return false;
        }
    }
    // Validate no duplicates and same size
    if (newOrder.size() != playlist->patterns.size()) {
        ESP_LOGW(TAG, "Reorder array size mismatch for playlist %s", playlistUuid.c_str());
        return false;
    }
    std::vector<std::string> sorted_old = playlist->patterns;
    std::vector<std::string> sorted_new = newOrder;
    std::sort(sorted_old.begin(), sorted_old.end());
    std::sort(sorted_new.begin(), sorted_new.end());
    if (sorted_old != sorted_new) {
        ESP_LOGW(TAG, "Reorder array does not match playlist contents for %s", playlistUuid.c_str());
        return false;
    }
    playlist->patterns = newOrder;
    return updatePlaylist(playlistUuid, *playlist) == ESP_OK;
}

// Statistics
size_t ManifestManager::getPatternCount() {
    if (!patterns_array) return 0;
    return cJSON_GetArraySize(patterns_array);
}

size_t ManifestManager::getPlaylistCount() {
    if (!playlists_array) return 0;
    return cJSON_GetArraySize(playlists_array);
}
