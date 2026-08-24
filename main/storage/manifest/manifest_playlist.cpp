/**
 * @file manifest_playlist.cpp
 * @brief ManifestDatabase playlist CRUD and playlist-pattern operations
 */

#include "manifest_internal.h"
#include "esp_log.h"
#include <algorithm>
#include <utility>

static const char* TAG = MANIFEST_TAG;

// ─────────────────────────────────────────────────────────────────────────────
// Storage converters
// ─────────────────────────────────────────────────────────────────────────────

cJSON* playlist_to_storage(const Playlist& pl) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) return nullptr;

    cJSON_AddNumberToObject(obj, "id", pl.id);
    cJSON_AddStringToObject(obj, "uuid", pl.external_uuid.c_str());
    cJSON_AddStringToObject(obj, "name", pl.name.c_str());
    cJSON_AddStringToObject(obj, "description", pl.description.c_str());
    cJSON_AddNumberToObject(obj, "featured_pattern_id", pl.featured_pattern_id);
    cJSON_AddStringToObject(obj, "date", pl.date.c_str());
    cJSON_AddStringToObject(obj, "created_at", pl.created_at.c_str());
    cJSON_AddStringToObject(obj, "updated_at", pl.updated_at.c_str());

    cJSON* ids = cJSON_CreateArray();
    if (!ids) {
        cJSON_Delete(obj);
        return nullptr;
    }
    for (uint32_t pid : pl.pattern_ids) {
        cJSON* entry = cJSON_CreateNumber(pid);
        if (entry && !cJSON_AddItemToArray(ids, entry)) {
            cJSON_Delete(entry);
        }
    }
    if (!cJSON_AddItemToObject(obj, "pattern_ids", ids)) {
        cJSON_Delete(ids);
        cJSON_Delete(obj);
        return nullptr;
    }
    return obj;
}

Playlist playlist_from_storage(const cJSON* obj) {
    Playlist pl;
    if (!obj) return pl;

    pl.id = static_cast<uint32_t>(storage_get_num(obj, "id", 0));
    pl.external_uuid = storage_get_str(obj, "uuid");
    pl.name = storage_get_str(obj, "name");
    pl.description = storage_get_str(obj, "description");
    pl.featured_pattern_id = static_cast<uint32_t>(storage_get_num(obj, "featured_pattern_id", 0));
    pl.date = storage_get_str(obj, "date");
    pl.created_at = storage_get_str(obj, "created_at");
    pl.updated_at = storage_get_str(obj, "updated_at");

    const cJSON* ids = cJSON_GetObjectItem(obj, "pattern_ids");
    if (ids && cJSON_IsArray(ids)) {
        const cJSON* item;
        cJSON_ArrayForEach(item, ids) {
            if (cJSON_IsNumber(item)) {
                pl.pattern_ids.push_back(static_cast<uint32_t>(item->valuedouble));
            }
        }
    }
    return pl;
}

// ─────────────────────────────────────────────────────────────────────────────
// Playlist CRUD
// ─────────────────────────────────────────────────────────────────────────────

esp_err_t ManifestDatabase::addPlaylist(Playlist& playlist) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    std::string ts = playlist.created_at.empty() ? currentTimestamp() : playlist.created_at;
    playlist.created_at = ts;
    if (playlist.updated_at.empty()) playlist.updated_at = ts;
    if (playlist.external_uuid.empty()) playlist.external_uuid = generateUUID();

    cJSON* obj = playlist_to_storage(playlist);
    if (!obj) return ESP_ERR_NO_MEM;

    uint32_t id = impl_->playlists.add(obj);
    cJSON_Delete(obj);
    if (id == 0) {
        ESP_LOGE(TAG, "Failed to add playlist");
        return ESP_ERR_NO_MEM;
    }
    playlist.id = id;
    if (!impl_->playlists.save()) return ESP_FAIL;

    ESP_LOGI(TAG, "Playlist added: id=%lu, uuid=%s", (unsigned long)playlist.id,
        playlist.external_uuid.c_str());
    return ESP_OK;
}

esp_err_t ManifestDatabase::updatePlaylist(uint32_t id, const Playlist& playlist) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    auto existing = getPlaylist(id);
    if (!existing) return ESP_ERR_NOT_FOUND;

    Playlist updated = playlist;
    updated.id = id;
    if (updated.external_uuid.empty()) updated.external_uuid = existing->external_uuid;
    if (updated.created_at.empty()) updated.created_at = existing->created_at;
    updated.updated_at = currentTimestamp();

    cJSON* obj = playlist_to_storage(updated);
    if (!obj) return ESP_ERR_NO_MEM;

    bool ok = impl_->playlists.update(id, obj);
    cJSON_Delete(obj);
    if (!ok) {
        ESP_LOGE(TAG, "Failed to update playlist %lu", (unsigned long)id);
        return ESP_FAIL;
    }
    return impl_->playlists.save() ? ESP_OK : ESP_FAIL;
}

esp_err_t ManifestDatabase::deletePlaylist(uint32_t id) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    if (!impl_->playlists.remove(id)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (!impl_->playlists.save()) return ESP_FAIL;

    ESP_LOGI(TAG, "Playlist deleted: id=%lu", (unsigned long)id);
    return ESP_OK;
}

std::vector<Playlist> ManifestDatabase::getAllPlaylists() {
    ManifestLock lock(impl_->mutex);
    std::vector<Playlist> result;
    if (!impl_->initialized) return result;

    impl_->playlists.foreach(
        [](uint32_t, const cJSON* obj, void* ctx) -> bool {
            auto* vec = static_cast<std::vector<Playlist>*>(ctx);
            vec->push_back(playlist_from_storage(obj));
            return true;
        }, &result);

    std::sort(result.begin(), result.end(),
        [](const Playlist& a, const Playlist& b) { return a.name < b.name; });

    return result;
}

PaginatedResult<Playlist> ManifestDatabase::getPlaylists(int page, int per_page) {
    ManifestLock lock(impl_->mutex);

    PaginatedResult<Playlist> result;
    result.pagination.page = page;
    result.pagination.per_page = per_page;
    if (!impl_->initialized || per_page <= 0) return result;

    // Lightweight {name, id} index instead of materializing every Playlist to
    // return one page; only the page's rows are expanded below.
    std::vector<std::pair<std::string, uint32_t>> index;
    index.reserve(impl_->playlists.count());
    impl_->playlists.foreach(
        [](uint32_t id, const cJSON* obj, void* ctx) -> bool {
            auto* idx = static_cast<std::vector<std::pair<std::string, uint32_t>>*>(ctx);
            idx->emplace_back(storage_get_str(obj, "name"), id);
            return true;
        }, &index);

    std::sort(index.begin(), index.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    result.pagination.total_items = static_cast<int>(index.size());
    result.pagination.total_pages = (result.pagination.total_items + per_page - 1) / per_page;

    int start = page * per_page;
    int end = std::min(start + per_page, static_cast<int>(index.size()));
    for (int i = start; i < end; i++) {
        cJSON* obj = impl_->playlists.get(index[i].second);
        if (!obj) continue;  // removed between indexing and materialization
        result.items.push_back(playlist_from_storage(obj));
        cJSON_Delete(obj);
    }
    return result;
}

std::optional<Playlist> ManifestDatabase::getPlaylist(uint32_t id) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return std::nullopt;

    cJSON* obj = impl_->playlists.get(id);
    if (!obj) return std::nullopt;

    Playlist pl = playlist_from_storage(obj);
    cJSON_Delete(obj);
    return pl;
}

bool ManifestDatabase::playlistExists(uint32_t id) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return false;
    return impl_->playlists.exists(id);
}

size_t ManifestDatabase::getPlaylistCount() {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return 0;
    return impl_->playlists.count();
}

std::optional<Playlist> ManifestDatabase::getPlaylistByExternalUuid(const std::string& uuid) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized || uuid.empty()) return std::nullopt;

    uint32_t id = impl_->playlists.findByUuid(uuid.c_str());
    if (id == 0) return std::nullopt;
    return getPlaylist(id);
}

std::optional<uint32_t> ManifestDatabase::findPlaylistIdByExternalUuid(const std::string& uuid) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized || uuid.empty()) return std::nullopt;

    uint32_t id = impl_->playlists.findByUuid(uuid.c_str());
    if (id == 0) return std::nullopt;
    return id;
}

// ─────────────────────────────────────────────────────────────────────────────
// Playlist-pattern operations
// ─────────────────────────────────────────────────────────────────────────────

esp_err_t ManifestDatabase::addPatternToPlaylist(uint32_t playlistId, uint32_t patternId) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    auto pl = getPlaylist(playlistId);
    if (!pl) return ESP_ERR_NOT_FOUND;
    if (!impl_->patterns.exists(patternId)) return ESP_ERR_NOT_FOUND;

    // Check if already in playlist
    if (std::find(pl->pattern_ids.begin(), pl->pattern_ids.end(), patternId) != pl->pattern_ids.end()) {
        return ESP_OK;  // Already exists
    }

    pl->pattern_ids.push_back(patternId);
    return updatePlaylist(playlistId, *pl);
}

esp_err_t ManifestDatabase::removePatternFromPlaylist(uint32_t playlistId, uint32_t patternId) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    auto pl = getPlaylist(playlistId);
    if (!pl) return ESP_ERR_NOT_FOUND;

    auto it = std::find(pl->pattern_ids.begin(), pl->pattern_ids.end(), patternId);
    if (it != pl->pattern_ids.end()) {
        pl->pattern_ids.erase(it);
        if (pl->featured_pattern_id == patternId) {
            pl->featured_pattern_id = 0;
        }
        return updatePlaylist(playlistId, *pl);
    }

    return ESP_OK;  // Pattern wasn't in playlist
}

esp_err_t ManifestDatabase::setFeaturedPattern(uint32_t playlistId, uint32_t patternId) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    auto pl = getPlaylist(playlistId);
    if (!pl) return ESP_ERR_NOT_FOUND;
    if (!impl_->patterns.exists(patternId)) return ESP_ERR_NOT_FOUND;
    if (std::find(pl->pattern_ids.begin(), pl->pattern_ids.end(), patternId) == pl->pattern_ids.end()) {
        return ESP_ERR_NOT_FOUND;  // Pattern not in playlist
    }

    pl->featured_pattern_id = patternId;
    return updatePlaylist(playlistId, *pl);
}

esp_err_t ManifestDatabase::reorderPlaylist(uint32_t playlistId, const std::vector<uint32_t>& newOrder) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    auto pl = getPlaylist(playlistId);
    if (!pl) return ESP_ERR_NOT_FOUND;

    if (newOrder.size() != pl->pattern_ids.size()) {
        return ESP_ERR_INVALID_ARG;
    }

    // Verify same elements
    auto sortedOld = pl->pattern_ids;
    auto sortedNew = newOrder;
    std::sort(sortedOld.begin(), sortedOld.end());
    std::sort(sortedNew.begin(), sortedNew.end());
    if (sortedOld != sortedNew) {
        return ESP_ERR_INVALID_ARG;
    }

    pl->pattern_ids = newOrder;
    return updatePlaylist(playlistId, *pl);
}

std::vector<Pattern> ManifestDatabase::getPlaylistPatterns(uint32_t playlistId) {
    ManifestLock lock(impl_->mutex);
    std::vector<Pattern> result;

    auto pl = getPlaylist(playlistId);
    if (!pl) return result;

    result.reserve(pl->pattern_ids.size());
    for (uint32_t pid : pl->pattern_ids) {
        auto p = getPattern(pid);
        if (p) result.push_back(std::move(*p));
    }

    return result;
}
