/**
 * @file manifest_playlist.cpp
 * @brief ManifestDatabase playlist CRUD and playlist-pattern operations
 */

#include "manifest_internal.h"
#include "esp_log.h"
#include <algorithm>

static const char* TAG = MANIFEST_TAG;

// ─────────────────────────────────────────────────────────────────────────────
// Playlist CRUD
// ─────────────────────────────────────────────────────────────────────────────

esp_err_t ManifestDatabase::addPlaylist(Playlist& playlist) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    std::string ts = playlist.created_at.empty() ? currentTimestamp() : playlist.created_at;
    playlist.created_at = ts;
    if (playlist.updated_at.empty()) playlist.updated_at = ts;
    if (playlist.external_uuid.empty()) playlist.external_uuid = generateUUID();

    tqdb_err_t err = tqdb_add(impl_->db, "Playlist", &playlist);
    if (err != TQDB_OK) {
        ESP_LOGE(TAG, "Failed to add playlist: %d", err);
        return (err == TQDB_ERR_FULL) ? ESP_ERR_NO_MEM : ESP_FAIL;
    }

    ESP_LOGI(TAG, "Playlist added: id=%lu, uuid=%s", (unsigned long)playlist.id,
        playlist.external_uuid.c_str());
    return ESP_OK;
}

esp_err_t ManifestDatabase::updatePlaylist(uint32_t id, const Playlist& playlist) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    Playlist existing;
    if (tqdb_get(impl_->db, "Playlist", id, &existing) != TQDB_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    Playlist updated = playlist;
    updated.id = id;
    if (updated.external_uuid.empty()) updated.external_uuid = existing.external_uuid;
    if (updated.created_at.empty()) updated.created_at = existing.created_at;
    updated.updated_at = currentTimestamp();

    tqdb_err_t err = tqdb_update(impl_->db, "Playlist", id, &updated);
    if (err != TQDB_OK) {
        ESP_LOGE(TAG, "Failed to update playlist: %d", err);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t ManifestDatabase::deletePlaylist(uint32_t id) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    if (!tqdb_exists(impl_->db, "Playlist", id)) {
        return ESP_ERR_NOT_FOUND;
    }

    tqdb_err_t err = tqdb_delete(impl_->db, "Playlist", id);
    if (err != TQDB_OK) {
        ESP_LOGE(TAG, "Failed to delete playlist: %d", err);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Playlist deleted: id=%lu", (unsigned long)id);
    return ESP_OK;
}

std::vector<Playlist> ManifestDatabase::getAllPlaylists() {
    std::vector<Playlist> result;
    if (!impl_->initialized) return result;

    tqdb_foreach(impl_->db, "Playlist",
        [](const void* entity, void* ctx) -> bool {
            auto* vec = static_cast<std::vector<Playlist>*>(ctx);
            vec->push_back(*static_cast<const Playlist*>(entity));
            return true;
        }, &result);

    std::sort(result.begin(), result.end(),
        [](const Playlist& a, const Playlist& b) { return a.name < b.name; });

    return result;
}

PaginatedResult<Playlist> ManifestDatabase::getPlaylists(int page, int per_page) {
    auto all = getAllPlaylists();

    PaginatedResult<Playlist> result;
    result.pagination.total_items = static_cast<int>(all.size());
    result.pagination.page = page;
    result.pagination.per_page = per_page;
    result.pagination.total_pages = (result.pagination.total_items + per_page - 1) / per_page;

    int start = page * per_page;
    int end = std::min(start + per_page, static_cast<int>(all.size()));
    for (int i = start; i < end; i++) {
        result.items.push_back(std::move(all[i]));
    }
    return result;
}

std::optional<Playlist> ManifestDatabase::getPlaylist(uint32_t id) {
    if (!impl_->initialized) return std::nullopt;

    Playlist pl;
    if (tqdb_get(impl_->db, "Playlist", id, &pl) == TQDB_OK) {
        return pl;
    }
    return std::nullopt;
}

bool ManifestDatabase::playlistExists(uint32_t id) {
    if (!impl_->initialized) return false;
    return tqdb_exists(impl_->db, "Playlist", id);
}

size_t ManifestDatabase::getPlaylistCount() {
    if (!impl_->initialized) return 0;
    return tqdb_count(impl_->db, "Playlist");
}

std::optional<Playlist> ManifestDatabase::getPlaylistByExternalUuid(const std::string& uuid) {
    if (!impl_->initialized || uuid.empty()) return std::nullopt;

    struct FindCtx {
        const std::string* target;
        Playlist result;
        bool found;
    } ctx = { &uuid, {}, false };

    tqdb_foreach(impl_->db, "Playlist",
        [](const void* entity, void* c) -> bool {
            auto* ctx = static_cast<FindCtx*>(c);
            const Playlist* pl = static_cast<const Playlist*>(entity);
            if (pl->external_uuid == *ctx->target) {
                ctx->result = *pl;
                ctx->found = true;
                return false;
            }
            return true;
        }, &ctx);

    if (ctx.found) return ctx.result;
    return std::nullopt;
}

std::optional<uint32_t> ManifestDatabase::findPlaylistIdByExternalUuid(const std::string& uuid) {
    auto pl = getPlaylistByExternalUuid(uuid);
    if (pl) return pl->id;
    return std::nullopt;
}

// ─────────────────────────────────────────────────────────────────────────────
// Playlist-pattern operations
// ─────────────────────────────────────────────────────────────────────────────

esp_err_t ManifestDatabase::addPatternToPlaylist(uint32_t playlistId, uint32_t patternId) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    Playlist pl;
    if (tqdb_get(impl_->db, "Playlist", playlistId, &pl) != TQDB_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    if (!tqdb_exists(impl_->db, "Pattern", patternId)) {
        return ESP_ERR_NOT_FOUND;
    }

    // Check if already in playlist
    if (std::find(pl.pattern_ids.begin(), pl.pattern_ids.end(), patternId) != pl.pattern_ids.end()) {
        return ESP_OK;  // Already exists
    }

    pl.pattern_ids.push_back(patternId);
    pl.updated_at = currentTimestamp();

    return (tqdb_update(impl_->db, "Playlist", playlistId, &pl) == TQDB_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t ManifestDatabase::removePatternFromPlaylist(uint32_t playlistId, uint32_t patternId) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    Playlist pl;
    if (tqdb_get(impl_->db, "Playlist", playlistId, &pl) != TQDB_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    auto it = std::find(pl.pattern_ids.begin(), pl.pattern_ids.end(), patternId);
    if (it != pl.pattern_ids.end()) {
        pl.pattern_ids.erase(it);
        if (pl.featured_pattern_id == patternId) {
            pl.featured_pattern_id = 0;
        }
        pl.updated_at = currentTimestamp();
        return (tqdb_update(impl_->db, "Playlist", playlistId, &pl) == TQDB_OK) ? ESP_OK : ESP_FAIL;
    }

    return ESP_OK;  // Pattern wasn't in playlist
}

esp_err_t ManifestDatabase::setFeaturedPattern(uint32_t playlistId, uint32_t patternId) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    Playlist pl;
    if (tqdb_get(impl_->db, "Playlist", playlistId, &pl) != TQDB_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    if (!tqdb_exists(impl_->db, "Pattern", patternId)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (std::find(pl.pattern_ids.begin(), pl.pattern_ids.end(), patternId) == pl.pattern_ids.end()) {
        return ESP_ERR_NOT_FOUND;  // Pattern not in playlist
    }

    pl.featured_pattern_id = patternId;
    pl.updated_at = currentTimestamp();

    return (tqdb_update(impl_->db, "Playlist", playlistId, &pl) == TQDB_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t ManifestDatabase::reorderPlaylist(uint32_t playlistId, const std::vector<uint32_t>& newOrder) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    Playlist pl;
    if (tqdb_get(impl_->db, "Playlist", playlistId, &pl) != TQDB_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    if (newOrder.size() != pl.pattern_ids.size()) {
        return ESP_ERR_INVALID_ARG;
    }

    // Verify same elements
    auto sortedOld = pl.pattern_ids;
    auto sortedNew = newOrder;
    std::sort(sortedOld.begin(), sortedOld.end());
    std::sort(sortedNew.begin(), sortedNew.end());
    if (sortedOld != sortedNew) {
        return ESP_ERR_INVALID_ARG;
    }

    pl.pattern_ids = newOrder;
    pl.updated_at = currentTimestamp();

    return (tqdb_update(impl_->db, "Playlist", playlistId, &pl) == TQDB_OK) ? ESP_OK : ESP_FAIL;
}

std::vector<Pattern> ManifestDatabase::getPlaylistPatterns(uint32_t playlistId) {
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
