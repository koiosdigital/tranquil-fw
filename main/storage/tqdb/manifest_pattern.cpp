/**
 * @file manifest_pattern.cpp
 * @brief ManifestDatabase pattern CRUD operations
 */

#include "manifest_internal.h"
#include "esp_log.h"
#include <algorithm>
#include <unistd.h>

static const char* TAG = MANIFEST_TAG;

// ─────────────────────────────────────────────────────────────────────────────
// Pattern CRUD
// ─────────────────────────────────────────────────────────────────────────────

esp_err_t ManifestDatabase::addPattern(Pattern& pattern) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    if (pattern.created_at.empty()) pattern.created_at = currentTimestamp();
    if (pattern.creator.empty()) pattern.creator = "Uploaded";
    if (pattern.external_uuid.empty()) pattern.external_uuid = generateUUID();

    tqdb_err_t err = tqdb_add(impl_->db, "Pattern", &pattern);
    if (err != TQDB_OK) {
        ESP_LOGE(TAG, "Failed to add pattern: %d", err);
        return (err == TQDB_ERR_FULL) ? ESP_ERR_NO_MEM : ESP_FAIL;
    }

    ESP_LOGI(TAG, "Pattern added: id=%lu, uuid=%s", (unsigned long)pattern.id,
        pattern.external_uuid.c_str());
    return ESP_OK;
}

esp_err_t ManifestDatabase::updatePattern(uint32_t id, const Pattern& pattern) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    // Get existing to preserve certain fields
    Pattern existing;
    if (tqdb_get(impl_->db, "Pattern", id, &existing) != TQDB_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    Pattern updated = pattern;
    updated.id = id;
    if (updated.external_uuid.empty()) updated.external_uuid = existing.external_uuid;
    if (updated.created_at.empty()) updated.created_at = existing.created_at;
    if (updated.downloaded_at.empty()) updated.downloaded_at = existing.downloaded_at;

    tqdb_err_t err = tqdb_update(impl_->db, "Pattern", id, &updated);
    if (err != TQDB_OK) {
        ESP_LOGE(TAG, "Failed to update pattern: %d", err);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t ManifestDatabase::deletePattern(uint32_t id) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    // Get pattern to delete files and remove from playlists
    Pattern p;
    if (tqdb_get(impl_->db, "Pattern", id, &p) != TQDB_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    // Remove pattern from all playlists
    struct RemoveCtx { uint32_t pattern_id; };
    RemoveCtx ctx = { id };

    tqdb_modify_where(impl_->db, "Playlist", nullptr, nullptr,
        [](void* entity, void* c) {
            Playlist* pl = static_cast<Playlist*>(entity);
            auto* ctx = static_cast<RemoveCtx*>(c);
            auto& pids = pl->pattern_ids;
            pids.erase(std::remove(pids.begin(), pids.end(), ctx->pattern_id), pids.end());
            if (pl->featured_pattern_id == ctx->pattern_id) {
                pl->featured_pattern_id = 0;
            }
        }, &ctx);

    // Delete the pattern
    tqdb_err_t err = tqdb_delete(impl_->db, "Pattern", id);
    if (err != TQDB_OK) {
        ESP_LOGE(TAG, "Failed to delete pattern: %d", err);
        return ESP_FAIL;
    }

    // Delete pattern files
    if (!p.external_uuid.empty()) {
        char path[128];
        snprintf(path, sizeof(path), "/sd/patterns/%s.thr", p.external_uuid.c_str());
        unlink(path);
        snprintf(path, sizeof(path), "/sd/patterns/%s.dat", p.external_uuid.c_str());
        unlink(path);
        snprintf(path, sizeof(path), "/sd/previews/%s.png", p.external_uuid.c_str());
        unlink(path);
    }

    ESP_LOGI(TAG, "Pattern deleted: id=%lu", (unsigned long)id);
    return ESP_OK;
}

std::vector<Pattern> ManifestDatabase::getAllPatterns() {
    std::vector<Pattern> result;
    if (!impl_->initialized) return result;

    tqdb_foreach(impl_->db, "Pattern",
        [](const void* entity, void* ctx) -> bool {
            auto* vec = static_cast<std::vector<Pattern>*>(ctx);
            vec->push_back(*static_cast<const Pattern*>(entity));
            return true;  // Continue
        }, &result);

    // Sort by name
    std::sort(result.begin(), result.end(),
        [](const Pattern& a, const Pattern& b) { return a.name < b.name; });

    return result;
}

PaginatedResult<Pattern> ManifestDatabase::getPatterns(int page, int per_page) {
    auto all = getAllPatterns();

    PaginatedResult<Pattern> result;
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

std::optional<Pattern> ManifestDatabase::getPattern(uint32_t id) {
    if (!impl_->initialized) return std::nullopt;

    Pattern p;
    if (tqdb_get(impl_->db, "Pattern", id, &p) == TQDB_OK) {
        return p;
    }
    return std::nullopt;
}

bool ManifestDatabase::patternExists(uint32_t id) {
    if (!impl_->initialized) return false;
    return tqdb_exists(impl_->db, "Pattern", id);
}

size_t ManifestDatabase::getPatternCount() {
    if (!impl_->initialized) return 0;
    return tqdb_count(impl_->db, "Pattern");
}

size_t ManifestDatabase::getSubscriptionPatternCount() {
    if (!impl_->initialized) return 0;

    size_t count = 0;
    tqdb_foreach(impl_->db, "Pattern",
        [](const void* entity, void* ctx) -> bool {
            const Pattern* p = static_cast<const Pattern*>(entity);
            if (!p->purchased) {
                (*static_cast<size_t*>(ctx))++;
            }
            return true;
        }, &count);

    return count;
}

std::optional<Pattern> ManifestDatabase::getPatternByExternalUuid(const std::string& uuid) {
    if (!impl_->initialized || uuid.empty()) return std::nullopt;

    struct FindCtx {
        const std::string* target;
        Pattern result;
        bool found;
    } ctx = { &uuid, {}, false };

    tqdb_foreach(impl_->db, "Pattern",
        [](const void* entity, void* c) -> bool {
            auto* ctx = static_cast<FindCtx*>(c);
            const Pattern* p = static_cast<const Pattern*>(entity);
            if (p->external_uuid == *ctx->target) {
                ctx->result = *p;
                ctx->found = true;
                return false;  // Stop iteration
            }
            return true;
        }, &ctx);

    if (ctx.found) return ctx.result;
    return std::nullopt;
}

std::optional<uint32_t> ManifestDatabase::findPatternIdByExternalUuid(const std::string& uuid) {
    auto p = getPatternByExternalUuid(uuid);
    if (p) return p->id;
    return std::nullopt;
}

esp_err_t ManifestDatabase::updateLastPlayed(uint32_t id) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    Pattern p;
    if (tqdb_get(impl_->db, "Pattern", id, &p) != TQDB_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    p.last_played_at = currentTimestamp();
    return (tqdb_update(impl_->db, "Pattern", id, &p) == TQDB_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t ManifestDatabase::incrementPopularity(uint32_t id) {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    Pattern p;
    if (tqdb_get(impl_->db, "Pattern", id, &p) != TQDB_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    p.popularity++;
    return (tqdb_update(impl_->db, "Pattern", id, &p) == TQDB_OK) ? ESP_OK : ESP_FAIL;
}
