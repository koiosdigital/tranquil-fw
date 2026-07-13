/**
 * @file manifest_pattern.cpp
 * @brief ManifestDatabase pattern CRUD operations
 */

#include "manifest_internal.h"
#include "esp_log.h"
#include <algorithm>
#include <cstdio>
#include <unistd.h>

static const char* TAG = MANIFEST_TAG;

// ─────────────────────────────────────────────────────────────────────────────
// Storage converters
// ─────────────────────────────────────────────────────────────────────────────

// Shared helpers for all storage converters (patterns, playlists, jobs)
std::string storage_get_str(const cJSON* obj, const char* key) {
    const cJSON* item = cJSON_GetObjectItem(obj, key);
    return (item && cJSON_IsString(item)) ? item->valuestring : "";
}

double storage_get_num(const cJSON* obj, const char* key, double def) {
    const cJSON* item = cJSON_GetObjectItem(obj, key);
    return (item && cJSON_IsNumber(item)) ? item->valuedouble : def;
}

bool storage_get_bool(const cJSON* obj, const char* key) {
    const cJSON* item = cJSON_GetObjectItem(obj, key);
    return item && cJSON_IsTrue(item);
}

cJSON* pattern_to_storage(const Pattern& p) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) return nullptr;

    cJSON_AddNumberToObject(obj, "id", p.id);
    cJSON_AddStringToObject(obj, "uuid", p.external_uuid.c_str());
    cJSON_AddStringToObject(obj, "name", p.name.c_str());
    cJSON_AddStringToObject(obj, "creator", p.creator.c_str());
    cJSON_AddStringToObject(obj, "date", p.date.c_str());
    cJSON_AddNumberToObject(obj, "popularity", p.popularity);
    cJSON_AddBoolToObject(obj, "reversible", p.reversible);
    cJSON_AddNumberToObject(obj, "start_point", p.start_point);
    cJSON_AddBoolToObject(obj, "encrypted", p.encrypted);
    cJSON_AddNumberToObject(obj, "size_bytes", static_cast<double>(p.size_bytes));
    cJSON_AddStringToObject(obj, "created_at", p.created_at.c_str());
    cJSON_AddStringToObject(obj, "last_played_at", p.last_played_at.c_str());
    cJSON_AddStringToObject(obj, "downloaded_at", p.downloaded_at.c_str());
    cJSON_AddBoolToObject(obj, "purchased", p.purchased);
    cJSON_AddNumberToObject(obj, "purchased_at", static_cast<double>(p.purchased_at));
    cJSON_AddStringToObject(obj, "receipt_id", p.receipt_id.c_str());
    return obj;
}

Pattern pattern_from_storage(const cJSON* obj) {
    Pattern p;
    if (!obj) return p;

    p.id = static_cast<uint32_t>(storage_get_num(obj, "id", 0));
    p.external_uuid = storage_get_str(obj, "uuid");
    p.name = storage_get_str(obj, "name");
    p.creator = storage_get_str(obj, "creator");
    p.date = storage_get_str(obj, "date");
    p.popularity = static_cast<int>(storage_get_num(obj, "popularity", 0));
    p.reversible = storage_get_bool(obj, "reversible");
    p.start_point = static_cast<int>(storage_get_num(obj, "start_point", 0));
    p.encrypted = storage_get_bool(obj, "encrypted");
    p.size_bytes = static_cast<size_t>(storage_get_num(obj, "size_bytes", 0));
    p.created_at = storage_get_str(obj, "created_at");
    p.last_played_at = storage_get_str(obj, "last_played_at");
    p.downloaded_at = storage_get_str(obj, "downloaded_at");
    p.purchased = storage_get_bool(obj, "purchased");
    p.purchased_at = static_cast<int64_t>(storage_get_num(obj, "purchased_at", 0));
    p.receipt_id = storage_get_str(obj, "receipt_id");
    return p;
}

// ─────────────────────────────────────────────────────────────────────────────
// Pattern CRUD
// ─────────────────────────────────────────────────────────────────────────────

esp_err_t ManifestDatabase::addPattern(Pattern& pattern) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    if (pattern.created_at.empty()) pattern.created_at = currentTimestamp();
    if (pattern.creator.empty()) pattern.creator = "Uploaded";
    if (pattern.external_uuid.empty()) pattern.external_uuid = generateUUID();

    cJSON* obj = pattern_to_storage(pattern);
    if (!obj) return ESP_ERR_NO_MEM;

    uint32_t id = impl_->patterns.add(obj);
    cJSON_Delete(obj);
    if (id == 0) {
        ESP_LOGE(TAG, "Failed to add pattern");
        return ESP_ERR_NO_MEM;
    }
    pattern.id = id;
    if (!impl_->patterns.save()) return ESP_FAIL;

    ESP_LOGI(TAG, "Pattern added: id=%lu, uuid=%s", (unsigned long)pattern.id,
        pattern.external_uuid.c_str());
    return ESP_OK;
}

esp_err_t ManifestDatabase::updatePattern(uint32_t id, const Pattern& pattern) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    // Get existing to preserve certain fields
    auto existing = getPattern(id);
    if (!existing) return ESP_ERR_NOT_FOUND;

    Pattern updated = pattern;
    updated.id = id;
    if (updated.external_uuid.empty()) updated.external_uuid = existing->external_uuid;
    if (updated.created_at.empty()) updated.created_at = existing->created_at;
    if (updated.downloaded_at.empty()) updated.downloaded_at = existing->downloaded_at;

    cJSON* obj = pattern_to_storage(updated);
    if (!obj) return ESP_ERR_NO_MEM;

    bool ok = impl_->patterns.update(id, obj);
    cJSON_Delete(obj);
    if (!ok) {
        ESP_LOGE(TAG, "Failed to update pattern %lu", (unsigned long)id);
        return ESP_FAIL;
    }
    return impl_->patterns.save() ? ESP_OK : ESP_FAIL;
}

esp_err_t ManifestDatabase::deletePattern(uint32_t id) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    // Get pattern to delete files and remove from playlists
    auto p = getPattern(id);
    if (!p) return ESP_ERR_NOT_FOUND;

    // Remove pattern from all playlists
    struct RemoveCtx { uint32_t pattern_id; };
    RemoveCtx ctx = { id };

    impl_->playlists.modify(
        [](uint32_t, cJSON** obj, void* c) -> manifest::JsonlTable::Action {
            auto* ctx = static_cast<RemoveCtx*>(c);
            Playlist pl = playlist_from_storage(*obj);
            auto& pids = pl.pattern_ids;
            size_t before = pids.size();
            pids.erase(std::remove(pids.begin(), pids.end(), ctx->pattern_id), pids.end());
            bool changed = pids.size() != before;
            if (pl.featured_pattern_id == ctx->pattern_id) {
                pl.featured_pattern_id = 0;
                changed = true;
            }
            if (!changed) return manifest::JsonlTable::Action::Keep;

            cJSON* updated = playlist_to_storage(pl);
            if (!updated) return manifest::JsonlTable::Action::Keep;
            cJSON_Delete(*obj);
            *obj = updated;
            return manifest::JsonlTable::Action::Update;
        }, &ctx);

    if (!impl_->patterns.remove(id)) {
        ESP_LOGE(TAG, "Failed to delete pattern %lu", (unsigned long)id);
        return ESP_FAIL;
    }

    bool saved = impl_->patterns.save();
    saved = impl_->playlists.save() && saved;

    // Delete pattern files
    if (!p->external_uuid.empty()) {
        char path[128];
        snprintf(path, sizeof(path), "/sd/patterns/%s.thr", p->external_uuid.c_str());
        unlink(path);
        snprintf(path, sizeof(path), "/sd/patterns/%s.dat", p->external_uuid.c_str());
        unlink(path);
        snprintf(path, sizeof(path), "/sd/previews/%s.png", p->external_uuid.c_str());
        unlink(path);
    }

    ESP_LOGI(TAG, "Pattern deleted: id=%lu", (unsigned long)id);
    return saved ? ESP_OK : ESP_FAIL;
}

std::vector<Pattern> ManifestDatabase::getAllPatterns() {
    ManifestLock lock(impl_->mutex);
    std::vector<Pattern> result;
    if (!impl_->initialized) return result;

    impl_->patterns.foreach(
        [](uint32_t, const cJSON* obj, void* ctx) -> bool {
            auto* vec = static_cast<std::vector<Pattern>*>(ctx);
            vec->push_back(pattern_from_storage(obj));
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
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return std::nullopt;

    cJSON* obj = impl_->patterns.get(id);
    if (!obj) return std::nullopt;

    Pattern p = pattern_from_storage(obj);
    cJSON_Delete(obj);
    return p;
}

bool ManifestDatabase::patternExists(uint32_t id) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return false;
    return impl_->patterns.exists(id);
}

size_t ManifestDatabase::getPatternCount() {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return 0;
    return impl_->patterns.count();
}

size_t ManifestDatabase::getSubscriptionPatternCount() {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return 0;

    size_t count = 0;
    impl_->patterns.foreach(
        [](uint32_t, const cJSON* obj, void* ctx) -> bool {
            if (!storage_get_bool(obj, "purchased")) {
                (*static_cast<size_t*>(ctx))++;
            }
            return true;
        }, &count);

    return count;
}

std::optional<Pattern> ManifestDatabase::getPatternByExternalUuid(const std::string& uuid) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized || uuid.empty()) return std::nullopt;

    uint32_t id = impl_->patterns.findByUuid(uuid.c_str());
    if (id == 0) return std::nullopt;
    return getPattern(id);
}

std::optional<uint32_t> ManifestDatabase::findPatternIdByExternalUuid(const std::string& uuid) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized || uuid.empty()) return std::nullopt;

    uint32_t id = impl_->patterns.findByUuid(uuid.c_str());
    if (id == 0) return std::nullopt;
    return id;
}

esp_err_t ManifestDatabase::updateLastPlayed(uint32_t id) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    auto p = getPattern(id);
    if (!p) return ESP_ERR_NOT_FOUND;

    p->last_played_at = currentTimestamp();
    return updatePattern(id, *p);
}

esp_err_t ManifestDatabase::incrementPopularity(uint32_t id) {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    auto p = getPattern(id);
    if (!p) return ESP_ERR_NOT_FOUND;

    p->popularity++;
    return updatePattern(id, *p);
}
