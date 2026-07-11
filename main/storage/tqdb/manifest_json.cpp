/**
 * @file manifest_json.cpp
 * @brief ManifestDatabase JSON conversion functions for HTTP API
 */

#include "manifest_internal.h"

// ─────────────────────────────────────────────────────────────────────────────
// JSON conversion
// ─────────────────────────────────────────────────────────────────────────────

cJSON* ManifestDatabase::patternToJson(const Pattern& p) {
    cJSON* json = cJSON_CreateObject();
    if (!json) return nullptr;

    // Use external_uuid as "uuid" for API compatibility
    cJSON_AddStringToObject(json, "uuid", p.external_uuid.c_str());
    cJSON_AddNumberToObject(json, "id", p.id);  // Also expose internal ID
    cJSON_AddStringToObject(json, "name", p.name.c_str());
    cJSON_AddStringToObject(json, "creator", p.creator.c_str());
    cJSON_AddStringToObject(json, "date", p.date.c_str());
    cJSON_AddNumberToObject(json, "popularity", p.popularity);
    cJSON_AddBoolToObject(json, "reversible", p.reversible);
    cJSON_AddNumberToObject(json, "start_point", p.start_point);
    cJSON_AddBoolToObject(json, "encrypted", p.encrypted);
    cJSON_AddNumberToObject(json, "size", static_cast<double>(p.size_bytes));
    if (!p.created_at.empty())
        cJSON_AddStringToObject(json, "created_at", p.created_at.c_str());
    if (!p.last_played_at.empty())
        cJSON_AddStringToObject(json, "last_played_at", p.last_played_at.c_str());
    if (!p.downloaded_at.empty())
        cJSON_AddStringToObject(json, "downloaded_at", p.downloaded_at.c_str());

    cJSON_AddBoolToObject(json, "is_owned", p.purchased);
    if (p.purchased) {
        cJSON_AddNumberToObject(json, "purchased_at", static_cast<double>(p.purchased_at));
        if (!p.receipt_id.empty())
            cJSON_AddStringToObject(json, "receipt_id", p.receipt_id.c_str());
    }

    return json;
}

cJSON* ManifestDatabase::playlistToJson(const Playlist& pl) {
    cJSON* json = cJSON_CreateObject();
    if (!json) return nullptr;

    auto* impl = instance().getImpl();

    // Use external_uuid as "uuid" for API compatibility
    cJSON_AddStringToObject(json, "uuid", pl.external_uuid.c_str());
    cJSON_AddNumberToObject(json, "id", pl.id);  // Also expose internal ID
    cJSON_AddStringToObject(json, "name", pl.name.c_str());
    cJSON_AddStringToObject(json, "description", pl.description.c_str());

    // Convert featured_pattern_id to external UUID
    if (pl.featured_pattern_id != 0) {
        Pattern fp;
        if (tqdb_get(impl->db, "Pattern", pl.featured_pattern_id, &fp) == TQDB_OK) {
            cJSON_AddStringToObject(json, "featured_pattern", fp.external_uuid.c_str());
        }
        else {
            cJSON_AddStringToObject(json, "featured_pattern", "");
        }
    }
    else {
        cJSON_AddStringToObject(json, "featured_pattern", "");
    }

    cJSON_AddStringToObject(json, "date", pl.date.c_str());
    if (!pl.created_at.empty())
        cJSON_AddStringToObject(json, "created_at", pl.created_at.c_str());
    if (!pl.updated_at.empty())
        cJSON_AddStringToObject(json, "updated_at", pl.updated_at.c_str());

    // Convert pattern_ids to external UUIDs for API
    cJSON* patterns = cJSON_CreateArray();
    if (patterns) {
        for (uint32_t pid : pl.pattern_ids) {
            Pattern p;
            if (tqdb_get(impl->db, "Pattern", pid, &p) == TQDB_OK) {
                cJSON* entry = cJSON_CreateString(p.external_uuid.c_str());
                // cJSON_AddItemToArray does NOT free the item on failure —
                // an unchecked add leaks the node under OOM.
                if (entry && !cJSON_AddItemToArray(patterns, entry)) {
                    cJSON_Delete(entry);
                }
            }
        }
        cJSON_AddItemToObject(json, "patterns", patterns);
    }

    return json;
}

Pattern ManifestDatabase::jsonToPattern(const cJSON* json) {
    Pattern p;
    if (!json) return p;

    auto getText = [json](const char* key) -> std::string {
        const cJSON* item = cJSON_GetObjectItem(json, key);
        return (item && cJSON_IsString(item)) ? item->valuestring : "";
    };
    auto getInt = [json](const char* key, int def = 0) -> int {
        const cJSON* item = cJSON_GetObjectItem(json, key);
        return (item && cJSON_IsNumber(item)) ? item->valueint : def;
    };
    auto getBool = [json](const char* key) -> bool {
        const cJSON* item = cJSON_GetObjectItem(json, key);
        return item && cJSON_IsTrue(item);
    };

    p.external_uuid = getText("uuid");  // "uuid" in JSON maps to external_uuid
    p.name = getText("name");
    p.creator = getText("creator");
    if (p.creator.empty()) p.creator = "Uploaded";
    p.date = getText("date");
    p.popularity = getInt("popularity");
    p.reversible = getBool("reversible");
    p.start_point = getInt("start_point");
    p.encrypted = getBool("encrypted");
    p.size_bytes = getInt("size");
    p.created_at = getText("created_at");
    p.last_played_at = getText("last_played_at");
    p.downloaded_at = getText("downloaded_at");

    p.purchased = getBool("is_owned");
    const cJSON* purchasedAt = cJSON_GetObjectItem(json, "purchased_at");
    p.purchased_at = (purchasedAt && cJSON_IsNumber(purchasedAt))
        ? static_cast<int64_t>(purchasedAt->valuedouble) : 0;
    p.receipt_id = getText("receipt_id");

    return p;
}

Playlist ManifestDatabase::jsonToPlaylist(const cJSON* json) {
    Playlist pl;
    if (!json) return pl;

    auto getText = [json](const char* key) -> std::string {
        const cJSON* item = cJSON_GetObjectItem(json, key);
        return (item && cJSON_IsString(item)) ? item->valuestring : "";
    };

    pl.external_uuid = getText("uuid");  // "uuid" in JSON maps to external_uuid
    pl.name = getText("name");
    pl.description = getText("description");

    // Convert featured_pattern UUID to internal ID
    std::string featured_uuid = getText("featured_pattern");
    if (!featured_uuid.empty()) {
        auto pid = instance().findPatternIdByExternalUuid(featured_uuid);
        pl.featured_pattern_id = pid.value_or(0);
    }

    pl.date = getText("date");
    pl.created_at = getText("created_at");
    pl.updated_at = getText("updated_at");

    // Convert pattern UUIDs to internal IDs.
    // "pattern_uuids" per the swagger contract; "patterns" kept for older payloads.
    const cJSON* patterns = cJSON_GetObjectItem(json, "pattern_uuids");
    if (!patterns) patterns = cJSON_GetObjectItem(json, "patterns");
    if (patterns && cJSON_IsArray(patterns)) {
        const cJSON* item;
        cJSON_ArrayForEach(item, patterns) {
            if (cJSON_IsString(item)) {
                auto pid = instance().findPatternIdByExternalUuid(item->valuestring);
                if (pid) {
                    pl.pattern_ids.push_back(*pid);
                }
            }
        }
    }

    return pl;
}
