#pragma once

/**
 * @file manifest_internal.h
 * @brief Internal implementation details for ManifestDatabase
 *
 * This header is NOT part of the public API. Include ManifestDatabase.h instead.
 */

#include "ManifestDatabase.h"
#include "manifest_store.h"
#include "../jobs/job_types.h"

#include <mutex>
#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────

static constexpr const char* MANIFEST_TAG = "ManifestDB";
static constexpr const char* MANIFEST_DIR = "/sd/manifest";
static constexpr const char* PATTERNS_PATH = "/sd/manifest/patterns.jsonl";
static constexpr const char* PLAYLISTS_PATH = "/sd/manifest/playlists.jsonl";
static constexpr const char* JOBS_PATH = "/sd/manifest/jobs.jsonl";

// Retired tqdb database file; renamed aside on first boot after the migration.
static constexpr const char* LEGACY_TQDB_PATH = "/sd/manifest.tqdb";

// Max sanity limits for corrupt file detection
static constexpr uint32_t MAX_PATTERNS = 10000;
static constexpr uint32_t MAX_PLAYLISTS = 5000;
static constexpr uint32_t MAX_JOBS = 5000;

// ─────────────────────────────────────────────────────────────────────────────
// Implementation class
// ─────────────────────────────────────────────────────────────────────────────

class ManifestDatabase::Impl {
public:
    bool initialized = false;

    // Guards all table access. Recursive: public methods call each other.
    std::recursive_mutex mutex;

    manifest::JsonlTable patterns{"Pattern", PATTERNS_PATH, MAX_PATTERNS};
    manifest::JsonlTable playlists{"Playlist", PLAYLISTS_PATH, MAX_PLAYLISTS};
    manifest::JsonlTable jobs{"Job", JOBS_PATH, MAX_JOBS};
};

using ManifestLock = std::lock_guard<std::recursive_mutex>;

// ─────────────────────────────────────────────────────────────────────────────
// Storage converters (full-fidelity, unlike the API-shaped converters in
// ManifestDatabase.h — these round-trip every field)
// ─────────────────────────────────────────────────────────────────────────────

std::string storage_get_str(const cJSON* obj, const char* key);
double storage_get_num(const cJSON* obj, const char* key, double def);
bool storage_get_bool(const cJSON* obj, const char* key);

cJSON* pattern_to_storage(const Pattern& p);
Pattern pattern_from_storage(const cJSON* obj);

cJSON* playlist_to_storage(const Playlist& pl);
Playlist playlist_from_storage(const cJSON* obj);

cJSON* job_to_storage(const jobs::Job& j);
jobs::Job job_from_storage(const cJSON* obj);
