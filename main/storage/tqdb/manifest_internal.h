#pragma once

/**
 * @file manifest_internal.h
 * @brief Internal implementation details for ManifestDatabase
 *
 * This header is NOT part of the public API. Include ManifestDatabase.h instead.
 */

#include "ManifestDatabase.h"
#include "tqdb.h"
#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────

static constexpr const char* MANIFEST_DB_PATH = "/sd/manifest.tqdb";
static constexpr const char* MANIFEST_TAG = "ManifestDB";

// Max sanity limits for corrupt file detection
static constexpr uint32_t MAX_PATTERNS = 10000;
static constexpr uint32_t MAX_PLAYLISTS = 5000;
static constexpr uint32_t MAX_JOBS = 5000;

// ─────────────────────────────────────────────────────────────────────────────
// Implementation class
// ─────────────────────────────────────────────────────────────────────────────

class ManifestDatabase::Impl {
public:
    tqdb_t db = nullptr;
    bool initialized = false;

    // Scratch buffer for TQDB (SPIRAM preferred)
    uint8_t* scratch = nullptr;
    static constexpr size_t SCRATCH_SIZE = 8192;

    // WAL memory buffer for hybrid WAL (avoids tmpfile() on ESP32)
    uint8_t* wal_buf = nullptr;
    static constexpr size_t WAL_BUF_SIZE = 16384;
    static constexpr size_t WAL_FLUSH_THRESHOLD = 10;  // Flush every 10 writes
};

// ─────────────────────────────────────────────────────────────────────────────
// TQDB Platform Wrappers
// ─────────────────────────────────────────────────────────────────────────────

// FreeRTOS mutex operations for TQDB
extern tqdb_mutex_ops_t g_freertos_mutex_ops;

// SPIRAM-preferred allocator for TQDB
extern tqdb_alloc_t g_spiram_alloc;

// ─────────────────────────────────────────────────────────────────────────────
// TQDB Traits
// ─────────────────────────────────────────────────────────────────────────────

// Entity traits for TQDB serialization
extern const tqdb_trait_t g_pattern_trait;
extern const tqdb_trait_t g_playlist_trait;
extern const tqdb_trait_t g_job_trait;
