/**
 * @file manifest_core.cpp
 * @brief ManifestDatabase core: singleton, lifecycle, and utility functions
 */

#include "manifest_internal.h"
#include "sd.h"
#include "esp_log.h"
#include "esp_random.h"
#include <sys/stat.h>
#include <unistd.h>
#include <cstdio>
#include <ctime>
#include <memory>

static const char* TAG = MANIFEST_TAG;

// ─────────────────────────────────────────────────────────────────────────────
// Singleton
// ─────────────────────────────────────────────────────────────────────────────

ManifestDatabase::ManifestDatabase() : impl_(std::make_unique<Impl>()) {}
ManifestDatabase::~ManifestDatabase() { shutdown(); }

ManifestDatabase& ManifestDatabase::instance() {
    static ManifestDatabase inst;
    return inst;
}

bool ManifestDatabase::isInitialized() const {
    return impl_->initialized;
}

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

esp_err_t ManifestDatabase::initialize() {
    ManifestLock lock(impl_->mutex);
    if (impl_->initialized) return ESP_OK;

    init_sd();

    struct stat st = { 0 };
    if (stat("/sd/patterns", &st) == -1) {
        if (mkdir("/sd/patterns", 0775) != 0) {
            ESP_LOGE(TAG, "Failed to create /sd/patterns");
            return ESP_FAIL;
        }
    }
    if (stat(MANIFEST_DIR, &st) == -1) {
        if (mkdir(MANIFEST_DIR, 0775) != 0) {
            ESP_LOGE(TAG, "Failed to create %s", MANIFEST_DIR);
            return ESP_FAIL;
        }
    }

    // Retire a leftover tqdb database. Its contents are not migrated; the
    // library re-syncs from the cloud. Keep the file aside for forensics.
    if (stat(LEGACY_TQDB_PATH, &st) == 0) {
        char bak[64];
        snprintf(bak, sizeof(bak), "%s.bak", LEGACY_TQDB_PATH);
        unlink(bak);
        if (rename(LEGACY_TQDB_PATH, bak) == 0) {
            ESP_LOGW(TAG, "Retired legacy tqdb database to %s (not migrated)", bak);
        }
    }

    // Refuse to come up on read errors (as opposed to missing snapshots):
    // initializing with silently-empty tables would overwrite real data on
    // the first mutation.
    bool loaded = impl_->patterns.load();
    loaded = impl_->playlists.load() && loaded;
    loaded = impl_->jobs.load() && loaded;
    if (!loaded) {
        ESP_LOGE(TAG, "Snapshot load failed, manifest unavailable");
        return ESP_FAIL;
    }

    impl_->initialized = true;
    ESP_LOGI(TAG, "Initialized: %zu patterns, %zu playlists, %zu jobs",
        impl_->patterns.count(), impl_->playlists.count(), impl_->jobs.count());
    return ESP_OK;
}

void ManifestDatabase::shutdown() {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return;

    // Every mutation already snapshots, so this is just belt and braces.
    impl_->patterns.save();
    impl_->playlists.save();
    impl_->jobs.save();

    impl_->initialized = false;
    ESP_LOGI(TAG, "Shutdown complete");
}

// ─────────────────────────────────────────────────────────────────────────────
// Utility Functions
// ─────────────────────────────────────────────────────────────────────────────

std::string ManifestDatabase::generateUUID() {
    uint32_t r1 = esp_random();
    uint32_t r2 = esp_random();
    uint32_t r3 = esp_random();
    uint32_t r4 = esp_random();

    char uuid[37];
    snprintf(uuid, sizeof(uuid), "%08lx-%04lx-%04lx-%04lx-%08lx%04lx",
        (unsigned long)r1,
        (unsigned long)((r2 >> 16) & 0xFFFF),
        (unsigned long)(r2 & 0xFFFF),
        (unsigned long)((r3 >> 16) & 0xFFFF),
        (unsigned long)r4,
        (unsigned long)(r3 & 0xFFFF));
    return uuid;
}

std::string ManifestDatabase::currentTimestamp() {
    time_t now = time(nullptr);
    struct tm tm_info;
    gmtime_r(&now, &tm_info);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm_info);
    return buf;
}

void ManifestDatabase::releaseMemory() {
    // No-op - records live in PSRAM already
}

esp_err_t ManifestDatabase::vacuum() {
    ManifestLock lock(impl_->mutex);
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    // Delete completed and failed jobs
    impl_->jobs.modify(
        [](uint32_t, cJSON** obj, void*) -> manifest::JsonlTable::Action {
            jobs::Job j = job_from_storage(*obj);
            if (j.status == jobs::JobStatus::Completed ||
                j.status == jobs::JobStatus::Failed) {
                return manifest::JsonlTable::Action::Remove;
            }
            return manifest::JsonlTable::Action::Keep;
        }, nullptr);

    // Snapshots rewrite in full, so saving IS compaction.
    bool ok = impl_->patterns.save() && impl_->playlists.save() && impl_->jobs.save();
    if (ok) {
        ESP_LOGI(TAG, "Vacuum complete");
        return ESP_OK;
    }
    return ESP_FAIL;
}
