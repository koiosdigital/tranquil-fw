/**
 * @file manifest_core.cpp
 * @brief ManifestDatabase core: singleton, lifecycle, and utility functions
 */

#include "manifest_internal.h"
#include "../jobs/job_types.h"
#include "sd.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_heap_caps.h"
#include "kdc_heap_tracing.h"
#include <sys/stat.h>
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
    if (impl_->initialized) return ESP_OK;

    init_sd();
    //format_sd();

    struct stat st = { 0 };
    if (stat("/sd/patterns", &st) == -1) {
        if (mkdir("/sd/patterns", 0775) != 0) {
            ESP_LOGE(TAG, "Failed to create /sd/patterns");
            return ESP_FAIL;
        }
    }

    kdc_heap_log_status("post-sd-init");

    // Allocate scratch buffer from SPIRAM
    impl_->scratch = static_cast<uint8_t*>(
        heap_caps_malloc(Impl::SCRATCH_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!impl_->scratch) {
        impl_->scratch = static_cast<uint8_t*>(malloc(Impl::SCRATCH_SIZE));
    }
    if (!impl_->scratch) {
        ESP_LOGE(TAG, "Failed to allocate scratch buffer");
        return ESP_ERR_NO_MEM;
    }

    // Allocate WAL memory buffer from SPIRAM (for hybrid WAL mode)
    impl_->wal_buf = static_cast<uint8_t*>(
        heap_caps_malloc(Impl::WAL_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!impl_->wal_buf) {
        impl_->wal_buf = static_cast<uint8_t*>(malloc(Impl::WAL_BUF_SIZE));
    }
    if (!impl_->wal_buf) {
        ESP_LOGW(TAG, "Failed to allocate WAL buffer, WAL disabled");
    }

    // Configure and open TQDB
    tqdb_config_t cfg = {};
    cfg.db_path = MANIFEST_DB_PATH;
    cfg.alloc = &g_spiram_alloc;
    cfg.mutex = &g_freertos_mutex_ops;
    cfg.scratch_buf = impl_->scratch;
    cfg.scratch_size = Impl::SCRATCH_SIZE;

    // Enable hybrid WAL if we have memory buffer (avoids tmpfile on ESP32)
    if (impl_->wal_buf) {
        cfg.enable_wal = true;
        cfg.wal_mem_buf = impl_->wal_buf;
        cfg.wal_mem_buf_size = Impl::WAL_BUF_SIZE;
        cfg.wal_flush_threshold = Impl::WAL_FLUSH_THRESHOLD;
    }
    else {
        cfg.enable_wal = false;
    }

    tqdb_err_t err = tqdb_open(&cfg, &impl_->db);
    if (err != TQDB_OK) {
        ESP_LOGE(TAG, "Failed to open TQDB: %d", err);
        heap_caps_free(impl_->scratch);
        impl_->scratch = nullptr;
        if (impl_->wal_buf) { heap_caps_free(impl_->wal_buf); impl_->wal_buf = nullptr; }
        return ESP_FAIL;
    }

    // Register entity types
    err = tqdb_register(impl_->db, &g_pattern_trait);
    if (err != TQDB_OK) {
        ESP_LOGE(TAG, "Failed to register Pattern trait: %d", err);
        tqdb_close(impl_->db);
        impl_->db = nullptr;
        heap_caps_free(impl_->scratch);
        impl_->scratch = nullptr;
        if (impl_->wal_buf) { heap_caps_free(impl_->wal_buf); impl_->wal_buf = nullptr; }
        return ESP_FAIL;
    }

    err = tqdb_register(impl_->db, &g_playlist_trait);
    if (err != TQDB_OK) {
        ESP_LOGE(TAG, "Failed to register Playlist trait: %d", err);
        tqdb_close(impl_->db);
        impl_->db = nullptr;
        heap_caps_free(impl_->scratch);
        impl_->scratch = nullptr;
        if (impl_->wal_buf) { heap_caps_free(impl_->wal_buf); impl_->wal_buf = nullptr; }
        return ESP_FAIL;
    }

    err = tqdb_register(impl_->db, &g_job_trait);
    if (err != TQDB_OK) {
        ESP_LOGE(TAG, "Failed to register Job trait: %d", err);
        tqdb_close(impl_->db);
        impl_->db = nullptr;
        heap_caps_free(impl_->scratch);
        impl_->scratch = nullptr;
        if (impl_->wal_buf) { heap_caps_free(impl_->wal_buf); impl_->wal_buf = nullptr; }
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Registered 3 entity types: Pattern, Playlist, Job");

    kdc_heap_log_status("post-init");

    impl_->initialized = true;
    ESP_LOGI(TAG, "Initialized with TQDB (8KB scratch buffer)");
    return ESP_OK;
}

void ManifestDatabase::shutdown() {
    if (!impl_->initialized) return;

    if (impl_->db) {
        // WAL buffer is freed by tqdb_close (flushes pending entries first)
        tqdb_close(impl_->db);
        impl_->db = nullptr;
    }

    if (impl_->scratch) {
        heap_caps_free(impl_->scratch);
        impl_->scratch = nullptr;
    }

    if (impl_->wal_buf) {
        heap_caps_free(impl_->wal_buf);
        impl_->wal_buf = nullptr;
    }

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
    // No-op - TQDB handles its own memory
}

esp_err_t ManifestDatabase::vacuum() {
    if (!impl_->initialized) return ESP_ERR_INVALID_STATE;

    // Delete completed and failed jobs
    tqdb_delete_where(impl_->db, "Job",
        [](const void* entity, void*) -> bool {
            const jobs::Job* j = static_cast<const jobs::Job*>(entity);
            return j->status != jobs::JobStatus::Completed &&
                j->status != jobs::JobStatus::Failed;  // Keep if true
        }, nullptr);

    tqdb_err_t err = tqdb_vacuum(impl_->db);
    if (err == TQDB_OK) {
        ESP_LOGI(TAG, "Vacuum complete");
        return ESP_OK;
    }
    return ESP_FAIL;
}
