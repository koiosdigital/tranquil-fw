#include "ManifestDatabase.h"
#include "jobs/job_types.h"
#include "sd.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <sys/stat.h>
#include <ctime>
#include <algorithm>
#include <unistd.h>
#include <unordered_map>
#include <cstring>

static const char* TAG = "ManifestDB";
static constexpr const char* DB_PATH = "/sd/manifest.tqdb";
static constexpr const char* DB_TMP_PATH = "/sd/manifest.tqdb.tmp";
static constexpr const char* DB_BAK_PATH = "/sd/manifest.tqdb.bak";

static constexpr uint32_t TQDB_MAGIC = 0x42445154;  // "TQDB" little-endian
static constexpr uint16_t TQDB_VERSION = 1;

// Max sanity limits for corrupt file detection
static constexpr uint32_t MAX_PATTERNS = 10000;
static constexpr uint32_t MAX_PLAYLISTS = 5000;
static constexpr uint32_t MAX_JOBS = 5000;
static constexpr uint16_t MAX_STRING = 4096;

// I/O buffer size (4KB matches SD card allocation and HTTP chunk patterns)
static constexpr size_t IO_BUFFER_SIZE = 4096;

// ─────────────────────────────────────────────────────────────────────────────
// CRC32 (standard polynomial 0xEDB88320)
// ─────────────────────────────────────────────────────────────────────────────

static uint32_t s_crc32_table[256];
static bool s_crc32_ready = false;

static void crc32_init() {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int j = 0; j < 8; j++)
            c = (c >> 1) ^ (c & 1 ? 0xEDB88320u : 0);
        s_crc32_table[i] = c;
    }
    s_crc32_ready = true;
}

static inline uint32_t crc32_byte(uint32_t crc, uint8_t b) {
    return s_crc32_table[(crc ^ b) & 0xFF] ^ (crc >> 8);
}

// ─────────────────────────────────────────────────────────────────────────────
// Buffered binary stream writers/readers with integrated CRC tracking
// Uses caller-provided 4KB buffer to reduce syscall overhead on SD card
// ─────────────────────────────────────────────────────────────────────────────

class BinaryWriter {
    FILE* f_;
    uint32_t crc_;
    bool error_;
    uint8_t* buf_;
    size_t buf_size_;
    size_t buf_pos_;

public:
    BinaryWriter(FILE* f, uint8_t* buffer, size_t buffer_size)
        : f_(f), crc_(0xFFFFFFFF), error_(false),
        buf_(buffer), buf_size_(buffer_size), buf_pos_(0) {
    }

    bool hasError() const { return error_; }
    uint32_t crc() const { return ~crc_; }

    void flush() {
        if (error_ || buf_pos_ == 0) return;
        if (fwrite(buf_, 1, buf_pos_, f_) != buf_pos_) {
            error_ = true;
        }
        buf_pos_ = 0;
    }

    void raw(const void* data, size_t len) {
        if (error_) return;
        auto p = static_cast<const uint8_t*>(data);

        // Update CRC as data arrives
        for (size_t i = 0; i < len; i++)
            crc_ = crc32_byte(crc_, p[i]);

        // Fast path: fits in buffer
        if (len <= buf_size_ - buf_pos_) {
            memcpy(buf_ + buf_pos_, p, len);
            buf_pos_ += len;
            return;
        }

        // Flush current buffer
        flush();
        if (error_) return;

        // Large write: bypass buffer
        if (len >= buf_size_) {
            if (fwrite(p, 1, len, f_) != len) error_ = true;
            return;
        }

        // Copy to fresh buffer
        memcpy(buf_, p, len);
        buf_pos_ = len;
    }

    void u8(uint8_t  v) { raw(&v, 1); }
    void u16(uint16_t v) { raw(&v, 2); }
    void u32(uint32_t v) { raw(&v, 4); }
    void i32(int32_t  v) { raw(&v, 4); }
    void i64(int64_t  v) { raw(&v, 8); }

    void str(const std::string& s) {
        uint16_t len = static_cast<uint16_t>(std::min(s.size(), size_t(0xFFFF)));
        u16(len);
        if (len > 0) raw(s.data(), len);
    }
};

class BinaryReader {
    FILE* f_;
    uint32_t crc_;
    bool error_;
    uint8_t* buf_;
    size_t buf_size_;
    size_t buf_pos_;
    size_t buf_filled_;

public:
    BinaryReader(FILE* f, uint8_t* buffer, size_t buffer_size)
        : f_(f), crc_(0xFFFFFFFF), error_(false),
        buf_(buffer), buf_size_(buffer_size), buf_pos_(0), buf_filled_(0) {
    }

    bool hasError() const { return error_; }
    uint32_t crc() const { return ~crc_; }

    void raw(void* data, size_t len) {
        if (error_) return;
        auto p = static_cast<uint8_t*>(data);
        size_t written = 0;

        while (written < len && !error_) {
            size_t avail = buf_filled_ - buf_pos_;
            if (avail > 0) {
                size_t to_copy = std::min(avail, len - written);
                memcpy(p + written, buf_ + buf_pos_, to_copy);
                buf_pos_ += to_copy;
                written += to_copy;
            }
            else {
                // Refill buffer
                buf_filled_ = fread(buf_, 1, buf_size_, f_);
                buf_pos_ = 0;
                if (buf_filled_ == 0) {
                    error_ = true;
                    return;
                }
            }
        }

        // Update CRC
        for (size_t i = 0; i < len; i++)
            crc_ = crc32_byte(crc_, p[i]);
    }

    uint8_t  u8() { uint8_t  v = 0; raw(&v, 1); return v; }
    uint16_t u16() { uint16_t v = 0; raw(&v, 2); return v; }
    uint32_t u32() { uint32_t v = 0; raw(&v, 4); return v; }
    int32_t  i32() { int32_t  v = 0; raw(&v, 4); return v; }
    int64_t  i64() { int64_t  v = 0; raw(&v, 8); return v; }

    std::string str() {
        uint16_t len = u16();
        if (error_ || len > MAX_STRING) { error_ = true; return ""; }
        std::string s(len, '\0');
        if (len > 0) raw(s.data(), len);
        return s;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Impl – in-memory storage + binary persistence
// ─────────────────────────────────────────────────────────────────────────────

class ManifestDatabase::Impl {
public:
    std::unordered_map<std::string, Pattern>    patterns;
    std::unordered_map<std::string, Playlist>   playlists;
    std::unordered_map<std::string, jobs::Job>  jobs;

    SemaphoreHandle_t mutex = nullptr;
    bool initialized = false;

    // RAII mutex guard
    class Lock {
    public:
        explicit Lock(SemaphoreHandle_t m, TickType_t timeout = pdMS_TO_TICKS(5000))
            : mutex_(m), acquired_(false) {
            if (mutex_ && xSemaphoreTake(mutex_, timeout) == pdTRUE)
                acquired_ = true;
        }
        ~Lock() { if (acquired_ && mutex_) xSemaphoreGive(mutex_); }
        bool acquired() const { return acquired_; }
    private:
        SemaphoreHandle_t mutex_;
        bool acquired_;
    };

    // ── Sorted helpers (sort-on-demand; fast for <1000 items) ────────────

    std::vector<const Pattern*> patternsSortedByName() const {
        std::vector<const Pattern*> v;
        v.reserve(patterns.size());
        for (auto& [_, p] : patterns) v.push_back(&p);
        std::sort(v.begin(), v.end(),
            [](const Pattern* a, const Pattern* b) { return a->name < b->name; });
        return v;
    }

    std::vector<const Playlist*> playlistsSortedByName() const {
        std::vector<const Playlist*> v;
        v.reserve(playlists.size());
        for (auto& [_, pl] : playlists) v.push_back(&pl);
        std::sort(v.begin(), v.end(),
            [](const Playlist* a, const Playlist* b) { return a->name < b->name; });
        return v;
    }

    // ── Binary serialization ─────────────────────────────────────────────

    static void writePattern(BinaryWriter& w, const Pattern& p) {
        w.str(p.uuid);
        w.str(p.name);
        w.str(p.creator);
        w.str(p.date);
        w.i32(p.popularity);
        uint8_t flags = (p.reversible ? 0x01 : 0)
            | (p.encrypted ? 0x02 : 0)
            | (p.purchased ? 0x04 : 0);
        w.u8(flags);
        w.i32(p.start_point);
        w.i64(p.size_bytes);
        w.str(p.created_at);
        w.str(p.last_played_at);
        w.str(p.downloaded_at);
        w.i64(p.purchased_at);
        w.str(p.receipt_id);
    }

    static Pattern readPattern(BinaryReader& r) {
        Pattern p;
        p.uuid = r.str();
        p.name = r.str();
        p.creator = r.str();
        p.date = r.str();
        p.popularity = r.i32();
        uint8_t flags = r.u8();
        p.reversible = flags & 0x01;
        p.encrypted = flags & 0x02;
        p.purchased = flags & 0x04;
        p.start_point = r.i32();
        p.size_bytes = r.i64();
        p.created_at = r.str();
        p.last_played_at = r.str();
        p.downloaded_at = r.str();
        p.purchased_at = r.i64();
        p.receipt_id = r.str();
        return p;
    }

    static void writePlaylist(BinaryWriter& w, const Playlist& pl) {
        w.str(pl.uuid);
        w.str(pl.name);
        w.str(pl.description);
        w.str(pl.featured_pattern);
        w.str(pl.date);
        w.str(pl.created_at);
        w.str(pl.updated_at);
        w.u32(static_cast<uint32_t>(pl.patterns.size()));
        for (auto& uuid : pl.patterns) w.str(uuid);
    }

    static Playlist readPlaylist(BinaryReader& r) {
        Playlist pl;
        pl.uuid = r.str();
        pl.name = r.str();
        pl.description = r.str();
        pl.featured_pattern = r.str();
        pl.date = r.str();
        pl.created_at = r.str();
        pl.updated_at = r.str();
        uint32_t n = r.u32();
        if (n > MAX_PATTERNS) { return pl; }  // sanity
        pl.patterns.reserve(n);
        for (uint32_t i = 0; i < n; i++) pl.patterns.push_back(r.str());
        return pl;
    }

    static void writeJob(BinaryWriter& w, const jobs::Job& j) {
        w.str(j.uuid);
        w.str(jobs::jobTypeToString(j.type));
        w.str(j.pattern_uuid);
        w.str(jobs::jobStatusToString(j.status));
        w.i32(j.priority);
        w.i32(j.retry_count);
        w.i32(j.max_retries);
        w.str(j.created_at);
        w.str(j.started_at);
        w.str(j.completed_at);
        w.str(j.error_message);
        w.str(j.job_data);
    }

    static jobs::Job readJob(BinaryReader& r) {
        jobs::Job j;
        j.uuid = r.str();
        j.type = jobs::jobTypeFromString(r.str().c_str());
        j.pattern_uuid = r.str();
        j.status = jobs::jobStatusFromString(r.str().c_str());
        j.priority = r.i32();
        j.retry_count = r.i32();
        j.max_retries = r.i32();
        j.created_at = r.str();
        j.started_at = r.str();
        j.completed_at = r.str();
        j.error_message = r.str();
        j.job_data = r.str();
        return j;
    }

    // ── Persistence ──────────────────────────────────────────────────────

    esp_err_t save() {
        FILE* f = fopen(DB_TMP_PATH, "wb");
        if (!f) {
            ESP_LOGE(TAG, "Failed to open %s for writing", DB_TMP_PATH);
            return ESP_FAIL;
        }

        // Write 16-byte header (CRC placeholder at offset 8)
        uint32_t magic = TQDB_MAGIC;
        uint16_t version = TQDB_VERSION;
        uint16_t flags = 0;
        uint32_t crc_placeholder = 0;
        uint32_t reserved = 0;
        fwrite(&magic, 4, 1, f);
        fwrite(&version, 2, 1, f);
        fwrite(&flags, 2, 1, f);
        fwrite(&crc_placeholder, 4, 1, f);
        fwrite(&reserved, 4, 1, f);

        // 4KB stack buffer for buffered writes
        uint8_t scratch[IO_BUFFER_SIZE];
        BinaryWriter w(f, scratch, sizeof(scratch));

        // Patterns
        w.u32(static_cast<uint32_t>(patterns.size()));
        for (auto& [_, p] : patterns) writePattern(w, p);

        // Playlists
        w.u32(static_cast<uint32_t>(playlists.size()));
        for (auto& [_, pl] : playlists) writePlaylist(w, pl);

        // Jobs
        w.u32(static_cast<uint32_t>(jobs.size()));
        for (auto& [_, j] : jobs) writeJob(w, j);

        // Flush remaining buffered data
        w.flush();

        if (w.hasError()) {
            fclose(f);
            unlink(DB_TMP_PATH);
            ESP_LOGE(TAG, "Write error during save");
            return ESP_FAIL;
        }

        // Patch CRC into header
        uint32_t crc = w.crc();
        fseek(f, 8, SEEK_SET);
        fwrite(&crc, 4, 1, f);
        fflush(f);
        fclose(f);

        // Atomic-ish swap: tmp → live (safe on FAT32 power loss)
        remove(DB_BAK_PATH);
        rename(DB_PATH, DB_BAK_PATH);    // old → bak (may fail if no old file)
        if (rename(DB_TMP_PATH, DB_PATH) != 0) {
            // Try to restore backup
            rename(DB_BAK_PATH, DB_PATH);
            ESP_LOGE(TAG, "Failed to rename tmp to live");
            return ESP_FAIL;
        }
        remove(DB_BAK_PATH);

        ESP_LOGD(TAG, "Saved: %zu patterns, %zu playlists, %zu jobs",
            patterns.size(), playlists.size(), jobs.size());
        return ESP_OK;
    }

    esp_err_t load() {
        // Recovery: handle incomplete previous writes
        struct stat st;
        if (stat(DB_PATH, &st) != 0) {
            // No main file — check for temp or backup
            if (stat(DB_TMP_PATH, &st) == 0) {
                rename(DB_TMP_PATH, DB_PATH);
            }
            else if (stat(DB_BAK_PATH, &st) == 0) {
                rename(DB_BAK_PATH, DB_PATH);
            }
            else {
                ESP_LOGI(TAG, "No existing database — starting fresh");
                return ESP_ERR_NOT_FOUND;
            }
        }
        else {
            // Main file exists — clean up any stale temp
            unlink(DB_TMP_PATH);
        }

        FILE* f = fopen(DB_PATH, "rb");
        if (!f) {
            ESP_LOGW(TAG, "Cannot open %s", DB_PATH);
            return ESP_ERR_NOT_FOUND;
        }

        // Read and validate header
        uint32_t magic = 0;
        uint16_t version = 0, flags = 0;
        uint32_t stored_crc = 0, reserved = 0;
        fread(&magic, 4, 1, f);
        fread(&version, 2, 1, f);
        fread(&flags, 2, 1, f);
        fread(&stored_crc, 4, 1, f);
        fread(&reserved, 4, 1, f);

        if (magic != TQDB_MAGIC) {
            ESP_LOGE(TAG, "Bad magic: 0x%08lx", (unsigned long)magic);
            fclose(f);
            return ESP_ERR_INVALID_STATE;
        }
        if (version > TQDB_VERSION) {
            ESP_LOGE(TAG, "Unsupported version: %u (max %u)", version, TQDB_VERSION);
            fclose(f);
            return ESP_ERR_INVALID_VERSION;
        }

        // 4KB stack buffer for buffered reads
        uint8_t scratch[IO_BUFFER_SIZE];
        BinaryReader r(f, scratch, sizeof(scratch));

        // Patterns
        uint32_t nPatterns = r.u32();
        if (nPatterns > MAX_PATTERNS) {
            ESP_LOGE(TAG, "Pattern count too large: %lu", (unsigned long)nPatterns);
            fclose(f);
            return ESP_ERR_INVALID_STATE;
        }
        for (uint32_t i = 0; i < nPatterns && !r.hasError(); i++) {
            Pattern p = readPattern(r);
            if (!p.uuid.empty()) patterns[p.uuid] = std::move(p);
        }

        // Playlists
        uint32_t nPlaylists = r.u32();
        if (nPlaylists > MAX_PLAYLISTS) {
            ESP_LOGE(TAG, "Playlist count too large: %lu", (unsigned long)nPlaylists);
            fclose(f);
            patterns.clear();
            return ESP_ERR_INVALID_STATE;
        }
        for (uint32_t i = 0; i < nPlaylists && !r.hasError(); i++) {
            Playlist pl = readPlaylist(r);
            if (!pl.uuid.empty()) playlists[pl.uuid] = std::move(pl);
        }

        // Jobs
        uint32_t nJobs = r.u32();
        if (nJobs > MAX_JOBS) {
            ESP_LOGE(TAG, "Job count too large: %lu", (unsigned long)nJobs);
            fclose(f);
            patterns.clear();
            playlists.clear();
            return ESP_ERR_INVALID_STATE;
        }
        for (uint32_t i = 0; i < nJobs && !r.hasError(); i++) {
            jobs::Job j = readJob(r);
            if (!j.uuid.empty()) jobs[j.uuid] = std::move(j);
        }

        fclose(f);

        if (r.hasError()) {
            ESP_LOGE(TAG, "Read error during load");
            patterns.clear();
            playlists.clear();
            jobs.clear();
            return ESP_ERR_INVALID_STATE;
        }

        // Validate CRC
        if (r.crc() != stored_crc) {
            ESP_LOGE(TAG, "CRC mismatch: computed=0x%08lx stored=0x%08lx",
                (unsigned long)r.crc(), (unsigned long)stored_crc);
            patterns.clear();
            playlists.clear();
            jobs.clear();
            return ESP_ERR_INVALID_CRC;
        }

        ESP_LOGI(TAG, "Loaded: %zu patterns, %zu playlists, %zu jobs",
            patterns.size(), playlists.size(), jobs.size());
        return ESP_OK;
    }
};

// ═════════════════════════════════════════════════════════════════════════════
// ManifestDatabase public API implementation
// ═════════════════════════════════════════════════════════════════════════════

ManifestDatabase::ManifestDatabase() : impl_(std::make_unique<Impl>()) {}
ManifestDatabase::~ManifestDatabase() { shutdown(); }

ManifestDatabase& ManifestDatabase::instance() {
    static ManifestDatabase inst;
    return inst;
}

bool ManifestDatabase::isInitialized() const {
    return impl_->initialized;
}

esp_err_t ManifestDatabase::initialize() {
    if (impl_->initialized) return ESP_OK;

    if (!s_crc32_ready) crc32_init();

    init_sd();
    //format_sd();

    struct stat st = { 0 };
    if (stat("/sd/patterns", &st) == -1) {
        if (mkdir("/sd/patterns", 0775) != 0) {
            ESP_LOGE(TAG, "Failed to create /sd/patterns");
            return ESP_FAIL;
        }
    }

    impl_->mutex = xSemaphoreCreateMutex();
    if (!impl_->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    // Load existing data (ESP_ERR_NOT_FOUND is fine — first boot)
    esp_err_t rc = impl_->load();
    if (rc != ESP_OK && rc != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Load failed (rc=%d), starting with empty database", rc);
        // Continue with empty data — non-fatal
    }

    impl_->initialized = true;
    ESP_LOGI(TAG, "Initialized successfully");
    return ESP_OK;
}

void ManifestDatabase::shutdown() {
    if (!impl_->initialized) return;

    // Persist current state
    impl_->save();

    if (impl_->mutex) {
        vSemaphoreDelete(impl_->mutex);
        impl_->mutex = nullptr;
    }

    impl_->patterns.clear();
    impl_->playlists.clear();
    impl_->jobs.clear();
    impl_->initialized = false;
    ESP_LOGI(TAG, "Shutdown complete");
}

// ─────────────────────────────────────────────────────────────────────────────
// Pattern CRUD
// ─────────────────────────────────────────────────────────────────────────────

esp_err_t ManifestDatabase::addPattern(const Pattern& pattern) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    if (impl_->patterns.count(pattern.uuid)) {
        ESP_LOGW(TAG, "Pattern already exists: %s", pattern.uuid.c_str());
        return ESP_ERR_INVALID_STATE;
    }

    Pattern p = pattern;
    if (p.created_at.empty()) p.created_at = currentTimestamp();
    if (p.creator.empty()) p.creator = "Uploaded";

    impl_->patterns[p.uuid] = std::move(p);

    esp_err_t rc = impl_->save();
    if (rc == ESP_OK) {
        ESP_LOGI(TAG, "Pattern added: %s", pattern.uuid.c_str());
    }
    return rc;
}

esp_err_t ManifestDatabase::updatePattern(const std::string& uuid, const Pattern& pattern) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    auto it = impl_->patterns.find(uuid);
    if (it == impl_->patterns.end()) return ESP_ERR_NOT_FOUND;

    // Preserve fields not in the update
    Pattern& existing = it->second;
    existing.name = pattern.name;
    existing.creator = pattern.creator;
    existing.date = pattern.date;
    existing.popularity = pattern.popularity;
    existing.reversible = pattern.reversible;
    existing.start_point = pattern.start_point;
    existing.encrypted = pattern.encrypted;
    existing.size_bytes = pattern.size_bytes;
    existing.purchased = pattern.purchased;
    existing.purchased_at = pattern.purchased_at;
    existing.receipt_id = pattern.receipt_id;

    return impl_->save();
}

esp_err_t ManifestDatabase::deletePattern(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    if (!impl_->patterns.erase(uuid)) return ESP_ERR_NOT_FOUND;

    // Remove from all playlists
    for (auto& [_, pl] : impl_->playlists) {
        auto& pats = pl.patterns;
        pats.erase(std::remove(pats.begin(), pats.end(), uuid), pats.end());
        if (pl.featured_pattern == uuid) pl.featured_pattern.clear();
    }

    esp_err_t rc = impl_->save();
    if (rc == ESP_OK) {
        // Delete pattern files
        char path[128];
        snprintf(path, sizeof(path), "/sd/patterns/%s.thr", uuid.c_str());
        unlink(path);
        snprintf(path, sizeof(path), "/sd/patterns/%s.dat", uuid.c_str());
        unlink(path);
        snprintf(path, sizeof(path), "/sd/previews/%s.png", uuid.c_str());
        unlink(path);
        ESP_LOGI(TAG, "Pattern deleted: %s", uuid.c_str());
    }
    return rc;
}

std::vector<Pattern> ManifestDatabase::getAllPatterns() {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return {};

    auto sorted = impl_->patternsSortedByName();
    std::vector<Pattern> result;
    result.reserve(sorted.size());
    for (auto* p : sorted) result.push_back(*p);
    return result;
}

PaginatedResult<Pattern> ManifestDatabase::getPatterns(int page, int per_page) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return {};

    PaginatedResult<Pattern> result;
    auto sorted = impl_->patternsSortedByName();

    result.pagination.total_items = static_cast<int>(sorted.size());
    result.pagination.page = page;
    result.pagination.per_page = per_page;
    result.pagination.total_pages = (result.pagination.total_items + per_page - 1) / per_page;

    int start = page * per_page;
    int end = std::min(start + per_page, static_cast<int>(sorted.size()));
    for (int i = start; i < end; i++) {
        result.items.push_back(*sorted[i]);
    }
    return result;
}

std::optional<Pattern> ManifestDatabase::getPattern(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return std::nullopt;

    auto it = impl_->patterns.find(uuid);
    if (it != impl_->patterns.end()) return it->second;
    return std::nullopt;
}

bool ManifestDatabase::patternExists(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return false;
    return impl_->patterns.count(uuid) > 0;
}

size_t ManifestDatabase::getPatternCount() {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return 0;
    return impl_->patterns.size();
}

size_t ManifestDatabase::getSubscriptionPatternCount() {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return 0;

    size_t count = 0;
    for (auto& [_, p] : impl_->patterns) {
        if (!p.purchased) count++;
    }
    return count;
}

esp_err_t ManifestDatabase::updateLastPlayed(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    auto it = impl_->patterns.find(uuid);
    if (it == impl_->patterns.end()) return ESP_ERR_NOT_FOUND;

    it->second.last_played_at = currentTimestamp();
    return impl_->save();
}

esp_err_t ManifestDatabase::incrementPopularity(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    auto it = impl_->patterns.find(uuid);
    if (it == impl_->patterns.end()) return ESP_ERR_NOT_FOUND;

    it->second.popularity++;
    return impl_->save();
}

// ─────────────────────────────────────────────────────────────────────────────
// Playlist CRUD
// ─────────────────────────────────────────────────────────────────────────────

esp_err_t ManifestDatabase::addPlaylist(const Playlist& playlist) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    if (impl_->playlists.count(playlist.uuid)) return ESP_ERR_INVALID_STATE;

    Playlist pl = playlist;
    std::string ts = pl.created_at.empty() ? currentTimestamp() : pl.created_at;
    pl.created_at = ts;
    if (pl.updated_at.empty()) pl.updated_at = ts;

    // Filter to only patterns that exist
    std::vector<std::string> valid;
    for (auto& puuid : pl.patterns) {
        if (impl_->patterns.count(puuid)) valid.push_back(puuid);
    }
    pl.patterns = std::move(valid);

    impl_->playlists[pl.uuid] = std::move(pl);

    esp_err_t rc = impl_->save();
    if (rc == ESP_OK) {
        ESP_LOGI(TAG, "Playlist added: %s", playlist.uuid.c_str());
    }
    return rc;
}

esp_err_t ManifestDatabase::updatePlaylist(const std::string& uuid, const Playlist& playlist) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    auto it = impl_->playlists.find(uuid);
    if (it == impl_->playlists.end()) return ESP_ERR_NOT_FOUND;

    Playlist& existing = it->second;
    existing.name = playlist.name;
    existing.description = playlist.description;
    existing.featured_pattern = playlist.featured_pattern;
    existing.date = playlist.date;
    existing.updated_at = currentTimestamp();

    // Filter to existing patterns
    std::vector<std::string> valid;
    for (auto& puuid : playlist.patterns) {
        if (impl_->patterns.count(puuid)) valid.push_back(puuid);
    }
    existing.patterns = std::move(valid);

    return impl_->save();
}

esp_err_t ManifestDatabase::deletePlaylist(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    if (!impl_->playlists.erase(uuid)) return ESP_ERR_NOT_FOUND;

    esp_err_t rc = impl_->save();
    if (rc == ESP_OK) {
        ESP_LOGI(TAG, "Playlist deleted: %s", uuid.c_str());
    }
    return rc;
}

std::vector<Playlist> ManifestDatabase::getAllPlaylists() {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return {};

    auto sorted = impl_->playlistsSortedByName();
    std::vector<Playlist> result;
    result.reserve(sorted.size());
    for (auto* pl : sorted) result.push_back(*pl);
    return result;
}

PaginatedResult<Playlist> ManifestDatabase::getPlaylists(int page, int per_page) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return {};

    PaginatedResult<Playlist> result;
    auto sorted = impl_->playlistsSortedByName();

    result.pagination.total_items = static_cast<int>(sorted.size());
    result.pagination.page = page;
    result.pagination.per_page = per_page;
    result.pagination.total_pages = (result.pagination.total_items + per_page - 1) / per_page;

    int start = page * per_page;
    int end = std::min(start + per_page, static_cast<int>(sorted.size()));
    for (int i = start; i < end; i++) {
        result.items.push_back(*sorted[i]);
    }
    return result;
}

std::optional<Playlist> ManifestDatabase::getPlaylist(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return std::nullopt;

    auto it = impl_->playlists.find(uuid);
    if (it != impl_->playlists.end()) return it->second;
    return std::nullopt;
}

bool ManifestDatabase::playlistExists(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return false;
    return impl_->playlists.count(uuid) > 0;
}

size_t ManifestDatabase::getPlaylistCount() {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return 0;
    return impl_->playlists.size();
}

// ─────────────────────────────────────────────────────────────────────────────
// Playlist-pattern operations
// ─────────────────────────────────────────────────────────────────────────────

esp_err_t ManifestDatabase::addPatternToPlaylist(const std::string& playlistUuid,
    const std::string& patternUuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    auto plIt = impl_->playlists.find(playlistUuid);
    if (plIt == impl_->playlists.end()) return ESP_ERR_NOT_FOUND;
    if (!impl_->patterns.count(patternUuid)) return ESP_ERR_NOT_FOUND;

    auto& pats = plIt->second.patterns;
    // Don't add duplicates
    if (std::find(pats.begin(), pats.end(), patternUuid) != pats.end()) return ESP_OK;

    pats.push_back(patternUuid);
    plIt->second.updated_at = currentTimestamp();
    return impl_->save();
}

esp_err_t ManifestDatabase::removePatternFromPlaylist(const std::string& playlistUuid,
    const std::string& patternUuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    auto plIt = impl_->playlists.find(playlistUuid);
    if (plIt == impl_->playlists.end()) return ESP_ERR_NOT_FOUND;

    auto& pl = plIt->second;
    auto& pats = pl.patterns;
    auto it = std::find(pats.begin(), pats.end(), patternUuid);
    if (it == pats.end()) return ESP_ERR_NOT_FOUND;

    pats.erase(it);

    if (pl.featured_pattern == patternUuid) {
        pl.featured_pattern.clear();
    }
    pl.updated_at = currentTimestamp();
    return impl_->save();
}

esp_err_t ManifestDatabase::setFeaturedPattern(const std::string& playlistUuid,
    const std::string& patternUuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    auto plIt = impl_->playlists.find(playlistUuid);
    if (plIt == impl_->playlists.end()) return ESP_ERR_NOT_FOUND;
    if (!impl_->patterns.count(patternUuid)) return ESP_ERR_NOT_FOUND;

    auto& pl = plIt->second;
    // Verify pattern is in this playlist
    if (std::find(pl.patterns.begin(), pl.patterns.end(), patternUuid) == pl.patterns.end()) {
        return ESP_ERR_NOT_FOUND;
    }

    pl.featured_pattern = patternUuid;
    pl.updated_at = currentTimestamp();
    return impl_->save();
}

esp_err_t ManifestDatabase::reorderPlaylist(const std::string& playlistUuid,
    const std::vector<std::string>& newOrder) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    auto plIt = impl_->playlists.find(playlistUuid);
    if (plIt == impl_->playlists.end()) return ESP_ERR_NOT_FOUND;

    auto& pl = plIt->second;
    if (newOrder.size() != pl.patterns.size()) return ESP_ERR_INVALID_ARG;

    // Validate same elements
    auto sortedOld = pl.patterns;
    auto sortedNew = newOrder;
    std::sort(sortedOld.begin(), sortedOld.end());
    std::sort(sortedNew.begin(), sortedNew.end());
    if (sortedOld != sortedNew) return ESP_ERR_INVALID_ARG;

    pl.patterns = newOrder;
    pl.updated_at = currentTimestamp();
    return impl_->save();
}

std::vector<Pattern> ManifestDatabase::getPlaylistPatterns(const std::string& playlistUuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return {};

    auto plIt = impl_->playlists.find(playlistUuid);
    if (plIt == impl_->playlists.end()) return {};

    std::vector<Pattern> result;
    for (auto& puuid : plIt->second.patterns) {
        auto pIt = impl_->patterns.find(puuid);
        if (pIt != impl_->patterns.end()) {
            result.push_back(pIt->second);
        }
    }
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Job queue operations
// ─────────────────────────────────────────────────────────────────────────────

esp_err_t ManifestDatabase::enqueueJob(const jobs::Job& job) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    jobs::Job j = job;
    if (j.created_at.empty()) j.created_at = currentTimestamp();

    impl_->jobs[j.uuid] = std::move(j);

    esp_err_t rc = impl_->save();
    if (rc == ESP_OK) {
        ESP_LOGI(TAG, "Job enqueued: %s (type=%s, pattern=%s)",
            job.uuid.c_str(), jobs::jobTypeToString(job.type), job.pattern_uuid.c_str());
    }
    return rc;
}

std::optional<jobs::Job> ManifestDatabase::claimNextPendingJob() {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return std::nullopt;

    // Linear scan for highest priority, oldest created_at among pending jobs
    jobs::Job* best = nullptr;
    for (auto& [_, j] : impl_->jobs) {
        if (j.status != jobs::JobStatus::Pending) continue;
        if (!best ||
            j.priority > best->priority ||
            (j.priority == best->priority && j.created_at < best->created_at)) {
            best = &j;
        }
    }
    if (!best) return std::nullopt;

    best->status = jobs::JobStatus::InProgress;
    best->started_at = currentTimestamp();

    jobs::Job result = *best;  // Copy before save
    impl_->save();

    ESP_LOGI(TAG, "Job claimed: %s", result.uuid.c_str());
    return result;
}

std::optional<jobs::Job> ManifestDatabase::getJob(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return std::nullopt;

    auto it = impl_->jobs.find(uuid);
    if (it != impl_->jobs.end()) return it->second;
    return std::nullopt;
}

std::optional<jobs::Job> ManifestDatabase::getJobByPattern(const std::string& pattern_uuid,
    jobs::JobType type) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return std::nullopt;

    for (auto& [_, j] : impl_->jobs) {
        if (j.pattern_uuid == pattern_uuid &&
            j.type == type &&
            (j.status == jobs::JobStatus::Pending || j.status == jobs::JobStatus::InProgress)) {
            return j;
        }
    }
    return std::nullopt;
}

bool ManifestDatabase::hasJob(const std::string& pattern_uuid, jobs::JobType type) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return false;

    for (auto& [_, j] : impl_->jobs) {
        if (j.pattern_uuid == pattern_uuid &&
            j.type == type &&
            (j.status == jobs::JobStatus::Pending || j.status == jobs::JobStatus::InProgress)) {
            return true;
        }
    }
    return false;
}

esp_err_t ManifestDatabase::markJobCompleted(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    auto it = impl_->jobs.find(uuid);
    if (it == impl_->jobs.end()) return ESP_ERR_NOT_FOUND;

    it->second.status = jobs::JobStatus::Completed;
    it->second.completed_at = currentTimestamp();

    ESP_LOGI(TAG, "Job completed: %s", uuid.c_str());
    return impl_->save();
}

esp_err_t ManifestDatabase::markJobFailed(const std::string& uuid, const std::string& error) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    auto it = impl_->jobs.find(uuid);
    if (it == impl_->jobs.end()) return ESP_ERR_NOT_FOUND;

    auto& j = it->second;
    if (j.retry_count < j.max_retries) {
        // Re-queue for retry
        j.status = jobs::JobStatus::Pending;
        j.retry_count++;
        j.error_message = error;
        j.started_at.clear();
        ESP_LOGW(TAG, "Job retry queued: %s (attempt %d/%d) - %s",
            uuid.c_str(), j.retry_count, j.max_retries, error.c_str());
    }
    else {
        // Permanent failure
        j.status = jobs::JobStatus::Failed;
        j.error_message = error;
        j.completed_at = currentTimestamp();
        ESP_LOGE(TAG, "Job failed permanently: %s - %s", uuid.c_str(), error.c_str());
    }

    return impl_->save();
}

esp_err_t ManifestDatabase::deleteJob(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    if (!impl_->jobs.erase(uuid)) return ESP_ERR_NOT_FOUND;

    ESP_LOGI(TAG, "Job deleted: %s", uuid.c_str());
    return impl_->save();
}

esp_err_t ManifestDatabase::cancelJobsForPattern(const std::string& pattern_uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    int deleted = 0;
    for (auto it = impl_->jobs.begin(); it != impl_->jobs.end(); ) {
        if (it->second.pattern_uuid == pattern_uuid &&
            it->second.status == jobs::JobStatus::Pending) {
            it = impl_->jobs.erase(it);
            deleted++;
        }
        else {
            ++it;
        }
    }

    if (deleted > 0) {
        ESP_LOGI(TAG, "Cancelled %d pending jobs for pattern: %s", deleted, pattern_uuid.c_str());
        return impl_->save();
    }
    return ESP_OK;
}

size_t ManifestDatabase::getPendingJobCount() {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return 0;

    size_t count = 0;
    for (auto& [_, j] : impl_->jobs) {
        if (j.status == jobs::JobStatus::Pending) count++;
    }
    return count;
}

size_t ManifestDatabase::getInProgressJobCount() {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return 0;

    size_t count = 0;
    for (auto& [_, j] : impl_->jobs) {
        if (j.status == jobs::JobStatus::InProgress) count++;
    }
    return count;
}

// ─────────────────────────────────────────────────────────────────────────────
// Utility
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
    // No-op: kept for API compatibility (no external cache to release)
}

esp_err_t ManifestDatabase::vacuum() {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    // Prune completed/failed jobs
    int pruned = 0;
    for (auto it = impl_->jobs.begin(); it != impl_->jobs.end(); ) {
        auto& j = it->second;
        if (j.status == jobs::JobStatus::Completed || j.status == jobs::JobStatus::Failed) {
            it = impl_->jobs.erase(it);
            pruned++;
        }
        else {
            ++it;
        }
    }

    // Rewrite file (compacts any fragmentation)
    esp_err_t rc = impl_->save();
    if (rc == ESP_OK) {
        ESP_LOGI(TAG, "Vacuum complete (pruned %d finished jobs)", pruned);
    }
    return rc;
}

// ─────────────────────────────────────────────────────────────────────────────
// JSON conversion
// ─────────────────────────────────────────────────────────────────────────────

cJSON* ManifestDatabase::patternToJson(const Pattern& p) {
    cJSON* json = cJSON_CreateObject();
    if (!json) return nullptr;

    cJSON_AddStringToObject(json, "uuid", p.uuid.c_str());
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

    cJSON_AddStringToObject(json, "uuid", pl.uuid.c_str());
    cJSON_AddStringToObject(json, "name", pl.name.c_str());
    cJSON_AddStringToObject(json, "description", pl.description.c_str());
    cJSON_AddStringToObject(json, "featured_pattern", pl.featured_pattern.c_str());
    cJSON_AddStringToObject(json, "date", pl.date.c_str());
    if (!pl.created_at.empty())
        cJSON_AddStringToObject(json, "created_at", pl.created_at.c_str());
    if (!pl.updated_at.empty())
        cJSON_AddStringToObject(json, "updated_at", pl.updated_at.c_str());

    cJSON* patterns = cJSON_CreateArray();
    for (const auto& uuid : pl.patterns) {
        cJSON_AddItemToArray(patterns, cJSON_CreateString(uuid.c_str()));
    }
    cJSON_AddItemToObject(json, "patterns", patterns);

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

    p.uuid = getText("uuid");
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

    pl.uuid = getText("uuid");
    pl.name = getText("name");
    pl.description = getText("description");
    pl.featured_pattern = getText("featured_pattern");
    pl.date = getText("date");
    pl.created_at = getText("created_at");
    pl.updated_at = getText("updated_at");

    const cJSON* patterns = cJSON_GetObjectItem(json, "patterns");
    if (patterns && cJSON_IsArray(patterns)) {
        const cJSON* item;
        cJSON_ArrayForEach(item, patterns) {
            if (cJSON_IsString(item)) {
                pl.patterns.push_back(item->valuestring);
            }
        }
    }

    return pl;
}
