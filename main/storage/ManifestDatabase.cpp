#include "ManifestDatabase.h"
#include "sd.h"
#include "sqlite3.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <sys/stat.h>
#include <ctime>
#include <algorithm>
#include <unistd.h>
#include <functional>
#include <type_traits>

static const char* TAG = "ManifestDB";
static constexpr const char* DB_PATH = "/sd/manifest.db";

// SQLite task stack size - needs to be large for SQLite operations
static constexpr size_t SQLITE_TASK_STACK_SIZE = 8192;

// Command structure for the SQLite task queue
struct SqliteCommand {
    std::function<void()>* fn;
    SemaphoreHandle_t done;
};

// Pimpl implementation
class ManifestDatabase::Impl {
public:
    sqlite3* db = nullptr;
    SemaphoreHandle_t mutex = nullptr;
    bool initialized = false;

    // SQLite task handles
    TaskHandle_t sqliteTask = nullptr;
    QueueHandle_t commandQueue = nullptr;
    volatile bool taskRunning = false;

    // Execute a function on the SQLite task and wait for completion
    template<typename F>
    auto executeOnSqliteTask(F&& func) -> decltype(func()) {
        using ReturnType = decltype(func());

        if constexpr (std::is_void_v<ReturnType>) {
            std::function<void()> wrapper = [&func]() { func(); };
            SemaphoreHandle_t done = xSemaphoreCreateBinary();
            SqliteCommand cmd = { &wrapper, done };
            xQueueSend(commandQueue, &cmd, portMAX_DELAY);
            xSemaphoreTake(done, portMAX_DELAY);
            vSemaphoreDelete(done);
        } else {
            ReturnType result{};
            std::function<void()> wrapper = [&func, &result]() { result = func(); };
            SemaphoreHandle_t done = xSemaphoreCreateBinary();
            SqliteCommand cmd = { &wrapper, done };
            xQueueSend(commandQueue, &cmd, portMAX_DELAY);
            xSemaphoreTake(done, portMAX_DELAY);
            vSemaphoreDelete(done);
            return result;
        }
    }

    static void sqliteTaskFunction(void* param) {
        auto* impl = static_cast<Impl*>(param);
        SqliteCommand cmd;
        ESP_LOGI(TAG, "SQLite task started");

        while (impl->taskRunning) {
            if (xQueueReceive(impl->commandQueue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (cmd.fn) {
                    (*cmd.fn)();
                }
                if (cmd.done) {
                    xSemaphoreGive(cmd.done);
                }
            }
        }

        ESP_LOGI(TAG, "SQLite task exiting");
        vTaskDelete(nullptr);
    }

    // RAII mutex guard
    class Lock {
    public:
        explicit Lock(SemaphoreHandle_t m, TickType_t timeout = pdMS_TO_TICKS(5000))
            : mutex_(m), acquired_(false) {
            if (mutex_ && xSemaphoreTake(mutex_, timeout) == pdTRUE) {
                acquired_ = true;
            }
        }
        ~Lock() {
            if (acquired_ && mutex_) {
                xSemaphoreGive(mutex_);
            }
        }
        bool acquired() const { return acquired_; }
    private:
        SemaphoreHandle_t mutex_;
        bool acquired_;
    };

    // RAII transaction
    class Transaction {
    public:
        explicit Transaction(sqlite3* db) : db_(db), committed_(false) {
            sqlite3_exec(db_, "BEGIN IMMEDIATE", nullptr, nullptr, nullptr);
        }
        ~Transaction() {
            if (!committed_) {
                sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
            }
        }
        void commit() {
            sqlite3_exec(db_, "COMMIT", nullptr, nullptr, nullptr);
            committed_ = true;
        }
    private:
        sqlite3* db_;
        bool committed_;
    };

    // RAII statement wrapper
    class Statement {
    public:
        Statement(sqlite3* db, const char* sql) : stmt_(nullptr) {
            sqlite3_prepare_v2(db, sql, -1, &stmt_, nullptr);
        }
        ~Statement() {
            if (stmt_) sqlite3_finalize(stmt_);
        }
        operator sqlite3_stmt*() { return stmt_; }
        sqlite3_stmt* get() { return stmt_; }
        bool valid() const { return stmt_ != nullptr; }

        void bindText(int idx, const std::string& val) {
            sqlite3_bind_text(stmt_, idx, val.c_str(), -1, SQLITE_TRANSIENT);
        }
        void bindInt(int idx, int val) {
            sqlite3_bind_int(stmt_, idx, val);
        }
        void bindInt64(int idx, int64_t val) {
            sqlite3_bind_int64(stmt_, idx, val);
        }
        void bindNull(int idx) {
            sqlite3_bind_null(stmt_, idx);
        }

        int step() { return sqlite3_step(stmt_); }
        void reset() { sqlite3_reset(stmt_); sqlite3_clear_bindings(stmt_); }

        const char* columnText(int idx) {
            const char* t = reinterpret_cast<const char*>(sqlite3_column_text(stmt_, idx));
            return t ? t : "";
        }
        int columnInt(int idx) { return sqlite3_column_int(stmt_, idx); }
        int64_t columnInt64(int idx) { return sqlite3_column_int64(stmt_, idx); }

    private:
        sqlite3_stmt* stmt_;
    };

    esp_err_t createSchema() {
        const char* schema = R"(
            CREATE TABLE IF NOT EXISTS patterns (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                uuid TEXT NOT NULL UNIQUE,
                name TEXT NOT NULL,
                creator TEXT DEFAULT 'Uploaded',
                date TEXT,
                popularity INTEGER DEFAULT 0,
                reversible INTEGER DEFAULT 0,
                start_point INTEGER DEFAULT 0,
                encrypted INTEGER DEFAULT 0,
                size_bytes INTEGER DEFAULT 0,
                created_at TEXT,
                last_played_at TEXT,
                downloaded_at TEXT
            );

            CREATE INDEX IF NOT EXISTS idx_patterns_uuid ON patterns(uuid);
            CREATE INDEX IF NOT EXISTS idx_patterns_popularity ON patterns(popularity DESC);
            CREATE INDEX IF NOT EXISTS idx_patterns_last_played ON patterns(last_played_at);

            CREATE TABLE IF NOT EXISTS playlists (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                uuid TEXT NOT NULL UNIQUE,
                name TEXT NOT NULL,
                description TEXT DEFAULT '',
                featured_pattern_uuid TEXT,
                date TEXT,
                created_at TEXT,
                updated_at TEXT
            );

            CREATE INDEX IF NOT EXISTS idx_playlists_uuid ON playlists(uuid);

            CREATE TABLE IF NOT EXISTS playlist_patterns (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                playlist_id INTEGER NOT NULL,
                pattern_id INTEGER NOT NULL,
                position INTEGER NOT NULL,
                FOREIGN KEY (playlist_id) REFERENCES playlists(id) ON DELETE CASCADE,
                FOREIGN KEY (pattern_id) REFERENCES patterns(id) ON DELETE CASCADE,
                UNIQUE(playlist_id, pattern_id),
                UNIQUE(playlist_id, position)
            );

            CREATE INDEX IF NOT EXISTS idx_pp_playlist ON playlist_patterns(playlist_id, position);

            PRAGMA foreign_keys = ON;
        )";

        char* err_msg = nullptr;
        int rc = sqlite3_exec(db, schema, nullptr, nullptr, &err_msg);
        if (rc != SQLITE_OK) {
            ESP_LOGE(TAG, "Schema error: %s", err_msg);
            sqlite3_free(err_msg);
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    Pattern rowToPattern(Statement& stmt) {
        Pattern p;
        p.uuid = stmt.columnText(0);
        p.name = stmt.columnText(1);
        p.creator = stmt.columnText(2);
        p.date = stmt.columnText(3);
        p.popularity = stmt.columnInt(4);
        p.reversible = stmt.columnInt(5) != 0;
        p.start_point = stmt.columnInt(6);
        p.encrypted = stmt.columnInt(7) != 0;
        p.size_bytes = stmt.columnInt64(8);
        p.created_at = stmt.columnText(9);
        p.last_played_at = stmt.columnText(10);
        p.downloaded_at = stmt.columnText(11);
        return p;
    }

    Playlist rowToPlaylist(Statement& stmt) {
        Playlist pl;
        pl.uuid = stmt.columnText(0);
        pl.name = stmt.columnText(1);
        pl.description = stmt.columnText(2);
        pl.featured_pattern = stmt.columnText(3);
        pl.date = stmt.columnText(4);
        pl.created_at = stmt.columnText(5);
        pl.updated_at = stmt.columnText(6);
        return pl;
    }

    int getPlaylistId(const std::string& uuid) {
        Statement stmt(db, "SELECT id FROM playlists WHERE uuid = ?");
        stmt.bindText(1, uuid);
        if (stmt.step() == SQLITE_ROW) {
            return stmt.columnInt(0);
        }
        return -1;
    }

    int getPatternId(const std::string& uuid) {
        Statement stmt(db, "SELECT id FROM patterns WHERE uuid = ?");
        stmt.bindText(1, uuid);
        if (stmt.step() == SQLITE_ROW) {
            return stmt.columnInt(0);
        }
        return -1;
    }

    std::vector<std::string> getPlaylistPatternUuids(int playlistId) {
        std::vector<std::string> uuids;
        Statement stmt(db,
            "SELECT p.uuid FROM patterns p "
            "JOIN playlist_patterns pp ON p.id = pp.pattern_id "
            "WHERE pp.playlist_id = ? ORDER BY pp.position");
        stmt.bindInt(1, playlistId);
        while (stmt.step() == SQLITE_ROW) {
            uuids.push_back(stmt.columnText(0));
        }
        return uuids;
    }
};

// ManifestDatabase implementation

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

    // Initialize SD card (can run on main task - not SQLite)
    init_sd();

    // Ensure patterns directory exists
    struct stat st = {0};
    if (stat("/sd/patterns", &st) == -1) {
        if (mkdir("/sd/patterns", 0775) != 0) {
            ESP_LOGE(TAG, "Failed to create /sd/patterns");
            return ESP_FAIL;
        }
    }

    // Create mutex for serializing SQLite operations
    impl_->mutex = xSemaphoreCreateMutex();
    if (!impl_->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    // Create command queue for SQLite task
    impl_->commandQueue = xQueueCreate(4, sizeof(SqliteCommand));
    if (!impl_->commandQueue) {
        ESP_LOGE(TAG, "Failed to create command queue");
        vSemaphoreDelete(impl_->mutex);
        impl_->mutex = nullptr;
        return ESP_FAIL;
    }

    // Start SQLite task with large stack (SQLite is stack-hungry)
    impl_->taskRunning = true;
    BaseType_t ret = xTaskCreatePinnedToCore(
        Impl::sqliteTaskFunction,
        "sqlite_task",
        SQLITE_TASK_STACK_SIZE,
        impl_.get(),
        5,  // Priority
        &impl_->sqliteTask,
        1   // Core 1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SQLite task");
        impl_->taskRunning = false;
        vQueueDelete(impl_->commandQueue);
        impl_->commandQueue = nullptr;
        vSemaphoreDelete(impl_->mutex);
        impl_->mutex = nullptr;
        return ESP_FAIL;
    }

    // Run SQLite initialization on the dedicated task
    esp_err_t initResult = impl_->executeOnSqliteTask([this]() -> esp_err_t {
        // Initialize SQLite (required because SQLITE_OMIT_AUTOINIT is set)
        int rc = sqlite3_initialize();
        if (rc != SQLITE_OK) {
            ESP_LOGE(TAG, "Failed to initialize SQLite: %d", rc);
            return ESP_FAIL;
        }

        // Open database
        rc = sqlite3_open(DB_PATH, &impl_->db);
        if (rc != SQLITE_OK) {
            ESP_LOGE(TAG, "Failed to open database: %s",
                impl_->db ? sqlite3_errmsg(impl_->db) : "null db");
            return ESP_FAIL;
        }

        // Create schema
        return impl_->createSchema();
    });

    if (initResult != ESP_OK) {
        // Cleanup on failure
        impl_->taskRunning = false;
        vTaskDelay(pdMS_TO_TICKS(150));  // Let task exit
        vQueueDelete(impl_->commandQueue);
        impl_->commandQueue = nullptr;
        vSemaphoreDelete(impl_->mutex);
        impl_->mutex = nullptr;
        return initResult;
    }

    impl_->initialized = true;
    ESP_LOGI(TAG, "Initialized successfully");
    return ESP_OK;
}

void ManifestDatabase::shutdown() {
    if (!impl_->initialized) return;

    // Close database on SQLite task
    if (impl_->commandQueue && impl_->taskRunning) {
        impl_->executeOnSqliteTask([this]() {
            if (impl_->db) {
                sqlite3_close(impl_->db);
                impl_->db = nullptr;
            }
        });
    }

    // Stop the SQLite task
    impl_->taskRunning = false;
    vTaskDelay(pdMS_TO_TICKS(150));  // Let task exit cleanly

    if (impl_->commandQueue) {
        vQueueDelete(impl_->commandQueue);
        impl_->commandQueue = nullptr;
    }
    if (impl_->mutex) {
        vSemaphoreDelete(impl_->mutex);
        impl_->mutex = nullptr;
    }
    impl_->initialized = false;
    ESP_LOGI(TAG, "Shutdown complete");
}

// Pattern CRUD

esp_err_t ManifestDatabase::addPattern(const Pattern& pattern) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    return impl_->executeOnSqliteTask([this, &pattern]() -> esp_err_t {
        Impl::Statement stmt(impl_->db,
            "INSERT INTO patterns (uuid, name, creator, date, popularity, reversible, "
            "start_point, encrypted, size_bytes, created_at, last_played_at, downloaded_at) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");

        std::string ts = pattern.created_at.empty() ? currentTimestamp() : pattern.created_at;

        stmt.bindText(1, pattern.uuid);
        stmt.bindText(2, pattern.name);
        stmt.bindText(3, pattern.creator.empty() ? "Uploaded" : pattern.creator);
        stmt.bindText(4, pattern.date);
        stmt.bindInt(5, pattern.popularity);
        stmt.bindInt(6, pattern.reversible ? 1 : 0);
        stmt.bindInt(7, pattern.start_point);
        stmt.bindInt(8, pattern.encrypted ? 1 : 0);
        stmt.bindInt64(9, pattern.size_bytes);
        stmt.bindText(10, ts);
        if (pattern.last_played_at.empty()) stmt.bindNull(11);
        else stmt.bindText(11, pattern.last_played_at);
        if (pattern.downloaded_at.empty()) stmt.bindNull(12);
        else stmt.bindText(12, pattern.downloaded_at);

        if (stmt.step() != SQLITE_DONE) {
            ESP_LOGE(TAG, "Failed to add pattern: %s", sqlite3_errmsg(impl_->db));
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Pattern added: %s", pattern.uuid.c_str());
        return ESP_OK;
    });
}

esp_err_t ManifestDatabase::updatePattern(const std::string& uuid, const Pattern& pattern) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    return impl_->executeOnSqliteTask([this, &uuid, &pattern]() -> esp_err_t {
        Impl::Statement stmt(impl_->db,
            "UPDATE patterns SET name=?, creator=?, date=?, popularity=?, reversible=?, "
            "start_point=?, encrypted=?, size_bytes=? WHERE uuid=?");

        stmt.bindText(1, pattern.name);
        stmt.bindText(2, pattern.creator);
        stmt.bindText(3, pattern.date);
        stmt.bindInt(4, pattern.popularity);
        stmt.bindInt(5, pattern.reversible ? 1 : 0);
        stmt.bindInt(6, pattern.start_point);
        stmt.bindInt(7, pattern.encrypted ? 1 : 0);
        stmt.bindInt64(8, pattern.size_bytes);
        stmt.bindText(9, uuid);

        if (stmt.step() != SQLITE_DONE) {
            ESP_LOGE(TAG, "Failed to update pattern: %s", sqlite3_errmsg(impl_->db));
            return ESP_FAIL;
        }

        if (sqlite3_changes(impl_->db) == 0) {
            return ESP_ERR_NOT_FOUND;
        }

        return ESP_OK;
    });
}

esp_err_t ManifestDatabase::deletePattern(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    esp_err_t result = impl_->executeOnSqliteTask([this, &uuid]() -> esp_err_t {
        Impl::Transaction txn(impl_->db);

        // Get pattern ID for junction cleanup
        int patternId = impl_->getPatternId(uuid);
        if (patternId < 0) {
            return ESP_ERR_NOT_FOUND;
        }

        // Delete from junction table (CASCADE should handle this, but be explicit)
        Impl::Statement delJunction(impl_->db,
            "DELETE FROM playlist_patterns WHERE pattern_id = ?");
        delJunction.bindInt(1, patternId);
        delJunction.step();

        // Delete pattern
        Impl::Statement delPattern(impl_->db, "DELETE FROM patterns WHERE uuid = ?");
        delPattern.bindText(1, uuid);
        if (delPattern.step() != SQLITE_DONE) {
            ESP_LOGE(TAG, "Failed to delete pattern: %s", sqlite3_errmsg(impl_->db));
            return ESP_FAIL;
        }

        txn.commit();
        return ESP_OK;
    });

    if (result == ESP_OK) {
        // Delete pattern file (can run outside SQLite task)
        char path[128];
        snprintf(path, sizeof(path), "/sd/patterns/%s.thr", uuid.c_str());
        unlink(path);
        ESP_LOGI(TAG, "Pattern deleted: %s", uuid.c_str());
    }
    return result;
}

std::vector<Pattern> ManifestDatabase::getAllPatterns() {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return {};

    return impl_->executeOnSqliteTask([this]() -> std::vector<Pattern> {
        std::vector<Pattern> patterns;
        Impl::Statement stmt(impl_->db,
            "SELECT uuid, name, creator, date, popularity, reversible, start_point, "
            "encrypted, size_bytes, created_at, last_played_at, downloaded_at "
            "FROM patterns ORDER BY name");

        while (stmt.step() == SQLITE_ROW) {
            patterns.push_back(impl_->rowToPattern(stmt));
        }
        return patterns;
    });
}

PaginatedResult<Pattern> ManifestDatabase::getPatterns(int page, int per_page) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return {};

    return impl_->executeOnSqliteTask([this, page, per_page]() -> PaginatedResult<Pattern> {
        PaginatedResult<Pattern> result;

        // Get total count
        Impl::Statement countStmt(impl_->db, "SELECT COUNT(*) FROM patterns");
        if (countStmt.step() == SQLITE_ROW) {
            result.pagination.total_items = countStmt.columnInt(0);
        }

        result.pagination.page = page;
        result.pagination.per_page = per_page;
        result.pagination.total_pages = (result.pagination.total_items + per_page - 1) / per_page;

        // Get page of patterns
        Impl::Statement stmt(impl_->db,
            "SELECT uuid, name, creator, date, popularity, reversible, start_point, "
            "encrypted, size_bytes, created_at, last_played_at, downloaded_at "
            "FROM patterns ORDER BY name LIMIT ? OFFSET ?");
        stmt.bindInt(1, per_page);
        stmt.bindInt(2, page * per_page);

        while (stmt.step() == SQLITE_ROW) {
            result.items.push_back(impl_->rowToPattern(stmt));
        }
        return result;
    });
}

std::optional<Pattern> ManifestDatabase::getPattern(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return std::nullopt;

    return impl_->executeOnSqliteTask([this, &uuid]() -> std::optional<Pattern> {
        Impl::Statement stmt(impl_->db,
            "SELECT uuid, name, creator, date, popularity, reversible, start_point, "
            "encrypted, size_bytes, created_at, last_played_at, downloaded_at "
            "FROM patterns WHERE uuid = ?");
        stmt.bindText(1, uuid);

        if (stmt.step() == SQLITE_ROW) {
            return impl_->rowToPattern(stmt);
        }
        return std::nullopt;
    });
}

bool ManifestDatabase::patternExists(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return false;

    return impl_->executeOnSqliteTask([this, &uuid]() -> bool {
        Impl::Statement stmt(impl_->db, "SELECT 1 FROM patterns WHERE uuid = ? LIMIT 1");
        stmt.bindText(1, uuid);
        return stmt.step() == SQLITE_ROW;
    });
}

size_t ManifestDatabase::getPatternCount() {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return 0;

    return impl_->executeOnSqliteTask([this]() -> size_t {
        Impl::Statement stmt(impl_->db, "SELECT COUNT(*) FROM patterns");
        if (stmt.step() == SQLITE_ROW) {
            return stmt.columnInt(0);
        }
        return 0;
    });
}

esp_err_t ManifestDatabase::updateLastPlayed(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    return impl_->executeOnSqliteTask([this, &uuid]() -> esp_err_t {
        Impl::Statement stmt(impl_->db, "UPDATE patterns SET last_played_at = ? WHERE uuid = ?");
        stmt.bindText(1, currentTimestamp());
        stmt.bindText(2, uuid);

        if (stmt.step() != SQLITE_DONE || sqlite3_changes(impl_->db) == 0) {
            return ESP_ERR_NOT_FOUND;
        }
        return ESP_OK;
    });
}

esp_err_t ManifestDatabase::incrementPopularity(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    return impl_->executeOnSqliteTask([this, &uuid]() -> esp_err_t {
        Impl::Statement stmt(impl_->db, "UPDATE patterns SET popularity = popularity + 1 WHERE uuid = ?");
        stmt.bindText(1, uuid);

        if (stmt.step() != SQLITE_DONE || sqlite3_changes(impl_->db) == 0) {
            return ESP_ERR_NOT_FOUND;
        }
        return ESP_OK;
    });
}

// Playlist CRUD

esp_err_t ManifestDatabase::addPlaylist(const Playlist& playlist) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    return impl_->executeOnSqliteTask([this, &playlist]() -> esp_err_t {
        Impl::Transaction txn(impl_->db);

        std::string ts = playlist.created_at.empty() ? currentTimestamp() : playlist.created_at;

        Impl::Statement stmt(impl_->db,
            "INSERT INTO playlists (uuid, name, description, featured_pattern_uuid, date, created_at, updated_at) "
            "VALUES (?, ?, ?, ?, ?, ?, ?)");

        stmt.bindText(1, playlist.uuid);
        stmt.bindText(2, playlist.name);
        stmt.bindText(3, playlist.description);
        stmt.bindText(4, playlist.featured_pattern);
        stmt.bindText(5, playlist.date);
        stmt.bindText(6, ts);
        stmt.bindText(7, ts);

        if (stmt.step() != SQLITE_DONE) {
            ESP_LOGE(TAG, "Failed to add playlist: %s", sqlite3_errmsg(impl_->db));
            return ESP_FAIL;
        }

        int playlistId = sqlite3_last_insert_rowid(impl_->db);

        // Add patterns to junction table
        int position = 0;
        for (const auto& patternUuid : playlist.patterns) {
            int patternId = impl_->getPatternId(patternUuid);
            if (patternId < 0) continue;

            Impl::Statement ppStmt(impl_->db,
                "INSERT INTO playlist_patterns (playlist_id, pattern_id, position) VALUES (?, ?, ?)");
            ppStmt.bindInt(1, playlistId);
            ppStmt.bindInt(2, patternId);
            ppStmt.bindInt(3, position++);
            ppStmt.step();
        }

        txn.commit();
        ESP_LOGI(TAG, "Playlist added: %s", playlist.uuid.c_str());
        return ESP_OK;
    });
}

esp_err_t ManifestDatabase::updatePlaylist(const std::string& uuid, const Playlist& playlist) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    return impl_->executeOnSqliteTask([this, &uuid, &playlist]() -> esp_err_t {
        Impl::Transaction txn(impl_->db);

        int playlistId = impl_->getPlaylistId(uuid);
        if (playlistId < 0) return ESP_ERR_NOT_FOUND;

        Impl::Statement stmt(impl_->db,
            "UPDATE playlists SET name=?, description=?, featured_pattern_uuid=?, date=?, updated_at=? "
            "WHERE uuid=?");

        stmt.bindText(1, playlist.name);
        stmt.bindText(2, playlist.description);
        stmt.bindText(3, playlist.featured_pattern);
        stmt.bindText(4, playlist.date);
        stmt.bindText(5, currentTimestamp());
        stmt.bindText(6, uuid);

        if (stmt.step() != SQLITE_DONE) {
            ESP_LOGE(TAG, "Failed to update playlist: %s", sqlite3_errmsg(impl_->db));
            return ESP_FAIL;
        }

        // Update patterns: clear and re-add
        Impl::Statement delPp(impl_->db, "DELETE FROM playlist_patterns WHERE playlist_id = ?");
        delPp.bindInt(1, playlistId);
        delPp.step();

        int position = 0;
        for (const auto& patternUuid : playlist.patterns) {
            int patternId = impl_->getPatternId(patternUuid);
            if (patternId < 0) continue;

            Impl::Statement ppStmt(impl_->db,
                "INSERT INTO playlist_patterns (playlist_id, pattern_id, position) VALUES (?, ?, ?)");
            ppStmt.bindInt(1, playlistId);
            ppStmt.bindInt(2, patternId);
            ppStmt.bindInt(3, position++);
            ppStmt.step();
        }

        txn.commit();
        return ESP_OK;
    });
}

esp_err_t ManifestDatabase::deletePlaylist(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    return impl_->executeOnSqliteTask([this, &uuid]() -> esp_err_t {
        Impl::Statement stmt(impl_->db, "DELETE FROM playlists WHERE uuid = ?");
        stmt.bindText(1, uuid);

        if (stmt.step() != SQLITE_DONE) {
            ESP_LOGE(TAG, "Failed to delete playlist: %s", sqlite3_errmsg(impl_->db));
            return ESP_FAIL;
        }

        if (sqlite3_changes(impl_->db) == 0) {
            return ESP_ERR_NOT_FOUND;
        }

        ESP_LOGI(TAG, "Playlist deleted: %s", uuid.c_str());
        return ESP_OK;
    });
}

std::vector<Playlist> ManifestDatabase::getAllPlaylists() {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return {};

    return impl_->executeOnSqliteTask([this]() -> std::vector<Playlist> {
        std::vector<Playlist> playlists;
        Impl::Statement stmt(impl_->db,
            "SELECT uuid, name, description, featured_pattern_uuid, date, created_at, updated_at "
            "FROM playlists ORDER BY name");

        while (stmt.step() == SQLITE_ROW) {
            Playlist pl = impl_->rowToPlaylist(stmt);
            int plId = impl_->getPlaylistId(pl.uuid);
            pl.patterns = impl_->getPlaylistPatternUuids(plId);
            playlists.push_back(std::move(pl));
        }
        return playlists;
    });
}

PaginatedResult<Playlist> ManifestDatabase::getPlaylists(int page, int per_page) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return {};

    return impl_->executeOnSqliteTask([this, page, per_page]() -> PaginatedResult<Playlist> {
        PaginatedResult<Playlist> result;

        // Get total count
        Impl::Statement countStmt(impl_->db, "SELECT COUNT(*) FROM playlists");
        if (countStmt.step() == SQLITE_ROW) {
            result.pagination.total_items = countStmt.columnInt(0);
        }

        result.pagination.page = page;
        result.pagination.per_page = per_page;
        result.pagination.total_pages = (result.pagination.total_items + per_page - 1) / per_page;

        // Get page of playlists
        Impl::Statement stmt(impl_->db,
            "SELECT uuid, name, description, featured_pattern_uuid, date, created_at, updated_at "
            "FROM playlists ORDER BY name LIMIT ? OFFSET ?");
        stmt.bindInt(1, per_page);
        stmt.bindInt(2, page * per_page);

        while (stmt.step() == SQLITE_ROW) {
            Playlist pl = impl_->rowToPlaylist(stmt);
            int plId = impl_->getPlaylistId(pl.uuid);
            pl.patterns = impl_->getPlaylistPatternUuids(plId);
            result.items.push_back(std::move(pl));
        }
        return result;
    });
}

std::optional<Playlist> ManifestDatabase::getPlaylist(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return std::nullopt;

    return impl_->executeOnSqliteTask([this, &uuid]() -> std::optional<Playlist> {
        Impl::Statement stmt(impl_->db,
            "SELECT uuid, name, description, featured_pattern_uuid, date, created_at, updated_at "
            "FROM playlists WHERE uuid = ?");
        stmt.bindText(1, uuid);

        if (stmt.step() == SQLITE_ROW) {
            Playlist pl = impl_->rowToPlaylist(stmt);
            int plId = impl_->getPlaylistId(pl.uuid);
            pl.patterns = impl_->getPlaylistPatternUuids(plId);
            return pl;
        }
        return std::nullopt;
    });
}

bool ManifestDatabase::playlistExists(const std::string& uuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return false;

    return impl_->executeOnSqliteTask([this, &uuid]() -> bool {
        Impl::Statement stmt(impl_->db, "SELECT 1 FROM playlists WHERE uuid = ? LIMIT 1");
        stmt.bindText(1, uuid);
        return stmt.step() == SQLITE_ROW;
    });
}

size_t ManifestDatabase::getPlaylistCount() {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return 0;

    return impl_->executeOnSqliteTask([this]() -> size_t {
        Impl::Statement stmt(impl_->db, "SELECT COUNT(*) FROM playlists");
        if (stmt.step() == SQLITE_ROW) {
            return stmt.columnInt(0);
        }
        return 0;
    });
}

// Playlist-pattern operations

esp_err_t ManifestDatabase::addPatternToPlaylist(const std::string& playlistUuid, const std::string& patternUuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    return impl_->executeOnSqliteTask([this, &playlistUuid, &patternUuid]() -> esp_err_t {
        int playlistId = impl_->getPlaylistId(playlistUuid);
        if (playlistId < 0) return ESP_ERR_NOT_FOUND;

        int patternId = impl_->getPatternId(patternUuid);
        if (patternId < 0) return ESP_ERR_NOT_FOUND;

        // Get max position
        Impl::Statement maxStmt(impl_->db,
            "SELECT COALESCE(MAX(position), -1) + 1 FROM playlist_patterns WHERE playlist_id = ?");
        maxStmt.bindInt(1, playlistId);
        int position = 0;
        if (maxStmt.step() == SQLITE_ROW) {
            position = maxStmt.columnInt(0);
        }

        Impl::Statement stmt(impl_->db,
            "INSERT OR IGNORE INTO playlist_patterns (playlist_id, pattern_id, position) VALUES (?, ?, ?)");
        stmt.bindInt(1, playlistId);
        stmt.bindInt(2, patternId);
        stmt.bindInt(3, position);

        if (stmt.step() != SQLITE_DONE) {
            return ESP_FAIL;
        }

        // Update playlist updated_at
        Impl::Statement updateStmt(impl_->db, "UPDATE playlists SET updated_at = ? WHERE id = ?");
        updateStmt.bindText(1, currentTimestamp());
        updateStmt.bindInt(2, playlistId);
        updateStmt.step();

        return ESP_OK;
    });
}

esp_err_t ManifestDatabase::removePatternFromPlaylist(const std::string& playlistUuid, const std::string& patternUuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    return impl_->executeOnSqliteTask([this, &playlistUuid, &patternUuid]() -> esp_err_t {
        int playlistId = impl_->getPlaylistId(playlistUuid);
        if (playlistId < 0) return ESP_ERR_NOT_FOUND;

        int patternId = impl_->getPatternId(patternUuid);
        if (patternId < 0) return ESP_ERR_NOT_FOUND;

        Impl::Statement stmt(impl_->db,
            "DELETE FROM playlist_patterns WHERE playlist_id = ? AND pattern_id = ?");
        stmt.bindInt(1, playlistId);
        stmt.bindInt(2, patternId);

        if (stmt.step() != SQLITE_DONE || sqlite3_changes(impl_->db) == 0) {
            return ESP_ERR_NOT_FOUND;
        }

        // Clear featured if it was this pattern
        Impl::Statement clearFeatured(impl_->db,
            "UPDATE playlists SET featured_pattern_uuid = '', updated_at = ? "
            "WHERE id = ? AND featured_pattern_uuid = ?");
        clearFeatured.bindText(1, currentTimestamp());
        clearFeatured.bindInt(2, playlistId);
        clearFeatured.bindText(3, patternUuid);
        clearFeatured.step();

        return ESP_OK;
    });
}

esp_err_t ManifestDatabase::setFeaturedPattern(const std::string& playlistUuid, const std::string& patternUuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    return impl_->executeOnSqliteTask([this, &playlistUuid, &patternUuid]() -> esp_err_t {
        // Verify pattern is in playlist
        int playlistId = impl_->getPlaylistId(playlistUuid);
        if (playlistId < 0) return ESP_ERR_NOT_FOUND;

        int patternId = impl_->getPatternId(patternUuid);
        if (patternId < 0) return ESP_ERR_NOT_FOUND;

        Impl::Statement checkStmt(impl_->db,
            "SELECT 1 FROM playlist_patterns WHERE playlist_id = ? AND pattern_id = ?");
        checkStmt.bindInt(1, playlistId);
        checkStmt.bindInt(2, patternId);
        if (checkStmt.step() != SQLITE_ROW) {
            return ESP_ERR_NOT_FOUND;  // Pattern not in playlist
        }

        Impl::Statement stmt(impl_->db,
            "UPDATE playlists SET featured_pattern_uuid = ?, updated_at = ? WHERE id = ?");
        stmt.bindText(1, patternUuid);
        stmt.bindText(2, currentTimestamp());
        stmt.bindInt(3, playlistId);

        if (stmt.step() != SQLITE_DONE) {
            return ESP_FAIL;
        }
        return ESP_OK;
    });
}

esp_err_t ManifestDatabase::reorderPlaylist(const std::string& playlistUuid, const std::vector<std::string>& newOrder) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return ESP_ERR_TIMEOUT;

    return impl_->executeOnSqliteTask([this, &playlistUuid, &newOrder]() -> esp_err_t {
        int playlistId = impl_->getPlaylistId(playlistUuid);
        if (playlistId < 0) return ESP_ERR_NOT_FOUND;

        auto currentOrder = impl_->getPlaylistPatternUuids(playlistId);

        // Validate: same patterns, no duplicates
        if (newOrder.size() != currentOrder.size()) return ESP_ERR_INVALID_ARG;

        auto sortedCurrent = currentOrder;
        auto sortedNew = newOrder;
        std::sort(sortedCurrent.begin(), sortedCurrent.end());
        std::sort(sortedNew.begin(), sortedNew.end());
        if (sortedCurrent != sortedNew) return ESP_ERR_INVALID_ARG;

        Impl::Transaction txn(impl_->db);

        // Delete existing positions
        Impl::Statement delStmt(impl_->db, "DELETE FROM playlist_patterns WHERE playlist_id = ?");
        delStmt.bindInt(1, playlistId);
        delStmt.step();

        // Re-insert with new order
        int position = 0;
        for (const auto& patternUuid : newOrder) {
            int patternId = impl_->getPatternId(patternUuid);
            if (patternId < 0) continue;

            Impl::Statement insStmt(impl_->db,
                "INSERT INTO playlist_patterns (playlist_id, pattern_id, position) VALUES (?, ?, ?)");
            insStmt.bindInt(1, playlistId);
            insStmt.bindInt(2, patternId);
            insStmt.bindInt(3, position++);
            insStmt.step();
        }

        // Update timestamp
        Impl::Statement updateStmt(impl_->db, "UPDATE playlists SET updated_at = ? WHERE id = ?");
        updateStmt.bindText(1, currentTimestamp());
        updateStmt.bindInt(2, playlistId);
        updateStmt.step();

        txn.commit();
        return ESP_OK;
    });
}

std::vector<Pattern> ManifestDatabase::getPlaylistPatterns(const std::string& playlistUuid) {
    Impl::Lock lock(impl_->mutex);
    if (!lock.acquired()) return {};

    return impl_->executeOnSqliteTask([this, &playlistUuid]() -> std::vector<Pattern> {
        std::vector<Pattern> patterns;

        int playlistId = impl_->getPlaylistId(playlistUuid);
        if (playlistId < 0) return patterns;

        Impl::Statement stmt(impl_->db,
            "SELECT p.uuid, p.name, p.creator, p.date, p.popularity, p.reversible, p.start_point, "
            "p.encrypted, p.size_bytes, p.created_at, p.last_played_at, p.downloaded_at "
            "FROM patterns p "
            "JOIN playlist_patterns pp ON p.id = pp.pattern_id "
            "WHERE pp.playlist_id = ? ORDER BY pp.position");
        stmt.bindInt(1, playlistId);

        while (stmt.step() == SQLITE_ROW) {
            patterns.push_back(impl_->rowToPattern(stmt));
        }
        return patterns;
    });
}

// Utility functions

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
