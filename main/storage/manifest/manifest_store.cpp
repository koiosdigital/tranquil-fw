/**
 * @file manifest_store.cpp
 * @brief JSONL-backed record store implementation
 */

#include "manifest_store.h"

#include "esp_log.h"
#include "esp_heap_caps.h"

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <sys/stat.h>

static const char* TAG = "ManifestStore";

namespace manifest {

// Longest accepted snapshot line. Job rows carry base64 receipt blobs in
// job_data, so this needs headroom over typical pattern/playlist rows.
static constexpr size_t MAX_LINE = 16384;

static constexpr size_t INITIAL_CAPACITY = 32;

// ─────────────────────────────────────────────────────────────────────────────
// PSRAM-preferred allocation
// ─────────────────────────────────────────────────────────────────────────────

void* ps_malloc(size_t size) {
    void* p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) p = malloc(size);
    return p;
}

void* ps_realloc(void* ptr, size_t size) {
    void* p = heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) p = realloc(ptr, size);
    return p;
}

void ps_free(void* ptr) {
    heap_caps_free(ptr);
}

char* ps_strdup(const char* s) {
    size_t len = strlen(s) + 1;
    char* p = static_cast<char*>(ps_malloc(len));
    if (p) memcpy(p, s, len);
    return p;
}

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

JsonlTable::JsonlTable(const char* name, const char* path, uint32_t max_records)
    : name_(name), path_(path), max_records_(max_records) {}

JsonlTable::~JsonlTable() {
    for (size_t i = 0; i < count_; i++) {
        ps_free(records_[i].json);
    }
    ps_free(records_);
}

bool JsonlTable::ensureCapacity(size_t needed) {
    if (needed <= capacity_) return true;

    size_t new_cap = capacity_ ? capacity_ * 2 : INITIAL_CAPACITY;
    while (new_cap < needed) new_cap *= 2;

    Record* grown = static_cast<Record*>(ps_realloc(records_, new_cap * sizeof(Record)));
    if (!grown) {
        ESP_LOGE(TAG, "%s: failed to grow record array to %zu", name_, new_cap);
        return false;
    }
    records_ = grown;
    capacity_ = new_cap;
    return true;
}

Record* JsonlTable::find(uint32_t id) const {
    for (size_t i = 0; i < count_; i++) {
        if (records_[i].id == id) return &records_[i];
    }
    return nullptr;
}

// Serialize obj into rec (does not touch rec.id).
bool JsonlTable::storeJson(Record& rec, cJSON* obj) {
    char* line = cJSON_PrintUnformatted(obj);
    if (!line) {
        ESP_LOGE(TAG, "%s: serialization failed", name_);
        return false;
    }
    // Lines past MAX_LINE would be silently dropped at the next load —
    // surface the failure at write time instead.
    if (strlen(line) >= MAX_LINE - 1) {
        ESP_LOGE(TAG, "%s: record too large (%zu bytes)", name_, strlen(line));
        cJSON_free(line);
        return false;
    }
    char* stored = ps_strdup(line);
    cJSON_free(line);
    if (!stored) {
        ESP_LOGE(TAG, "%s: out of memory storing record", name_);
        return false;
    }

    // Cache only when the full uuid fits: a truncated cache entry would make
    // findByUuid miss records that actually match.
    rec.uuid[0] = '\0';
    const cJSON* uuid = cJSON_GetObjectItem(obj, "uuid");
    if (uuid && cJSON_IsString(uuid) && strlen(uuid->valuestring) < sizeof(rec.uuid)) {
        strcpy(rec.uuid, uuid->valuestring);
    }

    ps_free(rec.json);
    rec.json = stored;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Load / save
// ─────────────────────────────────────────────────────────────────────────────

JsonlTable::LoadResult JsonlTable::loadFile(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) {
        // Distinguish "no snapshot yet" from a sick SD card: silently starting
        // empty on an I/O error would overwrite real data at the next save.
        return (errno == ENOENT) ? LoadResult::Missing : LoadResult::Error;
    }

    char* line = static_cast<char*>(ps_malloc(MAX_LINE));
    if (!line) {
        fclose(f);
        return LoadResult::Error;
    }

    uint32_t max_id = 0;
    bool first_line = true;
    size_t skipped = 0;

    while (fgets(line, MAX_LINE, f)) {
        size_t len = strlen(line);
        // Line longer than MAX_LINE: consume and drop the remainder
        if (len == MAX_LINE - 1 && line[len - 1] != '\n') {
            int c;
            while ((c = fgetc(f)) != EOF && c != '\n') {}
            skipped++;
            continue;
        }
        while (len && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        if (len == 0) continue;

        cJSON* obj = cJSON_Parse(line);
        if (!obj) {
            skipped++;
            continue;
        }

        if (first_line) {
            first_line = false;
            const cJSON* next = cJSON_GetObjectItem(obj, "next_id");
            if (next && cJSON_IsNumber(next)) {
                next_id_ = static_cast<uint32_t>(next->valuedouble);
                cJSON_Delete(obj);
                continue;
            }
            // Not a meta line — fall through and treat as a record.
        }

        const cJSON* id_item = cJSON_GetObjectItem(obj, "id");
        uint32_t id = (id_item && cJSON_IsNumber(id_item))
            ? static_cast<uint32_t>(id_item->valuedouble) : 0;

        if (id == 0 || find(id) || count_ >= max_records_ || !ensureCapacity(count_ + 1)) {
            skipped++;
            cJSON_Delete(obj);
            continue;
        }

        Record& rec = records_[count_];
        rec.id = id;
        rec.json = nullptr;
        if (!storeJson(rec, obj)) {
            skipped++;
            cJSON_Delete(obj);
            continue;
        }
        count_++;
        if (id > max_id) max_id = id;
        cJSON_Delete(obj);
    }

    ps_free(line);
    fclose(f);

    if (next_id_ <= max_id) next_id_ = max_id + 1;
    if (skipped) {
        ESP_LOGW(TAG, "%s: skipped %zu unreadable lines in %s", name_, skipped, path);
    }
    ESP_LOGI(TAG, "%s: loaded %zu records (next_id=%lu)", name_, count_,
        (unsigned long)next_id_);
    return LoadResult::Ok;
}

bool JsonlTable::load() {
    char scratch[64];

    // A leftover .tmp is an interrupted save — never a source of truth.
    snprintf(scratch, sizeof(scratch), "%s.tmp", path_);
    unlink(scratch);

    LoadResult res = loadFile(path_);
    if (res == LoadResult::Ok) return true;
    if (res == LoadResult::Error) {
        ESP_LOGE(TAG, "%s: failed to read snapshot %s", name_, path_);
        return false;
    }

    snprintf(scratch, sizeof(scratch), "%s.bak", path_);
    res = loadFile(scratch);
    if (res == LoadResult::Ok) {
        ESP_LOGW(TAG, "%s: recovered from backup snapshot", name_);
        save();
        return true;
    }
    if (res == LoadResult::Error) {
        ESP_LOGE(TAG, "%s: failed to read backup snapshot %s", name_, scratch);
        return false;
    }

    ESP_LOGI(TAG, "%s: no snapshot found, starting empty", name_);
    return true;
}

bool JsonlTable::save() {
    char tmp[64];
    char bak[64];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path_);
    snprintf(bak, sizeof(bak), "%s.bak", path_);

    FILE* f = fopen(tmp, "w");
    if (!f) {
        ESP_LOGE(TAG, "%s: failed to open %s for writing", name_, tmp);
        return false;
    }

    bool ok = fprintf(f, "{\"next_id\":%lu}\n", (unsigned long)next_id_) > 0;
    for (size_t i = 0; ok && i < count_; i++) {
        ok = fprintf(f, "%s\n", records_[i].json) > 0;
    }

    if (ok) ok = (fflush(f) == 0);
    if (ok) fsync(fileno(f));
    if (fclose(f) != 0) ok = false;

    if (!ok) {
        ESP_LOGE(TAG, "%s: snapshot write failed", name_);
        unlink(tmp);
        return false;
    }

    // FATFS rename does not overwrite, so rotate: current -> .bak, tmp -> current.
    // Only rotate when a current snapshot exists — otherwise unlink(bak) would
    // destroy the last committed copy while the new one is still in .tmp.
    struct stat st;
    if (stat(path_, &st) == 0) {
        unlink(bak);
        rename(path_, bak);
    }
    if (rename(tmp, path_) != 0) {
        ESP_LOGE(TAG, "%s: failed to activate snapshot %s", name_, path_);
        return false;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// CRUD
// ─────────────────────────────────────────────────────────────────────────────

uint32_t JsonlTable::add(cJSON* obj) {
    if (count_ >= max_records_) {
        ESP_LOGE(TAG, "%s: table full (%lu records)", name_, (unsigned long)max_records_);
        return 0;
    }
    if (!ensureCapacity(count_ + 1)) return 0;

    uint32_t id = next_id_;
    cJSON_DeleteItemFromObject(obj, "id");
    if (!cJSON_AddNumberToObject(obj, "id", id)) {
        ESP_LOGE(TAG, "%s: failed to stamp record id", name_);
        return 0;
    }

    Record& rec = records_[count_];
    rec.id = id;
    rec.json = nullptr;
    if (!storeJson(rec, obj)) return 0;

    count_++;
    next_id_++;
    return id;
}

bool JsonlTable::update(uint32_t id, cJSON* obj) {
    Record* rec = find(id);
    if (!rec) return false;

    cJSON_DeleteItemFromObject(obj, "id");
    if (!cJSON_AddNumberToObject(obj, "id", id)) {
        ESP_LOGE(TAG, "%s: failed to stamp record id", name_);
        return false;
    }
    return storeJson(*rec, obj);
}

bool JsonlTable::remove(uint32_t id) {
    Record* rec = find(id);
    if (!rec) return false;

    ps_free(rec->json);
    size_t idx = rec - records_;
    if (idx + 1 < count_) {
        memmove(&records_[idx], &records_[idx + 1], (count_ - idx - 1) * sizeof(Record));
    }
    count_--;
    return true;
}

bool JsonlTable::exists(uint32_t id) const {
    return find(id) != nullptr;
}

cJSON* JsonlTable::get(uint32_t id) const {
    Record* rec = find(id);
    if (!rec) return nullptr;

    cJSON* obj = cJSON_Parse(rec->json);
    if (!obj) {
        ESP_LOGE(TAG, "%s: stored record %lu unparseable", name_, (unsigned long)id);
    }
    return obj;
}

uint32_t JsonlTable::findByUuid(const char* uuid) const {
    if (!uuid || !uuid[0]) return 0;
    for (size_t i = 0; i < count_; i++) {
        if (records_[i].uuid[0]) {
            if (strcmp(records_[i].uuid, uuid) == 0) return records_[i].id;
            continue;
        }
        // Uncached (uuid absent or longer than the cache slot): compare parsed
        cJSON* obj = cJSON_Parse(records_[i].json);
        if (!obj) continue;
        const cJSON* item = cJSON_GetObjectItem(obj, "uuid");
        bool match = item && cJSON_IsString(item) && strcmp(item->valuestring, uuid) == 0;
        cJSON_Delete(obj);
        if (match) return records_[i].id;
    }
    return 0;
}

void JsonlTable::foreach(bool (*fn)(uint32_t id, const cJSON* obj, void* ctx), void* ctx) const {
    for (size_t i = 0; i < count_; i++) {
        cJSON* obj = cJSON_Parse(records_[i].json);
        if (!obj) continue;
        bool keep_going = fn(records_[i].id, obj, ctx);
        cJSON_Delete(obj);
        if (!keep_going) return;
    }
}

void JsonlTable::modify(Action (*fn)(uint32_t id, cJSON** obj, void* ctx), void* ctx) {
    for (size_t i = 0; i < count_; ) {
        cJSON* obj = cJSON_Parse(records_[i].json);
        if (!obj) {
            i++;
            continue;
        }

        Action action = fn(records_[i].id, &obj, ctx);
        if (action == Action::Update && obj) {
            storeJson(records_[i], obj);
        }
        cJSON_Delete(obj);

        if (action == Action::Remove) {
            ps_free(records_[i].json);
            if (i + 1 < count_) {
                memmove(&records_[i], &records_[i + 1], (count_ - i - 1) * sizeof(Record));
            }
            count_--;
        } else {
            i++;
        }
    }
}

} // namespace manifest
