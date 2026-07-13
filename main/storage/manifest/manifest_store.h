#pragma once

/**
 * @file manifest_store.h
 * @brief Minimal JSONL-backed record store for ManifestDatabase
 *
 * Each table keeps its records in RAM as serialized cJSON lines (PSRAM
 * preferred) and mirrors them to a JSONL snapshot file on the SD card.
 * Snapshots are written atomically (tmp -> bak rotation -> rename) so a
 * power cut mid-write never leaves a half-written table behind.
 *
 * Deliberately boring: no WAL, no paging, no caches, no custom allocator
 * bookkeeping. Serialization goes through cJSON, bytes go through stdio.
 */

#include <cstdint>
#include <cstddef>
#include "cJSON.h"

namespace manifest {

// PSRAM-preferred allocation (falls back to internal heap)
void* ps_malloc(size_t size);
void* ps_realloc(void* ptr, size_t size);
void ps_free(void* ptr);
char* ps_strdup(const char* s);

// One record: id + serialized single-line JSON (owned, PSRAM) + cached
// "uuid" field so external-UUID lookups don't have to parse every record.
struct Record {
    uint32_t id;
    char uuid[40];
    char* json;
};

class JsonlTable {
public:
    // File layout: first line {"next_id":N}, then one record object per line.
    JsonlTable(const char* name, const char* path, uint32_t max_records);
    ~JsonlTable();

    JsonlTable(const JsonlTable&) = delete;
    JsonlTable& operator=(const JsonlTable&) = delete;

    // Load snapshot from disk (falls back to the .bak rotation copy).
    // A missing file is not an error (the table starts empty); an I/O or
    // allocation failure IS (returns false) so the owner can refuse to come
    // up and overwrite real data with an empty table.
    bool load();

    // Write the full snapshot atomically. Mutating methods below only touch
    // RAM; the owner calls save() once per public ManifestDatabase op.
    bool save();

    // Stamps obj's "id", serializes and stores it. Caller keeps ownership of
    // obj. Returns the assigned id, or 0 when full / out of memory.
    uint32_t add(cJSON* obj);

    // Replaces the record's JSON (obj's "id" is forced to id).
    bool update(uint32_t id, cJSON* obj);

    bool remove(uint32_t id);
    bool exists(uint32_t id) const;
    size_t count() const { return count_; }

    // Parsed copy of one record; caller owns the returned object.
    cJSON* get(uint32_t id) const;

    // Fast lookup via the cached "uuid" field. Returns 0 when absent.
    uint32_t findByUuid(const char* uuid) const;

    // Iterate parsed records in insertion order. Return false to stop.
    void foreach(bool (*fn)(uint32_t id, const cJSON* obj, void* ctx), void* ctx) const;

    // Iterate with mutation: fn decides per record. On Update the record is
    // re-serialized from *obj; fn may replace *obj with a new object (the
    // table deletes whatever *obj points to afterwards).
    enum class Action { Keep, Update, Remove };
    void modify(Action (*fn)(uint32_t id, cJSON** obj, void* ctx), void* ctx);

private:
    enum class LoadResult { Ok, Missing, Error };
    LoadResult loadFile(const char* path);
    bool ensureCapacity(size_t needed);
    Record* find(uint32_t id) const;
    bool storeJson(Record& rec, cJSON* obj);

    const char* name_;
    const char* path_;
    uint32_t max_records_;
    uint32_t next_id_ = 1;
    Record* records_ = nullptr;   // ps_malloc'd, compact
    size_t count_ = 0;
    size_t capacity_ = 0;
};

} // namespace manifest
