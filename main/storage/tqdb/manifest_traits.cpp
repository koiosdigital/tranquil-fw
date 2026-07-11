/**
 * @file manifest_traits.cpp
 * @brief TQDB traits and platform wrappers for ManifestDatabase
 */

#include "manifest_internal.h"
#include "../jobs/job_types.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstdlib>
#include <new>

// ─────────────────────────────────────────────────────────────────────────────
// Heap scratch for trait read functions
//
// TQDB_MAX_STRING_LEN is 4KB — a stack buffer that size overflows the small
// tasks these readers run on (app_main is 3584 bytes, job workers 4KB var).
// tqdb_read_str preserves stream framing when the buffer is too small, so
// the tiny OOM fallback truncates string fields instead of derailing the
// read or crashing.
// ─────────────────────────────────────────────────────────────────────────────

namespace {
struct StringScratch {
    char* buf;
    size_t size;
    char fallback[64];

    StringScratch() {
        buf = static_cast<char*>(
            heap_caps_malloc(TQDB_MAX_STRING_LEN, MALLOC_CAP_SPIRAM));
        if (!buf) {
            buf = static_cast<char*>(malloc(TQDB_MAX_STRING_LEN));
        }
        if (buf) {
            size = TQDB_MAX_STRING_LEN;
        } else {
            buf = fallback;
            size = sizeof(fallback);
        }
    }
    ~StringScratch() {
        if (buf != fallback) free(buf);
    }
    StringScratch(const StringScratch&) = delete;
    StringScratch& operator=(const StringScratch&) = delete;
};
} // namespace

// ─────────────────────────────────────────────────────────────────────────────
// FreeRTOS Mutex Wrapper for TQDB
// ─────────────────────────────────────────────────────────────────────────────

static void* freertos_mutex_create() {
    return xSemaphoreCreateRecursiveMutex();
}

static void freertos_mutex_destroy(void* mutex) {
    if (mutex) vSemaphoreDelete(static_cast<SemaphoreHandle_t>(mutex));
}

static bool freertos_mutex_lock(void* mutex, uint32_t timeout_ms) {
    if (!mutex) return false;
    return xSemaphoreTakeRecursive(static_cast<SemaphoreHandle_t>(mutex),
        pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

static void freertos_mutex_unlock(void* mutex) {
    if (mutex) xSemaphoreGiveRecursive(static_cast<SemaphoreHandle_t>(mutex));
}

tqdb_mutex_ops_t g_freertos_mutex_ops = {
    .create = freertos_mutex_create,
    .destroy = freertos_mutex_destroy,
    .lock = freertos_mutex_lock,
    .unlock = freertos_mutex_unlock
};

// ─────────────────────────────────────────────────────────────────────────────
// SPIRAM-preferred allocator for TQDB
// ─────────────────────────────────────────────────────────────────────────────

static void* spiram_malloc(size_t size) {
    void* ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ptr) ptr = malloc(size);
    return ptr;
}

static void spiram_free(void* ptr) {
    heap_caps_free(ptr);
}

static void* spiram_realloc(void* ptr, size_t size) {
    void* new_ptr = heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!new_ptr) new_ptr = realloc(ptr, size);
    return new_ptr;
}

tqdb_alloc_t g_spiram_alloc = {
    .malloc = spiram_malloc,
    .free = spiram_free,
    .realloc = spiram_realloc
};

// ─────────────────────────────────────────────────────────────────────────────
// Pattern Trait for TQDB
// ─────────────────────────────────────────────────────────────────────────────

static void pattern_init(void* entity) {
    new (entity) Pattern();  // Placement new
}

static void pattern_destroy(void* entity) {
    static_cast<Pattern*>(entity)->~Pattern();  // Explicit destructor
}

static void pattern_copy(void* dst, const void* src) {
    new (dst) Pattern(*static_cast<const Pattern*>(src));
}

static uint32_t pattern_get_id(const void* entity) {
    return static_cast<const Pattern*>(entity)->id;
}

static void pattern_set_id(void* entity, uint32_t id) {
    static_cast<Pattern*>(entity)->id = id;
}

static void pattern_write(tqdb_writer_t* w, const void* entity) {
    const Pattern* p = static_cast<const Pattern*>(entity);
    tqdb_write_u32(w, p->id);
    tqdb_write_str(w, p->external_uuid.c_str());
    tqdb_write_str(w, p->name.c_str());
    tqdb_write_str(w, p->creator.c_str());
    tqdb_write_str(w, p->date.c_str());
    tqdb_write_i32(w, p->popularity);
    uint8_t flags = (p->reversible ? 0x01 : 0)
        | (p->encrypted ? 0x02 : 0)
        | (p->purchased ? 0x04 : 0);
    tqdb_write_u8(w, flags);
    tqdb_write_i32(w, p->start_point);
    tqdb_write_i64(w, static_cast<int64_t>(p->size_bytes));
    tqdb_write_str(w, p->created_at.c_str());
    tqdb_write_str(w, p->last_played_at.c_str());
    tqdb_write_str(w, p->downloaded_at.c_str());
    tqdb_write_i64(w, p->purchased_at);
    tqdb_write_str(w, p->receipt_id.c_str());
}

static void pattern_read(tqdb_reader_t* r, void* entity) {
    Pattern* p = static_cast<Pattern*>(entity);
    StringScratch scratch;
    char* buf = scratch.buf;

    p->id = tqdb_read_u32(r);
    tqdb_read_str(r, buf, scratch.size); p->external_uuid = buf;
    tqdb_read_str(r, buf, scratch.size); p->name = buf;
    tqdb_read_str(r, buf, scratch.size); p->creator = buf;
    tqdb_read_str(r, buf, scratch.size); p->date = buf;
    p->popularity = tqdb_read_i32(r);
    uint8_t flags = tqdb_read_u8(r);
    p->reversible = flags & 0x01;
    p->encrypted = flags & 0x02;
    p->purchased = flags & 0x04;
    p->start_point = tqdb_read_i32(r);
    p->size_bytes = static_cast<size_t>(tqdb_read_i64(r));
    tqdb_read_str(r, buf, scratch.size); p->created_at = buf;
    tqdb_read_str(r, buf, scratch.size); p->last_played_at = buf;
    tqdb_read_str(r, buf, scratch.size); p->downloaded_at = buf;
    p->purchased_at = tqdb_read_i64(r);
    tqdb_read_str(r, buf, scratch.size); p->receipt_id = buf;
}

static void pattern_skip(tqdb_reader_t* r) {
    tqdb_read_u32(r);       // id
    tqdb_read_skip_str(r);  // external_uuid
    tqdb_read_skip_str(r);  // name
    tqdb_read_skip_str(r);  // creator
    tqdb_read_skip_str(r);  // date
    tqdb_read_i32(r);       // popularity
    tqdb_read_u8(r);        // flags
    tqdb_read_i32(r);       // start_point
    tqdb_read_i64(r);       // size_bytes
    tqdb_read_skip_str(r);  // created_at
    tqdb_read_skip_str(r);  // last_played_at
    tqdb_read_skip_str(r);  // downloaded_at
    tqdb_read_i64(r);       // purchased_at
    tqdb_read_skip_str(r);  // receipt_id
}

const tqdb_trait_t g_pattern_trait = {
    .name = "Pattern",
    .max_count = MAX_PATTERNS,
    .struct_size = sizeof(Pattern),
    .write = pattern_write,
    .read = pattern_read,
    .get_id = pattern_get_id,
    .set_id = pattern_set_id,
    .init = pattern_init,
    .destroy = pattern_destroy,
    .skip = pattern_skip,
    .copy = pattern_copy
};

// ─────────────────────────────────────────────────────────────────────────────
// Playlist Trait for TQDB
// ─────────────────────────────────────────────────────────────────────────────

static void playlist_init(void* entity) {
    new (entity) Playlist();
}

static void playlist_destroy(void* entity) {
    static_cast<Playlist*>(entity)->~Playlist();
}

static void playlist_copy(void* dst, const void* src) {
    new (dst) Playlist(*static_cast<const Playlist*>(src));
}

static uint32_t playlist_get_id(const void* entity) {
    return static_cast<const Playlist*>(entity)->id;
}

static void playlist_set_id(void* entity, uint32_t id) {
    static_cast<Playlist*>(entity)->id = id;
}

static void playlist_write(tqdb_writer_t* w, const void* entity) {
    const Playlist* pl = static_cast<const Playlist*>(entity);
    tqdb_write_u32(w, pl->id);
    tqdb_write_str(w, pl->external_uuid.c_str());
    tqdb_write_str(w, pl->name.c_str());
    tqdb_write_str(w, pl->description.c_str());
    tqdb_write_u32(w, pl->featured_pattern_id);
    tqdb_write_str(w, pl->date.c_str());
    tqdb_write_str(w, pl->created_at.c_str());
    tqdb_write_str(w, pl->updated_at.c_str());

    // Write pattern IDs array
    tqdb_write_u32(w, static_cast<uint32_t>(pl->pattern_ids.size()));
    for (uint32_t pid : pl->pattern_ids) {
        tqdb_write_u32(w, pid);
    }
}

static void playlist_read(tqdb_reader_t* r, void* entity) {
    Playlist* pl = static_cast<Playlist*>(entity);
    StringScratch scratch;
    char* buf = scratch.buf;

    pl->id = tqdb_read_u32(r);
    tqdb_read_str(r, buf, scratch.size); pl->external_uuid = buf;
    tqdb_read_str(r, buf, scratch.size); pl->name = buf;
    tqdb_read_str(r, buf, scratch.size); pl->description = buf;
    pl->featured_pattern_id = tqdb_read_u32(r);
    tqdb_read_str(r, buf, scratch.size); pl->date = buf;
    tqdb_read_str(r, buf, scratch.size); pl->created_at = buf;
    tqdb_read_str(r, buf, scratch.size); pl->updated_at = buf;

    // Read pattern IDs array
    uint32_t count = tqdb_read_u32(r);
    pl->pattern_ids.clear();
    if (count <= MAX_PATTERNS) {
        pl->pattern_ids.reserve(count);
        for (uint32_t i = 0; i < count; i++) {
            pl->pattern_ids.push_back(tqdb_read_u32(r));
        }
    }
}

static void playlist_skip(tqdb_reader_t* r) {
    tqdb_read_u32(r);       // id
    tqdb_read_skip_str(r);  // external_uuid
    tqdb_read_skip_str(r);  // name
    tqdb_read_skip_str(r);  // description
    tqdb_read_u32(r);       // featured_pattern_id
    tqdb_read_skip_str(r);  // date
    tqdb_read_skip_str(r);  // created_at
    tqdb_read_skip_str(r);  // updated_at
    uint32_t count = tqdb_read_u32(r);
    for (uint32_t i = 0; i < count && i < MAX_PATTERNS; i++) {
        tqdb_read_u32(r);
    }
}

const tqdb_trait_t g_playlist_trait = {
    .name = "Playlist",
    .max_count = MAX_PLAYLISTS,
    .struct_size = sizeof(Playlist),
    .write = playlist_write,
    .read = playlist_read,
    .get_id = playlist_get_id,
    .set_id = playlist_set_id,
    .init = playlist_init,
    .destroy = playlist_destroy,
    .skip = playlist_skip,
    .copy = playlist_copy
};

// ─────────────────────────────────────────────────────────────────────────────
// Job Trait for TQDB
// ─────────────────────────────────────────────────────────────────────────────

static void job_init(void* entity) {
    new (entity) jobs::Job();
}

static void job_destroy(void* entity) {
    static_cast<jobs::Job*>(entity)->~Job();
}

static void job_copy(void* dst, const void* src) {
    new (dst) jobs::Job(*static_cast<const jobs::Job*>(src));
}

static uint32_t job_get_id(const void* entity) {
    return static_cast<const jobs::Job*>(entity)->id;
}

static void job_set_id(void* entity, uint32_t id) {
    static_cast<jobs::Job*>(entity)->id = id;
}

static void job_write(tqdb_writer_t* w, const void* entity) {
    const jobs::Job* j = static_cast<const jobs::Job*>(entity);
    tqdb_write_u32(w, j->id);
    tqdb_write_str(w, jobs::jobTypeToString(j->type));
    tqdb_write_u32(w, j->pattern_id);
    tqdb_write_str(w, j->pattern_external_uuid.c_str());
    tqdb_write_str(w, jobs::jobStatusToString(j->status));
    tqdb_write_i32(w, j->priority);
    tqdb_write_i32(w, j->retry_count);
    tqdb_write_i32(w, j->max_retries);
    tqdb_write_str(w, j->created_at.c_str());
    tqdb_write_str(w, j->started_at.c_str());
    tqdb_write_str(w, j->completed_at.c_str());
    tqdb_write_str(w, j->error_message.c_str());
    tqdb_write_str(w, j->job_data.c_str());
}

static void job_read(tqdb_reader_t* r, void* entity) {
    jobs::Job* j = static_cast<jobs::Job*>(entity);
    StringScratch scratch;
    char* buf = scratch.buf;

    j->id = tqdb_read_u32(r);
    tqdb_read_str(r, buf, scratch.size); j->type = jobs::jobTypeFromString(buf);
    j->pattern_id = tqdb_read_u32(r);
    tqdb_read_str(r, buf, scratch.size); j->pattern_external_uuid = buf;
    tqdb_read_str(r, buf, scratch.size); j->status = jobs::jobStatusFromString(buf);
    j->priority = tqdb_read_i32(r);
    j->retry_count = tqdb_read_i32(r);
    j->max_retries = tqdb_read_i32(r);
    tqdb_read_str(r, buf, scratch.size); j->created_at = buf;
    tqdb_read_str(r, buf, scratch.size); j->started_at = buf;
    tqdb_read_str(r, buf, scratch.size); j->completed_at = buf;
    tqdb_read_str(r, buf, scratch.size); j->error_message = buf;
    tqdb_read_str(r, buf, scratch.size); j->job_data = buf;
}

static void job_skip(tqdb_reader_t* r) {
    tqdb_read_u32(r);       // id
    tqdb_read_skip_str(r);  // type
    tqdb_read_u32(r);       // pattern_id
    tqdb_read_skip_str(r);  // pattern_external_uuid
    tqdb_read_skip_str(r);  // status
    tqdb_read_i32(r);       // priority
    tqdb_read_i32(r);       // retry_count
    tqdb_read_i32(r);       // max_retries
    tqdb_read_skip_str(r);  // created_at
    tqdb_read_skip_str(r);  // started_at
    tqdb_read_skip_str(r);  // completed_at
    tqdb_read_skip_str(r);  // error_message
    tqdb_read_skip_str(r);  // job_data
}

const tqdb_trait_t g_job_trait = {
    .name = "Job",
    .max_count = MAX_JOBS,
    .struct_size = sizeof(jobs::Job),
    .write = job_write,
    .read = job_read,
    .get_id = job_get_id,
    .set_id = job_set_id,
    .init = job_init,
    .destroy = job_destroy,
    .skip = job_skip,
    .copy = job_copy
};
