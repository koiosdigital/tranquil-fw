// Library operations implementation
#include "library_ops.h"
#include "psram_array.h"
#include "library_notifier.h"
#include "SandTablePlayer.h"

#include <esp_log.h>
#include <sys/stat.h>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>

static const char* TAG = "library_ops";

using namespace api_common;

namespace library_ops {

namespace {

// Both encrypted (KDEP) and unencrypted (THRB) patterns are stored as .dat
void pattern_file_path(const std::string& uuid, char* path, size_t path_size) {
    snprintf(path, path_size, "/sd/patterns/%s.dat", uuid.c_str());
}

std::string thumb_url_for(const std::string& uuid) {
    return "/api/pattern_thumbs/" + uuid + ".png";
}

// Download only for unencrypted patterns (swagger contract).
std::string download_url_for(const Pattern& p) {
    if (p.encrypted) return std::string();
    return "/api/pattern_download/" + p.external_uuid;
}

// A client-chosen playlist uuid must fit the player's 64-byte buffers and
// be safe in a URL path.
bool uuid_valid(const std::string& s) {
    if (s.empty() || s.size() > kMaxUuidChars) return false;
    for (unsigned char c : s) {
        if (!(std::isalnum(c) || c == '-' || c == '_' || c == '.')) return false;
    }
    return true;
}

std::vector<uint32_t> resolve_pattern_ids(const std::vector<std::string>& uuids) {
    std::vector<uint32_t> ids;
    ids.reserve(uuids.size());
    auto& db = ManifestDatabase::instance();
    for (const auto& u : uuids) {
        if (u.empty()) continue;
        auto id = db.findPatternIdByExternalUuid(u);
        if (id) {
            ids.push_back(*id);
        } else {
            ESP_LOGW(TAG, "Pattern not found, skipped: %s", u.c_str());
        }
    }
    return ids;
}

OpResult refetch_playlist(uint32_t id, Playlist& out) {
    auto pl = ManifestDatabase::instance().getPlaylist(id);
    if (!pl) return OpResult::fail(ESP_FAIL, "Playlist vanished");
    out = std::move(*pl);
    return OpResult::success();
}

// Everything a packed PatternInfo points at. For the list builder these are
// interned in a PSRAM arena; for a single reply they are the Pattern's own
// strings plus two locals.
struct PatternStrings {
    const char* uuid;
    const char* name;
    const char* creator;
    const char* created_at;
    const char* last_played_at;
    const char* receipt_id;
    const char* thumb_url;
    const char* download_url;
};

void fill_pattern_info(Kd__V1__PatternInfo& info, const Pattern& p, const PatternStrings& s) {
    info = KD__V1__PATTERN_INFO__INIT;
    info.uuid = const_cast<char*>(s.uuid);
    info.name = const_cast<char*>(s.name);
    info.creator = const_cast<char*>(s.creator);
    info.encrypted = p.encrypted;
    info.size_bytes = static_cast<uint32_t>(p.size_bytes);
    info.reversible = p.reversible;
    info.start_point = static_cast<uint32_t>(p.start_point);
    info.created_at = const_cast<char*>(s.created_at);
    info.last_played_at = const_cast<char*>(s.last_played_at);
    info.download_url = const_cast<char*>(s.download_url);
    info.thumb_url = const_cast<char*>(s.thumb_url);
    info.is_owned = p.purchased;
    info.purchased_at = p.purchased_at;
    info.receipt_id = const_cast<char*>(s.receipt_id);
}

// Wire overhead of one repeated sub-message: tag byte + length varint.
constexpr size_t kRepeatedOverhead = 3;

}  // namespace

// ---------------------------------------------------------------------------
// Patterns
// ---------------------------------------------------------------------------

void fill_size_from_disk(Pattern& p) {
    if (p.size_bytes != 0) return;
    char path[128];
    pattern_file_path(p.external_uuid, path, sizeof(path));
    struct stat st;
    if (stat(path, &st) == 0) {
        p.size_bytes = static_cast<size_t>(st.st_size);
    }
}

std::optional<Pattern> get_pattern(const std::string& uuid) {
    if (uuid.empty()) return std::nullopt;
    auto p = ManifestDatabase::instance().getPatternByExternalUuid(uuid);
    if (p) fill_size_from_disk(*p);
    return p;
}

OpResult delete_pattern(const std::string& uuid) {
    auto& db = ManifestDatabase::instance();
    auto p = db.getPatternByExternalUuid(uuid);
    if (!p) return OpResult::fail(ESP_ERR_NOT_FOUND, "Pattern not found");

    // Never pull the file out from under the player.
    if (SandTablePlayer::isPlayingPattern(uuid.c_str())) {
        ESP_LOGI(TAG, "Deleting the playing pattern %s: stopping playback first", uuid.c_str());
        SandTablePlayer::stop();
    }

    esp_err_t err = db.deletePattern(p->id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Delete pattern %s failed: %s", uuid.c_str(), esp_err_to_name(err));
        return OpResult::fail(err, "Failed to delete pattern");
    }

    ESP_LOGI(TAG, "Deleted pattern %s", uuid.c_str());
    library_notifier::patterns_changed();
    return OpResult::success();
}

OpResult rename_pattern(const std::string& uuid, const std::string& name) {
    auto& db = ManifestDatabase::instance();
    auto p = db.getPatternByExternalUuid(uuid);
    if (!p) return OpResult::fail(ESP_ERR_NOT_FOUND, "Pattern not found");
    if (name.empty()) return OpResult::success();
    if (name.size() > 128) return OpResult::fail(ESP_ERR_INVALID_ARG, "Name too long");

    p->name = name;
    esp_err_t err = db.updatePattern(p->id, *p);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Rename pattern %s failed: %s", uuid.c_str(), esp_err_to_name(err));
        return OpResult::fail(err, "Failed to update pattern");
    }

    ESP_LOGI(TAG, "Renamed pattern %s to '%s'", uuid.c_str(), name.c_str());
    library_notifier::patterns_changed();
    return OpResult::success();
}

void pattern_to_json(ChunkBuf* cb, const Pattern& p) {
    chunk_buf_write_str(cb, "{\"uuid\":");
    chunk_buf_write_json_string(cb, p.external_uuid.c_str());
    chunk_buf_printf(cb, ",\"id\":%lu,\"name\":", (unsigned long)p.id);
    chunk_buf_write_json_string(cb, p.name.c_str());
    chunk_buf_write_str(cb, ",\"creator\":");
    chunk_buf_write_json_string(cb, p.creator.c_str());
    chunk_buf_write_str(cb, ",\"date\":");
    chunk_buf_write_json_string(cb, p.date.c_str());
    chunk_buf_printf(cb, ",\"popularity\":%d,\"reversible\":%s,\"start_point\":%u,\"encrypted\":%s,\"size_bytes\":%lu",
        p.popularity,
        p.reversible ? "true" : "false",
        (unsigned)p.start_point,
        p.encrypted ? "true" : "false",
        (unsigned long)p.size_bytes);
    // URLs per the swagger contract (download only for unencrypted patterns)
    chunk_buf_write_str(cb, ",\"thumb_url\":");
    chunk_buf_write_json_string(cb, thumb_url_for(p.external_uuid).c_str());
    chunk_buf_write_str(cb, ",\"download_url\":");
    chunk_buf_write_json_string(cb, download_url_for(p).c_str());
    if (!p.created_at.empty()) {
        chunk_buf_write_str(cb, ",\"created_at\":");
        chunk_buf_write_json_string(cb, p.created_at.c_str());
    }
    if (!p.last_played_at.empty()) {
        chunk_buf_write_str(cb, ",\"last_played_at\":");
        chunk_buf_write_json_string(cb, p.last_played_at.c_str());
    }
    if (!p.downloaded_at.empty()) {
        chunk_buf_write_str(cb, ",\"downloaded_at\":");
        chunk_buf_write_json_string(cb, p.downloaded_at.c_str());
    }
    chunk_buf_printf(cb, ",\"is_owned\":%s", p.purchased ? "true" : "false");
    if (p.purchased) {
        chunk_buf_printf(cb, ",\"purchased_at\":%lld", (long long)p.purchased_at);
        if (!p.receipt_id.empty()) {
            chunk_buf_write_str(cb, ",\"receipt_id\":");
            chunk_buf_write_json_string(cb, p.receipt_id.c_str());
        }
    }
    chunk_buf_write(cb, "}", 1);
}

ResponseMessage build_pattern_info_message(const Pattern& p, const char* request_id) {
    // Locals, not statics: httpd and the cloudlink task can run this
    // concurrently.
    const std::string thumb = thumb_url_for(p.external_uuid);
    const std::string download = download_url_for(p);

    PatternStrings s = {
        p.external_uuid.c_str(), p.name.c_str(), p.creator.c_str(),
        p.created_at.c_str(), p.last_played_at.c_str(), p.receipt_id.c_str(),
        thumb.c_str(), download.c_str()
    };
    Kd__V1__PatternInfo info;
    fill_pattern_info(info, p, s);

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_PATTERN_INFO;
    resp.pattern_info = &info;
    return pack_tranquil_message(&resp, request_id);
}

ResponseMessage build_pattern_list_message(const char* request_id) {
    auto& db = ManifestDatabase::instance();

    // Entries and every string they point at live in PSRAM; the arena is
    // sized past the packed cap because a packed message is never smaller
    // than the strings in it.
    PsramArena arena(kListCapBytes + 4096);
    PsramArray<Kd__V1__PatternInfo> infos;
    if (!arena.valid() || !infos.reserve(64)) {
        ESP_LOGE(TAG, "Pattern list: out of PSRAM");
        return ResponseMessage();
    }

    // Walk the manifest one page at a time (the paged query materialises
    // only that page) and stop as soon as the next entry would push the
    // packed size over the cap.
    constexpr int kPage = 32;
    size_t packed = 0;
    size_t count = 0;
    int total = 0;
    bool full = false;

    for (int page = 0; !full; page++) {
        PaginatedResult<Pattern> result = db.getPatterns(page, kPage);
        total = result.pagination.total_items;
        if (result.items.empty()) break;

        for (const Pattern& p : result.items) {
            const std::string thumb = thumb_url_for(p.external_uuid);
            const std::string download = download_url_for(p);

            PatternStrings s;
            s.uuid = arena.intern(p.external_uuid);
            s.name = arena.intern(p.name);
            s.creator = arena.intern(p.creator);
            s.created_at = arena.intern(p.created_at);
            s.last_played_at = arena.intern(p.last_played_at);
            s.receipt_id = arena.intern(p.receipt_id);
            s.thumb_url = arena.intern(thumb);
            s.download_url = arena.intern(download);
            if (!s.uuid || !s.name || !s.creator || !s.created_at || !s.last_played_at ||
                !s.receipt_id || !s.thumb_url || !s.download_url) {
                full = true;
                break;
            }

            Kd__V1__PatternInfo info;
            fill_pattern_info(info, p, s);

            const size_t entry = kd__v1__pattern_info__get_packed_size(&info) + kRepeatedOverhead;
            if (packed + entry > kListCapBytes || !infos.push_back(info)) {
                full = true;
                break;
            }
            packed += entry;
            count++;
        }

        if (result.items.size() < static_cast<size_t>(kPage)) break;  // last page
    }

    PsramArray<Kd__V1__PatternInfo*> ptrs;
    if (!ptrs.reserve(infos.size() ? infos.size() : 1)) {
        ESP_LOGE(TAG, "Pattern list: out of PSRAM");
        return ResponseMessage();
    }
    for (size_t i = 0; i < infos.size(); i++) {
        ptrs.push_back(&infos[i]);
    }

    Kd__V1__DownloadedPatterns downloaded = KD__V1__DOWNLOADED_PATTERNS__INIT;
    downloaded.n_patterns = ptrs.size();
    downloaded.patterns = ptrs.data();

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_DOWNLOADED_PATTERNS;
    resp.downloaded_patterns = &downloaded;

    if (static_cast<int>(count) < total) {
        ESP_LOGW(TAG, "DownloadedPatterns truncated to %zu of %d patterns (%zu bytes); use the REST list",
            count, total, packed);
    } else {
        ESP_LOGD(TAG, "DownloadedPatterns: %zu patterns, %zu bytes", count, packed);
    }
    return pack_tranquil_message(&resp, request_id);
}

// ---------------------------------------------------------------------------
// Playlists
// ---------------------------------------------------------------------------

std::optional<Playlist> get_playlist(const std::string& uuid) {
    if (uuid.empty()) return std::nullopt;
    return ManifestDatabase::instance().getPlaylistByExternalUuid(uuid);
}

OpResult create_playlist(const PlaylistDraft& draft, Playlist& created) {
    auto& db = ManifestDatabase::instance();
    if (draft.name.empty()) return OpResult::fail(ESP_ERR_INVALID_ARG, "Missing playlist name");

    Playlist pl;
    pl.id = 0;  // Assigned by ManifestDatabase
    if (!draft.uuid.empty()) {
        if (!uuid_valid(draft.uuid)) return OpResult::fail(ESP_ERR_INVALID_ARG, "Invalid playlist uuid");
        if (db.getPlaylistByExternalUuid(draft.uuid)) {
            return OpResult::fail(ESP_ERR_INVALID_STATE, "Playlist uuid already exists");
        }
        pl.external_uuid = draft.uuid;
    } else {
        pl.external_uuid = ManifestDatabase::generateUUID();
    }
    pl.name = draft.name;
    pl.description = draft.description;
    if (!draft.featured_pattern.empty()) {
        auto id = db.findPatternIdByExternalUuid(draft.featured_pattern);
        pl.featured_pattern_id = id.value_or(0);
    }
    pl.pattern_ids = resolve_pattern_ids(draft.pattern_uuids);
    pl.created_at = ManifestDatabase::currentTimestamp();
    pl.updated_at = pl.created_at;

    esp_err_t err = db.addPlaylist(pl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Create playlist failed: %s", esp_err_to_name(err));
        return OpResult::fail(err, "Failed to create playlist");
    }

    ESP_LOGI(TAG, "Created playlist '%s' (%s)", pl.name.c_str(), pl.external_uuid.c_str());
    created = std::move(pl);
    library_notifier::playlists_changed();
    return OpResult::success();
}

OpResult update_playlist(const std::string& uuid, const PlaylistPatch& patch, Playlist& updated) {
    auto& db = ManifestDatabase::instance();
    auto existing = db.getPlaylistByExternalUuid(uuid);
    if (!existing) return OpResult::fail(ESP_ERR_NOT_FOUND, "Playlist not found");

    Playlist u = *existing;
    if (patch.name && !patch.name->empty()) u.name = *patch.name;
    if (patch.description) u.description = *patch.description;
    if (patch.featured_pattern) {
        if (patch.featured_pattern->empty()) {
            u.featured_pattern_id = 0;
        } else {
            auto id = db.findPatternIdByExternalUuid(*patch.featured_pattern);
            if (id) {
                u.featured_pattern_id = *id;
            } else {
                ESP_LOGW(TAG, "Featured pattern not found: %s", patch.featured_pattern->c_str());
            }
        }
    }
    if (patch.pattern_uuids) u.pattern_ids = resolve_pattern_ids(*patch.pattern_uuids);
    u.updated_at = ManifestDatabase::currentTimestamp();

    esp_err_t err = db.updatePlaylist(existing->id, u);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Update playlist %s failed: %s", uuid.c_str(), esp_err_to_name(err));
        return OpResult::fail(err, "Failed to update playlist");
    }

    OpResult r = refetch_playlist(existing->id, updated);
    if (!r.ok()) return r;
    ESP_LOGI(TAG, "Updated playlist %s", uuid.c_str());
    library_notifier::playlists_changed();
    return OpResult::success();
}

OpResult delete_playlist(const std::string& uuid) {
    auto& db = ManifestDatabase::instance();
    auto pl = db.getPlaylistByExternalUuid(uuid);
    if (!pl) return OpResult::fail(ESP_ERR_NOT_FOUND, "Playlist not found");

    esp_err_t err = db.deletePlaylist(pl->id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Delete playlist %s failed: %s", uuid.c_str(), esp_err_to_name(err));
        return OpResult::fail(err, "Failed to delete playlist");
    }

    ESP_LOGI(TAG, "Deleted playlist %s", uuid.c_str());
    library_notifier::playlists_changed();
    return OpResult::success();
}

OpResult add_pattern_to_playlist(const std::string& playlist_uuid, const std::string& pattern_uuid, Playlist& updated) {
    auto& db = ManifestDatabase::instance();
    auto pl = db.getPlaylistByExternalUuid(playlist_uuid);
    if (!pl) return OpResult::fail(ESP_ERR_NOT_FOUND, "Playlist not found");
    auto pid = db.findPatternIdByExternalUuid(pattern_uuid);
    if (!pid) return OpResult::fail(ESP_ERR_INVALID_ARG, "Pattern not found");

    esp_err_t err = db.addPatternToPlaylist(pl->id, *pid);
    if (err != ESP_OK) return OpResult::fail(err, "Failed to add pattern to playlist");

    OpResult r = refetch_playlist(pl->id, updated);
    if (!r.ok()) return r;
    library_notifier::playlists_changed();
    return OpResult::success();
}

OpResult remove_pattern_from_playlist(const std::string& playlist_uuid, const std::string& pattern_uuid, Playlist& updated) {
    auto& db = ManifestDatabase::instance();
    auto pl = db.getPlaylistByExternalUuid(playlist_uuid);
    if (!pl) return OpResult::fail(ESP_ERR_NOT_FOUND, "Playlist not found");
    auto pid = db.findPatternIdByExternalUuid(pattern_uuid);
    if (!pid) return OpResult::fail(ESP_ERR_INVALID_ARG, "Pattern not found");

    esp_err_t err = db.removePatternFromPlaylist(pl->id, *pid);
    if (err != ESP_OK) return OpResult::fail(err, "Failed to remove pattern from playlist");

    OpResult r = refetch_playlist(pl->id, updated);
    if (!r.ok()) return r;
    library_notifier::playlists_changed();
    return OpResult::success();
}

OpResult reorder_playlist(const std::string& playlist_uuid, const std::vector<std::string>& pattern_uuids, Playlist& updated) {
    auto& db = ManifestDatabase::instance();
    auto pl = db.getPlaylistByExternalUuid(playlist_uuid);
    if (!pl) return OpResult::fail(ESP_ERR_NOT_FOUND, "Playlist not found");

    std::vector<uint32_t> order = resolve_pattern_ids(pattern_uuids);
    esp_err_t err = db.reorderPlaylist(pl->id, order);
    if (err != ESP_OK) return OpResult::fail(err, "Failed to reorder playlist");

    OpResult r = refetch_playlist(pl->id, updated);
    if (!r.ok()) return r;
    library_notifier::playlists_changed();
    return OpResult::success();
}

void playlist_to_json(ChunkBuf* cb, const Playlist& pl) {
    auto& db = ManifestDatabase::instance();

    chunk_buf_write_str(cb, "{\"uuid\":");
    chunk_buf_write_json_string(cb, pl.external_uuid.c_str());
    chunk_buf_printf(cb, ",\"id\":%lu,\"name\":", (unsigned long)pl.id);
    chunk_buf_write_json_string(cb, pl.name.c_str());
    chunk_buf_write_str(cb, ",\"description\":");
    chunk_buf_write_json_string(cb, pl.description.c_str());

    chunk_buf_write_str(cb, ",\"featured_pattern\":");
    std::string featured;
    if (pl.featured_pattern_id != 0) {
        auto u = db.getPatternUuid(pl.featured_pattern_id);
        if (u) featured = *u;
    }
    chunk_buf_write_json_string(cb, featured.c_str());

    chunk_buf_write_str(cb, ",\"date\":");
    chunk_buf_write_json_string(cb, pl.date.c_str());
    if (!pl.created_at.empty()) {
        chunk_buf_write_str(cb, ",\"created_at\":");
        chunk_buf_write_json_string(cb, pl.created_at.c_str());
    }
    if (!pl.updated_at.empty()) {
        chunk_buf_write_str(cb, ",\"updated_at\":");
        chunk_buf_write_json_string(cb, pl.updated_at.c_str());
    }

    // Pattern ids resolve to uuids one at a time so the full list never has
    // to sit in a JSON tree on the heap.
    chunk_buf_write_str(cb, ",\"pattern_uuids\":[");
    bool first = true;
    for (uint32_t pid : pl.pattern_ids) {
        auto u = db.getPatternUuid(pid);
        if (!u) continue;
        if (!first) chunk_buf_write(cb, ",", 1);
        chunk_buf_write_json_string(cb, u->c_str());
        first = false;
    }
    chunk_buf_write_str(cb, "]}");
}

ResponseMessage build_playlist_list_message(const char* request_id) {
    auto& db = ManifestDatabase::instance();
    std::vector<Playlist> playlists = db.getAllPlaylists();

    size_t total_refs = 0;
    for (const auto& pl : playlists) total_refs += pl.pattern_ids.size();

    PsramArena arena(kListCapBytes + 4096);
    PsramArray<Kd__V1__PlaylistInfo> infos;
    PsramArray<char*> uuid_ptrs;
    // uuid_ptrs is reserved for every reference up front so the per-playlist
    // pointer runs handed to protobuf never move.
    if (!arena.valid() || !infos.reserve(playlists.size() ? playlists.size() : 1) ||
        !uuid_ptrs.reserve(total_refs ? total_refs : 1)) {
        ESP_LOGE(TAG, "Playlist list: out of PSRAM");
        return ResponseMessage();
    }

    size_t packed = 0;
    size_t count = 0;
    for (const Playlist& pl : playlists) {
        Kd__V1__PlaylistInfo info = KD__V1__PLAYLIST_INFO__INIT;

        const char* uuid = arena.intern(pl.external_uuid);
        const char* name = arena.intern(pl.name);
        const char* description = arena.intern(pl.description);
        const char* created_at = arena.intern(pl.created_at);
        const char* updated_at = arena.intern(pl.updated_at);
        const char* featured = "";
        if (pl.featured_pattern_id != 0) {
            auto u = db.getPatternUuid(pl.featured_pattern_id);
            if (u) featured = arena.intern(*u);
        }
        if (!uuid || !name || !description || !created_at || !updated_at || !featured) break;

        const size_t first = uuid_ptrs.size();
        bool overflow = false;
        for (uint32_t pid : pl.pattern_ids) {
            auto u = db.getPatternUuid(pid);
            if (!u) continue;
            const char* s = arena.intern(*u);
            if (!s || !uuid_ptrs.push_back(const_cast<char*>(s))) {
                overflow = true;
                break;
            }
        }
        if (overflow) {
            uuid_ptrs.truncate(first);
            break;
        }

        info.uuid = const_cast<char*>(uuid);
        info.name = const_cast<char*>(name);
        info.description = const_cast<char*>(description);
        info.featured_pattern = const_cast<char*>(featured);
        info.created_at = const_cast<char*>(created_at);
        info.updated_at = const_cast<char*>(updated_at);
        info.n_pattern_uuids = uuid_ptrs.size() - first;
        info.pattern_uuids = uuid_ptrs.data() + first;

        const size_t entry = kd__v1__playlist_info__get_packed_size(&info) + kRepeatedOverhead;
        if (packed + entry > kListCapBytes || !infos.push_back(info)) {
            uuid_ptrs.truncate(first);
            break;
        }
        packed += entry;
        count++;
    }

    PsramArray<Kd__V1__PlaylistInfo*> ptrs;
    if (!ptrs.reserve(infos.size() ? infos.size() : 1)) {
        ESP_LOGE(TAG, "Playlist list: out of PSRAM");
        return ResponseMessage();
    }
    for (size_t i = 0; i < infos.size(); i++) {
        ptrs.push_back(&infos[i]);
    }

    Kd__V1__Playlists playlists_msg = KD__V1__PLAYLISTS__INIT;
    playlists_msg.n_playlists = ptrs.size();
    playlists_msg.playlists = ptrs.data();

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLISTS;
    resp.playlists = &playlists_msg;

    if (count < playlists.size()) {
        ESP_LOGW(TAG, "Playlists truncated to %zu of %zu (%zu bytes); use the REST list",
            count, playlists.size(), packed);
    } else {
        ESP_LOGD(TAG, "Playlists: %zu playlists, %zu bytes", count, packed);
    }
    return pack_tranquil_message(&resp, request_id);
}

}  // namespace library_ops
