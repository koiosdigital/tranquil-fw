// Library operations: patterns and playlists. Every mutation validates,
// applies, and posts the matching library_notifier event so REST and
// WebSocket changes alike are pushed to LAN clients and the cloud. Both
// transports share the serialisers here.
#pragma once

#include "op_result.h"
#include "api_common.h"
#include "handler_base.h"
#include "ManifestDatabase.h"

#include <optional>
#include <string>
#include <vector>

namespace library_ops {

// Socket list messages are capped by packed size, not entry count; larger
// libraries page through the REST list.
constexpr size_t kListCapBytes = 30 * 1024;

// ---- Patterns ----

// Lookup by external uuid. size_bytes is filled from the SD file when the
// manifest has 0.
std::optional<Pattern> get_pattern(const std::string& uuid);
void fill_size_from_disk(Pattern& p);

// Stops the player first when the pattern is the one playing.
OpResult delete_pattern(const std::string& uuid);

// Empty name = no change (still verifies the pattern exists).
OpResult rename_pattern(const std::string& uuid, const std::string& name);

// One pattern object, REST shape (docs/swagger.json Pattern).
void pattern_to_json(api_common::ChunkBuf* cb, const Pattern& p);

// TranquilMessage{pattern_info}. thumb_url / download_url carry the LAN
// paths (download_url empty for encrypted patterns), as REST does.
ResponseMessage build_pattern_info_message(const Pattern& p, const char* request_id = nullptr);

// TranquilMessage{downloaded_patterns}: name-sorted, stops below
// kListCapBytes. Entries and strings live in PSRAM.
ResponseMessage build_pattern_list_message(const char* request_id = nullptr);

// ---- Playlists ----

struct PlaylistDraft {
    std::string uuid;                        // empty = device assigns
    std::string name;                        // required
    std::string description;
    std::string featured_pattern;            // pattern uuid; empty = none
    std::vector<std::string> pattern_uuids;  // unknown uuids are skipped
};

// Partial update. Absent fields are unchanged.
struct PlaylistPatch {
    std::optional<std::string> name;                        // empty = unchanged
    std::optional<std::string> description;                 // "" allowed
    std::optional<std::string> featured_pattern;            // "" clears
    std::optional<std::vector<std::string>> pattern_uuids;  // replaces; empty allowed
};

std::optional<Playlist> get_playlist(const std::string& uuid);
OpResult create_playlist(const PlaylistDraft& draft, Playlist& created);
OpResult update_playlist(const std::string& uuid, const PlaylistPatch& patch, Playlist& updated);
OpResult delete_playlist(const std::string& uuid);
OpResult add_pattern_to_playlist(const std::string& playlist_uuid, const std::string& pattern_uuid, Playlist& updated);
OpResult remove_pattern_from_playlist(const std::string& playlist_uuid, const std::string& pattern_uuid, Playlist& updated);
OpResult reorder_playlist(const std::string& playlist_uuid, const std::vector<std::string>& pattern_uuids, Playlist& updated);

// One playlist object, REST shape (docs/swagger.json Playlist).
void playlist_to_json(api_common::ChunkBuf* cb, const Playlist& pl);

// TranquilMessage{playlists}, stops below kListCapBytes.
ResponseMessage build_playlist_list_message(const char* request_id = nullptr);

}  // namespace library_ops
