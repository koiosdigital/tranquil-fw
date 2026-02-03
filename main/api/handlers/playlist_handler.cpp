// Unified playlist handler implementation
#include "playlist_handler.h"
#include "ManifestDatabase.h"

#include <esp_log.h>
#include <cstring>

static const char* TAG = "playlist_handler";

PlaylistHandler& PlaylistHandler::instance() {
    static PlaylistHandler instance;
    return instance;
}

HandleResult PlaylistHandler::handle(
    const Kd__V1__TranquilMessage* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    (void)ctx;

    switch (msg->message_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLISTS_REQUEST:
            return handlePlaylistsRequest(response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_CREATE_PLAYLIST:
            return handleCreatePlaylist(msg->create_playlist, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_UPDATE_PLAYLIST:
            return handleUpdatePlaylist(msg->update_playlist, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_DELETE_PLAYLIST:
            return handleDeletePlaylist(msg->delete_playlist, response);

        default:
            return HandleResult::notHandled();
    }
}

bool PlaylistHandler::canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const {
    switch (msg_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLISTS_REQUEST:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_CREATE_PLAYLIST:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_UPDATE_PLAYLIST:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_DELETE_PLAYLIST:
            return true;
        default:
            return false;
    }
}

std::vector<Kd__V1__TranquilMessage__MessageCase> PlaylistHandler::supportedMessages() const {
    return {
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLISTS_REQUEST,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_CREATE_PLAYLIST,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_UPDATE_PLAYLIST,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_DELETE_PLAYLIST
    };
}

HandleResult PlaylistHandler::handlePlaylistsRequest(ResponseMessage& response) {
    auto playlists = ManifestDatabase::instance().getAllPlaylists();

    ESP_LOGI(TAG, "PlaylistsRequest: returning %zu playlists", playlists.size());

    // Build response - use static storage for protobuf data
    static Kd__V1__Playlists playlists_msg = KD__V1__PLAYLISTS__INIT;
    static std::vector<Kd__V1__PlaylistInfo*> playlist_ptrs;
    static std::vector<Kd__V1__PlaylistInfo> playlist_infos;
    static std::vector<std::vector<char*>> pattern_uuid_ptrs;
    static std::vector<std::vector<std::string>> pattern_uuid_strs;  // Storage for converted UUIDs
    static std::vector<std::string> featured_pattern_strs;  // Storage for featured pattern UUIDs

    playlist_ptrs.clear();
    playlist_infos.clear();
    pattern_uuid_ptrs.clear();
    pattern_uuid_strs.clear();
    featured_pattern_strs.clear();
    playlist_infos.resize(playlists.size());
    pattern_uuid_ptrs.resize(playlists.size());
    pattern_uuid_strs.resize(playlists.size());
    featured_pattern_strs.resize(playlists.size());

    for (size_t i = 0; i < playlists.size(); i++) {
        playlist_infos[i] = KD__V1__PLAYLIST_INFO__INIT;
        // Use external_uuid as "uuid" for API compatibility
        playlist_infos[i].uuid = const_cast<char*>(playlists[i].external_uuid.c_str());
        playlist_infos[i].name = const_cast<char*>(playlists[i].name.c_str());
        playlist_infos[i].description = const_cast<char*>(playlists[i].description.c_str());

        // Convert featured_pattern_id to external UUID
        if (playlists[i].featured_pattern_id != 0) {
            auto featured = ManifestDatabase::instance().getPattern(playlists[i].featured_pattern_id);
            if (featured) {
                featured_pattern_strs[i] = featured->external_uuid;
            }
        }
        playlist_infos[i].featured_pattern = const_cast<char*>(featured_pattern_strs[i].c_str());
        playlist_infos[i].created_at = const_cast<char*>(playlists[i].created_at.c_str());
        playlist_infos[i].updated_at = const_cast<char*>(playlists[i].updated_at.c_str());

        // Convert pattern IDs to external UUIDs
        pattern_uuid_ptrs[i].clear();
        pattern_uuid_strs[i].clear();
        for (uint32_t pattern_id : playlists[i].pattern_ids) {
            auto pattern = ManifestDatabase::instance().getPattern(pattern_id);
            if (pattern) {
                pattern_uuid_strs[i].push_back(pattern->external_uuid);
            }
        }
        for (auto& uuid : pattern_uuid_strs[i]) {
            pattern_uuid_ptrs[i].push_back(const_cast<char*>(uuid.c_str()));
        }
        playlist_infos[i].n_pattern_uuids = pattern_uuid_ptrs[i].size();
        playlist_infos[i].pattern_uuids = pattern_uuid_ptrs[i].data();

        playlist_ptrs.push_back(&playlist_infos[i]);
    }

    playlists_msg.n_playlists = playlist_ptrs.size();
    playlists_msg.playlists = playlist_ptrs.data();

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLISTS;
    resp.playlists = &playlists_msg;

    response = serialize(&resp);
    return HandleResult::ok();
}

HandleResult PlaylistHandler::handleCreatePlaylist(
    const Kd__V1__CreatePlaylist* msg,
    ResponseMessage& response
) {
    if (!msg || !msg->name) {
        ESP_LOGW(TAG, "CreatePlaylist: missing name");
        response = makeCommandResult(false, "Missing playlist name");
        return HandleResult::ok();
    }

    Playlist playlist;
    playlist.id = 0;  // Will be assigned by TQDB
    playlist.external_uuid = ManifestDatabase::generateUUID();
    playlist.name = msg->name;
    playlist.description = msg->description ? msg->description : "";
    playlist.created_at = ManifestDatabase::currentTimestamp();
    playlist.updated_at = playlist.created_at;

    // Convert pattern UUIDs to internal IDs
    for (size_t i = 0; i < msg->n_pattern_uuids; i++) {
        if (msg->pattern_uuids[i]) {
            auto pattern_id = ManifestDatabase::instance().findPatternIdByExternalUuid(msg->pattern_uuids[i]);
            if (pattern_id) {
                playlist.pattern_ids.push_back(*pattern_id);
            } else {
                ESP_LOGW(TAG, "CreatePlaylist: pattern not found: %s", msg->pattern_uuids[i]);
            }
        }
    }

    esp_err_t ret = ManifestDatabase::instance().addPlaylist(playlist);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CreatePlaylist failed: %s", esp_err_to_name(ret));
        response = makeCommandResult(false, "Failed to create playlist");
        return HandleResult::ok();
    }

    ESP_LOGI(TAG, "Created playlist: %s (%s)", playlist.name.c_str(), playlist.external_uuid.c_str());
    response = makeCommandResult(true);
    return HandleResult::ok(true);  // Broadcast to locals
}

HandleResult PlaylistHandler::handleUpdatePlaylist(
    const Kd__V1__UpdatePlaylist* msg,
    ResponseMessage& response
) {
    if (!msg || !msg->uuid) {
        ESP_LOGW(TAG, "UpdatePlaylist: missing uuid");
        response = makeCommandResult(false, "Missing playlist UUID");
        return HandleResult::ok();
    }

    // Lookup by external UUID
    auto existing = ManifestDatabase::instance().getPlaylistByExternalUuid(msg->uuid);
    if (!existing) {
        ESP_LOGW(TAG, "UpdatePlaylist: playlist not found: %s", msg->uuid);
        response = makeCommandResult(false, "Playlist not found");
        return HandleResult::ok();
    }

    Playlist updated = *existing;
    if (msg->name) updated.name = msg->name;
    if (msg->description) updated.description = msg->description;

    // Convert featured_pattern UUID to internal ID
    if (msg->featured_pattern) {
        auto pattern_id = ManifestDatabase::instance().findPatternIdByExternalUuid(msg->featured_pattern);
        if (pattern_id) {
            updated.featured_pattern_id = *pattern_id;
        } else {
            ESP_LOGW(TAG, "UpdatePlaylist: featured pattern not found: %s", msg->featured_pattern);
        }
    }
    updated.updated_at = ManifestDatabase::currentTimestamp();

    // Update pattern list if provided - convert UUIDs to internal IDs
    if (msg->n_pattern_uuids > 0) {
        updated.pattern_ids.clear();
        for (size_t i = 0; i < msg->n_pattern_uuids; i++) {
            if (msg->pattern_uuids[i]) {
                auto pattern_id = ManifestDatabase::instance().findPatternIdByExternalUuid(msg->pattern_uuids[i]);
                if (pattern_id) {
                    updated.pattern_ids.push_back(*pattern_id);
                } else {
                    ESP_LOGW(TAG, "UpdatePlaylist: pattern not found: %s", msg->pattern_uuids[i]);
                }
            }
        }
    }

    // Update by internal ID
    esp_err_t ret = ManifestDatabase::instance().updatePlaylist(existing->id, updated);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UpdatePlaylist failed: %s", esp_err_to_name(ret));
        response = makeCommandResult(false, "Failed to update playlist");
        return HandleResult::ok();
    }

    ESP_LOGI(TAG, "Updated playlist: %s", msg->uuid);
    response = makeCommandResult(true);
    return HandleResult::ok(true);  // Broadcast to locals
}

HandleResult PlaylistHandler::handleDeletePlaylist(
    const Kd__V1__DeletePlaylist* msg,
    ResponseMessage& response
) {
    if (!msg || !msg->uuid) {
        ESP_LOGW(TAG, "DeletePlaylist: missing uuid");
        response = makeCommandResult(false, "Missing playlist UUID");
        return HandleResult::ok();
    }

    // Lookup by external UUID to get internal ID
    auto playlist = ManifestDatabase::instance().getPlaylistByExternalUuid(msg->uuid);
    if (!playlist) {
        ESP_LOGW(TAG, "DeletePlaylist: not found: %s", msg->uuid);
        response = makeCommandResult(false, "Playlist not found");
        return HandleResult::ok();
    }

    esp_err_t ret = ManifestDatabase::instance().deletePlaylist(playlist->id);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "DeletePlaylist: failed: %s", msg->uuid);
        response = makeCommandResult(false, "Failed to delete playlist");
        return HandleResult::ok();
    }

    ESP_LOGI(TAG, "Deleted playlist: %s", msg->uuid);
    response = makeCommandResult(true);
    return HandleResult::ok(true);  // Broadcast to locals
}
