// Unified playlist handler implementation
#include "playlist_handler.h"
#include "ops/library_ops.h"

#include <esp_log.h>
#include <cstring>
#include <string>
#include <vector>

static const char* TAG = "playlist_handler";

namespace {

bool has(const char* s) { return s && s[0] != '\0'; }

std::vector<std::string> string_list(char** items, size_t n) {
    std::vector<std::string> out;
    out.reserve(n);
    for (size_t i = 0; i < n; i++) {
        if (items[i] && items[i][0]) out.emplace_back(items[i]);
    }
    return out;
}

}  // namespace

PlaylistHandler& PlaylistHandler::instance() {
    static PlaylistHandler instance;
    return instance;
}

HandleResult PlaylistHandler::handle(
    const Kd__V1__TranquilMessage* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    const char* rid = ctx.request_id;

    switch (msg->message_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLISTS_REQUEST:
            response = library_ops::build_playlist_list_message(rid);
            return HandleResult::ok();

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_CREATE_PLAYLIST: {
            const auto* m = msg->create_playlist;
            if (!m || !has(m->name)) {
                ESP_LOGW(TAG, "CreatePlaylist: missing name");
                response = makeCommandResult(false, "Missing playlist name", rid, ESP_ERR_INVALID_ARG);
                return HandleResult::ok();
            }
            library_ops::PlaylistDraft draft;
            if (has(m->uuid)) draft.uuid = m->uuid;   // client-chosen id (matches REST)
            draft.name = m->name;
            if (m->description) draft.description = m->description;
            draft.pattern_uuids = string_list(m->pattern_uuids, m->n_pattern_uuids);

            Playlist created;
            // Requester gets the CommandResult; the list push to LAN + cloud
            // is posted by library_ops.
            response = makeCommandResult(library_ops::create_playlist(draft, created), rid);
            return HandleResult::ok();
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_UPDATE_PLAYLIST: {
            const auto* m = msg->update_playlist;
            if (!m || !has(m->uuid)) {
                ESP_LOGW(TAG, "UpdatePlaylist: missing uuid");
                response = makeCommandResult(false, "Missing playlist UUID", rid, ESP_ERR_INVALID_ARG);
                return HandleResult::ok();
            }

            // proto3 `optional` fields carry presence (protobuf-c exposes it
            // as the synthetic *_case). protobuf-c hands us "" (never NULL)
            // for an absent plain string, so presence is the only reliable
            // "was this field sent" signal. The pattern list is replaced only
            // when replace_patterns is set (an empty list is then allowed).
            library_ops::PlaylistPatch patch;
            if (m->_name_case == KD__V1__UPDATE_PLAYLIST___NAME_NAME) {
                patch.name = std::string(m->name ? m->name : "");
            }
            if (m->_description_case == KD__V1__UPDATE_PLAYLIST___DESCRIPTION_DESCRIPTION) {
                patch.description = std::string(m->description ? m->description : "");
            }
            if (m->_featured_pattern_case == KD__V1__UPDATE_PLAYLIST___FEATURED_PATTERN_FEATURED_PATTERN) {
                patch.featured_pattern = std::string(m->featured_pattern ? m->featured_pattern : "");
            }
            if (m->replace_patterns) {
                patch.pattern_uuids = string_list(m->pattern_uuids, m->n_pattern_uuids);
            }

            Playlist updated;
            response = makeCommandResult(library_ops::update_playlist(m->uuid, patch, updated), rid);
            return HandleResult::ok();
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_DELETE_PLAYLIST: {
            const auto* m = msg->delete_playlist;
            if (!m || !has(m->uuid)) {
                ESP_LOGW(TAG, "DeletePlaylist: missing uuid");
                response = makeCommandResult(false, "Missing playlist UUID", rid, ESP_ERR_INVALID_ARG);
                return HandleResult::ok();
            }
            response = makeCommandResult(library_ops::delete_playlist(m->uuid), rid);
            return HandleResult::ok();
        }

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
