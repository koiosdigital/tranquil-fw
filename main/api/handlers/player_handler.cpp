// Unified player control handler implementation
#include "player_handler.h"
#include "player_state_broadcaster.h"
#include "ops/player_ops.h"

#include <esp_log.h>

static const char* TAG = "player_handler";

namespace {

bool has(const char* s) { return s && s[0] != '\0'; }

OpResult invalid_message() {
    return OpResult::fail(ESP_ERR_INVALID_ARG, "Invalid message");
}

}  // namespace

PlayerHandler& PlayerHandler::instance() {
    static PlayerHandler instance;
    return instance;
}

// Every mutating command answers the REQUESTER with a CommandResult
// (unicast, or back to the cloud). The resulting PlayerState reaches every
// client - LAN and cloud - through PlayerStateBroadcaster, which notices the
// change on its next 100 ms tick. Broadcasting the CommandResult itself to
// all clients (the old behaviour) told everyone about a result they could
// not correlate, and told nobody what actually changed.
HandleResult PlayerHandler::handle(
    const Kd__V1__TranquilMessage* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    const char* rid = ctx.request_id;
    auto reply = [&](const OpResult& r) {
        response = makeCommandResult(r, rid);
        return HandleResult::ok();
    };

    switch (msg->message_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAY_PATTERN: {
            const auto* m = msg->play_pattern;
            return reply(player_ops::play_pattern(m ? m->pattern_uuid : nullptr));
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_PAUSED: {
            const auto* m = msg->set_paused;
            if (!m) return reply(invalid_message());
            return reply(player_ops::set_paused(m->paused != 0));
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_STOP_PLAYBACK: {
            const auto* m = msg->stop_playback;
            return reply(player_ops::stop(m && m->emergency));
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLIST_PLAY: {
            const auto* m = msg->playlist_play;
            if (!m || !has(m->playlist_uuid)) {
                return reply(OpResult::fail(ESP_ERR_INVALID_ARG, "Missing playlist_uuid"));
            }
            if (has(m->start_pattern_uuid)) {
                return reply(player_ops::play_playlist_from_pattern(
                    m->playlist_uuid, m->start_pattern_uuid, m->shuffle != 0, m->loop != 0));
            }
            return reply(player_ops::play_playlist(m->playlist_uuid, m->shuffle != 0, m->loop != 0));
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_SHUFFLE: {
            const auto* m = msg->set_shuffle;
            if (!m) return reply(invalid_message());
            return reply(player_ops::set_shuffle(m->shuffle != 0));
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_LOOP: {
            const auto* m = msg->set_loop;
            if (!m) return reply(invalid_message());
            return reply(player_ops::set_loop(m->enabled != 0));
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAY_RANDOM: {
            const auto* m = msg->play_random;
            if (!m) return reply(invalid_message());
            return reply(player_ops::play_random(m->loop != 0));
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_RANDOM_LOOP: {
            const auto* m = msg->set_random_loop;
            if (!m) return reply(invalid_message());
            return reply(player_ops::set_random_loop(m->enabled != 0));
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_FEED_RATE: {
            const auto* m = msg->set_feed_rate;
            if (!m) return reply(invalid_message());
            return reply(player_ops::set_feed_rate(m->feed_rate_rpm));
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLIST_NAVIGATE_COMMAND: {
            const auto* m = msg->playlist_navigate_command;
            if (!m) return reply(invalid_message());
            if (m->direction == KD__V1__PLAYLIST_NAVIGATE_COMMAND__DIRECTION__DIRECTION_NEXT) {
                return reply(player_ops::next());
            }
            if (m->direction == KD__V1__PLAYLIST_NAVIGATE_COMMAND__DIRECTION__DIRECTION_PREVIOUS) {
                return reply(player_ops::previous());
            }
            ESP_LOGW(TAG, "PlaylistNavigate: unspecified direction");
            return reply(OpResult::fail(ESP_ERR_INVALID_ARG, "Unspecified direction"));
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_PLAYER_STATE: {
            // A cloud query carrying watch_seconds means a remote user has the
            // player page open: keep progress flowing at the fast cadence.
            const auto* m = msg->get_player_state;
            if (ctx.isCloud() && m && m->watch_seconds > 0) {
                PlayerStateBroadcaster::instance().noteCloudAttention(m->watch_seconds);
            }
            response = player_ops::build_state_message(rid);
            // GetPlayerState is a query - unicast response only
            return HandleResult::ok(false, false);
        }

        default:
            return HandleResult::notHandled();
    }
}

bool PlayerHandler::canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const {
    switch (msg_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAY_PATTERN:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_PAUSED:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_STOP_PLAYBACK:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLIST_PLAY:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_SHUFFLE:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_LOOP:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAY_RANDOM:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_RANDOM_LOOP:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_FEED_RATE:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLIST_NAVIGATE_COMMAND:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_PLAYER_STATE:
            return true;
        default:
            return false;
    }
}

std::vector<Kd__V1__TranquilMessage__MessageCase> PlayerHandler::supportedMessages() const {
    return {
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAY_PATTERN,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_PAUSED,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_STOP_PLAYBACK,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLIST_PLAY,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_SHUFFLE,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_LOOP,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAY_RANDOM,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_RANDOM_LOOP,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_FEED_RATE,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLIST_NAVIGATE_COMMAND,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_PLAYER_STATE
    };
}
