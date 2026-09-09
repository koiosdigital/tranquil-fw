// Unified player control handler implementation
#include "player_handler.h"
#include "player_state_broadcaster.h"
#include "SandTablePlayer.h"

#include <esp_log.h>
#include <cstring>

static const char* TAG = "player_handler";

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
    switch (msg->message_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAY_PATTERN:
            return handlePlayPattern(msg->play_pattern, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_PAUSED:
            return handleSetPaused(msg->set_paused, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_STOP_PLAYBACK:
            return handleStopPlayback(response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLIST_PLAY:
            return handlePlaylistPlay(msg->playlist_play, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_SHUFFLE:
            return handleSetShuffle(msg->set_shuffle, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_LOOP:
            return handleSetLoop(msg->set_loop, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_FEED_RATE:
            return handleSetFeedRate(msg->set_feed_rate, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLIST_NAVIGATE_COMMAND:
            return handlePlaylistNavigate(msg->playlist_navigate_command, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_PLAYER_STATE:
            return handleGetPlayerState(msg->get_player_state, ctx, response);

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
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_FEED_RATE,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLIST_NAVIGATE_COMMAND,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_PLAYER_STATE
    };
}

HandleResult PlayerHandler::handlePlayPattern(
    const Kd__V1__PlayPattern* msg,
    ResponseMessage& response
) {
    if (!msg || !msg->pattern_uuid) {
        ESP_LOGW(TAG, "PlayPattern: missing pattern_uuid");
        response = makeCommandResult(false, "Missing pattern_uuid");
        return HandleResult::ok();
    }

    ESP_LOGI(TAG, "PlayPattern: %s", msg->pattern_uuid);
    esp_err_t ret = SandTablePlayer::playPattern(msg->pattern_uuid);

    response = makeCommandResult(ret == ESP_OK, ret == ESP_OK ? nullptr : "Failed to play pattern");
    return HandleResult::ok();
}

HandleResult PlayerHandler::handleSetPaused(
    const Kd__V1__SetPaused* msg,
    ResponseMessage& response
) {
    if (!msg) {
        response = makeCommandResult(false, "Invalid message");
        return HandleResult::ok();
    }

    ESP_LOGI(TAG, "SetPaused: %s", msg->paused ? "true" : "false");
    esp_err_t ret = msg->paused ? SandTablePlayer::pause() : SandTablePlayer::resume();

    response = makeCommandResult(ret == ESP_OK);
    return HandleResult::ok();
}

HandleResult PlayerHandler::handleStopPlayback(ResponseMessage& response) {
    ESP_LOGI(TAG, "StopPlayback");
    esp_err_t ret = SandTablePlayer::stop();

    response = makeCommandResult(ret == ESP_OK);
    return HandleResult::ok();
}

HandleResult PlayerHandler::handlePlaylistPlay(
    const Kd__V1__PlaylistPlay* msg,
    ResponseMessage& response
) {
    if (!msg || !msg->playlist_uuid) {
        ESP_LOGW(TAG, "PlaylistPlay: missing playlist_uuid");
        response = makeCommandResult(false, "Missing playlist_uuid");
        return HandleResult::ok();
    }

    ESP_LOGI(TAG, "PlaylistPlay: %s (shuffle=%d, loop=%d, start=%s)",
        msg->playlist_uuid, msg->shuffle, msg->loop,
        (msg->start_pattern_uuid && msg->start_pattern_uuid[0]) ? msg->start_pattern_uuid : "-");

    esp_err_t ret;
    if (msg->start_pattern_uuid && msg->start_pattern_uuid[0]) {
        ret = SandTablePlayer::playPlaylistFromPattern(
            msg->playlist_uuid, msg->start_pattern_uuid, msg->shuffle, msg->loop);
    }
    else {
        ret = SandTablePlayer::playPlaylist(msg->playlist_uuid, msg->shuffle, msg->loop);
    }

    response = makeCommandResult(ret == ESP_OK, ret == ESP_OK ? nullptr : "Failed to play playlist");
    return HandleResult::ok();
}

HandleResult PlayerHandler::handleSetShuffle(
    const Kd__V1__PlaylistSetShuffle* msg,
    ResponseMessage& response
) {
    if (!msg) {
        response = makeCommandResult(false, "Invalid message");
        return HandleResult::ok();
    }

    ESP_LOGI(TAG, "SetShuffle: %s", msg->shuffle ? "true" : "false");
    esp_err_t ret = SandTablePlayer::setShuffle(msg->shuffle);

    response = makeCommandResult(ret == ESP_OK);
    return HandleResult::ok();
}

HandleResult PlayerHandler::handleSetLoop(
    const Kd__V1__SetLoop* msg,
    ResponseMessage& response
) {
    if (!msg) {
        response = makeCommandResult(false, "Invalid message");
        return HandleResult::ok();
    }

    ESP_LOGI(TAG, "SetLoop: %s", msg->enabled ? "true" : "false");
    esp_err_t ret = SandTablePlayer::setLoop(msg->enabled);

    response = makeCommandResult(ret == ESP_OK);
    return HandleResult::ok();
}

HandleResult PlayerHandler::handleSetFeedRate(
    const Kd__V1__SetFeedRate* msg,
    ResponseMessage& response
) {
    if (!msg) {
        response = makeCommandResult(false, "Invalid message");
        return HandleResult::ok();
    }

    ESP_LOGI(TAG, "SetFeedRate: %.2f", msg->feed_rate_rpm);
    SandTablePlayer::setFeedRate(msg->feed_rate_rpm);

    response = makeCommandResult(true);
    return HandleResult::ok();
}

HandleResult PlayerHandler::handlePlaylistNavigate(
    const Kd__V1__PlaylistNavigateCommand* msg,
    ResponseMessage& response
) {
    if (!msg) {
        response = makeCommandResult(false, "Invalid message");
        return HandleResult::ok();
    }

    esp_err_t ret = ESP_OK;

    if (msg->direction == KD__V1__PLAYLIST_NAVIGATE_COMMAND__DIRECTION__DIRECTION_NEXT) {
        ESP_LOGI(TAG, "PlaylistNavigate: next");
        ret = SandTablePlayer::skip();
    } else if (msg->direction == KD__V1__PLAYLIST_NAVIGATE_COMMAND__DIRECTION__DIRECTION_PREVIOUS) {
        ESP_LOGI(TAG, "PlaylistNavigate: previous");
        ret = SandTablePlayer::previous();
    } else {
        ESP_LOGW(TAG, "PlaylistNavigate: unspecified direction");
        ret = ESP_ERR_INVALID_ARG;
    }

    const char* detail = nullptr;
    if (ret == ESP_ERR_INVALID_STATE) detail = "Not in playlist mode";
    else if (ret != ESP_OK) detail = "Navigation failed";
    response = makeCommandResult(ret == ESP_OK, detail);
    return HandleResult::ok();
}

HandleResult PlayerHandler::handleGetPlayerState(
    const Kd__V1__GetPlayerState* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    // A cloud query carrying watch_seconds means a remote user has the
    // player page open: keep progress flowing at the fast cadence.
    if (ctx.isCloud() && msg && msg->watch_seconds > 0) {
        PlayerStateBroadcaster::instance().noteCloudAttention(msg->watch_seconds);
    }
    response = buildPlayerStateResponse();
    // GetPlayerState is a query - unicast response only
    return HandleResult::ok(false, false);
}

ResponseMessage PlayerHandler::buildPlayerStateResponse() {
    // Locals, not statics: this runs concurrently from the broadcaster timer,
    // httpd, and the cloudlink task. Shared static buffers raced between
    // get_packed_size() and pack(), overflowing the serialize buffer.
    Kd__V1__PlayerState state = KD__V1__PLAYER_STATE__INIT;
    char pattern_uuid[MAX_UUID_LEN] = {0};
    char playlist_uuid[MAX_UUID_LEN] = {0};

    PlaybackStatus status = SandTablePlayer::getStatus();

    // Map PlaybackState enum
    switch (status.state) {
        case PlaybackState::STOPPED:
            state.state = KD__V1__PLAYER_STATE__PLAYBACK_STATE__PLAYBACK_STATE_STOPPED;
            break;
        case PlaybackState::PLAYING:
            state.state = KD__V1__PLAYER_STATE__PLAYBACK_STATE__PLAYBACK_STATE_PLAYING;
            break;
        case PlaybackState::PAUSED:
            state.state = KD__V1__PLAYER_STATE__PLAYBACK_STATE__PLAYBACK_STATE_PAUSED;
            break;
    }

    // Map PlayMode enum
    switch (status.mode) {
        case PlayMode::SINGLE_PATTERN:
            state.mode = KD__V1__PLAYER_STATE__PLAY_MODE__PLAY_MODE_SINGLE;
            break;
        case PlayMode::PLAYLIST:
            state.mode = KD__V1__PLAYER_STATE__PLAY_MODE__PLAY_MODE_PLAYLIST;
            break;
        case PlayMode::PLAYLIST_LOOP:
            state.mode = KD__V1__PLAYER_STATE__PLAY_MODE__PLAY_MODE_PLAYLIST_LOOP;
            break;
        case PlayMode::PLAYLIST_SHUFFLE:
            state.mode = KD__V1__PLAYER_STATE__PLAY_MODE__PLAY_MODE_PLAYLIST_SHUFFLE;
            break;
    }

    strncpy(pattern_uuid, status.current_pattern_uuid.c_str(), sizeof(pattern_uuid) - 1);
    pattern_uuid[sizeof(pattern_uuid) - 1] = '\0';

    strncpy(playlist_uuid, status.current_playlist_uuid.c_str(), sizeof(playlist_uuid) - 1);
    playlist_uuid[sizeof(playlist_uuid) - 1] = '\0';

    state.current_pattern_uuid = pattern_uuid;
    state.current_playlist_uuid = playlist_uuid;
    state.progress_percent = status.progress_percent;
    state.pattern_index = status.pattern_index;
    state.playlist_size = status.playlist_size;
    state.feed_rate = static_cast<float>(status.feed_rate);
    state.shuffle = status.is_shuffle;
    state.loop = status.is_loop;

    // Build response message
    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYER_STATE;
    resp.player_state = &state;

    return serialize(&resp);
}
