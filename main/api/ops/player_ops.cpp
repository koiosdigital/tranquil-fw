// Player operations implementation
#include "player_ops.h"
#include "schedule_manager.h"

#include <esp_log.h>
#include <cmath>
#include <cstring>

static const char* TAG = "player_ops";

namespace player_ops {

namespace {

bool has(const char* s) { return s && s[0] != '\0'; }

// The player's uuid buffers are MAX_UUID_LEN bytes including the NUL: a
// longer id would be silently truncated into a different key.
bool uuid_ok(const char* s) {
    return has(s) && strlen(s) < MAX_UUID_LEN;
}

OpResult from_play_err(esp_err_t err, bool random) {
    switch (err) {
        case ESP_OK: return OpResult::success();
        case ESP_ERR_NOT_FOUND:
            return OpResult::fail(err, random ? "No playable patterns" : "Pattern or playlist not found");
        case ESP_ERR_INVALID_STATE: return OpResult::fail(err, "Not homed or not initialized");
        case ESP_ERR_TIMEOUT: return OpResult::fail(err, "Player busy");
        default: return OpResult::fail(err, "Failed to start playback");
    }
}

OpResult from_ctrl_err(esp_err_t err, const char* generic, const char* invalid_state) {
    switch (err) {
        case ESP_OK: return OpResult::success();
        case ESP_ERR_INVALID_STATE: return OpResult::fail(err, invalid_state);
        case ESP_ERR_TIMEOUT: return OpResult::fail(err, "Player busy");
        default: return OpResult::fail(err, generic);
    }
}

const char* state_name(PlaybackState s) {
    switch (s) {
        case PlaybackState::PLAYING: return "PLAYING";
        case PlaybackState::PAUSED: return "PAUSED";
        default: return "STOPPED";
    }
}

const char* mode_name(PlayMode m) {
    switch (m) {
        case PlayMode::PLAYLIST: return "PLAYLIST";
        case PlayMode::PLAYLIST_LOOP: return "PLAYLIST_LOOP";
        case PlayMode::PLAYLIST_SHUFFLE: return "PLAYLIST_SHUFFLE";
        case PlayMode::RANDOM_LOOP: return "RANDOM_LOOP";
        default: return "SINGLE_PATTERN";
    }
}

}  // namespace

OpResult play(const PlayRequest& r) {
    const bool has_pattern = has(r.pattern_uuid);
    const bool has_playlist = has(r.playlist_uuid);
    if (r.random) return play_random(r.loop);
    if (!has_pattern && !has_playlist) {
        return OpResult::fail(ESP_ERR_INVALID_ARG, "Missing pattern_uuid, playlist_uuid or random");
    }
    if (has_playlist && has_pattern) {
        return play_playlist_from_pattern(r.playlist_uuid, r.pattern_uuid, r.shuffle, r.loop);
    }
    if (has_playlist) return play_playlist(r.playlist_uuid, r.shuffle, r.loop);
    return play_pattern(r.pattern_uuid);
}

OpResult play_pattern(const char* pattern_uuid) {
    ScheduleManager::instance().notePlaybackCommand();
    if (!uuid_ok(pattern_uuid)) return OpResult::fail(ESP_ERR_INVALID_ARG, "Missing or invalid pattern_uuid");
    ESP_LOGI(TAG, "Play pattern %s", pattern_uuid);
    return from_play_err(SandTablePlayer::playPattern(pattern_uuid), false);
}

OpResult play_playlist(const char* playlist_uuid, bool shuffle, bool loop) {
    ScheduleManager::instance().notePlaybackCommand();
    if (!uuid_ok(playlist_uuid)) return OpResult::fail(ESP_ERR_INVALID_ARG, "Missing or invalid playlist_uuid");
    ESP_LOGI(TAG, "Play playlist %s (shuffle=%d loop=%d)", playlist_uuid, shuffle, loop);
    return from_play_err(SandTablePlayer::playPlaylist(playlist_uuid, shuffle, loop), false);
}

OpResult play_playlist_from_pattern(const char* playlist_uuid, const char* pattern_uuid, bool shuffle, bool loop) {
    ScheduleManager::instance().notePlaybackCommand();
    if (!uuid_ok(playlist_uuid)) return OpResult::fail(ESP_ERR_INVALID_ARG, "Missing or invalid playlist_uuid");
    if (!uuid_ok(pattern_uuid)) return OpResult::fail(ESP_ERR_INVALID_ARG, "Missing or invalid pattern_uuid");
    ESP_LOGI(TAG, "Play playlist %s from %s (shuffle=%d loop=%d)", playlist_uuid, pattern_uuid, shuffle, loop);
    return from_play_err(
        SandTablePlayer::playPlaylistFromPattern(playlist_uuid, pattern_uuid, shuffle, loop), false);
}

OpResult play_random(bool loop) {
    ScheduleManager::instance().notePlaybackCommand();
    ESP_LOGI(TAG, "Play random (loop=%d)", loop);
    return from_play_err(SandTablePlayer::playRandom(loop), true);
}

OpResult set_paused(bool paused) {
    ScheduleManager::instance().notePlaybackCommand();
    ESP_LOGI(TAG, "%s", paused ? "Pause" : "Resume");
    esp_err_t err = paused ? SandTablePlayer::pause() : SandTablePlayer::resume();
    return from_ctrl_err(err, paused ? "Failed to pause" : "Failed to resume", "Player not initialized");
}

OpResult stop(bool emergency) {
    ScheduleManager::instance().notePlaybackCommand();
    ESP_LOGI(TAG, "%s", emergency ? "Emergency stop" : "Stop");
    esp_err_t err = emergency ? SandTablePlayer::emergencyStop() : SandTablePlayer::stop();
    return from_ctrl_err(err, "Failed to stop", "Player not initialized");
}

OpResult next() {
    ESP_LOGI(TAG, "Next");
    return from_ctrl_err(SandTablePlayer::skip(), "Navigation failed", "Not in playlist mode");
}

OpResult previous() {
    ESP_LOGI(TAG, "Previous");
    return from_ctrl_err(SandTablePlayer::previous(), "Navigation failed", "Not in playlist mode");
}

OpResult set_shuffle(bool shuffle) {
    ESP_LOGI(TAG, "Shuffle %s", shuffle ? "on" : "off");
    return from_ctrl_err(SandTablePlayer::setShuffle(shuffle), "Failed to set shuffle", "Player not initialized");
}

OpResult set_loop(bool loop) {
    ESP_LOGI(TAG, "Loop %s", loop ? "on" : "off");
    return from_ctrl_err(SandTablePlayer::setLoop(loop), "Failed to set loop", "Player not initialized");
}

OpResult set_random_loop(bool enabled) {
    ESP_LOGI(TAG, "Random loop %s", enabled ? "on" : "off");
    esp_err_t err = SandTablePlayer::setRandomLoop(enabled);
    switch (err) {
        case ESP_OK: return OpResult::success();
        case ESP_ERR_NOT_FOUND: return OpResult::fail(err, "No playable patterns");
        case ESP_ERR_INVALID_STATE: return OpResult::fail(err, "Not homed or not initialized");
        case ESP_ERR_TIMEOUT: return OpResult::fail(err, "Player busy");
        default: return OpResult::fail(err, "Failed to set random loop");
    }
}

OpResult set_feed_rate(double feed_rate) {
    if (!std::isfinite(feed_rate)) {
        return OpResult::fail(ESP_ERR_INVALID_ARG, "feed_rate must be a finite number");
    }
    if (feed_rate < kFeedRateMin) feed_rate = kFeedRateMin;
    if (feed_rate > kFeedRateMax) feed_rate = kFeedRateMax;
    ESP_LOGI(TAG, "Feed rate %.2f", feed_rate);
    SandTablePlayer::setFeedRate(feed_rate);
    return OpResult::success();
}

PlaybackStatus snapshot() {
    return SandTablePlayer::getStatus();
}

void to_json(api_common::ChunkBuf* cb, const PlaybackStatus& s) {
    using namespace api_common;
    // Field names follow the documented PlayerState schema in docs/swagger.json.
    chunk_buf_printf(cb, "{\"state\":\"%s\",\"mode\":\"%s\",\"current_pattern_uuid\":",
        state_name(s.state), mode_name(s.mode));
    chunk_buf_write_json_string(cb, s.current_pattern_uuid.c_str());
    chunk_buf_write_str(cb, ",\"current_playlist_uuid\":");
    chunk_buf_write_json_string(cb, s.current_playlist_uuid.c_str());
    chunk_buf_printf(cb,
        ",\"progress_percent\":%d,\"pattern_index\":%lu,\"playlist_size\":%lu,"
        "\"feed_rate\":%g,\"shuffle\":%s,\"loop\":%s}",
        s.progress_percent,
        static_cast<unsigned long>(s.pattern_index),
        static_cast<unsigned long>(s.playlist_size),
        s.feed_rate,
        s.is_shuffle ? "true" : "false",
        s.is_loop ? "true" : "false");
}

ResponseMessage state_to_proto(const PlaybackStatus& status, const char* request_id) {
    // Locals, not statics: this runs concurrently from the broadcaster timer,
    // httpd, and the cloudlink task.
    Kd__V1__PlayerState state = KD__V1__PLAYER_STATE__INIT;
    char pattern_uuid[MAX_UUID_LEN] = {0};
    char playlist_uuid[MAX_UUID_LEN] = {0};

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
        case PlayMode::RANDOM_LOOP:
            state.mode = KD__V1__PLAYER_STATE__PLAY_MODE__PLAY_MODE_RANDOM_LOOP;
            break;
    }

    strncpy(pattern_uuid, status.current_pattern_uuid.c_str(), sizeof(pattern_uuid) - 1);
    strncpy(playlist_uuid, status.current_playlist_uuid.c_str(), sizeof(playlist_uuid) - 1);

    state.current_pattern_uuid = pattern_uuid;
    state.current_playlist_uuid = playlist_uuid;
    state.progress_percent = status.progress_percent;
    state.pattern_index = status.pattern_index;
    state.playlist_size = status.playlist_size;
    state.feed_rate = static_cast<float>(status.feed_rate);
    state.shuffle = status.is_shuffle;
    state.loop = status.is_loop;

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYER_STATE;
    resp.player_state = &state;

    return pack_tranquil_message(&resp, request_id);
}

ResponseMessage build_state_message(const char* request_id) {
    PlaybackStatus s = snapshot();
    return state_to_proto(s, request_id);
}

}  // namespace player_ops
