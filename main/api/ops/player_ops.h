// Player operations: every playback side effect and its validation, shared
// by the REST endpoints (api_player.cpp) and the WebSocket / cloud handler
// (player_handler.cpp). Transports only parse their wire format into these
// calls and serialise the result.
#pragma once

#include "op_result.h"
#include "api_common.h"
#include "handler_base.h"
#include "SandTablePlayer.h"

namespace player_ops {

constexpr double kFeedRateMin = 1.0;
constexpr double kFeedRateMax = 5.0;

// One play request shape for every transport. Precedence: random, then
// playlist (+ optional start pattern), then a single pattern.
struct PlayRequest {
    const char* pattern_uuid = nullptr;    // nullptr or "" = none
    const char* playlist_uuid = nullptr;
    bool random = false;
    bool shuffle = false;
    bool loop = false;
};

OpResult play(const PlayRequest& req);
OpResult play_pattern(const char* pattern_uuid);
OpResult play_playlist(const char* playlist_uuid, bool shuffle, bool loop);
OpResult play_playlist_from_pattern(const char* playlist_uuid, const char* pattern_uuid, bool shuffle, bool loop);
OpResult play_random(bool loop);

OpResult set_paused(bool paused);
// emergency = immediate halt without deceleration or state cleanup
OpResult stop(bool emergency);
OpResult next();
OpResult previous();
OpResult set_shuffle(bool shuffle);
OpResult set_loop(bool loop);
OpResult set_random_loop(bool enabled);
// Clamped to [kFeedRateMin, kFeedRateMax]; non-finite values are rejected.
OpResult set_feed_rate(double feed_rate);

// ---- State snapshot + serialisers ----
PlaybackStatus snapshot();

// REST shape (docs/swagger.json PlayerState).
void to_json(api_common::ChunkBuf* cb, const PlaybackStatus& s);

// TranquilMessage{player_state}. `request_id` is echoed on the envelope
// when replying to a GetPlayerState.
ResponseMessage state_to_proto(const PlaybackStatus& s, const char* request_id = nullptr);
ResponseMessage build_state_message(const char* request_id = nullptr);

}  // namespace player_ops
