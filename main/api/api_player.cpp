// Player REST API: a thin JSON codec over player_ops.
#include "api_player.h"
#include "api_common.h"
#include "ops/player_ops.h"

#include "esp_log.h"
#include "cJSON.h"
#include "esp_http_server.h"
#include <cstdio>
#include <cstring>

static const char* TAG = "api_player";

using namespace api_common;

namespace {

bool has(const char* s) { return s && s[0] != '\0'; }

// Shared response for GET/PATCH/play/stop/skip/previous.
esp_err_t send_state(httpd_req_t* req) {
    PlaybackStatus s = player_ops::snapshot();
    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);
    player_ops::to_json(&cb, s);
    chunk_buf_finish(&cb);
    return ESP_OK;
}

const char* json_str(const cJSON* obj, const char* key) {
    const cJSON* v = cJSON_GetObjectItem(obj, key);
    return (v && cJSON_IsString(v)) ? v->valuestring : nullptr;
}

// GET /api/player - get state
esp_err_t handle_get_state(httpd_req_t* req) {
    return send_state(req);
}

// POST /api/player/play - unified play endpoint
// Supports:
//   { "pattern_uuid": "..." } - play single pattern
//   { "playlist_uuid": "...", "shuffle": bool, "loop": bool } - play playlist from start
//   { "playlist_uuid": "...", "pattern_uuid": "...", "shuffle": bool, "loop": bool } - play playlist from pattern
//   { "random": true, "loop": bool } - play a random library pattern; loop = keep chaining random patterns
esp_err_t handle_play(httpd_req_t* req) {
    // Full body read (a truncated single recv left the remainder to be
    // parsed as the NEXT request on the keep-alive connection).
    cJSON* root = parse_json_body(req, 2048);
    if (!root) return ESP_FAIL;

    player_ops::PlayRequest r;
    r.pattern_uuid = json_str(root, "pattern_uuid");
    r.playlist_uuid = json_str(root, "playlist_uuid");
    const cJSON* random_item = cJSON_GetObjectItem(root, "random");
    r.random = random_item && cJSON_IsBool(random_item) && cJSON_IsTrue(random_item);
    const cJSON* shuffle_item = cJSON_GetObjectItem(root, "shuffle");
    r.shuffle = shuffle_item && cJSON_IsBool(shuffle_item) && cJSON_IsTrue(shuffle_item);
    const cJSON* loop_item = cJSON_GetObjectItem(root, "loop");
    r.loop = loop_item && cJSON_IsBool(loop_item) && cJSON_IsTrue(loop_item);

    OpResult res = player_ops::play(r);
    cJSON_Delete(root);

    if (!res.ok()) return send_op_error(req, res);
    return send_state(req);
}

// PATCH /api/player - update player state
// Supports (all optional):
//   is_paused: bool - pause/resume
//   loop: bool - set loop mode
//   shuffle: bool - set shuffle mode
//   feed_rate: number - set feed rate
//   random_loop: bool - random-loop mode (on: chain random patterns after the current one, or start one if idle; off: finish the current pattern then stop)
//   pattern_uuid: string - set playing pattern (stops current, plays new)
//   playlist_uuid: string - set playing playlist (stops current, plays new)
// If both pattern_uuid and playlist_uuid provided, starts playlist from pattern.
// Every present field is applied; the first failure names its field in the
// error body.
esp_err_t handle_patch(httpd_req_t* req) {
    cJSON* root = parse_json_body(req, 2048);
    if (!root) return ESP_FAIL;

    const char* failed_field = nullptr;
    OpResult failure;
    auto note = [&](const char* field, const OpResult& r) {
        if (!r.ok()) {
            ESP_LOGW(TAG, "PATCH %s failed: %s", field, r.detail);
            if (!failed_field) {
                failed_field = field;
                failure = r;
            }
        }
    };

    const cJSON* is_paused = cJSON_GetObjectItem(root, "is_paused");
    if (is_paused && cJSON_IsBool(is_paused)) {
        note("is_paused", player_ops::set_paused(cJSON_IsTrue(is_paused)));
    }

    const cJSON* loop = cJSON_GetObjectItem(root, "loop");
    if (loop && cJSON_IsBool(loop)) {
        note("loop", player_ops::set_loop(cJSON_IsTrue(loop)));
    }

    const cJSON* shuffle = cJSON_GetObjectItem(root, "shuffle");
    if (shuffle && cJSON_IsBool(shuffle)) {
        note("shuffle", player_ops::set_shuffle(cJSON_IsTrue(shuffle)));
    }

    const cJSON* feed_rate = cJSON_GetObjectItem(root, "feed_rate");
    if (feed_rate && cJSON_IsNumber(feed_rate)) {
        note("feed_rate", player_ops::set_feed_rate(feed_rate->valuedouble));
    }

    const cJSON* random_loop = cJSON_GetObjectItem(root, "random_loop");
    if (random_loop && cJSON_IsBool(random_loop)) {
        note("random_loop", player_ops::set_random_loop(cJSON_IsTrue(random_loop)));
    }

    // Pattern/playlist changes start new playback with the request's
    // shuffle/loop when given, else the player's current settings.
    const char* pattern_uuid = json_str(root, "pattern_uuid");
    const char* playlist_uuid = json_str(root, "playlist_uuid");
    if (has(pattern_uuid) || has(playlist_uuid)) {
        player_ops::PlayRequest r;
        r.pattern_uuid = pattern_uuid;
        r.playlist_uuid = playlist_uuid;
        r.shuffle = (shuffle && cJSON_IsBool(shuffle)) ? cJSON_IsTrue(shuffle) : SandTablePlayer::isShuffle();
        r.loop = (loop && cJSON_IsBool(loop)) ? cJSON_IsTrue(loop) : SandTablePlayer::isLoop();
        note(has(playlist_uuid) ? "playlist_uuid" : "pattern_uuid", player_ops::play(r));
    }

    cJSON_Delete(root);

    if (failed_field) {
        // The detail buffer only has to outlive this synchronous send.
        char detail[96];
        snprintf(detail, sizeof(detail), "%s: %s", failed_field, failure.detail);
        return send_op_error(req, OpResult{ failure.err, detail });
    }

    return send_state(req);
}

// POST /api/player/stop - stop playback
// Supports optional: { "emergency_stop": bool }
esp_err_t handle_stop(httpd_req_t* req) {
    bool emergency = false;

    if (req->content_len > 0) {
        cJSON* root = parse_json_body(req, 512);
        if (!root) return ESP_FAIL;
        const cJSON* emergency_item = cJSON_GetObjectItem(root, "emergency_stop");
        emergency = emergency_item && cJSON_IsBool(emergency_item) && cJSON_IsTrue(emergency_item);
        cJSON_Delete(root);
    }

    OpResult r = player_ops::stop(emergency);
    if (!r.ok()) return send_op_error(req, r);
    return send_state(req);
}

// POST /api/player/skip - next pattern (playlist / random-loop mode only)
esp_err_t handle_skip(httpd_req_t* req) {
    OpResult r = player_ops::next();
    if (!r.ok()) return send_op_error(req, r);
    return send_state(req);
}

// POST /api/player/previous - previous pattern (playlist mode only)
esp_err_t handle_previous(httpd_req_t* req) {
    OpResult r = player_ops::previous();
    if (!r.ok()) return send_op_error(req, r);
    return send_state(req);
}

// URI handler definitions
httpd_uri_t player_state_uri = {
    .uri = "/api/player",
    .method = HTTP_GET,
    .handler = handle_get_state,
    .user_ctx = NULL
};
httpd_uri_t player_patch_uri = {
    .uri = "/api/player",
    .method = HTTP_PATCH,
    .handler = handle_patch,
    .user_ctx = NULL
};
httpd_uri_t player_play_uri = {
    .uri = "/api/player/play",
    .method = HTTP_POST,
    .handler = handle_play,
    .user_ctx = NULL
};
httpd_uri_t player_stop_uri = {
    .uri = "/api/player/stop",
    .method = HTTP_POST,
    .handler = handle_stop,
    .user_ctx = NULL
};
httpd_uri_t player_skip_uri = {
    .uri = "/api/player/skip",
    .method = HTTP_POST,
    .handler = handle_skip,
    .user_ctx = NULL
};
httpd_uri_t player_previous_uri = {
    .uri = "/api/player/previous",
    .method = HTTP_POST,
    .handler = handle_previous,
    .user_ctx = NULL
};

}  // namespace

void api_player_register_endpoints(httpd_handle_t server) {
    kd_common_api_register_uri_handler(server, &player_state_uri);
    kd_common_api_register_uri_handler(server, &player_patch_uri);
    kd_common_api_register_uri_handler(server, &player_play_uri);
    kd_common_api_register_uri_handler(server, &player_stop_uri);
    kd_common_api_register_uri_handler(server, &player_skip_uri);
    kd_common_api_register_uri_handler(server, &player_previous_uri);
}
