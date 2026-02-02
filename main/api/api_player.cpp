#include "api_player.h"
#include "SandTablePlayer.h"
#include "esp_log.h"
#include "cJSON.h"
#include "esp_http_server.h"
#include <cstring>

static const char* TAG = "api_player";

// Helper: get state as cJSON
cJSON* api_player_get_state_json() {
    return SandTablePlayer::getStateJSON();
}

// GET /api/player - get state
static esp_err_t handle_get_state(httpd_req_t* req) {
    cJSON* state = api_player_get_state_json();
    char* resp = cJSON_PrintUnformatted(state);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    cJSON_Delete(state);
    free(resp);
    return ESP_OK;
}

// POST /api/player/play - unified play endpoint
// Supports:
//   { "pattern_uuid": "..." } - play single pattern
//   { "playlist_uuid": "...", "shuffle": bool, "loop": bool } - play playlist from start
//   { "playlist_uuid": "...", "pattern_uuid": "...", "shuffle": bool, "loop": bool } - play playlist from pattern
static esp_err_t handle_play(httpd_req_t* req) {
    char buf[512];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON* root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON* pattern_uuid = cJSON_GetObjectItem(root, "pattern_uuid");
    cJSON* playlist_uuid = cJSON_GetObjectItem(root, "playlist_uuid");

    bool has_pattern = pattern_uuid && cJSON_IsString(pattern_uuid) && strlen(pattern_uuid->valuestring) > 0;
    bool has_playlist = playlist_uuid && cJSON_IsString(playlist_uuid) && strlen(playlist_uuid->valuestring) > 0;

    if (!has_pattern && !has_playlist) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing pattern_uuid or playlist_uuid");
        return ESP_FAIL;
    }

    // Get optional flags
    bool shuffle = false;
    bool loop = false;
    cJSON* shuffle_item = cJSON_GetObjectItem(root, "shuffle");
    if (shuffle_item && cJSON_IsBool(shuffle_item)) {
        shuffle = cJSON_IsTrue(shuffle_item);
    }
    cJSON* loop_item = cJSON_GetObjectItem(root, "loop");
    if (loop_item && cJSON_IsBool(loop_item)) {
        loop = cJSON_IsTrue(loop_item);
    }

    esp_err_t ret;

    if (has_playlist && has_pattern) {
        // Playlist with pattern: start playlist from specific pattern
        ret = SandTablePlayer::playPlaylistFromPattern(
            playlist_uuid->valuestring, pattern_uuid->valuestring, shuffle, loop);
    } else if (has_playlist) {
        // Playlist only: start from beginning (or shuffle if enabled)
        ret = SandTablePlayer::playPlaylist(playlist_uuid->valuestring, shuffle, loop);
    } else {
        // Pattern only: play single pattern
        ret = SandTablePlayer::playPattern(pattern_uuid->valuestring);
    }

    cJSON_Delete(root);

    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NOT_FOUND) {
            httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Pattern or playlist not found");
        } else if (ret == ESP_ERR_INVALID_STATE) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not homed or not initialized");
        } else {
            httpd_resp_send_500(req);
        }
        return ESP_FAIL;
    }

    return handle_get_state(req);
}

// PATCH /api/player - update player state
// Supports (all optional):
//   is_paused: bool - pause/resume
//   loop: bool - set loop mode
//   shuffle: bool - set shuffle mode
//   feed_rate: number - set feed rate
//   pattern_uuid: string - set playing pattern (stops current, plays new)
//   playlist_uuid: string - set playing playlist (stops current, plays new)
// If both pattern_uuid and playlist_uuid provided, starts playlist from pattern
static esp_err_t handle_patch(httpd_req_t* req) {
    char buf[512];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON* root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_OK;

    // Handle is_paused first (pause/resume)
    cJSON* is_paused = cJSON_GetObjectItem(root, "is_paused");
    if (is_paused && cJSON_IsBool(is_paused)) {
        if (cJSON_IsTrue(is_paused)) {
            ret = SandTablePlayer::pause();
        } else {
            ret = SandTablePlayer::resume();
        }
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set paused state");
        }
    }

    // Handle loop
    cJSON* loop = cJSON_GetObjectItem(root, "loop");
    if (loop && cJSON_IsBool(loop)) {
        ret = SandTablePlayer::setLoop(cJSON_IsTrue(loop));
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set loop");
        }
    }

    // Handle shuffle
    cJSON* shuffle = cJSON_GetObjectItem(root, "shuffle");
    if (shuffle && cJSON_IsBool(shuffle)) {
        ret = SandTablePlayer::setShuffle(cJSON_IsTrue(shuffle));
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set shuffle");
        }
    }

    // Handle feed_rate
    cJSON* feed_rate = cJSON_GetObjectItem(root, "feed_rate");
    if (feed_rate && cJSON_IsNumber(feed_rate)) {
        SandTablePlayer::setFeedRate(feed_rate->valuedouble);
    }

    // Handle pattern/playlist changes (these start new playback)
    cJSON* pattern_uuid = cJSON_GetObjectItem(root, "pattern_uuid");
    cJSON* playlist_uuid = cJSON_GetObjectItem(root, "playlist_uuid");

    bool has_pattern = pattern_uuid && cJSON_IsString(pattern_uuid) && strlen(pattern_uuid->valuestring) > 0;
    bool has_playlist = playlist_uuid && cJSON_IsString(playlist_uuid) && strlen(playlist_uuid->valuestring) > 0;

    if (has_pattern || has_playlist) {
        // Get current shuffle/loop state for new playback
        bool cur_shuffle = SandTablePlayer::isShuffle();
        bool cur_loop = SandTablePlayer::isLoop();

        // Use values from request if provided, otherwise use current
        if (shuffle && cJSON_IsBool(shuffle)) {
            cur_shuffle = cJSON_IsTrue(shuffle);
        }
        if (loop && cJSON_IsBool(loop)) {
            cur_loop = cJSON_IsTrue(loop);
        }

        if (has_playlist && has_pattern) {
            ret = SandTablePlayer::playPlaylistFromPattern(
                playlist_uuid->valuestring, pattern_uuid->valuestring, cur_shuffle, cur_loop);
        } else if (has_playlist) {
            ret = SandTablePlayer::playPlaylist(playlist_uuid->valuestring, cur_shuffle, cur_loop);
        } else {
            ret = SandTablePlayer::playPattern(pattern_uuid->valuestring);
        }

        if (ret != ESP_OK) {
            cJSON_Delete(root);
            if (ret == ESP_ERR_NOT_FOUND) {
                httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Pattern or playlist not found");
            } else if (ret == ESP_ERR_INVALID_STATE) {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not homed or not initialized");
            } else {
                httpd_resp_send_500(req);
            }
            return ESP_FAIL;
        }
    }

    cJSON_Delete(root);
    return handle_get_state(req);
}

// POST /api/player/stop - stop playback
// Supports optional: { "emergency_stop": bool }
static esp_err_t handle_stop(httpd_req_t* req) {
    bool emergency = false;

    // Check if there's a body with emergency_stop
    int content_len = req->content_len;
    if (content_len > 0) {
        char buf[64];
        int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
        if (len > 0) {
            buf[len] = '\0';
            cJSON* root = cJSON_Parse(buf);
            if (root) {
                cJSON* emergency_item = cJSON_GetObjectItem(root, "emergency_stop");
                if (emergency_item && cJSON_IsBool(emergency_item)) {
                    emergency = cJSON_IsTrue(emergency_item);
                }
                cJSON_Delete(root);
            }
        }
    }

    esp_err_t ret;
    if (emergency) {
        ret = SandTablePlayer::emergencyStop();
    } else {
        ret = SandTablePlayer::stop();
    }

    if (ret != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    return handle_get_state(req);
}

// POST /api/player/skip - skip to next pattern (playlist mode only)
static esp_err_t handle_skip(httpd_req_t* req) {
    esp_err_t ret = SandTablePlayer::skip();
    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not in playlist mode");
        return ESP_FAIL;
    }
    return handle_get_state(req);
}

// URI handler definitions
static httpd_uri_t player_state_uri = {
    .uri = "/api/player",
    .method = HTTP_GET,
    .handler = handle_get_state,
    .user_ctx = NULL
};
static httpd_uri_t player_patch_uri = {
    .uri = "/api/player",
    .method = HTTP_PATCH,
    .handler = handle_patch,
    .user_ctx = NULL
};
static httpd_uri_t player_play_uri = {
    .uri = "/api/player/play",
    .method = HTTP_POST,
    .handler = handle_play,
    .user_ctx = NULL
};
static httpd_uri_t player_stop_uri = {
    .uri = "/api/player/stop",
    .method = HTTP_POST,
    .handler = handle_stop,
    .user_ctx = NULL
};
static httpd_uri_t player_skip_uri = {
    .uri = "/api/player/skip",
    .method = HTTP_POST,
    .handler = handle_skip,
    .user_ctx = NULL
};

void api_player_register_endpoints(httpd_handle_t server) {
    httpd_register_uri_handler(server, &player_state_uri);
    httpd_register_uri_handler(server, &player_patch_uri);
    httpd_register_uri_handler(server, &player_play_uri);
    httpd_register_uri_handler(server, &player_stop_uri);
    httpd_register_uri_handler(server, &player_skip_uri);
}
