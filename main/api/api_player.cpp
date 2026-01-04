#include "api_player.h"
#include "SandTablePlayer.h"
#include "esp_log.h"
#include "cJSON.h"
#include "esp_http_server.h"
#include <cstring>

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
// Supports: { "uuid": "...", "type": "pattern"|"playlist", "shuffle": bool, "loop": bool }
// For backward compatibility, defaults to pattern if type is not specified
static esp_err_t handle_play(httpd_req_t* req) {
    char buf[256];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) return ESP_FAIL;
    buf[len] = '\0';

    cJSON* root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON* uuid = cJSON_GetObjectItem(root, "uuid");
    if (!uuid || !cJSON_IsString(uuid)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing uuid");
        return ESP_FAIL;
    }

    // Check type: "pattern" or "playlist"
    cJSON* type = cJSON_GetObjectItem(root, "type");
    const char* type_str = "pattern";  // Default to pattern for backward compat
    if (type && cJSON_IsString(type)) {
        type_str = type->valuestring;
    }

    esp_err_t ret;
    if (strcmp(type_str, "playlist") == 0) {
        // Playlist playback
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

        ret = SandTablePlayer::playPlaylist(uuid->valuestring, shuffle, loop);
    } else {
        // Pattern playback
        ret = SandTablePlayer::playPattern(uuid->valuestring);
    }

    cJSON_Delete(root);

    if (ret != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    return handle_get_state(req);
}

// POST /api/player/pause - pause
static esp_err_t handle_pause(httpd_req_t* req) {
    SandTablePlayer::pause();
    return handle_get_state(req);
}

// POST /api/player/resume - resume
static esp_err_t handle_resume(httpd_req_t* req) {
    SandTablePlayer::resume();
    return handle_get_state(req);
}

// POST /api/player/stop - stop
static esp_err_t handle_stop(httpd_req_t* req) {
    SandTablePlayer::stop();
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

// POST /api/player/speed - set feed rate { "speed": ... }
static esp_err_t handle_speed(httpd_req_t* req) {
    char buf[64];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) return ESP_FAIL;
    buf[len] = '\0';

    cJSON* root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON* speed = cJSON_GetObjectItem(root, "speed");
    if (!speed || !cJSON_IsNumber(speed)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing speed");
        return ESP_FAIL;
    }

    SandTablePlayer::setFeedRate(speed->valuedouble);
    cJSON_Delete(root);

    return handle_get_state(req);
}

// POST /api/player/shuffle - set shuffle mode { "shuffle": bool }
static esp_err_t handle_shuffle(httpd_req_t* req) {
    char buf[64];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) return ESP_FAIL;
    buf[len] = '\0';

    cJSON* root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON* shuffle = cJSON_GetObjectItem(root, "shuffle");
    if (!shuffle || !cJSON_IsBool(shuffle)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing shuffle");
        return ESP_FAIL;
    }

    SandTablePlayer::setShuffle(cJSON_IsTrue(shuffle));
    cJSON_Delete(root);

    return handle_get_state(req);
}

// POST /api/player/loop - set loop mode { "loop": bool }
static esp_err_t handle_loop(httpd_req_t* req) {
    char buf[64];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) return ESP_FAIL;
    buf[len] = '\0';

    cJSON* root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON* loop = cJSON_GetObjectItem(root, "loop");
    if (!loop || !cJSON_IsBool(loop)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing loop");
        return ESP_FAIL;
    }

    SandTablePlayer::setLoop(cJSON_IsTrue(loop));
    cJSON_Delete(root);

    return handle_get_state(req);
}

// URI handler definitions
static httpd_uri_t player_state_uri = {
    .uri = "/api/player",
    .method = HTTP_GET,
    .handler = handle_get_state,
    .user_ctx = NULL
};
static httpd_uri_t player_play_uri = {
    .uri = "/api/player/play",
    .method = HTTP_POST,
    .handler = handle_play,
    .user_ctx = NULL
};
static httpd_uri_t player_pause_uri = {
    .uri = "/api/player/pause",
    .method = HTTP_POST,
    .handler = handle_pause,
    .user_ctx = NULL
};
static httpd_uri_t player_resume_uri = {
    .uri = "/api/player/resume",
    .method = HTTP_POST,
    .handler = handle_resume,
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
static httpd_uri_t player_speed_uri = {
    .uri = "/api/player/speed",
    .method = HTTP_POST,
    .handler = handle_speed,
    .user_ctx = NULL
};
static httpd_uri_t player_shuffle_uri = {
    .uri = "/api/player/shuffle",
    .method = HTTP_POST,
    .handler = handle_shuffle,
    .user_ctx = NULL
};
static httpd_uri_t player_loop_uri = {
    .uri = "/api/player/loop",
    .method = HTTP_POST,
    .handler = handle_loop,
    .user_ctx = NULL
};

void api_player_register_endpoints(httpd_handle_t server) {
    httpd_register_uri_handler(server, &player_state_uri);
    httpd_register_uri_handler(server, &player_play_uri);
    httpd_register_uri_handler(server, &player_pause_uri);
    httpd_register_uri_handler(server, &player_resume_uri);
    httpd_register_uri_handler(server, &player_stop_uri);
    httpd_register_uri_handler(server, &player_skip_uri);
    httpd_register_uri_handler(server, &player_speed_uri);
    httpd_register_uri_handler(server, &player_shuffle_uri);
    httpd_register_uri_handler(server, &player_loop_uri);
}
