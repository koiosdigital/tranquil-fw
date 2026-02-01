#include "proto_handler.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "kd_common.h"
#include "SandTablePlayer.h"
#include "ManifestDatabase.h"
#include <cstring>
#include <cstdlib>

// Include generated protobuf-c headers
#include "kd/v1/tranquil.pb-c.h"
#include "kd/v1/common.pb-c.h"

static const char* TAG = "proto_handler";

// Helper to create a CommandResult response
static void make_command_result(Kd__V1__TranquilMessage* resp, bool success, const char* detail = nullptr) {
    static Kd__V1__CommandResult result = KD__V1__COMMAND_RESULT__INIT;
    result.success = success;
    result.error_code = success ? 0 : -1;
    result.detail = detail ? const_cast<char*>(detail) : nullptr;

    resp->message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_COMMAND_RESULT;
    resp->command_result = &result;
}

esp_err_t proto_handle_message(const uint8_t* data, size_t len,
                                uint8_t** response, size_t* response_len) {
    *response = nullptr;
    *response_len = 0;

    // Decode incoming TranquilMessage
    Kd__V1__TranquilMessage* msg = kd__v1__tranquil_message__unpack(nullptr, len, data);
    if (!msg) {
        ESP_LOGE(TAG, "Failed to unpack TranquilMessage");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Received message type: %d", msg->message_case);

    // Prepare response
    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    bool has_response = false;

    switch (msg->message_case) {
        // === Ping/Pong ===
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PING: {
            static Kd__V1__Pong pong = KD__V1__PONG__INIT;
            pong.timestamp = msg->ping ? msg->ping->timestamp : 0;
            resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_PONG;
            resp.pong = &pong;
            has_response = true;
            break;
        }

        // === Player Control ===
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAY_PATTERN: {
            if (msg->play_pattern && msg->play_pattern->pattern_uuid) {
                esp_err_t ret = SandTablePlayer::playPattern(msg->play_pattern->pattern_uuid);
                make_command_result(&resp, ret == ESP_OK,
                    ret == ESP_OK ? nullptr : "Failed to play pattern");
            } else {
                make_command_result(&resp, false, "Missing pattern_uuid");
            }
            has_response = true;
            break;
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_PAUSED: {
            if (msg->set_paused) {
                esp_err_t ret = msg->set_paused->paused ?
                    SandTablePlayer::pause() : SandTablePlayer::resume();
                make_command_result(&resp, ret == ESP_OK);
            } else {
                make_command_result(&resp, false, "Invalid message");
            }
            has_response = true;
            break;
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_STOP_PLAYBACK: {
            esp_err_t ret = SandTablePlayer::stop();
            make_command_result(&resp, ret == ESP_OK);
            has_response = true;
            break;
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLIST_PLAY: {
            if (msg->playlist_play && msg->playlist_play->playlist_uuid) {
                esp_err_t ret = SandTablePlayer::playPlaylist(
                    msg->playlist_play->playlist_uuid,
                    msg->playlist_play->shuffle,
                    msg->playlist_play->loop
                );
                make_command_result(&resp, ret == ESP_OK,
                    ret == ESP_OK ? nullptr : "Failed to play playlist");
            } else {
                make_command_result(&resp, false, "Missing playlist_uuid");
            }
            has_response = true;
            break;
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_SHUFFLE: {
            if (msg->set_shuffle) {
                esp_err_t ret = SandTablePlayer::setShuffle(msg->set_shuffle->shuffle);
                make_command_result(&resp, ret == ESP_OK);
            } else {
                make_command_result(&resp, false, "Invalid message");
            }
            has_response = true;
            break;
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_LOOP: {
            if (msg->set_loop) {
                esp_err_t ret = SandTablePlayer::setLoop(msg->set_loop->enabled);
                make_command_result(&resp, ret == ESP_OK);
            } else {
                make_command_result(&resp, false, "Invalid message");
            }
            has_response = true;
            break;
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_FEED_RATE: {
            if (msg->set_feed_rate) {
                SandTablePlayer::setFeedRate(msg->set_feed_rate->feed_rate_rpm);
                make_command_result(&resp, true);
            } else {
                make_command_result(&resp, false, "Invalid message");
            }
            has_response = true;
            break;
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYLIST_NAVIGATE_COMMAND: {
            if (msg->playlist_navigate_command) {
                esp_err_t ret = ESP_OK;
                if (msg->playlist_navigate_command->direction == KD__V1__PLAYLIST_NAVIGATE_COMMAND__DIRECTION__DIRECTION_NEXT) {
                    ret = SandTablePlayer::skip();
                }
                // DIRECTION_PREVIOUS not implemented yet
                make_command_result(&resp, ret == ESP_OK);
            } else {
                make_command_result(&resp, false, "Invalid message");
            }
            has_response = true;
            break;
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_PLAYER_STATE: {
            static Kd__V1__PlayerState state = KD__V1__PLAYER_STATE__INIT;
            static char pattern_uuid[64] = {0};
            static char playlist_uuid[64] = {0};

            PlaybackStatus status = SandTablePlayer::getStatus();

            // Map enum values
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
            }

            strncpy(pattern_uuid, status.current_pattern_uuid.c_str(), sizeof(pattern_uuid) - 1);
            strncpy(playlist_uuid, status.current_playlist_uuid.c_str(), sizeof(playlist_uuid) - 1);

            state.current_pattern_uuid = pattern_uuid;
            state.current_playlist_uuid = playlist_uuid;
            state.progress_percent = status.progress_percent;
            state.pattern_index = status.pattern_index;
            state.playlist_size = status.playlist_size;
            state.feed_rate = status.feed_rate;
            state.shuffle = status.is_shuffle;
            state.loop = status.is_loop;

            resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_PLAYER_STATE;
            resp.player_state = &state;
            has_response = true;
            break;
        }

        // === Pattern Management ===
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_DOWNLOADED_PATTERNS_REQUEST: {
            // Return list of all patterns
            auto patterns = ManifestDatabase::instance().getAllPatterns();

            static Kd__V1__DownloadedPatterns downloaded = KD__V1__DOWNLOADED_PATTERNS__INIT;
            static std::vector<Kd__V1__PatternInfo*> pattern_ptrs;
            static std::vector<Kd__V1__PatternInfo> pattern_infos;

            pattern_ptrs.clear();
            pattern_infos.clear();
            pattern_infos.resize(patterns.size());

            for (size_t i = 0; i < patterns.size() && i < 100; i++) {
                pattern_infos[i] = KD__V1__PATTERN_INFO__INIT;
                // Note: These strings point to Pattern member data which stays valid
                // during this function call since patterns vector is on stack
                pattern_infos[i].uuid = const_cast<char*>(patterns[i].uuid.c_str());
                pattern_infos[i].name = const_cast<char*>(patterns[i].name.c_str());
                pattern_infos[i].creator = const_cast<char*>(patterns[i].creator.c_str());
                pattern_infos[i].encrypted = patterns[i].encrypted;
                pattern_infos[i].size_bytes = patterns[i].size_bytes;
                pattern_infos[i].reversible = patterns[i].reversible;
                pattern_infos[i].start_point = patterns[i].start_point;
                pattern_infos[i].created_at = const_cast<char*>(patterns[i].created_at.c_str());
                pattern_infos[i].last_played_at = const_cast<char*>(patterns[i].last_played_at.c_str());
                pattern_ptrs.push_back(&pattern_infos[i]);
            }

            downloaded.n_patterns = pattern_ptrs.size();
            downloaded.patterns = pattern_ptrs.data();

            resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_DOWNLOADED_PATTERNS;
            resp.downloaded_patterns = &downloaded;
            has_response = true;
            break;
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_DELETE_PATTERN: {
            if (msg->delete_pattern && msg->delete_pattern->uuid) {
                esp_err_t ret = ManifestDatabase::instance().deletePattern(msg->delete_pattern->uuid);
                make_command_result(&resp, ret == ESP_OK,
                    ret == ESP_OK ? nullptr : "Failed to delete pattern");
            } else {
                make_command_result(&resp, false, "Missing uuid");
            }
            has_response = true;
            break;
        }

        // === System ===
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_SYSTEM_INFO: {
            static Kd__V1__SystemInfo info = KD__V1__SYSTEM_INFO__INIT;
            static char model[] = "tranquil";

            const esp_app_desc_t* app_desc = esp_app_get_description();

            info.firmware_version = const_cast<char*>(app_desc->version);
            info.hardware_model = model;
            info.device_id = kd_common_get_device_name();
            info.is_homed = SandTablePlayer::isHomed();
            info.free_heap = esp_get_free_heap_size();
            info.hostname = kd_common_get_wifi_hostname();

            resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_SYSTEM_INFO;
            resp.system_info = &info;
            has_response = true;
            break;
        }

        default:
            ESP_LOGW(TAG, "Unhandled message type: %d", msg->message_case);
            break;
    }

    kd__v1__tranquil_message__free_unpacked(msg, nullptr);

    if (!has_response) {
        return ESP_OK;
    }

    // Encode response
    size_t buf_size = kd__v1__tranquil_message__get_packed_size(&resp);
    uint8_t* buf = static_cast<uint8_t*>(malloc(buf_size));
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate response buffer");
        return ESP_ERR_NO_MEM;
    }

    size_t packed = kd__v1__tranquil_message__pack(&resp, buf);
    if (packed != buf_size) {
        ESP_LOGE(TAG, "Pack size mismatch: expected %zu, got %zu", buf_size, packed);
        free(buf);
        return ESP_FAIL;
    }

    *response = buf;
    *response_len = packed;

    ESP_LOGI(TAG, "Response: %zu bytes", packed);
    return ESP_OK;
}
