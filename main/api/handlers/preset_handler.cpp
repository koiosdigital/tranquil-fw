// Unified preset handler implementation (local-only)
#include "preset_handler.h"
#include "config_manager.h"

#include <esp_log.h>

static const char* TAG = "preset_handler";

PresetHandler& PresetHandler::instance() {
    static PresetHandler instance;
    return instance;
}

HandleResult PresetHandler::handle(
    const Kd__V1__TranquilMessage* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    // Preset messages are local-only
    if (!ctx.isLocal()) {
        ESP_LOGW(TAG, "Preset message from non-local source, ignoring");
        return HandleResult::notHandled();
    }

    switch (msg->message_case) {
    case KD__V1__TRANQUIL_MESSAGE__MESSAGE_LIST_PRESETS_REQUEST:
        return handleListPresets(response);

    case KD__V1__TRANQUIL_MESSAGE__MESSAGE_LOAD_PRESET_REQUEST:
        return handleLoadPreset(msg->load_preset_request, response);

    default:
        return HandleResult::notHandled();
    }
}

bool PresetHandler::canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const {
    switch (msg_case) {
    case KD__V1__TRANQUIL_MESSAGE__MESSAGE_LIST_PRESETS_REQUEST:
    case KD__V1__TRANQUIL_MESSAGE__MESSAGE_LOAD_PRESET_REQUEST:
        return true;
    default:
        return false;
    }
}

std::vector<Kd__V1__TranquilMessage__MessageCase> PresetHandler::supportedMessages() const {
    return {
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_LIST_PRESETS_REQUEST,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_LOAD_PRESET_REQUEST
    };
}

HandleResult PresetHandler::handleListPresets(ResponseMessage& response) {
    auto& cfg = sand_table::ConfigManager::instance();
    auto presets = cfg.list_presets();

    ESP_LOGI(TAG, "ListPresets: returning %zu presets", presets.size());

    // Build response
    static Kd__V1__ListPresetsResponse list_resp = KD__V1__LIST_PRESETS_RESPONSE__INIT;
    static std::vector<Kd__V1__PresetInfoProto*> preset_ptrs;
    static std::vector<Kd__V1__PresetInfoProto> preset_infos;
    static char active_id[32] = { 0 };

    preset_ptrs.clear();
    preset_infos.clear();
    preset_infos.resize(presets.size());

    for (size_t i = 0; i < presets.size(); i++) {
        preset_infos[i] = KD__V1__PRESET_INFO_PROTO__INIT;
        preset_infos[i].id = const_cast<char*>(presets[i].id);
        preset_infos[i].name = const_cast<char*>(presets[i].name);
        preset_infos[i].description = const_cast<char*>(presets[i].description);
        preset_ptrs.push_back(&preset_infos[i]);
    }

    strncpy(active_id, cfg.active_preset_id(), sizeof(active_id));

    list_resp.n_presets = preset_ptrs.size();
    list_resp.presets = preset_ptrs.data();
    list_resp.active_preset_id = active_id;

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_LIST_PRESETS_RESPONSE;
    resp.list_presets_response = &list_resp;

    response = serialize(&resp);
    return HandleResult::ok();
}

HandleResult PresetHandler::handleLoadPreset(
    const Kd__V1__LoadPresetRequest* msg,
    ResponseMessage& response
) {
    if (!msg || !msg->preset_id) {
        static Kd__V1__LoadPresetResponse load_resp = KD__V1__LOAD_PRESET_RESPONSE__INIT;
        load_resp.success = false;
        load_resp.error = const_cast<char*>("Missing preset_id");

        Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
        resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_LOAD_PRESET_RESPONSE;
        resp.load_preset_response = &load_resp;

        response = serialize(&resp);
        return HandleResult::ok();
    }

    auto& cfg = sand_table::ConfigManager::instance();
    esp_err_t ret = cfg.load_preset(msg->preset_id);

    ESP_LOGI(TAG, "LoadPreset '%s': %s", msg->preset_id, esp_err_to_name(ret));

    // Build response
    static Kd__V1__LoadPresetResponse load_resp = KD__V1__LOAD_PRESET_RESPONSE__INIT;
    static char error_buf[64] = { 0 };

    load_resp.success = (ret == ESP_OK);
    if (ret != ESP_OK) {
        snprintf(error_buf, sizeof(error_buf), "Failed to load preset: %s", esp_err_to_name(ret));
        load_resp.error = error_buf;
    }
    else {
        load_resp.error = nullptr;
    }

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_LOAD_PRESET_RESPONSE;
    resp.load_preset_response = &load_resp;

    response = serialize(&resp);
    return HandleResult::ok();
}
