// LED operations implementation
#include "led_ops.h"

#include <esp_log.h>

static const char* TAG = "led_ops";

namespace led_ops {

OpResult set_channel(uint32_t index, const led_state::ChannelPatch& patch) {
    if (!led_state::apply_channel_patch(index, patch)) {
        return OpResult::fail(ESP_ERR_NOT_FOUND, "Channel not found");
    }
    ESP_LOGD(TAG, "Channel %lu updated", static_cast<unsigned long>(index));
    return OpResult::success();
}

std::vector<PixelEffectEngine::EffectInfo> effects() {
    PixelEffectEngine* engine = PixelDriver::getEffectEngine();
    if (!engine) return {};
    return engine->getAllEffects();
}

void effects_to_json(api_common::ChunkBuf* cb) {
    using namespace api_common;
    chunk_buf_write(cb, "[", 1);
    bool first = true;
    for (const auto& eff : effects()) {
        if (!first) chunk_buf_write(cb, ",", 1);
        chunk_buf_write_str(cb, "{\"id\":");
        chunk_buf_write_json_string(cb, eff.id.c_str());
        chunk_buf_write_str(cb, ",\"name\":");
        chunk_buf_write_json_string(cb, eff.display_name.c_str());
        chunk_buf_write(cb, "}", 1);
        first = false;
    }
    chunk_buf_write(cb, "]", 1);
}

ResponseMessage build_effects_message(const char* request_id) {
    std::vector<PixelEffectEngine::EffectInfo> list = effects();

    // Locals, not statics: concurrent handler tasks resizing shared static
    // vectors double-free their backing stores.
    Kd__V1__LEDEffectsList effects_list = KD__V1__LEDEFFECTS_LIST__INIT;
    std::vector<Kd__V1__LEDEffectInfo> effect_infos(list.size());
    std::vector<Kd__V1__LEDEffectInfo*> effect_ptrs(list.size());

    for (size_t i = 0; i < list.size(); i++) {
        effect_infos[i] = KD__V1__LEDEFFECT_INFO__INIT;
        effect_infos[i].id = const_cast<char*>(list[i].id.c_str());
        effect_infos[i].name = const_cast<char*>(list[i].display_name.c_str());
        effect_ptrs[i] = &effect_infos[i];
    }

    effects_list.n_effects = effect_ptrs.size();
    effects_list.effects = effect_ptrs.data();

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_LED_EFFECTS_LIST;
    resp.led_effects_list = &effects_list;

    return pack_tranquil_message(&resp, request_id);
}

}  // namespace led_ops
