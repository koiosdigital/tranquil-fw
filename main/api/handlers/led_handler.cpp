// Unified LED control handler implementation
#include "led_handler.h"
#include "led_state.h"
#include "kd_pixdriver.h"
#include "pixel_effects.h"

#include <esp_log.h>
#include <cstring>

static const char* TAG = "led_handler";

LEDHandler& LEDHandler::instance() {
    static LEDHandler instance;
    return instance;
}

HandleResult LEDHandler::handle(
    const Kd__V1__TranquilMessage* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    (void)ctx;

    switch (msg->message_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_LED_CONFIG_REQUEST:
            return handleLEDConfigRequest(response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_LED_CHANNEL:
            return handleSetLEDChannel(msg->set_led_channel, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_LED_EFFECTS:
            return handleGetLEDEffects(response);

        default:
            return HandleResult::notHandled();
    }
}

bool LEDHandler::canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const {
    switch (msg_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_LED_CONFIG_REQUEST:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_LED_CHANNEL:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_LED_EFFECTS:
            return true;
        default:
            return false;
    }
}

std::vector<Kd__V1__TranquilMessage__MessageCase> LEDHandler::supportedMessages() const {
    return {
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_LED_CONFIG_REQUEST,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_LED_CHANNEL,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_LED_EFFECTS
    };
}

HandleResult LEDHandler::handleLEDConfigRequest(ResponseMessage& response) {
    response = led_state::build_config_message();
    return HandleResult::ok();
}

HandleResult LEDHandler::handleSetLEDChannel(
    const Kd__V1__SetLEDChannel* msg,
    ResponseMessage& response
) {
    if (!msg) {
        response = makeCommandResult(false, "Invalid message");
        return HandleResult::ok();
    }

    // Partial update: proto3 `optional` fields carry presence (protobuf-c
    // exposes it as the synthetic *_case), effect_id "" = unchanged, absent
    // color = unchanged. So a cloud "power on" no longer zeroes brightness.
    led_state::ChannelPatch patch;
    if (msg->effect_id && msg->effect_id[0]) patch.effect_id = std::string(msg->effect_id);
    if (msg->_brightness_case == KD__V1__SET_LEDCHANNEL___BRIGHTNESS_BRIGHTNESS) {
        patch.brightness = static_cast<uint8_t>(msg->brightness > 255 ? 255 : msg->brightness);
    }
    if (msg->_speed_case == KD__V1__SET_LEDCHANNEL___SPEED_SPEED) {
        patch.speed = static_cast<uint8_t>(msg->speed > 10 ? 10 : (msg->speed < 1 ? 1 : msg->speed));
    }
    if (msg->_enabled_case == KD__V1__SET_LEDCHANNEL___ENABLED_ENABLED) {
        patch.enabled = msg->enabled;
    }
    if (msg->color) {
        patch.color = PixelColor(
            static_cast<uint8_t>(msg->color->r), static_cast<uint8_t>(msg->color->g),
            static_cast<uint8_t>(msg->color->b), static_cast<uint8_t>(msg->color->w),
            static_cast<uint8_t>(msg->color->cw));
    }

    if (!led_state::apply_channel_patch(msg->channel, patch)) {
        ESP_LOGW(TAG, "SetLEDChannel: channel %u not found", msg->channel);
        response = makeCommandResult(false, "Channel not found");
        return HandleResult::ok();
    }

    ESP_LOGI(TAG, "SetLEDChannel: ch=%u", msg->channel);

    // Requester gets the CommandResult; the resulting LEDConfig snapshot
    // reaches LAN clients and the cloud through led_state::notify_changed().
    response = makeCommandResult(true);
    return HandleResult::ok();
}

HandleResult LEDHandler::handleGetLEDEffects(ResponseMessage& response) {
    PixelEffectEngine* effect_engine = PixelDriver::getEffectEngine();
    std::vector<PixelEffectEngine::EffectInfo> effects =
        effect_engine ? effect_engine->getAllEffects() : std::vector<PixelEffectEngine::EffectInfo>{};

    ESP_LOGD(TAG, "GetLEDEffects: returning %zu effects", effects.size());

    // Locals, not statics: concurrent handler tasks resizing shared static
    // vectors double-free their backing stores.
    Kd__V1__LEDEffectsList effects_list = KD__V1__LEDEFFECTS_LIST__INIT;
    std::vector<Kd__V1__LEDEffectInfo*> effect_ptrs;
    std::vector<Kd__V1__LEDEffectInfo> effect_infos;

    effect_infos.resize(effects.size());

    for (size_t i = 0; i < effects.size(); i++) {
        effect_infos[i] = KD__V1__LEDEFFECT_INFO__INIT;
        effect_infos[i].id = const_cast<char*>(effects[i].id.c_str());
        effect_infos[i].name = const_cast<char*>(effects[i].display_name.c_str());
        effect_ptrs.push_back(&effect_infos[i]);
    }

    effects_list.n_effects = effect_ptrs.size();
    effects_list.effects = effect_ptrs.data();

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_LED_EFFECTS_LIST;
    resp.led_effects_list = &effects_list;

    response = serialize(&resp);
    return HandleResult::ok();
}
