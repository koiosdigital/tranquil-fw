// Unified LED control handler implementation
#include "led_handler.h"
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
    static Kd__V1__LEDConfig config = KD__V1__LEDCONFIG__INIT;
    static char version[32] = "1.0";

    std::vector<int32_t> channel_ids = PixelDriver::getChannelIds();
    bool has_leds = !channel_ids.empty();
    uint32_t total_leds = 0;
    bool is_rgbw = false;

    for (int32_t id : channel_ids) {
        const PixelChannel* ch = PixelDriver::getChannel(id);
        if (ch) {
            ChannelConfig cfg = ch->getConfig();
            total_leds += cfg.pixel_count;
            if (cfg.format == PixelFormat::RGBW) {
                is_rgbw = true;
            }
        }
    }

    config.has_leds = has_leds;
    config.led_count = total_leds;
    config.is_rgbw = is_rgbw;
    config.pixdriver_version = version;

    ESP_LOGI(TAG, "LEDConfigRequest: has_leds=%d, count=%u, rgbw=%d",
        has_leds, total_leds, is_rgbw);

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_LED_CONFIG;
    resp.led_config = &config;

    response = serialize(&resp);
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

    PixelChannel* ch = PixelDriver::getChannel(msg->channel);
    if (!ch) {
        ESP_LOGW(TAG, "SetLEDChannel: channel %u not found", msg->channel);
        response = makeCommandResult(false, "Channel not found");
        return HandleResult::ok();
    }

    EffectConfig eff_cfg = ch->getEffectConfig();

    // Update effect config from message
    if (msg->effect_id && strlen(msg->effect_id) > 0) {
        eff_cfg.effect = msg->effect_id;
    }
    eff_cfg.brightness = msg->brightness;
    eff_cfg.speed = msg->speed;
    eff_cfg.enabled = msg->enabled;

    if (msg->color) {
        eff_cfg.color.r = msg->color->r;
        eff_cfg.color.g = msg->color->g;
        eff_cfg.color.b = msg->color->b;
        eff_cfg.color.w = msg->color->w;
    }

    ch->setEffect(eff_cfg);

    ESP_LOGI(TAG, "SetLEDChannel: ch=%u effect=%s bright=%u enabled=%d",
        msg->channel, eff_cfg.effect.c_str(), eff_cfg.brightness, eff_cfg.enabled);

    response = makeCommandResult(true);
    return HandleResult::ok(true);  // Broadcast to locals
}

HandleResult LEDHandler::handleGetLEDEffects(ResponseMessage& response) {
    PixelEffectEngine* effect_engine = PixelDriver::getEffectEngine();
    std::vector<PixelEffectEngine::EffectInfo> effects = effect_engine->getAllEffects();

    ESP_LOGI(TAG, "GetLEDEffects: returning %zu effects", effects.size());

    static Kd__V1__LEDEffectsList effects_list = KD__V1__LEDEFFECTS_LIST__INIT;
    static std::vector<Kd__V1__LEDEffectInfo*> effect_ptrs;
    static std::vector<Kd__V1__LEDEffectInfo> effect_infos;

    effect_ptrs.clear();
    effect_infos.clear();
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
