// LED state implementation
#include "led_state.h"
#include "library_notifier.h"
#include "pixel_effects.h"

#include <esp_log.h>

#include <algorithm>
#include <vector>

static const char* TAG = "led_state";

namespace led_state {

namespace {

struct Packer : HandlerBase {
    HandleResult handle(const Kd__V1__TranquilMessage*, const MessageContext&, ResponseMessage&) override {
        return HandleResult::notHandled();
    }
    bool canHandle(Kd__V1__TranquilMessage__MessageCase) const override { return false; }
    std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override { return {}; }
    static ResponseMessage pack(const Kd__V1__TranquilMessage* msg) { return serialize(msg); }
};

Kd__V1__LEDFormat to_proto_format(PixelFormat fmt) {
    switch (fmt) {
        case PixelFormat::RGB: return KD__V1__LEDFORMAT__LED_FORMAT_RGB;
        case PixelFormat::RGBW: return KD__V1__LEDFORMAT__LED_FORMAT_RGBW;
        case PixelFormat::RGBCCT: return KD__V1__LEDFORMAT__LED_FORMAT_RGBCCT;
    }
    return KD__V1__LEDFORMAT__LED_FORMAT_UNSPECIFIED;
}

}  // namespace

std::optional<int32_t> channel_id(uint32_t index) {
    std::vector<int32_t> ids = PixelDriver::getChannelIds();
    if (index >= ids.size()) return std::nullopt;
    return ids[index];
}

size_t channel_count() {
    return PixelDriver::getChannelIds().size();
}

ResponseMessage build_config_message() {
    std::vector<int32_t> ids = PixelDriver::getChannelIds();

    // Locals, not statics: httpd, the cloudlink task and the api_events task
    // can build this concurrently.
    Kd__V1__LEDConfig config = KD__V1__LEDCONFIG__INIT;
    char version[32];
    snprintf(version, sizeof(version), "%s", getPixelDriverVersion());

    std::vector<Kd__V1__LEDChannelState> states(ids.size());
    std::vector<Kd__V1__LEDColor> colors(ids.size());
    std::vector<Kd__V1__LEDChannelState*> state_ptrs(ids.size());
    // Effect ids must outlive pack(); keep copies rather than pointers into
    // the channel's mutable std::string.
    std::vector<std::string> effect_ids(ids.size());

    uint32_t total_leds = 0;
    Kd__V1__LEDFormat format = KD__V1__LEDFORMAT__LED_FORMAT_UNSPECIFIED;

    for (size_t i = 0; i < ids.size(); i++) {
        const PixelChannel* ch = PixelDriver::getChannel(ids[i]);
        states[i] = KD__V1__LEDCHANNEL_STATE__INIT;
        colors[i] = KD__V1__LEDCOLOR__INIT;
        state_ptrs[i] = &states[i];
        states[i].channel = static_cast<uint32_t>(i);
        if (!ch) continue;

        const ChannelConfig& cfg = ch->getConfig();
        const EffectConfig& eff = ch->getEffectConfig();
        total_leds += cfg.pixel_count;
        format = to_proto_format(cfg.format);

        effect_ids[i] = eff.effect;
        states[i].effect_id = const_cast<char*>(effect_ids[i].c_str());
        states[i].brightness = eff.brightness;
        states[i].speed = eff.speed;
        states[i].enabled = eff.enabled;
        colors[i].r = eff.color.r;
        colors[i].g = eff.color.g;
        colors[i].b = eff.color.b;
        colors[i].w = eff.color.w;
        colors[i].cw = eff.color.cw;
        states[i].color = &colors[i];
    }

    config.has_leds = !ids.empty();
    config.led_count = total_leds;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    config.is_rgbw = (format == KD__V1__LEDFORMAT__LED_FORMAT_RGBW);
#pragma GCC diagnostic pop
    config.format = format;
    config.pixdriver_version = version;
    config.n_channels = state_ptrs.size();
    config.channels = state_ptrs.data();

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_LED_CONFIG;
    msg.led_config = &config;
    return Packer::pack(&msg);
}

bool apply_channel_patch(uint32_t index, const ChannelPatch& patch) {
    auto id = channel_id(index);
    if (!id) return false;
    PixelChannel* ch = PixelDriver::getChannel(*id);
    if (!ch) return false;

    EffectConfig cfg = ch->getEffectConfig();
    if (patch.effect_id && !patch.effect_id->empty()) cfg.effect = *patch.effect_id;
    if (patch.brightness) cfg.brightness = *patch.brightness;
    if (patch.speed) cfg.speed = std::clamp<uint8_t>(*patch.speed, 1, 10);
    if (patch.enabled) cfg.enabled = *patch.enabled;
    if (patch.color) cfg.color = *patch.color;
    ch->setEffect(cfg);

    ESP_LOGD(TAG, "Channel %lu: effect=%s bright=%u speed=%u on=%d",
        static_cast<unsigned long>(index), cfg.effect.c_str(), cfg.brightness, cfg.speed, cfg.enabled);
    notify_changed();
    return true;
}

void notify_changed() {
    library_notifier::led_changed();
}

}  // namespace led_state
