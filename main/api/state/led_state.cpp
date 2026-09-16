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

Kd__V1__LEDFormat to_proto_format(PixelFormat fmt) {
    switch (fmt) {
        case PixelFormat::RGB: return KD__V1__LEDFORMAT__LED_FORMAT_RGB;
        case PixelFormat::RGBW: return KD__V1__LEDFORMAT__LED_FORMAT_RGBW;
        case PixelFormat::RGBCCT: return KD__V1__LEDFORMAT__LED_FORMAT_RGBCCT;
    }
    return KD__V1__LEDFORMAT__LED_FORMAT_UNSPECIFIED;
}

// Caller holds PixelDriver::Lock.
PixelChannel* channel_at_locked(uint32_t index) {
    std::vector<int32_t> ids = PixelDriver::getChannelIds();
    if (index >= ids.size()) return nullptr;
    return PixelDriver::getChannel(ids[index]);
}

}  // namespace

std::optional<int32_t> channel_id(uint32_t index) {
    PixelDriver::Lock lock;
    std::vector<int32_t> ids = PixelDriver::getChannelIds();
    if (index >= ids.size()) return std::nullopt;
    return ids[index];
}

size_t channel_count() {
    PixelDriver::Lock lock;
    return PixelDriver::getChannelIds().size();
}

bool snapshot_channel(uint32_t index, ChannelSnapshot& out) {
    PixelDriver::Lock lock;
    PixelChannel* ch = channel_at_locked(index);
    if (!ch) return false;
    const ChannelConfig& cfg = ch->getConfig();
    out.index = index;
    out.pixel_count = cfg.pixel_count;
    out.format = cfg.format;
    out.effect = ch->getEffectConfigCopy();
    return true;
}

std::vector<ChannelSnapshot> snapshot_channels() {
    std::vector<ChannelSnapshot> out;
    PixelDriver::Lock lock;
    std::vector<int32_t> ids = PixelDriver::getChannelIds();
    out.reserve(ids.size());
    for (size_t i = 0; i < ids.size(); i++) {
        PixelChannel* ch = PixelDriver::getChannel(ids[i]);
        if (!ch) continue;
        ChannelSnapshot s;
        const ChannelConfig& cfg = ch->getConfig();
        s.index = static_cast<uint32_t>(i);
        s.pixel_count = cfg.pixel_count;
        s.format = cfg.format;
        s.effect = ch->getEffectConfigCopy();
        out.push_back(std::move(s));
    }
    return out;
}

ResponseMessage build_config_message(const char* request_id) {
    // Copy the channel state under the driver lock, then pack without it.
    // Locals, not statics: httpd, the cloudlink task and the api_events task
    // can build this concurrently.
    std::vector<ChannelSnapshot> snaps = snapshot_channels();

    Kd__V1__LEDConfig config = KD__V1__LEDCONFIG__INIT;
    char version[32];
    snprintf(version, sizeof(version), "%s", getPixelDriverVersion());

    std::vector<Kd__V1__LEDChannelState> states(snaps.size());
    std::vector<Kd__V1__LEDColor> colors(snaps.size());
    std::vector<Kd__V1__LEDChannelState*> state_ptrs(snaps.size());

    uint32_t total_leds = 0;
    Kd__V1__LEDFormat format = KD__V1__LEDFORMAT__LED_FORMAT_UNSPECIFIED;

    for (size_t i = 0; i < snaps.size(); i++) {
        const ChannelSnapshot& s = snaps[i];
        states[i] = KD__V1__LEDCHANNEL_STATE__INIT;
        colors[i] = KD__V1__LEDCOLOR__INIT;
        state_ptrs[i] = &states[i];

        total_leds += s.pixel_count;
        format = to_proto_format(s.format);

        states[i].channel = s.index;
        states[i].effect_id = const_cast<char*>(s.effect.effect.c_str());
        states[i].brightness = s.effect.brightness;
        states[i].speed = s.effect.speed;
        states[i].enabled = s.effect.enabled;
        colors[i].r = s.effect.color.r;
        colors[i].g = s.effect.color.g;
        colors[i].b = s.effect.color.b;
        colors[i].w = s.effect.color.w;
        colors[i].cw = s.effect.color.cw;
        states[i].color = &colors[i];
    }

    config.has_leds = !snaps.empty();
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
    return pack_tranquil_message(&msg, request_id);
}

bool apply_channel_patch(uint32_t index, const ChannelPatch& patch) {
    {
        PixelDriver::Lock lock;
        PixelChannel* ch = channel_at_locked(index);
        if (!ch) return false;

        EffectConfig cfg = ch->getEffectConfigCopy();
        if (patch.effect_id && !patch.effect_id->empty()) cfg.effect = *patch.effect_id;
        if (patch.brightness) cfg.brightness = static_cast<uint8_t>(std::clamp(*patch.brightness, 0, 255));
        if (patch.speed) cfg.speed = static_cast<uint8_t>(std::clamp(*patch.speed, 1, 10));
        if (patch.enabled) cfg.enabled = *patch.enabled;
        if (patch.color) cfg.color = *patch.color;
        ch->setEffect(cfg);

        ESP_LOGD(TAG, "Channel %lu: effect=%s bright=%u speed=%u on=%d",
            static_cast<unsigned long>(index), cfg.effect.c_str(), cfg.brightness, cfg.speed, cfg.enabled);
    }
    notify_changed();
    return true;
}

void notify_changed() {
    library_notifier::led_changed();
}

}  // namespace led_state
