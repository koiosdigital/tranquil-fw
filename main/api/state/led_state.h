// LED state: the single source of the LEDConfig snapshot (strip hardware +
// live per-channel effect state) used by the WS handler, the REST API, the
// library notifier and the cloud boot handshake.
#pragma once

#include "handler_base.h"
#include "kd_pixdriver.h"

#include <cstdint>
#include <optional>
#include <string>

namespace led_state {

// Public channel index (0-based) -> driver channel id, or nullopt.
std::optional<int32_t> channel_id(uint32_t index);

// Number of live channels.
size_t channel_count();

// Packed TranquilMessage{led_config}. Invalid on allocation failure.
ResponseMessage build_config_message();

// Apply a partial effect update to a channel. Fields left at nullopt are
// unchanged. Speed is clamped to the engine's 1-10 scale, brightness to
// 0-255. Returns false when the channel does not exist.
struct ChannelPatch {
    std::optional<std::string> effect_id;
    std::optional<uint8_t> brightness;
    std::optional<uint8_t> speed;
    std::optional<bool> enabled;
    std::optional<PixelColor> color;
};
bool apply_channel_patch(uint32_t index, const ChannelPatch& patch);

// Something changed a channel (any transport, or the scheduler): schedule a
// debounced LEDConfig push to LAN clients and the cloud.
void notify_changed();

}  // namespace led_state
