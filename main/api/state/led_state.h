// LED state: the single source of the LEDConfig snapshot (strip hardware +
// live per-channel effect state) used by the WS handler, the REST API, the
// library notifier and the cloud boot handshake.
//
// Every access to a PixelChannel happens under PixelDriver::Lock inside this
// module; callers only ever see copies (ChannelSnapshot), never a raw
// PixelChannel*.
#pragma once

#include "handler_base.h"
#include "kd_pixdriver.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace led_state {

// Public channel index (0-based) -> driver channel id, or nullopt.
std::optional<int32_t> channel_id(uint32_t index);

// Number of live channels.
size_t channel_count();

// Copy of one channel's hardware config and live effect state, taken under
// the driver lock.
struct ChannelSnapshot {
    uint32_t index = 0;
    uint16_t pixel_count = 0;
    PixelFormat format = PixelFormat::RGB;
    EffectConfig effect;
};

// False when the channel does not exist.
bool snapshot_channel(uint32_t index, ChannelSnapshot& out);

// Every live channel, in index order.
std::vector<ChannelSnapshot> snapshot_channels();

// Packed TranquilMessage{led_config}. Invalid on allocation failure.
// `request_id` is echoed on the envelope when replying to a request.
ResponseMessage build_config_message(const char* request_id = nullptr);

// Apply a partial effect update to a channel. Fields left at nullopt are
// unchanged. Wire values may be passed through unmodified: brightness is
// clamped to 0-255 and speed to the engine's 1-10 scale here. An empty
// effect_id means "unchanged". Returns false when the channel does not
// exist.
struct ChannelPatch {
    std::optional<std::string> effect_id;
    std::optional<int> brightness;
    std::optional<int> speed;
    std::optional<bool> enabled;
    std::optional<PixelColor> color;
};
bool apply_channel_patch(uint32_t index, const ChannelPatch& patch);

// Something changed a channel (any transport, or the scheduler): schedule a
// debounced LEDConfig push to LAN clients and the cloud.
void notify_changed();

}  // namespace led_state
