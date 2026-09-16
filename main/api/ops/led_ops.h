// LED operations: channel mutation and the effects list, shared by the REST
// endpoints (led_api.cpp) and the WebSocket / cloud handler (led_handler.cpp).
// Channel state snapshots and the LEDConfig builder live in led_state.
#pragma once

#include "op_result.h"
#include "api_common.h"
#include "handler_base.h"
#include "led_state.h"
#include "pixel_effects.h"

#include <vector>

namespace led_ops {

// Partial update of one channel. Every change is pushed to LAN clients and
// the cloud as an LEDConfig snapshot by led_state.
OpResult set_channel(uint32_t index, const led_state::ChannelPatch& patch);

// Static per firmware.
std::vector<PixelEffectEngine::EffectInfo> effects();

// REST shape: [{"id":..,"name":..}, ...]
void effects_to_json(api_common::ChunkBuf* cb);

// TranquilMessage{led_effects_list}.
ResponseMessage build_effects_message(const char* request_id = nullptr);

}  // namespace led_ops
