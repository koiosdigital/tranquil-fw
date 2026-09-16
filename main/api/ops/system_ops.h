// System operations: homing, reboot, factory reset and the device info
// snapshot, shared by system_api.cpp and system_handler.cpp.
#pragma once

#include "op_result.h"
#include "api_common.h"

#include <cstdint>

namespace system_ops {

// Start an asynchronous homing run (homing_service). ESP_ERR_INVALID_STATE
// when a run is already in progress.
OpResult home(bool force_full_calibration);

// Stop playback and restart in ~500 ms (settings and library kept).
OpResult reboot(const char* reason);

// Wipe NVS / SD state and restart in ~300 ms. LAN-only: the transports
// enforce that (dispatcher isLocalOnly, REST Origin refusal).
OpResult factory_reset(const char* reason);

// The one restart timer. Gives the transport time to flush its reply.
void schedule_restart(uint32_t delay_ms);

// REST shape (docs/swagger.json SystemInfo).
void info_to_json(api_common::ChunkBuf* cb);

}  // namespace system_ops
