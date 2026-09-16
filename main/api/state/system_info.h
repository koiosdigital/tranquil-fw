// SystemInfo builder shared by the WS handler, the REST endpoint and the
// cloud boot handshake, so every consumer reports the same fields.
#pragma once

#include "handler_base.h"

#include <cstdint>

namespace system_info {

// Plain snapshot (no protobuf ownership headaches for REST).
struct Snapshot {
    const char* firmware_version;
    const char* hardware_model;
    const char* device_id;      // cert CN (may be empty)
    const char* hostname;
    bool is_homed;
    bool is_homing;
    uint32_t free_heap;
    uint32_t free_psram;
    uint32_t uptime_s;
    int32_t wifi_rssi;          // dBm; 0 when not associated
    char ip_address[16];
    bool quiet_hours_active;    // a quiet-hours window is active right now
    const char* timezone;       // IANA zone the schedule runs in
};

// Fill a snapshot. `out` must outlive any use of the string pointers
// (they point at process-lifetime storage, except ip_address which is inline).
void snapshot(Snapshot& out);

// Packed TranquilMessage{system_info}. Invalid on allocation failure.
// `request_id` is echoed on the envelope when replying to a request.
ResponseMessage build_message(const char* request_id = nullptr);

}  // namespace system_info
