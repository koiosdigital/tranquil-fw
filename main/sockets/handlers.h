// Message handlers for incoming cloud WebSocket messages
#pragma once

#include <kd/v1/tranquil.pb-c.h>

// Process a received protobuf message from cloud
void cloud_handle_message(Kd__V1__TranquilMessage* message);

// Device-initiated boot handshake, run on cloudlink session-ready: report
// identity and request license / purchases / claim. The vn backend only
// answers requests, so the device must drive this on every (re)connect.
void handlers_on_connected();
