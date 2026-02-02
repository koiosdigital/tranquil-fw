// Message handlers for incoming cloud WebSocket messages
#pragma once

#include <kd/v1/tranquil.pb-c.h>

// Process a received protobuf message from cloud
void cloud_handle_message(Kd__V1__TranquilMessage* message);
