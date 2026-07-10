#pragma once

// Cloud WebSocket client (vn-sec.koios.sh), a thin layer over koios-sdk's
// cloudlink. Cloudlink owns the connection lifecycle end to end: waits for
// network + identity, connects with mTLS (device cert from kd_common crypto
// plus the DS peripheral context), expects the `welcome` control frame,
// reconnects with jittered backoff, and escalates persistent failures.
// The reverse proxy forwards the cert CN ("TRANQUIL-XXXX") which IS the
// device identity server-side.
//
// This layer keeps only the tranquil-specific parts: the TranquilMessage
// protobuf dispatch (DRM/license/purchase/pattern), and the device-initiated
// boot sequence on session ready.

// Initialize the cloud sockets module. Requires the default event loop and
// kd_common_init() (WiFi + crypto) to have run.
void cloud_sockets_init();

// Deinitialize and clean up
void cloud_sockets_deinit();

// Check if connected to cloud server
bool cloud_sockets_is_connected();

// Make the next connection present the freshly stored device cert instead of
// cloudlink's process-lifetime cached copy. Call after a successful cert
// renewal. Forces a reconnect (asynchronously).
void sockets_invalidate_cert_cache();

#define CLOUD_SOCKETS_URL "wss://vn-sec.koios.sh"
