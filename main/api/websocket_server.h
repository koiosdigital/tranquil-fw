#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
#include <cstdint>
#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize WebSocket server on existing HTTPD server
esp_err_t websocket_server_init(httpd_handle_t server);

// Broadcast message to all connected WebSocket clients
esp_err_t websocket_broadcast(const uint8_t* data, size_t len);

// Broadcast to every connected client except `exclude_fd` (-1 = none).
esp_err_t websocket_broadcast_except(const uint8_t* data, size_t len, int exclude_fd);

// Send message to a specific client by socket fd. Non-blocking: when the
// async send queue is saturated the frame is dropped at the source and
// ESP_ERR_TIMEOUT is returned.
esp_err_t websocket_send(int fd, const uint8_t* data, size_t len);

// Get number of connected clients
size_t websocket_client_count();

// Same as websocket_client_count(); used by the cloudlink restart policy.
size_t ws_connected_client_count();

#ifdef __cplusplus
}
#endif
