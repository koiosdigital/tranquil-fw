#pragma once

// Initialize the cloud WebSocket client module
void cloud_sockets_init();

// Deinitialize and clean up
void cloud_sockets_deinit();

// Check if connected to cloud server
bool cloud_sockets_is_connected();

#define CLOUD_SOCKETS_URL "wss://device.api.koiosdigital.net"
