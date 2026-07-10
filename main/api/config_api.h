#pragma once

#include <esp_http_server.h>

// REST endpoints: /api/config, /api/config/calibration, /api/presets,
// /api/presets/load
void config_api_register_handlers(httpd_handle_t server);
