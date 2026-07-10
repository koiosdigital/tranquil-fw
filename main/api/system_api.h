#pragma once

#include <esp_http_server.h>

// REST endpoints: /api/system/info, /api/system/home, /api/system/factory-reset
void system_api_register_handlers(httpd_handle_t server);
