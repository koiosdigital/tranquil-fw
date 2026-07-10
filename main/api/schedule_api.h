#pragma once

#include <esp_http_server.h>

// REST endpoints: /api/schedule (GET/PUT)
void schedule_api_register_handlers(httpd_handle_t server);
