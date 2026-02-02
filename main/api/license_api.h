#pragma once

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

// Register /api/license endpoints with the given httpd server
void license_api_register_handlers(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
