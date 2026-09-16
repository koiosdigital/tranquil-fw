#pragma once

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

    // Register all /api/player endpoints with the given httpd server
    void api_player_register_endpoints(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
