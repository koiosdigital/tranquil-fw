#pragma once

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

    void pattern_upload_register_handlers(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
