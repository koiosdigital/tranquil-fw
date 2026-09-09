// LED REST API (/api/led/*). Replaces kd_pixdriver's built-in attach_api so
// every write goes through led_state (speed clamp, RGBCCT whites, change
// notification to WS clients + cloud) and the config read includes the live
// channel state.
#pragma once

#include <esp_http_server.h>

void led_api_register_handlers(httpd_handle_t server);
