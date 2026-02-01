#pragma once

#include "esp_err.h"
#include <cstdint>
#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

// Handle incoming protobuf TranquilMessage and produce response
// If response is needed, allocates buffer and sets response/response_len
// Caller must free response buffer if non-null
// Returns ESP_OK if message was handled (even if no response generated)
esp_err_t proto_handle_message(const uint8_t* data, size_t len,
                                uint8_t** response, size_t* response_len);

#ifdef __cplusplus
}
#endif
