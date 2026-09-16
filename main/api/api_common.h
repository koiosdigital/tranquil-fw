#pragma once

#include <esp_http_server.h>
#include <cJSON.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <string>

// Re-exported so every handler file gets the wrapper declaration via
// api_common.h. Use kd_common_api_register_uri_handler() instead of the
// bare httpd_register_uri_handler() so the pre-handler hook (CORS, etc.)
// applies to every route.
#include "kd_api.h"

#include "ops/op_result.h"
#include "pixel_core.h"

// ============================================================================
// Shared HTTP / JSON helpers for the REST API.
// ============================================================================

namespace api_common {

// Receive entire request body into a heap buffer (PSRAM when available).
// On success, *out points to a NUL-terminated buffer the caller free()s
// and *out_len is the body length (excluding NUL).
// On failure, sends an error response and returns false.
bool recv_body(httpd_req_t* req, char** out, size_t* out_len, size_t max_len = 16384);

// Parse request body as JSON. On error, sends 400 and returns nullptr.
// Caller must cJSON_Delete the returned pointer.
cJSON* parse_json_body(httpd_req_t* req, size_t max_len = 16384);

// Send a cJSON object as 200 application/json. Consumes json (deletes it).
esp_err_t send_json(httpd_req_t* req, cJSON* json);

// Send `{ "ok": true }` (no allocation).
esp_err_t send_ok(httpd_req_t* req);

// The one esp_err -> HTTP status mapping for the REST API. Sends the error
// response (detail as the body) and returns ESP_FAIL so handlers can
// `return send_op_error(req, r);`.
//   ESP_ERR_NOT_FOUND                              -> 404
//   ESP_ERR_INVALID_ARG / _SIZE / _STATE           -> 400
//   ESP_ERR_NOT_ALLOWED / ESP_ERR_NOT_SUPPORTED    -> 403
//   ESP_ERR_TIMEOUT                                -> 503
//   anything else                                  -> 500
esp_err_t send_op_error(httpd_req_t* req, const OpResult& r);

// Send an arbitrary status line with a text/plain body; returns ESP_FAIL.
esp_err_t send_error_status(httpd_req_t* req, const char* status, const char* message);

// Routes that must never be reachable from a browser page (store token,
// factory reset, reboot). Browsers always send an Origin header on
// cross-origin requests; native apps and curl send none. Refuses the request
// with 403 and returns true when an Origin header is present.
bool reject_browser_origin(httpd_req_t* req);

// Extract UUID path segment from URI like {base}{uuid} or {base}{uuid}/suffix.
// base must end with '/'. Returns empty string if missing or longer than 63
// characters (the player's uuid buffers are 64 bytes including the NUL).
std::string extract_uuid(const char* uri, const char* base);

// Longest uuid the REST API accepts in a path or body.
constexpr size_t kMaxUuidChars = 63;

// Parse ?page= and ?per_page= query params with clamping.
void parse_pagination(httpd_req_t* req, int& page, int& per_page);

// Read obj[key] as a finite number. Returns false when absent, not a number,
// or NaN/inf, so callers can range-check and cast safely.
bool json_number(const cJSON* obj, const char* key, double& out);

// "#rrggbb" / "rrggbb" / "#rrggbbww". Untouched components keep their value.
bool parse_hex_color(const char* s, PixelColor& out);

// ============================================================================
// Buffered chunked writer
//
// httpd_resp_send_chunk allocates an lwIP pbuf per call (~80-130 B overhead
// each — pbufs live in heap until TCP ACKs them). Streaming an N-item
// response as N small chunks could pile up more heap than the cJSON tree
// it replaced. ChunkBuf accumulates writes into a 1 KB stack buffer and
// flushes on overflow or finish — keeps streaming benefits while making
// only a handful of TCP sends per response.
// ============================================================================

struct ChunkBuf {
    httpd_req_t* req;
    char         buf[1024];
    size_t       pos;
};

inline void chunk_buf_init(ChunkBuf* cb, httpd_req_t* req) {
    cb->req = req;
    cb->pos = 0;
}

// Write data to the buffer; auto-flushes on overflow. Data larger than the
// buffer's capacity is sent directly (with a preceding flush of any pending).
void chunk_buf_write(ChunkBuf* cb, const char* data, size_t len);

inline void chunk_buf_write_str(ChunkBuf* cb, const char* s) {
    chunk_buf_write(cb, s, strlen(s));
}

// printf-style write.
void chunk_buf_printf(ChunkBuf* cb, const char* fmt, ...) __attribute__((format(printf, 2, 3)));

// Flush any pending bytes, then emit the terminating zero-length chunk.
void chunk_buf_finish(ChunkBuf* cb);

// Write a JSON-escaped, quoted string literal ("foo\nbar" -> "\"foo\\nbar\"").
// Caller must NOT pre-write the surrounding quotes.
void chunk_buf_write_json_string(ChunkBuf* cb, const char* s);

} // namespace api_common
