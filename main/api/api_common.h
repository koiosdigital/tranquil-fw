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

// ============================================================================
// Shared HTTP / JSON helpers for the REST API.
// ============================================================================

namespace api_common {

// Receive entire request body into a heap buffer.
// On success, *out points to a malloc'd NUL-terminated buffer (caller frees)
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

// Extract UUID path segment from URI like {base}{uuid} or {base}{uuid}/suffix.
// base must end with '/'. Returns empty string if missing/too long.
std::string extract_uuid(const char* uri, const char* base);

// Parse ?page= and ?per_page= query params with clamping.
void parse_pagination(httpd_req_t* req, int& page, int& per_page);

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

// printf-style write (formatted output must fit in 128 bytes).
void chunk_buf_printf(ChunkBuf* cb, const char* fmt, ...) __attribute__((format(printf, 2, 3)));

// Flush any pending bytes, then emit the terminating zero-length chunk.
void chunk_buf_finish(ChunkBuf* cb);

// Write a JSON-escaped, quoted string literal ("foo\nbar" -> "\"foo\\nbar\"").
// Caller must NOT pre-write the surrounding quotes.
void chunk_buf_write_json_string(ChunkBuf* cb, const char* s);

} // namespace api_common
