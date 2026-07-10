#include "api_common.h"

#include <esp_log.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace api_common {

bool recv_body(httpd_req_t* req, char** out, size_t* out_len, size_t max_len) {
    if (!out || !out_len) return false;
    *out = nullptr;
    *out_len = 0;

    size_t total = req->content_len;
    if (total == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return false;
    }
    if (total > max_len) {
        httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "Body too large");
        return false;
    }

    char* buf = static_cast<char*>(malloc(total + 1));
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return false;
    }

    size_t received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, buf + received, total - received);
        if (r <= 0) {
            free(buf);
            if (r == HTTPD_SOCK_ERR_TIMEOUT) {
                httpd_resp_send_408(req);
            } else {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Recv failed");
            }
            return false;
        }
        received += r;
    }
    buf[total] = '\0';

    *out = buf;
    *out_len = total;
    return true;
}

cJSON* parse_json_body(httpd_req_t* req, size_t max_len) {
    char* buf = nullptr;
    size_t len = 0;
    if (!recv_body(req, &buf, &len, max_len)) return nullptr;

    (void)len;
    cJSON* json = cJSON_Parse(buf);
    free(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return nullptr;
    }
    return json;
}

esp_err_t send_json(httpd_req_t* req, cJSON* json) {
    if (!json) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Null JSON");
        return ESP_FAIL;
    }
    char* s = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    if (!s) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON print failed");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_send(req, s, strlen(s));
    free(s);
    return err;
}

esp_err_t send_ok(httpd_req_t* req) {
    static const char OK_BODY[] = "{\"ok\":true}";
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, OK_BODY, sizeof(OK_BODY) - 1);
}

std::string extract_uuid(const char* uri, const char* base) {
    if (strncmp(uri, base, strlen(base)) != 0) {
        return "";
    }
    const char* uuid_start = uri + strlen(base);
    const char* slash = strchr(uuid_start, '/');
    size_t uuid_len = slash ? static_cast<size_t>(slash - uuid_start) : strlen(uuid_start);
    // Strip query string if present
    const char* quest = static_cast<const char*>(memchr(uuid_start, '?', uuid_len));
    if (quest) uuid_len = quest - uuid_start;
    if (uuid_len == 0 || uuid_len > 64) {
        return "";
    }
    return std::string(uuid_start, uuid_len);
}

void parse_pagination(httpd_req_t* req, int& page, int& per_page) {
    page = 0;
    per_page = 20;

    char query[128] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char param[16] = {0};
        if (httpd_query_key_value(query, "page", param, sizeof(param)) == ESP_OK) {
            page = atoi(param);
            if (page < 0) page = 0;
        }
        if (httpd_query_key_value(query, "per_page", param, sizeof(param)) == ESP_OK) {
            per_page = atoi(param);
            if (per_page < 1) per_page = 1;
            if (per_page > 100) per_page = 100;
        }
    }
}

void chunk_buf_write(ChunkBuf* cb, const char* data, size_t len) {
    if (!cb || !data || len == 0) return;
    if (cb->pos + len > sizeof(cb->buf)) {
        if (cb->pos > 0) {
            httpd_resp_send_chunk(cb->req, cb->buf, cb->pos);
            cb->pos = 0;
        }
        if (len >= sizeof(cb->buf)) {
            // Single write larger than scratch — emit directly so we
            // don't add a copy step. Rare path (only very long strings).
            httpd_resp_send_chunk(cb->req, data, len);
            return;
        }
    }
    memcpy(cb->buf + cb->pos, data, len);
    cb->pos += len;
}

void chunk_buf_printf(ChunkBuf* cb, const char* fmt, ...) {
    char tmp[128];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    if (n > 0) {
        chunk_buf_write(cb, tmp, (n < (int)sizeof(tmp)) ? (size_t)n : sizeof(tmp) - 1);
    }
}

void chunk_buf_finish(ChunkBuf* cb) {
    if (!cb) return;
    if (cb->pos > 0) {
        httpd_resp_send_chunk(cb->req, cb->buf, cb->pos);
        cb->pos = 0;
    }
    httpd_resp_send_chunk(cb->req, nullptr, 0);
}

void chunk_buf_write_json_string(ChunkBuf* cb, const char* s) {
    if (!cb) return;
    chunk_buf_write(cb, "\"", 1);
    if (!s) { chunk_buf_write(cb, "\"", 1); return; }
    const char* run = s;
    size_t run_len = 0;
    char esc[8];
    for (const char* p = s; *p; p++) {
        unsigned char c = static_cast<unsigned char>(*p);
        const char* repl = nullptr;
        size_t repl_len = 0;
        switch (c) {
            case '"':  repl = "\\\""; repl_len = 2; break;
            case '\\': repl = "\\\\"; repl_len = 2; break;
            case '\b': repl = "\\b";  repl_len = 2; break;
            case '\f': repl = "\\f";  repl_len = 2; break;
            case '\n': repl = "\\n";  repl_len = 2; break;
            case '\r': repl = "\\r";  repl_len = 2; break;
            case '\t': repl = "\\t";  repl_len = 2; break;
            default:
                if (c < 0x20) {
                    repl_len = snprintf(esc, sizeof(esc), "\\u%04x", c);
                    repl = esc;
                }
                break;
        }
        if (repl) {
            if (run_len > 0) chunk_buf_write(cb, run, run_len);
            chunk_buf_write(cb, repl, repl_len);
            run = p + 1;
            run_len = 0;
        } else {
            run_len++;
        }
    }
    if (run_len > 0) chunk_buf_write(cb, run, run_len);
    chunk_buf_write(cb, "\"", 1);
}

} // namespace api_common
