#include "patterns_api.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "cJSON.h"
#include "api_common.h"
#include "ManifestDatabase.h"
#include "PatternReader.h"
#include "jobs/job_queue.h"
#include "jobs/job_types.h"
#include "jobs/job_processor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <vector>
#include <unistd.h>
#include <cstdio>
#include <cmath>
#include <algorithm>

static const char* TAG = "patterns_api";

// Upload constants
static constexpr size_t UPLOAD_CHUNK_SIZE = 8 * 1024;  // HTTP server stack is 8KB (set in kd_common)
static constexpr size_t MAX_BOUNDARY_SIZE = 128;
static constexpr size_t MAX_FILENAME_SIZE = 256;
static constexpr size_t MAX_HEADER_SIZE = 1024;

// Upload context for tracking state during multipart parsing
struct UploadContext {
    FILE* file = nullptr;
    char uuid[64] = { 0 };
    char filename[MAX_FILENAME_SIZE] = { 0 };
    char temp_path[128] = { 0 };
    char final_path[128] = { 0 };
    size_t bytes_written = 0;
    bool in_content = false;
    char boundary[MAX_BOUNDARY_SIZE] = { 0 };
    size_t boundary_len = 0;
};

namespace {

    // Helper: get pattern file path with correct extension
    // Both encrypted (KDEP) and unencrypted (THRB) patterns use .dat
    void getPatternFilePath(const std::string& uuid, bool encrypted, char* path, size_t path_size) {
        (void)encrypted;  // Now unused - all patterns are .dat
        snprintf(path, path_size, "/sd/patterns/%s.dat", uuid.c_str());
    }

    // Helper: get pattern file size on disk
    size_t getPatternFileSize(const std::string& uuid, bool encrypted) {
        char path[128];
        getPatternFilePath(uuid, encrypted, path, sizeof(path));
        struct stat st;
        if (stat(path, &st) == 0) {
            return st.st_size;
        }
        return 0;
    }

    // Stream a pattern object without building a cJSON tree. Field set
    // mirrors ManifestDatabase::patternToJson.
    void write_pattern_json(api_common::ChunkBuf* cb, const Pattern& p) {
        using namespace api_common;
        chunk_buf_write_str(cb, "{\"uuid\":");
        chunk_buf_write_json_string(cb, p.external_uuid.c_str());
        chunk_buf_printf(cb, ",\"id\":%lu,\"name\":", (unsigned long)p.id);
        chunk_buf_write_json_string(cb, p.name.c_str());
        chunk_buf_write_str(cb, ",\"creator\":");
        chunk_buf_write_json_string(cb, p.creator.c_str());
        chunk_buf_write_str(cb, ",\"date\":");
        chunk_buf_write_json_string(cb, p.date.c_str());
        chunk_buf_printf(cb, ",\"popularity\":%d,\"reversible\":%s,\"start_point\":%u,\"encrypted\":%s,\"size_bytes\":%lu",
            p.popularity,
            p.reversible ? "true" : "false",
            (unsigned)p.start_point,
            p.encrypted ? "true" : "false",
            (unsigned long)p.size_bytes);
        // URLs per the swagger contract (download only for unencrypted patterns)
        chunk_buf_write_str(cb, ",\"thumb_url\":\"/api/pattern_thumbs/");
        chunk_buf_write_str(cb, p.external_uuid.c_str());
        chunk_buf_write_str(cb, ".png\",\"download_url\":\"");
        if (!p.encrypted) {
            chunk_buf_write_str(cb, "/api/pattern_download/");
            chunk_buf_write_str(cb, p.external_uuid.c_str());
        }
        chunk_buf_write(cb, "\"", 1);
        if (!p.created_at.empty()) {
            chunk_buf_write_str(cb, ",\"created_at\":");
            chunk_buf_write_json_string(cb, p.created_at.c_str());
        }
        if (!p.last_played_at.empty()) {
            chunk_buf_write_str(cb, ",\"last_played_at\":");
            chunk_buf_write_json_string(cb, p.last_played_at.c_str());
        }
        if (!p.downloaded_at.empty()) {
            chunk_buf_write_str(cb, ",\"downloaded_at\":");
            chunk_buf_write_json_string(cb, p.downloaded_at.c_str());
        }
        chunk_buf_printf(cb, ",\"is_owned\":%s", p.purchased ? "true" : "false");
        if (p.purchased) {
            chunk_buf_printf(cb, ",\"purchased_at\":%lld", (long long)p.purchased_at);
            if (!p.receipt_id.empty()) {
                chunk_buf_write_str(cb, ",\"receipt_id\":");
                chunk_buf_write_json_string(cb, p.receipt_id.c_str());
            }
        }
        chunk_buf_write(cb, "}", 1);
    }

    // Extract boundary from Content-Type header
    bool extract_boundary(httpd_req_t* req, char* boundary, size_t max_len) {
        char content_type[256] = { 0 };
        if (httpd_req_get_hdr_value_str(req, "Content-Type", content_type, sizeof(content_type)) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get Content-Type header");
            return false;
        }

        const char* boundary_start = strstr(content_type, "boundary=");
        if (!boundary_start) {
            ESP_LOGE(TAG, "No boundary in Content-Type: %s", content_type);
            return false;
        }

        boundary_start += 9;  // Skip "boundary="

        // Handle quoted boundary
        if (*boundary_start == '"') {
            boundary_start++;
            const char* boundary_end = strchr(boundary_start, '"');
            if (!boundary_end) return false;
            size_t len = boundary_end - boundary_start;
            if (len >= max_len) return false;
            strncpy(boundary, boundary_start, len);
            boundary[len] = '\0';
        }
        else {
            // Unquoted - copy until space or semicolon
            size_t i = 0;
            while (*boundary_start&&* boundary_start != ' ' && *boundary_start != ';' && i < max_len - 1) {
                boundary[i++] = *boundary_start++;
            }
            boundary[i] = '\0';
        }

        return strlen(boundary) > 0;
    }

    // Extract filename from Content-Disposition header
    bool parse_content_disposition(const char* header, char* filename, size_t max_len) {
        const char* fname_start = strstr(header, "filename=\"");
        if (!fname_start) {
            fname_start = strstr(header, "filename=");
            if (!fname_start) return false;
            fname_start += 9;
        }
        else {
            fname_start += 10;  // Skip 'filename="'
        }

        size_t i = 0;
        while (*fname_start&&* fname_start != '"' && *fname_start != '\r' && *fname_start != '\n' && i < max_len - 1) {
            filename[i++] = *fname_start++;
        }
        filename[i] = '\0';

        // Strip extension for pattern name
        char* dot = strrchr(filename, '.');
        if (dot) *dot = '\0';

        return strlen(filename) > 0;
    }

    // Find pattern in buffer, returns pointer or nullptr
    const char* find_pattern(const char* buf, size_t buf_len, const char* pattern, size_t pattern_len) {
        if (pattern_len > buf_len) return nullptr;
        for (size_t i = 0; i <= buf_len - pattern_len; i++) {
            if (memcmp(buf + i, pattern, pattern_len) == 0) {
                return buf + i;
            }
        }
        return nullptr;
    }

} // anonymous namespace

// GET /api/patterns - paginated list (streamed, no JSON tree)
static esp_err_t patterns_list_handler(httpd_req_t* req) {
    using namespace api_common;

    int page, per_page;
    parse_pagination(req, page, per_page);

    auto result = ManifestDatabase::instance().getPatterns(page, per_page);

    httpd_resp_set_type(req, "application/json");
    ChunkBuf cb;
    chunk_buf_init(&cb, req);

    chunk_buf_printf(&cb,
        "{\"pagination\":{\"page\":%d,\"per_page\":%d,\"total_pages\":%d,\"total_items\":%d},\"patterns\":[",
        result.pagination.page, result.pagination.per_page,
        result.pagination.total_pages, result.pagination.total_items);

    bool first = true;
    for (auto& pattern : result.items) {
        // Update size from disk if not set
        if (pattern.size_bytes == 0) {
            pattern.size_bytes = getPatternFileSize(pattern.external_uuid, pattern.encrypted);
        }
        if (!first) chunk_buf_write(&cb, ",", 1);
        write_pattern_json(&cb, pattern);
        first = false;
    }

    chunk_buf_write_str(&cb, "]}");
    chunk_buf_finish(&cb);
    return ESP_OK;
}

// POST /api/patterns - streaming multipart upload
static esp_err_t patterns_upload_handler(httpd_req_t* req) {
    esp_err_t ret = ESP_OK;

    // Stack-allocate buffers (total ~5.7KB - fits HTTP server 8KB stack)
    UploadContext ctx_storage = {};
    UploadContext* ctx = &ctx_storage;
    char header_buf[MAX_HEADER_SIZE] = { 0 };
    char buffer[UPLOAD_CHUNK_SIZE];

    // Extract multipart boundary
    if (!extract_boundary(req, ctx->boundary, sizeof(ctx->boundary))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Content-Type: missing boundary");
        return ESP_FAIL;
    }
    ctx->boundary_len = strlen(ctx->boundary);
    ESP_LOGI(TAG, "Upload boundary: %s", ctx->boundary);

    // Generate UUID for new pattern
    std::string uuid = ManifestDatabase::generateUUID();
    strncpy(ctx->uuid, uuid.c_str(), sizeof(ctx->uuid) - 1);

    // Prepare file paths (%.36s limits UUID to 36 chars - standard UUID length)
    snprintf(ctx->temp_path, sizeof(ctx->temp_path), "/sd/patterns/%.36s.tmp", ctx->uuid);
    snprintf(ctx->final_path, sizeof(ctx->final_path), "/sd/patterns/%.36s.thr", ctx->uuid);

    // Pre-build boundary strings (avoid repeated stack allocation in loop)
    // first_boundary: "--" + boundary + null = 2 + 127 + 1 = 130 max
    // close_boundary: "\r\n--" + boundary + null = 4 + 127 + 1 = 132 max
    char first_boundary[MAX_BOUNDARY_SIZE + 8];
    char close_boundary[MAX_BOUNDARY_SIZE + 8];
    snprintf(first_boundary, sizeof(first_boundary), "--%.*s",
             (int)(MAX_BOUNDARY_SIZE - 1), ctx->boundary);
    snprintf(close_boundary, sizeof(close_boundary), "\r\n--%.*s",
             (int)(MAX_BOUNDARY_SIZE - 1), ctx->boundary);
    size_t fb_len = strlen(first_boundary);
    size_t cb_len = strlen(close_boundary);

    // We'll use a simple state machine for multipart parsing
    // States: PREAMBLE -> HEADERS -> BODY -> EPILOGUE
    enum State { PREAMBLE, HEADERS, BODY, DONE } state = PREAMBLE;

    size_t header_len = 0;

    size_t total_received = 0;
    size_t content_len = req->content_len;

    while (total_received < content_len && state != DONE) {
        size_t to_read = content_len - total_received;
        if (to_read > UPLOAD_CHUNK_SIZE) to_read = UPLOAD_CHUNK_SIZE;

        int received = httpd_req_recv(req, buffer, to_read);
        if (received <= 0) {
            ESP_LOGE(TAG, "Receive error: %d", received);
            ret = ESP_FAIL;
            break;
        }

        total_received += received;
        char* data = buffer;
        size_t data_len = received;

        // Process data based on state
        while (data_len > 0 && state != DONE) {
            if (state == PREAMBLE) {
                // Look for first boundary: --<boundary>
                const char* found = find_pattern(data, data_len, first_boundary, fb_len);
                if (found) {
                    // Skip to after boundary and CRLF
                    size_t skip = (found - data) + fb_len;
                    if (skip + 2 <= data_len && data[skip] == '\r' && data[skip + 1] == '\n') {
                        skip += 2;
                    }
                    data += skip;
                    data_len -= skip;
                    state = HEADERS;
                    header_len = 0;
                }
                else {
                    // Skip this chunk, boundary not found yet
                    break;
                }
            }

            if (state == HEADERS) {
                // Accumulate headers until we see \r\n\r\n
                while (data_len > 0 && header_len < MAX_HEADER_SIZE - 1) {
                    header_buf[header_len++] = *data++;
                    data_len--;

                    // Check for end of headers
                    if (header_len >= 4 &&
                        header_buf[header_len - 4] == '\r' &&
                        header_buf[header_len - 3] == '\n' &&
                        header_buf[header_len - 2] == '\r' &&
                        header_buf[header_len - 1] == '\n') {
                        header_buf[header_len] = '\0';

                        // Parse Content-Disposition for filename
                        parse_content_disposition(header_buf, ctx->filename, sizeof(ctx->filename));
                        ESP_LOGI(TAG, "Upload filename: %s", ctx->filename[0] ? ctx->filename : "(unnamed)");

                        // Open temp file
                        ctx->file = fopen(ctx->temp_path, "wb");
                        if (!ctx->file) {
                            ESP_LOGE(TAG, "Failed to open temp file: %s", ctx->temp_path);
                            ret = ESP_FAIL;
                            state = DONE;
                            break;
                        }

                        state = BODY;
                        header_len = 0;
                        break;
                    }
                }

                if (header_len >= MAX_HEADER_SIZE - 1) {
                    ESP_LOGE(TAG, "Headers too large");
                    ret = ESP_FAIL;
                    state = DONE;
                }
            }

            if (state == BODY && ctx->file) {
                // Search for boundary in current data
                const char* bound = find_pattern(data, data_len, close_boundary, cb_len);
                if (bound) {
                    // Write data up to boundary
                    size_t write_len = bound - data;
                    if (write_len > 0) {
                        fwrite(data, 1, write_len, ctx->file);
                        ctx->bytes_written += write_len;
                    }
                    state = DONE;
                    break;
                }
                else {
                    // No boundary found, but it might span chunks
                    // Keep last cb_len-1 bytes as carry-over
                    size_t safe_len = (data_len > cb_len - 1) ? data_len - (cb_len - 1) : 0;
                    if (safe_len > 0) {
                        fwrite(data, 1, safe_len, ctx->file);
                        ctx->bytes_written += safe_len;
                        data += safe_len;
                        data_len -= safe_len;
                    }

                    // Store potential boundary start as carry-over for next iteration
                    // For simplicity, just write remaining and check in next chunk
                    // (This is a slight simplification - may write trailing boundary chars)
                    if (data_len > 0 && total_received >= content_len) {
                        // Last chunk - don't write potential boundary
                        // Check if it ends with boundary
                        if (data_len >= cb_len && memcmp(data + data_len - cb_len, close_boundary, cb_len) == 0) {
                            fwrite(data, 1, data_len - cb_len, ctx->file);
                            ctx->bytes_written += data_len - cb_len;
                        }
                        else {
                            fwrite(data, 1, data_len, ctx->file);
                            ctx->bytes_written += data_len;
                        }
                    }
                    else if (data_len > 0) {
                        fwrite(data, 1, data_len, ctx->file);
                        ctx->bytes_written += data_len;
                    }
                    break;
                }
            }
        }
    }

    // Close file
    if (ctx->file) {
        fclose(ctx->file);
        ctx->file = nullptr;
    }

    if (ret != ESP_OK || ctx->bytes_written == 0) {
        // Cleanup on error
        unlink(ctx->temp_path);
        if (ret == ESP_OK) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No file data received");
            return ESP_FAIL;
        }
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Create pattern in database first to get internal ID
    Pattern pattern;
    pattern.id = 0;  // Will be assigned by TQDB
    pattern.external_uuid = ctx->uuid;
    pattern.name = strlen(ctx->filename) > 0 ? ctx->filename : ctx->uuid;
    pattern.creator = "Uploaded";
    pattern.encrypted = false;
    pattern.size_bytes = 0;  // Will be updated after conversion
    pattern.created_at = ManifestDatabase::currentTimestamp();

    if (ManifestDatabase::instance().addPattern(pattern) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add pattern to database");
        unlink(ctx->temp_path);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Look up the newly created pattern to get its internal ID
    auto created_pattern = ManifestDatabase::instance().getPatternByExternalUuid(ctx->uuid);
    if (!created_pattern) {
        ESP_LOGE(TAG, "Failed to find newly created pattern");
        unlink(ctx->temp_path);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Enqueue conversion job via job queue using internal ID
    jobs::ConversionJobData conv_data;
    conv_data.temp_path = ctx->temp_path;
    conv_data.name = pattern.name;
    conv_data.encrypted = false;

    esp_err_t enqueue_result = jobs::JobQueue::instance().enqueueConversion(created_pattern->id, conv_data);
    if (enqueue_result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enqueue conversion job");
        ManifestDatabase::instance().deletePattern(created_pattern->id);
        unlink(ctx->temp_path);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Trigger job processor to start immediately
    jobs::JobProcessor::instance().triggerProcessing();

    ESP_LOGI(TAG, "Pattern uploaded, conversion queued: %s (%zu bytes)", ctx->uuid, ctx->bytes_written);

    // Return 200 OK with processing status
    cJSON* response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "uuid", ctx->uuid);
    cJSON_AddStringToObject(response, "name", conv_data.name.c_str());
    cJSON_AddStringToObject(response, "status", "processing");
    cJSON_AddStringToObject(response, "message", "Pattern uploaded, conversion in progress");

    char* json_str = cJSON_PrintUnformatted(response);

    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(response);

    return ESP_OK;
}

// GET /api/patterns/{uuid} - detail
static esp_err_t patterns_detail_handler(httpd_req_t* req) {
    const char* uri = req->uri;
    const char* base = "/api/patterns/";

    if (strncmp(uri, base, strlen(base)) != 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    const char* uuid = uri + strlen(base);
    if (!uuid || strlen(uuid) == 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // uuid is external_uuid
    auto pattern = ManifestDatabase::instance().getPatternByExternalUuid(uuid);
    if (!pattern) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Update size from disk if not set
    if (pattern->size_bytes == 0) {
        pattern->size_bytes = getPatternFileSize(pattern->external_uuid, pattern->encrypted);
    }

    httpd_resp_set_type(req, "application/json");
    api_common::ChunkBuf cb;
    api_common::chunk_buf_init(&cb, req);
    write_pattern_json(&cb, *pattern);
    api_common::chunk_buf_finish(&cb);
    return ESP_OK;
}

// DELETE /api/patterns/{uuid}
static esp_err_t patterns_delete_handler(httpd_req_t* req) {
    const char* uri = req->uri;
    const char* base = "/api/patterns/";

    if (strncmp(uri, base, strlen(base)) != 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    const char* uuid = uri + strlen(base);
    if (!uuid || strlen(uuid) == 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // uuid is external_uuid - look up to get internal ID
    auto pattern = ManifestDatabase::instance().getPatternByExternalUuid(uuid);
    if (!pattern) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    if (ManifestDatabase::instance().deletePattern(pattern->id) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"deleted\"}", -1);
    return ESP_OK;
}

// GET /api/pattern_download/{uuid} - download pattern file
static esp_err_t patterns_download_handler(httpd_req_t* req) {
    const char* uri = req->uri;
    const char* base = "/api/pattern_download/";

    if (strncmp(uri, base, strlen(base)) != 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    const char* uuid = uri + strlen(base);
    if (!uuid || strlen(uuid) == 0 || strlen(uuid) >= 64) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Look up pattern (uuid is external_uuid)
    auto pattern = ManifestDatabase::instance().getPatternByExternalUuid(uuid);
    if (!pattern) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Deny download of encrypted patterns
    if (pattern->encrypted) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Cannot download encrypted patterns");
        return ESP_FAIL;
    }

    // Get file path (use external_uuid for file naming)
    char file_path[128];
    getPatternFilePath(pattern->external_uuid, pattern->encrypted, file_path, sizeof(file_path));

    FILE* f = fopen(file_path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open pattern file: %s", file_path);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Get file size
    fseek(f, 0, SEEK_END);
    size_t file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    // Set response headers (no Content-Length when using chunked transfer)
    httpd_resp_set_type(req, "application/octet-stream");

    char content_disp[256];
    snprintf(content_disp, sizeof(content_disp), "attachment; filename=\"%s.thrb\"",
        pattern->name.empty() ? uuid : pattern->name.c_str());
    httpd_resp_set_hdr(req, "Content-Disposition", content_disp);

    // Stream file in chunks
    constexpr size_t CHUNK_SIZE = 4096;
    char* chunk = static_cast<char*>(
        heap_caps_malloc(CHUNK_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
    );
    if (!chunk) {
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t bytes_sent = 0;
    esp_err_t err = ESP_OK;
    while (bytes_sent < file_size) {
        size_t to_read = std::min(CHUNK_SIZE, file_size - bytes_sent);
        size_t read = fread(chunk, 1, to_read, f);
        if (read == 0) {
            break;
        }

        err = httpd_resp_send_chunk(req, chunk, read);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Send failed at %zu/%zu bytes", bytes_sent, file_size);
            break;
        }
        bytes_sent += read;
    }

    // End chunked response
    httpd_resp_send_chunk(req, nullptr, 0);

    heap_caps_free(chunk);
    fclose(f);

    if (err != ESP_OK || bytes_sent != file_size) {
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Pattern downloaded: %s (%zu bytes)", uuid, bytes_sent);
    return ESP_OK;
}

// GET /api/pattern_thumbs/{uuid}.png - serve thumbnail with long cache
static esp_err_t pattern_thumb_handler(httpd_req_t* req) {
    const char* uri = req->uri;
    const char* base = "/api/pattern_thumbs/";

    if (strncmp(uri, base, strlen(base)) != 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Extract UUID (strip .png extension)
    const char* uuid_start = uri + strlen(base);
    char uuid[64] = { 0 };
    size_t uuid_len = 0;

    // Copy until '.' or end
    while (*uuid_start && *uuid_start != '.' && uuid_len < sizeof(uuid) - 1) {
        uuid[uuid_len++] = *uuid_start++;
    }

    if (uuid_len == 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Build thumbnail path
    char thumb_path[128];
    snprintf(thumb_path, sizeof(thumb_path), "/sd/previews/%s.png", uuid);

    // Check if thumbnail exists
    struct stat st;
    if (stat(thumb_path, &st) != 0) {
        // Thumbnail not found - check if pattern exists and enqueue generation
        // uuid is external_uuid
        auto pattern = ManifestDatabase::instance().getPatternByExternalUuid(uuid);
        if (pattern && !jobs::JobQueue::instance().hasJobForPatternId(pattern->id, jobs::JobType::Thumbnail)) {
            jobs::ThumbnailJobData data;
            data.encrypted = pattern->encrypted;
            data.output_path = thumb_path;
            jobs::JobQueue::instance().enqueueThumbnail(pattern->id, data, 0);
        }
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // Open thumbnail file
    FILE* f = fopen(thumb_path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open thumbnail: %s", thumb_path);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Get file size
    size_t file_size = st.st_size;

    // Set response headers
    httpd_resp_set_type(req, "image/png");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=31536000");  // 1 year

    // Stream file in chunks (use SPIRAM for cold-path download)
    constexpr size_t CHUNK_SIZE = 4096;
    char* chunk = static_cast<char*>(
        heap_caps_malloc(CHUNK_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
        );
    if (!chunk) {
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t bytes_sent = 0;
    while (bytes_sent < file_size) {
        size_t to_read = std::min(CHUNK_SIZE, file_size - bytes_sent);
        size_t read = fread(chunk, 1, to_read, f);
        if (read == 0) break;

        if (httpd_resp_send_chunk(req, chunk, read) != ESP_OK) {
            break;
        }
        bytes_sent += read;
    }

    // End chunked response
    httpd_resp_send_chunk(req, nullptr, 0);

    heap_caps_free(chunk);
    fclose(f);
    return ESP_OK;
}

void patterns_api_register_handlers(httpd_handle_t server) {
    static httpd_uri_t patterns_list_uri = {
        .uri = "/api/patterns",
        .method = HTTP_GET,
        .handler = patterns_list_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &patterns_list_uri);

    static httpd_uri_t patterns_upload_uri = {
        .uri = "/api/patterns",
        .method = HTTP_POST,
        .handler = patterns_upload_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &patterns_upload_uri);

    static httpd_uri_t patterns_download_uri = {
        .uri = "/api/pattern_download/*",
        .method = HTTP_GET,
        .handler = patterns_download_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &patterns_download_uri);

    static httpd_uri_t pattern_thumbs_uri = {
        .uri = "/api/pattern_thumbs/*",
        .method = HTTP_GET,
        .handler = pattern_thumb_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &pattern_thumbs_uri);

    static httpd_uri_t patterns_detail_uri = {
        .uri = "/api/patterns/*",
        .method = HTTP_GET,
        .handler = patterns_detail_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &patterns_detail_uri);

    static httpd_uri_t patterns_delete_uri = {
        .uri = "/api/patterns/*",
        .method = HTTP_DELETE,
        .handler = patterns_delete_handler,
        .user_ctx = nullptr
    };
    kd_common_api_register_uri_handler(server, &patterns_delete_uri);
}
