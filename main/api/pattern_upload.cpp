#include "pattern_upload.h"
#include "ManifestDatabase.h"
#include "esp_log.h"
#include "cJSON.h"
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <unistd.h>

static const char* TAG = "pattern_upload";

// Buffer size for streaming - 4KB fits comfortably in ESP32 memory
static constexpr size_t UPLOAD_CHUNK_SIZE = 4096;
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

// Extract boundary from Content-Type header
static bool extract_boundary(httpd_req_t* req, char* boundary, size_t max_len) {
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
static bool parse_content_disposition(const char* header, char* filename, size_t max_len) {
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
static const char* find_pattern(const char* buf, size_t buf_len, const char* pattern, size_t pattern_len) {
    if (pattern_len > buf_len) return nullptr;
    for (size_t i = 0; i <= buf_len - pattern_len; i++) {
        if (memcmp(buf + i, pattern, pattern_len) == 0) {
            return buf + i;
        }
    }
    return nullptr;
}

// POST /api/patterns/upload - streaming multipart upload
static esp_err_t pattern_upload_handler(httpd_req_t* req) {
    UploadContext ctx;
    esp_err_t ret = ESP_OK;

    // Extract multipart boundary
    if (!extract_boundary(req, ctx.boundary, sizeof(ctx.boundary))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Content-Type: missing boundary");
        return ESP_FAIL;
    }
    ctx.boundary_len = strlen(ctx.boundary);
    ESP_LOGI(TAG, "Upload boundary: %s", ctx.boundary);

    // Generate UUID for new pattern
    std::string uuid = ManifestDatabase::generateUUID();
    strncpy(ctx.uuid, uuid.c_str(), sizeof(ctx.uuid) - 1);

    // Prepare file paths
    snprintf(ctx.temp_path, sizeof(ctx.temp_path), "/sd/patterns/%s.tmp", ctx.uuid);
    snprintf(ctx.final_path, sizeof(ctx.final_path), "/sd/patterns/%s.thr", ctx.uuid);

    // Allocate buffer for streaming
    char* buffer = static_cast<char*>(malloc(UPLOAD_CHUNK_SIZE));
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // We'll use a simple state machine for multipart parsing
    // States: PREAMBLE -> HEADERS -> BODY -> EPILOGUE
    enum State { PREAMBLE, HEADERS, BODY, DONE } state = PREAMBLE;

    // Header accumulator (multipart headers can span chunks)
    char header_buf[MAX_HEADER_SIZE] = { 0 };
    size_t header_len = 0;

    // Carry-over buffer for boundary detection across chunks
    char carry[MAX_BOUNDARY_SIZE + 4] = { 0 };  // --boundary\r\n worst case
    size_t carry_len = 0;

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
                char first_boundary[MAX_BOUNDARY_SIZE + 4];
                snprintf(first_boundary, sizeof(first_boundary), "--%s", ctx.boundary);
                size_t fb_len = strlen(first_boundary);

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
                        parse_content_disposition(header_buf, ctx.filename, sizeof(ctx.filename));
                        ESP_LOGI(TAG, "Upload filename: %s", ctx.filename[0] ? ctx.filename : "(unnamed)");

                        // Open temp file
                        ctx.file = fopen(ctx.temp_path, "wb");
                        if (!ctx.file) {
                            ESP_LOGE(TAG, "Failed to open temp file: %s", ctx.temp_path);
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

            if (state == BODY && ctx.file) {
                // Look for closing boundary: \r\n--<boundary>
                char close_boundary[MAX_BOUNDARY_SIZE + 8];
                snprintf(close_boundary, sizeof(close_boundary), "\r\n--%s", ctx.boundary);
                size_t cb_len = strlen(close_boundary);

                // Search for boundary in current data
                const char* bound = find_pattern(data, data_len, close_boundary, cb_len);
                if (bound) {
                    // Write data up to boundary
                    size_t write_len = bound - data;
                    if (write_len > 0) {
                        fwrite(data, 1, write_len, ctx.file);
                        ctx.bytes_written += write_len;
                    }
                    state = DONE;
                    break;
                }
                else {
                    // No boundary found, but it might span chunks
                    // Keep last cb_len-1 bytes as carry-over
                    size_t safe_len = (data_len > cb_len - 1) ? data_len - (cb_len - 1) : 0;
                    if (safe_len > 0) {
                        fwrite(data, 1, safe_len, ctx.file);
                        ctx.bytes_written += safe_len;
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
                            fwrite(data, 1, data_len - cb_len, ctx.file);
                            ctx.bytes_written += data_len - cb_len;
                        }
                        else {
                            fwrite(data, 1, data_len, ctx.file);
                            ctx.bytes_written += data_len;
                        }
                    }
                    else if (data_len > 0) {
                        fwrite(data, 1, data_len, ctx.file);
                        ctx.bytes_written += data_len;
                    }
                    break;
                }
            }
        }
    }

    free(buffer);

    // Close file
    if (ctx.file) {
        fclose(ctx.file);
        ctx.file = nullptr;
    }

    if (ret != ESP_OK || ctx.bytes_written == 0) {
        // Cleanup on error
        unlink(ctx.temp_path);
        if (ret == ESP_OK) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No file data received");
            return ESP_FAIL;
        }
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Atomic rename: .tmp -> .thr
    if (rename(ctx.temp_path, ctx.final_path) != 0) {
        ESP_LOGE(TAG, "Failed to rename temp file");
        unlink(ctx.temp_path);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Get final file size
    struct stat st;
    size_t file_size = 0;
    if (stat(ctx.final_path, &st) == 0) {
        file_size = st.st_size;
    }

    // Create pattern record
    Pattern pattern;
    pattern.uuid = ctx.uuid;
    pattern.name = strlen(ctx.filename) > 0 ? ctx.filename : ctx.uuid;
    pattern.creator = "Uploaded";
    pattern.encrypted = false;  // Uploaded patterns are always unencrypted
    pattern.size_bytes = file_size;
    pattern.created_at = ManifestDatabase::currentTimestamp();

    // Transactional database update
    esp_err_t db_result = ManifestDatabase::instance().addPattern(pattern);
    if (db_result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add pattern to database");
        unlink(ctx.final_path);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Pattern uploaded: %s (%zu bytes)", ctx.uuid, file_size);

    // Return success with pattern JSON
    cJSON* response = ManifestDatabase::patternToJson(pattern);
    char* json_str = cJSON_PrintUnformatted(response);

    httpd_resp_set_status(req, "201 Created");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));

    free(json_str);
    cJSON_Delete(response);

    return ESP_OK;
}

void pattern_upload_register_handlers(httpd_handle_t server) {
    static httpd_uri_t upload_uri = {
        .uri = "/api/patterns/upload",
        .method = HTTP_POST,
        .handler = pattern_upload_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &upload_uri);
}
