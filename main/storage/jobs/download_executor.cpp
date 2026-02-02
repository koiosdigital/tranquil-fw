#include "download_executor.h"
#include "job_queue.h"
#include "ManifestDatabase.h"
#include "drm/drm_license.h"
#include "drm/drm_purchase.h"

#include <esp_log.h>
#include <esp_http_client.h>
#include <esp_crt_bundle.h>
#include <mbedtls/base64.h>

#include <cstdio>
#include <cstring>
#include <vector>

static const char* TAG = "DownloadExecutor";

static constexpr size_t HTTP_BUFFER_SIZE = 4096;
static constexpr const char* PATTERNS_PATH = "/sd/patterns";

namespace jobs {

//------------------------------------------------------------------------------
// RAII Wrappers
//------------------------------------------------------------------------------

class FileHandle {
public:
    FileHandle() = default;
    explicit FileHandle(const char* path, const char* mode) {
        file_ = fopen(path, mode);
    }
    ~FileHandle() { close(); }

    FileHandle(FileHandle&& other) noexcept : file_(other.file_) {
        other.file_ = nullptr;
    }
    FileHandle& operator=(FileHandle&& other) noexcept {
        if (this != &other) {
            close();
            file_ = other.file_;
            other.file_ = nullptr;
        }
        return *this;
    }

    FileHandle(const FileHandle&) = delete;
    FileHandle& operator=(const FileHandle&) = delete;

    bool isOpen() const { return file_ != nullptr; }
    FILE* get() const { return file_; }

    void close() {
        if (file_) {
            fclose(file_);
            file_ = nullptr;
        }
    }

    size_t write(const void* data, size_t size) {
        return file_ ? fwrite(data, 1, size, file_) : 0;
    }

private:
    FILE* file_ = nullptr;
};

class HttpClient {
public:
    HttpClient() = default;
    explicit HttpClient(const esp_http_client_config_t* config) {
        client_ = esp_http_client_init(config);
    }
    ~HttpClient() { cleanup(); }

    HttpClient(HttpClient&& other) noexcept : client_(other.client_) {
        other.client_ = nullptr;
    }
    HttpClient& operator=(HttpClient&& other) noexcept {
        if (this != &other) {
            cleanup();
            client_ = other.client_;
            other.client_ = nullptr;
        }
        return *this;
    }

    HttpClient(const HttpClient&) = delete;
    HttpClient& operator=(const HttpClient&) = delete;

    bool isValid() const { return client_ != nullptr; }
    esp_http_client_handle_t get() const { return client_; }

    void cleanup() {
        if (client_) {
            esp_http_client_cleanup(client_);
            client_ = nullptr;
        }
    }

    esp_err_t perform() {
        return client_ ? esp_http_client_perform(client_) : ESP_ERR_INVALID_STATE;
    }

    int getStatusCode() const {
        return client_ ? esp_http_client_get_status_code(client_) : -1;
    }

private:
    esp_http_client_handle_t client_ = nullptr;
};

//------------------------------------------------------------------------------
// Implementation
//------------------------------------------------------------------------------

JobResult DownloadExecutor::execute(const Job& job) {
    DownloadJobData data = DownloadJobData::fromJson(job.job_data);

    if (data.download_url.empty()) {
        return JobResult::fail("Missing download URL");
    }

    return performDownload(job.pattern_uuid, data);
}

JobResult DownloadExecutor::performDownload(const std::string& pattern_uuid,
                                             const DownloadJobData& data) {
    ESP_LOGI(TAG, "Starting download: %s", pattern_uuid.c_str());
    ESP_LOGI(TAG, "  URL: %s", data.download_url.c_str());
    ESP_LOGI(TAG, "  Encrypted: %s", data.encrypted ? "yes" : "no");
    ESP_LOGI(TAG, "  Purchased: %s", data.hasPurchaseReceipt() ? "yes" : "no");

    // For subscription patterns (no receipt), check license validity and limits
    // For purchased patterns (has receipt), we can download without a valid subscription
    if (!data.hasPurchaseReceipt()) {
        if (!drm_license_is_valid()) {
            return JobResult::fail("License invalid");
        }

        if (!drm_license_can_download()) {
            return JobResult::fail("Pattern download limit reached");
        }
    }

    // Determine output file path
    char file_path[256];
    if (data.encrypted) {
        snprintf(file_path, sizeof(file_path), "%s/%s.dat",
                 PATTERNS_PATH, pattern_uuid.c_str());
    } else {
        snprintf(file_path, sizeof(file_path), "%s/%s.thr",
                 PATTERNS_PATH, pattern_uuid.c_str());
    }

    // Open output file
    FileHandle file(file_path, "wb");
    if (!file.isOpen()) {
        ESP_LOGE(TAG, "Failed to open output file: %s", file_path);
        return JobResult::fail("Failed to create output file");
    }

    // HTTP event handler context
    struct DownloadContext {
        FileHandle* file;
        size_t bytes_written = 0;
        esp_err_t error = ESP_OK;
    };

    DownloadContext ctx = {&file};

    // Event handler that streams to file
    auto event_handler = [](esp_http_client_event_t* evt) -> esp_err_t {
        auto* ctx = static_cast<DownloadContext*>(evt->user_data);
        if (!ctx || !ctx->file) return ESP_FAIL;

        switch (evt->event_id) {
            case HTTP_EVENT_ON_DATA:
                if (evt->data && evt->data_len > 0) {
                    size_t written = ctx->file->write(evt->data, evt->data_len);
                    if (written != static_cast<size_t>(evt->data_len)) {
                        ESP_LOGE(TAG, "Write failed: %zu != %d", written, evt->data_len);
                        ctx->error = ESP_FAIL;
                        return ESP_FAIL;
                    }
                    ctx->bytes_written += written;
                }
                break;
            default:
                break;
        }
        return ESP_OK;
    };

    esp_http_client_config_t config = {};
    config.url = data.download_url.c_str();
    config.event_handler = event_handler;
    config.user_data = &ctx;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.buffer_size = HTTP_BUFFER_SIZE;
    config.timeout_ms = 30000;

    HttpClient client(&config);
    if (!client.isValid()) {
        file.close();
        remove(file_path);
        return JobResult::fail("Failed to init HTTP client");
    }

    // Perform download
    esp_err_t err = client.perform();
    if (err != ESP_OK || ctx.error != ESP_OK) {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
        file.close();
        remove(file_path);
        return JobResult::fail("Download failed: " + std::string(esp_err_to_name(err)));
    }

    int status = client.getStatusCode();
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP status: %d", status);
        file.close();
        remove(file_path);
        return JobResult::fail("HTTP error: " + std::to_string(status));
    }

    file.close();
    ESP_LOGI(TAG, "Downloaded %zu bytes", ctx.bytes_written);

    // Build pattern info for database
    Pattern pattern_info;
    pattern_info.uuid = pattern_uuid;
    pattern_info.name = data.pattern_name;
    pattern_info.creator = data.pattern_creator;
    pattern_info.encrypted = data.encrypted;
    pattern_info.size_bytes = data.size_bytes;
    pattern_info.reversible = data.reversible;
    pattern_info.start_point = data.start_point;
    pattern_info.created_at = data.created_at;

    // If we have a purchase receipt, decode and save it
    if (data.hasPurchaseReceipt()) {
        // Decode base64 payload
        size_t payload_len = 0;
        mbedtls_base64_decode(nullptr, 0, &payload_len,
            reinterpret_cast<const unsigned char*>(data.receipt_payload_b64.c_str()),
            data.receipt_payload_b64.size());

        std::vector<uint8_t> payload(payload_len);
        if (mbedtls_base64_decode(payload.data(), payload.size(), &payload_len,
            reinterpret_cast<const unsigned char*>(data.receipt_payload_b64.c_str()),
            data.receipt_payload_b64.size()) != 0) {
            ESP_LOGE(TAG, "Failed to decode receipt payload");
            remove(file_path);
            return JobResult::fail("Failed to decode receipt payload");
        }

        // Decode base64 signature
        size_t sig_len = 0;
        mbedtls_base64_decode(nullptr, 0, &sig_len,
            reinterpret_cast<const unsigned char*>(data.receipt_signature_b64.c_str()),
            data.receipt_signature_b64.size());

        std::vector<uint8_t> signature(sig_len);
        if (mbedtls_base64_decode(signature.data(), signature.size(), &sig_len,
            reinterpret_cast<const unsigned char*>(data.receipt_signature_b64.c_str()),
            data.receipt_signature_b64.size()) != 0) {
            ESP_LOGE(TAG, "Failed to decode receipt signature");
            remove(file_path);
            return JobResult::fail("Failed to decode receipt signature");
        }

        // Save the purchase receipt (verifies signature internally)
        esp_err_t save_err = drm_purchase_save(pattern_uuid.c_str(),
            payload.data(), payload_len, signature.data(), sig_len);

        if (save_err == ESP_OK) {
            ESP_LOGI(TAG, "Saved purchase receipt for pattern: %s", pattern_uuid.c_str());
            pattern_info.purchased = true;

            // Try to get purchase info for the timestamp and receipt_id
            drm_purchase_info_t purchase_info;
            if (drm_purchase_verify(pattern_uuid.c_str(), &purchase_info) == ESP_OK) {
                pattern_info.purchased_at = purchase_info.purchased_at;
                pattern_info.receipt_id = purchase_info.receipt_id;
            }
        } else {
            ESP_LOGW(TAG, "Failed to save purchase receipt: %s (continuing anyway)", esp_err_to_name(save_err));
            // Don't fail the download, just log the warning - pattern can still be used with subscription
        }
    }

    // Add pattern to manifest database
    if (ManifestDatabase::instance().addPattern(pattern_info) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add pattern to manifest");
        remove(file_path);
        return JobResult::fail("Failed to add to manifest");
    }

    // Enqueue thumbnail generation job
    ThumbnailJobData thumb_data;
    thumb_data.encrypted = data.encrypted;
    thumb_data.output_path = "/sd/previews/" + pattern_uuid + ".png";
    JobQueue::instance().enqueueThumbnail(pattern_uuid, thumb_data, -1);

    ESP_LOGI(TAG, "Download complete: %s -> %s", pattern_uuid.c_str(), file_path);
    return JobResult::ok();
}

} // namespace jobs
