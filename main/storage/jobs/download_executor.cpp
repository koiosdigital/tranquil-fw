#include "download_executor.h"
#include "job_queue.h"
#include "ManifestDatabase.h"
#include "PatternReader.h"
#include "EncryptedPatternReader.h"
#include "download_progress.h"
#include "drm/drm_license.h"
#include "drm/drm_purchase.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <kd_http.h>
#include <mbedtls/base64.h>

#include <cstdio>
#include <cstring>
#include <vector>

static const char* TAG = "DownloadExecutor";

static constexpr const char* PATTERNS_PATH = "/sd/patterns";

// How long to wait for the shared kd_http client if another user (OTA
// check, TZ fetch, a concurrent download job) currently holds it. Failed
// jobs are retried by the processor, so giving up here is not fatal.
static constexpr int HTTP_LOCK_TIMEOUT_MS = 30000;

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

//------------------------------------------------------------------------------
// Implementation
//------------------------------------------------------------------------------

JobResult DownloadExecutor::execute(const Job& job) {
    DownloadJobData data = DownloadJobData::fromJson(job.job_data);

    if (data.download_url.empty()) {
        return JobResult::fail("Missing download URL");
    }

    // Download jobs use pattern_external_uuid (pattern doesn't exist yet)
    return performDownload(job.pattern_external_uuid, data);
}

JobResult DownloadExecutor::performDownload(const std::string& pattern_uuid,
                                             const DownloadJobData& data) {
    ESP_LOGI(TAG, "Starting download: %s (encrypted=%d, purchased=%d, expected=%lld bytes)",
        pattern_uuid.c_str(), data.encrypted, data.hasPurchaseReceipt(),
        (long long)data.size_bytes);
    ESP_LOGI(TAG, "  URL: %s", data.download_url.c_str());

    // 10% = job picked up by a worker
    DownloadProgressBroadcaster::instance().report(pattern_uuid, 10);

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

    // Download to a neutral temp path first. The final location and the
    // processing pipeline are decided by sniffing the file's magic bytes —
    // the server's encrypted flag has been observed wrong (KDEP payloads
    // announced as unencrypted), and the content is authoritative.
    char file_path[256];
    snprintf(file_path, sizeof(file_path), "%s/%s.tmp",
             PATTERNS_PATH, pattern_uuid.c_str());

    // Open output file
    FileHandle file(file_path, "wb");
    if (!file.isOpen()) {
        ESP_LOGE(TAG, "Failed to open output file: %s", file_path);
        return JobResult::fail("Failed to create output file");
    }

    // HTTP event handler context
    struct DownloadContext {
        FileHandle* file;
        const char* pattern_uuid;
        size_t bytes_written = 0;
        size_t next_progress_log = 0;
        uint8_t last_progress_pct = 0;
        size_t expected_size = 0;  // metadata size, fallback when no Content-Length
        esp_err_t error = ESP_OK;
    };

    DownloadContext ctx = {&file, pattern_uuid.c_str()};
    // Rough denominator for chunked responses that omit Content-Length; the
    // server's metadata size is imperfect but keeps the bar moving.
    ctx.expected_size = data.size_bytes;

    // Event handler that streams to file
    auto event_handler = [](esp_http_client_event_t* evt) -> esp_err_t {
        auto* ctx = static_cast<DownloadContext*>(evt->user_data);
        if (!ctx || !ctx->file) return ESP_FAIL;

        switch (evt->event_id) {
            case HTTP_EVENT_ON_CONNECTED:
                ESP_LOGI(TAG, "HTTP connected");
                // Bridge the 10 (picked up) -> 20 (first bytes) gap so the bar
                // doesn't stall through TLS setup on a slow link.
                DownloadProgressBroadcaster::instance().report(ctx->pattern_uuid, 15);
                break;
            case HTTP_EVENT_ON_DATA:
                if (evt->data && evt->data_len > 0) {
                    size_t written = ctx->file->write(evt->data, evt->data_len);
                    if (written != static_cast<size_t>(evt->data_len)) {
                        ESP_LOGE(TAG, "Write failed: %zu != %d", written, evt->data_len);
                        ctx->error = ESP_FAIL;
                        return ESP_FAIL;
                    }
                    ctx->bytes_written += written;
                    if (ctx->bytes_written >= ctx->next_progress_log) {
                        ESP_LOGI(TAG, "  received %zu bytes", ctx->bytes_written);
                        ctx->next_progress_log = ctx->bytes_written + 65536;
                    }
                    // Map byte progress into the 20-80 band, in 5% steps.
                    {
                        int64_t total = esp_http_client_get_content_length(evt->client);
                        size_t denom = total > 0 ? static_cast<size_t>(total)
                                                 : ctx->expected_size;
                        uint8_t pct;
                        if (denom > 0) {
                            pct = 20 + static_cast<uint8_t>(
                                (ctx->bytes_written * 60) / denom);
                            if (pct > 80) pct = 80;
                        } else {
                            // Truly unknown length (chunked, no metadata): creep
                            // 20->75 by volume (~+1% per 16 KiB) so the bar still
                            // advances instead of freezing at 10 until 80.
                            size_t creep = ctx->bytes_written / 16384;
                            pct = 20 + static_cast<uint8_t>(creep > 55 ? 55 : creep);
                        }
                        pct -= pct % 5;
                        if (pct > ctx->last_progress_pct) {
                            ctx->last_progress_pct = pct;
                            DownloadProgressBroadcaster::instance().report(
                                ctx->pattern_uuid, pct);
                        }
                    }
                }
                break;
            case HTTP_EVENT_ERROR:
                ESP_LOGW(TAG, "HTTP transport error event");
                break;
            case HTTP_EVENT_DISCONNECTED:
                ESP_LOGI(TAG, "HTTP disconnected (%zu bytes received)", ctx->bytes_written);
                break;
            default:
                break;
        }
        return ESP_OK;
    };

    // Use the app-wide shared HTTP client: serializes TLS with the other
    // HTTP users (OTA check, TZ fetch) and reuses the kept-alive connection
    // across back-to-back downloads from the same host.
    int64_t t_acquire = esp_timer_get_time();
    esp_http_client_handle_t client = kd_http_acquire(
        data.download_url.c_str(), event_handler, &ctx, HTTP_LOCK_TIMEOUT_MS);
    if (client == nullptr) {
        ESP_LOGE(TAG, "kd_http busy for %dms, giving up (job will retry)",
            HTTP_LOCK_TIMEOUT_MS);
        file.close();
        remove(file_path);
        return JobResult::fail("HTTP client busy");
    }

    int64_t t_start = esp_timer_get_time();
    if (t_start - t_acquire > 100000) {
        ESP_LOGI(TAG, "Waited %lldms for shared HTTP client",
            (long long)((t_start - t_acquire) / 1000));
    }

    // Perform download
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    int64_t elapsed_ms = (esp_timer_get_time() - t_start) / 1000;
    ESP_LOGI(TAG, "HTTP perform: err=%s status=%d bytes=%zu elapsed=%lldms",
        esp_err_to_name(err), status, ctx.bytes_written, (long long)elapsed_ms);
    if (err != ESP_OK) {
        // Transport-level failure: drop the possibly-poisoned connection
        // so the next acquire starts clean.
        kd_http_invalidate();
    }
    kd_http_release();

    if (err != ESP_OK || ctx.error != ESP_OK) {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
        file.close();
        remove(file_path);
        return JobResult::fail("Download failed: " + std::string(esp_err_to_name(err)));
    }

    if (status != 200) {
        ESP_LOGE(TAG, "HTTP status: %d", status);
        file.close();
        remove(file_path);
        // Auth/not-found responses never resolve on retry — fail permanently
        if (status == 401 || status == 403 || status == 404) {
            return JobResult::fail_permanent("HTTP error: " + std::to_string(status));
        }
        return JobResult::fail("HTTP error: " + std::to_string(status));
    }

    file.close();

    // Do NOT validate against the metadata size: the server's size_bytes does
    // not describe the download payload (observed consistently smaller bodies
    // with a clean 200 + matching Content-Length). Truncation is already
    // caught by esp_http_client (ESP_ERR_HTTP_INCOMPLETE_DATA), and the magic
    // sniff below rejects unusable files.
    if (data.size_bytes > 0 &&
        ctx.bytes_written != static_cast<size_t>(data.size_bytes)) {
        ESP_LOGW(TAG, "Metadata size %lld != payload size %zu (informational)",
            (long long)data.size_bytes, ctx.bytes_written);
    }

    ESP_LOGI(TAG, "Downloaded %zu bytes -> %s", ctx.bytes_written, file_path);

    // 80% = transfer done; conversion (if any) runs before the thumbnailer's 90
    DownloadProgressBroadcaster::instance().report(pattern_uuid, 80);

    // Sniff the actual format from the file's magic bytes.
    enum class PatternFormat { Kdep, Thrb, Text };
    PatternFormat format = PatternFormat::Text;
    {
        FileHandle sniff(file_path, "rb");
        uint32_t magic = 0;
        if (!sniff.isOpen() ||
            fread(&magic, 1, sizeof(magic), sniff.get()) != sizeof(magic)) {
            ESP_LOGE(TAG, "Downloaded file unreadable: %s", file_path);
            remove(file_path);
            return JobResult::fail("Downloaded file unreadable");
        }
        if (magic == ENCRYPTED_PATTERN_MAGIC) {
            format = PatternFormat::Kdep;
        } else if (magic == BINARY_PATTERN_MAGIC) {
            format = PatternFormat::Thrb;
        }
    }

    const bool is_encrypted = (format == PatternFormat::Kdep);
    if (is_encrypted != data.encrypted) {
        ESP_LOGW(TAG, "Server metadata said encrypted=%d but content is %s — trusting content",
            data.encrypted,
            format == PatternFormat::Kdep ? "KDEP" :
            format == PatternFormat::Thrb ? "THRB" : "text");
    }

    // Move to the final location: binary formats (KDEP/THRB) live at
    // <uuid>.dat; raw .thr text keeps a .thr name as conversion input.
    char final_path[256];
    snprintf(final_path, sizeof(final_path), "%s/%s.%s",
             PATTERNS_PATH, pattern_uuid.c_str(),
             format == PatternFormat::Text ? "thr" : "dat");
    if (rename(file_path, final_path) != 0) {
        ESP_LOGE(TAG, "Failed to move %s -> %s", file_path, final_path);
        remove(file_path);
        return JobResult::fail("Failed to finalize downloaded file");
    }

    // Build pattern info for database
    Pattern pattern_info;
    pattern_info.id = 0;  // Assigned by ManifestDatabase
    pattern_info.external_uuid = pattern_uuid;  // Server UUID for linking
    pattern_info.name = data.pattern_name;
    pattern_info.creator = data.pattern_creator;
    pattern_info.encrypted = is_encrypted;
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
            remove(final_path);
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
            remove(final_path);
            return JobResult::fail("Failed to decode receipt signature");
        }

        // Save the purchase receipt (verifies signature internally)
        esp_err_t save_err = drm_purchase_save(pattern_uuid.c_str(),
            payload.data(), payload_len, signature.data(), sig_len);

        if (save_err == ESP_OK) {
            ESP_LOGD(TAG, "Saved purchase receipt for pattern: %s", pattern_uuid.c_str());
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

    // Add pattern to manifest database. A row may already exist for this
    // UUID (e.g. a reboot after addPattern but before markCompleted re-ran
    // the job) — reuse/update it instead of inserting a duplicate.
    auto existing = ManifestDatabase::instance().getPatternByExternalUuid(pattern_uuid);
    if (existing) {
        ESP_LOGW(TAG, "Pattern %s already in manifest (id=%u), updating instead of duplicating",
            pattern_uuid.c_str(), existing->id);
        pattern_info.id = existing->id;
        if (ManifestDatabase::instance().updatePattern(existing->id, pattern_info) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to update existing pattern in manifest");
            remove(final_path);
            return JobResult::fail("Failed to update manifest");
        }
    }
    else if (ManifestDatabase::instance().addPattern(pattern_info) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add pattern to manifest");
        remove(final_path);
        return JobResult::fail("Failed to add to manifest");
    }

    // Get the newly created pattern's internal ID for thumbnail job
    auto created_pattern = ManifestDatabase::instance().getPatternByExternalUuid(pattern_uuid);
    if (!created_pattern) {
        ESP_LOGE(TAG, "Failed to find newly created pattern");
        remove(final_path);
        return JobResult::fail("Failed to find created pattern");
    }

    if (format == PatternFormat::Text) {
        // Raw .thr text: the player and thumbnail renderer both read THRB
        // binary from <uuid>.dat, so convert first — the conversion job
        // chains the thumbnail itself and deletes the .thr input on success.
        ConversionJobData conv_data;
        conv_data.temp_path = final_path;
        conv_data.name = data.pattern_name;
        conv_data.encrypted = false;
        JobQueue::instance().enqueueConversion(created_pattern->id, conv_data, -1);
        ESP_LOGI(TAG, "Download complete: %s (pattern id=%u, conversion queued)",
            pattern_uuid.c_str(), created_pattern->id);
    } else {
        // Already in final binary form (KDEP or THRB) — thumbnail directly.
        ThumbnailJobData thumb_data;
        thumb_data.encrypted = is_encrypted;
        thumb_data.output_path = "/sd/previews/" + pattern_uuid + ".png";
        JobQueue::instance().enqueueThumbnail(created_pattern->id, thumb_data, -1);
        ESP_LOGI(TAG, "Download complete: %s (pattern id=%u, %s, thumbnail queued)",
            pattern_uuid.c_str(), created_pattern->id,
            is_encrypted ? "KDEP" : "THRB");
    }
    return JobResult::ok();
}

} // namespace jobs
