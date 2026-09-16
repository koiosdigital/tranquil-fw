// Message handlers for incoming cloud WebSocket messages
#include "handlers.h"
#include "sockets.h"
#include "messages.h"
#include "PatternDownloader.h"
#include "message_dispatcher.h"

#include <esp_log.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <kd_common.h>

#include <atomic>
#include <cstring>
#include <vector>

#include "drm/drm_license.h"
#include "drm/drm_purchase.h"
#include "ManifestDatabase.h"
#include "download_progress.h"
#include "download_tracker.h"
#include "library_notifier.h"
#include "player_state_broadcaster.h"

static const char* TAG = "cloud_handlers";

namespace {

    void handle_license_response(Kd__V1__LicenseResponse* response) {
        if (response == nullptr) return;

        if (!response->success) {
            ESP_LOGE(TAG, "License request failed: %s",
                response->error ? response->error : "unknown error");
            return;
        }

        // Freshness hit: the license we hold is the one the server last
        // issued and it is not near expiry. Nothing to verify or re-save.
        if (response->unchanged) {
            ESP_LOGI(TAG, "License up to date");
            return;
        }

        if (response->license == nullptr) {
            ESP_LOGE(TAG, "License response missing license payload");
            return;
        }

        if (response->signature.data == nullptr || response->signature.len == 0) {
            ESP_LOGE(TAG, "License response missing signature");
            return;
        }

        ESP_LOGI(TAG, "Received license response");

        // Serialize the license payload for verification and storage
        size_t payload_len = kd__v1__license_payload__get_packed_size(response->license);
        auto* payload_buf = static_cast<uint8_t*>(heap_caps_malloc(payload_len, MALLOC_CAP_SPIRAM));
        if (payload_buf == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate license payload buffer");
            return;
        }

        kd__v1__license_payload__pack(response->license, payload_buf);

        // Save the license (this verifies signature internally)
        esp_err_t ret = drm_license_save(payload_buf, payload_len,
            response->signature.data, response->signature.len);

        heap_caps_free(payload_buf);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "License saved successfully");
            ESP_LOGI(TAG, "  Device: %s", response->license->for_device ? response->license->for_device : "");
            ESP_LOGI(TAG, "  Max patterns: %u", response->license->max_patterns);
            ESP_LOGI(TAG, "  License ID: %s", response->license->license_id ? response->license->license_id : "");
        }
        else {
            ESP_LOGE(TAG, "Failed to save license: %s", esp_err_to_name(ret));
        }
    }

    void handle_pattern_download_response(Kd__V1__PatternDownloadResponse* response) {
        if (response == nullptr) return;

        ESP_LOGI(TAG, "Pattern download response:");
        ESP_LOGI(TAG, "  UUID: %s", response->pattern_uuid ? response->pattern_uuid : "");
        ESP_LOGI(TAG, "  URL: %s", response->download_url ? response->download_url : "");

        // An empty download_url is the server's terminal decline (device not
        // claimed to an account, pattern missing from the catalog, ...). Fail
        // the tracked download so clients clear their UI instead of hanging
        // at 0%, and drop the requester routing.
        if (response->pattern_uuid &&
            (response->download_url == nullptr || response->download_url[0] == '\0')) {
            ESP_LOGW(TAG, "Cloud declined pattern download for %s", response->pattern_uuid);
            DownloadProgressBroadcaster::instance().fail(
                response->pattern_uuid, "Download refused by server");
            DownloadTracker::instance().removeDownload(response->pattern_uuid);
            return;
        }

        // Ensure the download is tracked from 0% even when it was cloud-initiated
        // (no local RequestPatternDownload ran to call track()). No-op if already
        // tracked from the local request path.
        if (response->pattern_uuid) {
            DownloadProgressBroadcaster::instance().track(response->pattern_uuid);
        }

        // Queue the download with completion callback
        auto result = PatternDownloader::instance().queueDownload(response,
            [](const DownloadResult& result) {
                if (result.success()) {
                    ESP_LOGI("cloud_handlers", "Pattern %s downloaded successfully",
                        result.pattern_uuid.c_str());
                }
                else {
                    ESP_LOGE("cloud_handlers", "Pattern %s download failed: %s (%s)",
                        result.pattern_uuid.c_str(),
                        downloadStatusToString(result.status),
                        result.error.c_str());
                }
            });

        if (result.status == DownloadStatus::Queued ||
            result.status == DownloadStatus::InProgress) {
            ESP_LOGI(TAG, "Download %s: %s",
                result.pattern_uuid.c_str(), downloadStatusToString(result.status));
        }
        else {
            ESP_LOGE(TAG, "Failed to queue download: %s - %s",
                downloadStatusToString(result.status), result.error.c_str());
            // Never reached the job queue, so the job-complete failure path
            // won't fire — emit the terminal signal here.
            if (response->pattern_uuid) {
                DownloadProgressBroadcaster::instance().fail(
                    response->pattern_uuid,
                    result.error.empty() ? "Failed to queue download" : result.error);
                DownloadTracker::instance().removeDownload(response->pattern_uuid);
            }
        }
    }

    void handle_join_response(Kd__V1__JoinResponse* response) {
        if (response == nullptr) return;

        ESP_LOGI(TAG, "Join response: claimed=%d, needs_claimed=%d",
            response->is_claimed, response->needs_claimed);

        bool needs_claim = response->needs_claimed || !response->is_claimed;
        if (!needs_claim) {
            kd_common_clear_claim_token();
        }
        else {
            cloud_msg_send_claim_if_needed();
        }
    }

    void handle_cert_renew_required(Kd__V1__CertRenewRequired* request) {
        if (request != nullptr && request->reason != nullptr) {
            ESP_LOGI(TAG, "Cert renewal required: %s", request->reason);
        }
        else {
            ESP_LOGI(TAG, "Cert renewal required");
        }

        // Get CSR - it should already exist from provisioning
        size_t csr_len = 0;
        if (kd_common_get_csr(nullptr, &csr_len) != ESP_OK || csr_len == 0) {
            ESP_LOGE(TAG, "No CSR available for renewal");
            return;
        }

        auto* csr_buf = static_cast<char*>(heap_caps_malloc(csr_len + 1, MALLOC_CAP_SPIRAM));
        if (csr_buf == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate CSR buffer");
            return;
        }

        if (kd_common_get_csr(csr_buf, &csr_len) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get CSR");
            heap_caps_free(csr_buf);
            return;
        }
        csr_buf[csr_len] = '\0';

        cloud_msg_send_cert_renew_request(csr_buf, csr_len);
        heap_caps_free(csr_buf);
    }

    void handle_cert_renew_response(Kd__V1__CertRenewResponse* response) {
        if (response == nullptr) return;

        if (!response->success) {
            ESP_LOGE(TAG, "Cert renewal failed: %s",
                response->error ? response->error : "unknown error");
            return;
        }

        if (response->device_cert.data == nullptr || response->device_cert.len == 0) {
            ESP_LOGE(TAG, "Cert renewal response missing certificate");
            return;
        }

        ESP_LOGI(TAG, "Received new certificate (%zu bytes)", response->device_cert.len);

        esp_err_t err = kd_common_set_device_cert(
            reinterpret_cast<const char*>(response->device_cert.data),
            response->device_cert.len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to store new certificate: %s", esp_err_to_name(err));
            return;
        }

        ESP_LOGI(TAG, "Certificate renewed successfully");
        // cloudlink caches the device cert for the process lifetime, so a plain
        // reconnect would re-present the OLD cert. Force a deinit+reinit so the
        // next connection fetches the freshly stored one.
        sockets_invalidate_cert_cache();
    }

    // --- Purchase receipt sync -------------------------------------------
    //
    // The server answers SyncPurchasesRequest with one bundle per owned
    // pattern. Each receipt costs an RSA verify + a file write, and stamping
    // ownership on the manifest used to REWRITE (and fsync) the whole
    // patterns snapshot once per receipt — a 100-receipt sync was seconds of
    // SD traffic on cloudlink's event task, stalling every other cloud
    // message. Now the bundles are copied out of the protobuf (PSRAM), the
    // work moves to a short-lived task, receipt files are written first (no
    // manifest lock held across the RSA work), and the manifest rows are
    // updated inside ONE BatchScope: a single snapshot save at the end.

    struct ReceiptCopy {
        char pattern_uuid[40];
        uint16_t payload_len;
        uint16_t signature_len;
        uint8_t payload[512];    // drm_purchase_verify rejects larger anyway
        uint8_t signature[512];
    };

    struct ReceiptBatch {
        ReceiptCopy* items;  // PSRAM, count entries
        size_t count;
    };

    std::atomic<bool> s_receipt_sync_running{ false };

    void process_receipt_batch(const ReceiptBatch& batch) {
        struct Owned {
            const ReceiptCopy* receipt;
            int64_t purchased_at;
            std::string receipt_id;
        };
        std::vector<Owned> owned;
        owned.reserve(batch.count);
        size_t failed = 0;

        // Phase 1: verify + save every receipt file, and read back the
        // ownership metadata while the receipt is fresh. All the crypto
        // happens here, with no manifest lock held.
        for (size_t i = 0; i < batch.count; i++) {
            const ReceiptCopy& r = batch.items[i];
            esp_err_t err = drm_purchase_save(r.pattern_uuid,
                r.payload, r.payload_len, r.signature, r.signature_len);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to save receipt for %s: %s",
                    r.pattern_uuid, esp_err_to_name(err));
                failed++;
                continue;
            }
            ESP_LOGI(TAG, "Saved receipt for pattern: %s", r.pattern_uuid);

            Owned o{ &r, 0, "" };
            drm_purchase_info_t info;
            if (drm_purchase_verify(r.pattern_uuid, &info) == ESP_OK) {
                o.purchased_at = info.purchased_at;
                o.receipt_id = info.receipt_id;
            }
            owned.push_back(std::move(o));
        }

        // Phase 2: stamp ownership on the manifest rows that exist locally
        // (pattern_uuid is the external uuid). One batch = one snapshot
        // write, regardless of how many rows change.
        size_t updated_rows = 0;
        if (!owned.empty()) {
            ManifestDatabase::BatchScope batch_scope;
            auto& db = ManifestDatabase::instance();
            for (const Owned& o : owned) {
                auto existing = db.getPatternByExternalUuid(o.receipt->pattern_uuid);
                if (!existing.has_value()) continue;

                Pattern updated = existing.value();
                updated.purchased = true;
                updated.purchased_at = o.purchased_at;
                updated.receipt_id = o.receipt_id;
                if (db.updatePattern(existing->id, updated) == ESP_OK) {
                    updated_rows++;
                }
            }
        }

        ESP_LOGI(TAG, "Purchase sync complete: %zu saved, %zu failed, %zu manifest rows updated",
            owned.size(), failed, updated_rows);
    }

    void receipt_sync_task(void* arg) {
        auto* batch = static_cast<ReceiptBatch*>(arg);
        process_receipt_batch(*batch);
        heap_caps_free(batch->items);
        delete batch;
        s_receipt_sync_running.store(false, std::memory_order_release);
        vTaskDelete(nullptr);
    }

    void handle_sync_purchases_response(Kd__V1__SyncPurchasesResponse* response) {
        if (response == nullptr) return;

        const size_t count = response->n_receipts;
        ESP_LOGI(TAG, "Received %zu purchase receipts from server", count);
        if (count == 0) return;

        // One sync at a time. A second response while one is still being
        // written can only come from a reconnect storm; the request is
        // re-sent on every connect and daily, so dropping it is safe and
        // avoids two tasks racing on the same receipt files.
        bool expected = false;
        if (!s_receipt_sync_running.compare_exchange_strong(expected, true)) {
            ESP_LOGW(TAG, "Receipt sync already in progress, dropping %zu receipts (re-synced later)",
                count);
            return;
        }

        auto* items = static_cast<ReceiptCopy*>(
            heap_caps_calloc(count, sizeof(ReceiptCopy), MALLOC_CAP_SPIRAM));
        if (items == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate receipt batch (%zu receipts)", count);
            s_receipt_sync_running.store(false, std::memory_order_release);
            return;
        }

        // Copy the bundles out: the protobuf is freed as soon as this
        // handler returns, and the work continues on its own task.
        size_t copied = 0;
        for (size_t i = 0; i < count; i++) {
            Kd__V1__PurchaseReceiptBundle* bundle = response->receipts[i];
            if (!bundle || !bundle->pattern_uuid ||
                bundle->payload.data == nullptr || bundle->payload.len == 0 ||
                bundle->signature.data == nullptr || bundle->signature.len == 0) {
                ESP_LOGW(TAG, "Skipping receipt %zu: missing data", i);
                continue;
            }
            ReceiptCopy& r = items[copied];
            if (strlen(bundle->pattern_uuid) >= sizeof(r.pattern_uuid) ||
                bundle->payload.len > sizeof(r.payload) ||
                bundle->signature.len > sizeof(r.signature)) {
                ESP_LOGW(TAG, "Skipping receipt for %s: oversized (payload=%zu, sig=%zu)",
                    bundle->pattern_uuid, bundle->payload.len, bundle->signature.len);
                continue;
            }
            strcpy(r.pattern_uuid, bundle->pattern_uuid);
            r.payload_len = static_cast<uint16_t>(bundle->payload.len);
            r.signature_len = static_cast<uint16_t>(bundle->signature.len);
            memcpy(r.payload, bundle->payload.data, bundle->payload.len);
            memcpy(r.signature, bundle->signature.data, bundle->signature.len);
            copied++;
        }

        if (copied == 0) {
            heap_caps_free(items);
            s_receipt_sync_running.store(false, std::memory_order_release);
            ESP_LOGI(TAG, "Purchase sync: nothing usable in %zu receipts", count);
            return;
        }

        auto* batch = new ReceiptBatch{ items, copied };
        // 8 KB: drm_purchase_save/verify nest RSA verification + protobuf
        // unpack + x509 parsing of the device cert.
        if (xTaskCreate(receipt_sync_task, "receipt_sync", 8192, batch, 3, nullptr) != pdPASS) {
            ESP_LOGW(TAG, "Failed to spawn receipt sync task, processing inline");
            process_receipt_batch(*batch);
            heap_caps_free(items);
            delete batch;
            s_receipt_sync_running.store(false, std::memory_order_release);
        }
    }

}  // namespace

void cloud_handle_message(Kd__V1__TranquilMessage* message) {
    if (message == nullptr) return;

    // First check for cloud-only messages that need special handling
    switch (message->message_case) {
    case KD__V1__TRANQUIL_MESSAGE__MESSAGE_LICENSE_RESPONSE:
        handle_license_response(message->license_response);
        return;

    case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PATTERN_DOWNLOAD_RESPONSE:
        handle_pattern_download_response(message->pattern_download_response);
        return;

    case KD__V1__TRANQUIL_MESSAGE__MESSAGE_JOIN_RESPONSE:
        handle_join_response(message->join_response);
        return;

    case KD__V1__TRANQUIL_MESSAGE__MESSAGE_CERT_RENEW_REQUIRED:
        handle_cert_renew_required(message->cert_renew_required);
        return;

    case KD__V1__TRANQUIL_MESSAGE__MESSAGE_CERT_RENEW_RESPONSE:
        handle_cert_renew_response(message->cert_renew_response);
        return;

    case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SYNC_PURCHASES_RESPONSE:
        handle_sync_purchases_response(message->sync_purchases_response);
        return;

    default:
        break;
    }

    // Try the unified message dispatcher for bidirectional messages
    // (PlayPattern, PlayerControl, FactoryReset, etc. from cloud)
    MessageContext ctx = MessageContext::cloud();
    ResponseMessage response;
    HandleResult result;

    esp_err_t ret = MessageDispatcher::instance().dispatch(message, ctx, &response, &result);

    // The (msg, ctx, out, out) dispatch overload only fills the out-params:
    // it neither routes the response nor honours forward_to_cloud. Do both
    // here. forward_to_cloud on the cloud path is the designed round-trip for
    // cloud-initiated pattern downloads (Worker -> us -> back to Worker with a
    // proper RequestPatternDownload -> PatternDownloadResponse); dropping it
    // used to leave every app-initiated cloud download stuck at 0%.
    if (result.forward_to_cloud &&
        message->message_case == KD__V1__TRANQUIL_MESSAGE__MESSAGE_REQUEST_PATTERN_DOWNLOAD &&
        message->request_pattern_download && message->request_pattern_download->pattern_uuid) {
        cloud_msg_send_pattern_download_request(message->request_pattern_download->pattern_uuid);
    }

    if (result.has_response) {
        if (response.valid()) {
            ResponseRouter::instance().route(response, ctx, result);
        }
        else {
            ESP_LOGE(TAG, "Cloud handler for type %d produced no payload (response too large?)",
                message->message_case);
        }
    }

    if (ret == ESP_OK && result.handled) {
        ESP_LOGD(TAG, "Cloud message %d handled by dispatcher", message->message_case);
        return;
    }

    // Unhandled message
    ESP_LOGD(TAG, "Unhandled cloud message: %d", message->message_case);
}

namespace {

// The license (and its embedded store token) is only refreshed when the
// server sees a LicenseRequest, and that used to happen on connect alone. A
// table that stays connected for weeks would sail past the 30-day validity,
// so re-ask once a day; the server answers `unchanged` until the license is
// close to expiry, at which point it mints a fresh one.
constexpr int64_t kLicenseRefreshUs = 24LL * 60 * 60 * 1000 * 1000;
esp_timer_handle_t s_license_refresh_timer = nullptr;

void license_refresh_task(void*) {
    if (cloud_sockets_is_ready()) {
        cloud_msg_send_license_request();
        cloud_msg_send_sync_purchases_request();
    }
    vTaskDelete(nullptr);
}

// esp_timer task: 4 KB stack, so hand the hashing + packing to a task.
void license_refresh_cb(void*) {
    if (xTaskCreate(license_refresh_task, "lic_refresh", 6144, nullptr, 3, nullptr) != pdPASS) {
        ESP_LOGW(TAG, "Failed to spawn license refresh task");
    }
}

void arm_license_refresh() {
    if (!s_license_refresh_timer) {
        const esp_timer_create_args_t args = {
            .callback = license_refresh_cb,
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "lic_refresh",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&args, &s_license_refresh_timer) != ESP_OK) {
            s_license_refresh_timer = nullptr;
            return;
        }
    }
    esp_timer_stop(s_license_refresh_timer);
    esp_timer_start_periodic(s_license_refresh_timer, kLicenseRefreshUs);
}

}  // namespace

void handlers_on_connected() {
    // Device-initiated handshake. Claim first (the Worker answers ClaimDevice
    // and never pushes an unsolicited JoinResponse), then the license and
    // purchase receipts. CertReport is gone: SaaS PKI owns renewal and the
    // Worker discarded the ~2 KB PEM on every connect. Both requests carry a
    // freshness digest, so a reconnect with nothing changed costs the server
    // a lookup, not a re-mint.
    cloud_msg_upload_coredump();
    cloud_msg_send_claim_if_needed();
    cloud_msg_send_license_request();
    cloud_msg_send_sync_purchases_request();
    arm_license_refresh();

    // Then the state the cloud caches for off-LAN control: player state now,
    // the list snapshots (patterns / playlists / schedule / LED / system info)
    // through the notifier task. The Worker never has to ask.
    PlayerStateBroadcaster::instance().pushToCloudNow();
    library_notifier::cloud_snapshot();
}
