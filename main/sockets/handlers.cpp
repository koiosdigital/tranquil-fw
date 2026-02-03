// Message handlers for incoming cloud WebSocket messages
#include "handlers.h"
#include "messages.h"
#include "PatternDownloader.h"
#include "message_dispatcher.h"

#include <esp_log.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
#include <kd_common.h>

#include "drm/drm_license.h"
#include "drm/drm_purchase.h"
#include "ManifestDatabase.h"

static const char* TAG = "cloud_handlers";

namespace {

    void handle_license_response(Kd__V1__LicenseResponse* response) {
        if (response == nullptr) return;

        if (!response->success) {
            ESP_LOGE(TAG, "License request failed: %s",
                response->error ? response->error : "unknown error");
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
        }
    }

    void handle_join_response(Kd__V1__JoinResponse* response) {
        if (response == nullptr) return;

        ESP_LOGI(TAG, "Join response: claimed=%d, needs_claimed=%d",
            response->is_claimed, response->needs_claimed);

        cloud_msg_send_device_info();

        bool needs_claim = response->needs_claimed || !response->is_claimed;
        if (!needs_claim) {
            kd_common_clear_claim_token();
        }
        else {
            cloud_msg_send_claim_if_needed();
        }
    }

    void handle_factory_reset(Kd__V1__FactoryResetRequest* request) {
        const char* reason = (request && request->reason) ? request->reason : nullptr;
        ESP_LOGI(TAG, "Factory reset requested%s%s",
            reason ? ": " : "", reason ? reason : "");

        // Erase NVS and reboot
        //kd_common_factory_reset();
        esp_restart();
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
        // Server will disconnect us to reconnect with new cert
    }

    void handle_sync_purchases_response(Kd__V1__SyncPurchasesResponse* response) {
        if (response == nullptr) return;

        size_t count = response->n_receipts;
        ESP_LOGI(TAG, "Received %zu purchase receipts from server", count);

        size_t saved = 0;
        size_t failed = 0;

        for (size_t i = 0; i < count; i++) {
            Kd__V1__PurchaseReceiptBundle* bundle = response->receipts[i];
            if (!bundle || !bundle->pattern_uuid) {
                ESP_LOGW(TAG, "Skipping receipt %zu: missing data", i);
                failed++;
                continue;
            }

            if (bundle->payload.data == nullptr || bundle->payload.len == 0 ||
                bundle->signature.data == nullptr || bundle->signature.len == 0) {
                ESP_LOGW(TAG, "Skipping receipt for %s: missing payload or signature",
                    bundle->pattern_uuid);
                failed++;
                continue;
            }

            // Save the purchase receipt (this verifies signature and device binding)
            esp_err_t err = drm_purchase_save(bundle->pattern_uuid,
                bundle->payload.data, bundle->payload.len,
                bundle->signature.data, bundle->signature.len);

            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Saved receipt for pattern: %s", bundle->pattern_uuid);
                saved++;

                // Update ManifestDatabase if this pattern exists (pattern_uuid is external_uuid)
                auto existing = ManifestDatabase::instance().getPatternByExternalUuid(bundle->pattern_uuid);
                if (existing.has_value()) {
                    Pattern updated = existing.value();
                    updated.purchased = true;

                    // Get purchase info for timestamp and receipt_id
                    drm_purchase_info_t info;
                    if (drm_purchase_verify(bundle->pattern_uuid, &info) == ESP_OK) {
                        updated.purchased_at = info.purchased_at;
                        updated.receipt_id = info.receipt_id;
                    }

                    // Update by internal ID
                    ManifestDatabase::instance().updatePattern(existing->id, updated);
                    ESP_LOGD(TAG, "Updated manifest for pattern: %s", bundle->pattern_uuid);
                }
            }
            else {
                ESP_LOGW(TAG, "Failed to save receipt for %s: %s",
                    bundle->pattern_uuid, esp_err_to_name(err));
                failed++;
            }
        }

        ESP_LOGI(TAG, "Purchase sync complete: %zu saved, %zu failed", saved, failed);
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

    if (ret == ESP_OK && result.handled) {
        ESP_LOGD(TAG, "Cloud message %d handled by dispatcher", message->message_case);
        // Response routing is handled by the dispatcher
        return;
    }

    // Unhandled message
    ESP_LOGD(TAG, "Unhandled cloud message: %d", message->message_case);
}
