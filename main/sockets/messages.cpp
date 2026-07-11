// Outbound messages for cloud WebSocket client
#include "messages.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_partition.h>
#include <esp_app_desc.h>
#include <esp_timer.h>
#include <psa/crypto.h>
#include <kd_common.h>
#include <ctime>
#include <kd/v1/common.pb-c.h>
#include <koios/cloudlink.h>

#include "drm/drm_license.h"
#include "sockets.h"

#include <cinttypes>

static const char* TAG = "cloud_messages";

namespace {

    bool g_needs_claim = false;
    int64_t g_last_claim_ms = 0;

    constexpr int64_t CLAIM_RETRY_MS = 5000;
    constexpr size_t CLAIM_TOKEN_MAX = 2048;

}  // namespace

bool cloud_msg_queue_raw(const uint8_t* data, size_t len) {
    if (data == nullptr || len == 0) return false;
    // cloudlink copies the data into its outbox; no need to buffer here.
    return koios_cloudlink_send(data, len);
}

bool cloud_msg_queue(const Kd__V1__TranquilMessage* message) {
    if (message == nullptr) return false;

    size_t len = kd__v1__tranquil_message__get_packed_size(message);
    auto* buf = static_cast<uint8_t*>(heap_caps_malloc(len, MALLOC_CAP_SPIRAM));
    if (buf == nullptr) {
        ESP_LOGE(TAG, "Failed to alloc %zu bytes", len);
        return false;
    }

    kd__v1__tranquil_message__pack(message, buf);
    bool ok = koios_cloudlink_send(buf, len);  // copies buf
    heap_caps_free(buf);
    if (!ok) {
        // send() fails for oversize messages and uninitialized cloudlink,
        // not just a full outbox — don't claim to know which.
        ESP_LOGW(TAG, "cloudlink send failed, dropped message (%zu bytes, max %zu)",
                 len, cloud_sockets_max_msg_size());
    }
    return ok;
}

void cloud_msg_send_device_info() {
    static Kd__V1__SystemInfo info = KD__V1__SYSTEM_INFO__INIT;
    static char model[] = "tranquil";

    const esp_app_desc_t* app_desc = esp_app_get_description();

    info.firmware_version = const_cast<char*>(app_desc->version);
    info.hardware_model = model;
    info.device_id = kd_common_get_device_name();
    info.free_heap = esp_get_free_heap_size();
    info.hostname = kd_common_get_wifi_hostname();

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_SYSTEM_INFO;
    msg.system_info = &info;
    cloud_msg_queue(&msg);

    ESP_LOGI(TAG, "Sent device info");
}

void cloud_msg_upload_coredump() {
    const esp_partition_t* part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, "coredump");
    if (part == nullptr) return;

    // Flash core dumps begin with a 32-bit total image size. Reading the
    // whole partition (64 KB) always exceeded cloudlink's message limit, so
    // the upload could never succeed and retried every boot.
    uint32_t dump_size = 0;
    if (esp_partition_read(part, 0, &dump_size, sizeof(dump_size)) != ESP_OK) return;
    if (dump_size == 0xFFFFFFFF) return;  // Erased - nothing to upload

    if (dump_size < 8 || dump_size > part->size) {
        ESP_LOGW(TAG, "Invalid coredump header (size=%" PRIu32 "), erasing partition",
                 dump_size);
        esp_partition_erase_range(part, 0, part->size);
        return;
    }

    auto* data = static_cast<uint8_t*>(heap_caps_malloc(dump_size, MALLOC_CAP_SPIRAM));
    if (data == nullptr) return;

    if (esp_partition_read(part, 0, data, dump_size) != ESP_OK) {
        heap_caps_free(data);
        return;
    }

    const esp_app_desc_t* app = esp_app_get_description();

    Kd__V1__UploadCoreDump upload = KD__V1__UPLOAD_CORE_DUMP__INIT;
    upload.core_dump.data = data;
    upload.core_dump.len = dump_size;
    upload.firmware_project = const_cast<char*>(app->project_name);
    upload.firmware_version = const_cast<char*>(app->version);

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_UPLOAD_CORE_DUMP;
    msg.upload_core_dump = &upload;

    size_t packed = kd__v1__tranquil_message__get_packed_size(&msg);
    if (packed > cloud_sockets_max_msg_size()) {
        // Undeliverable at any retry - erase so it doesn't churn every boot.
        ESP_LOGE(TAG, "Coredump message (%zu bytes) exceeds cloud limit (%zu), discarding",
                 packed, cloud_sockets_max_msg_size());
        esp_partition_erase_range(part, 0, part->size);
    } else {
        ESP_LOGI(TAG, "Uploading coredump (%" PRIu32 " bytes)", dump_size);
        if (cloud_msg_queue(&msg)) {
            esp_partition_erase_range(part, 0, part->size);
        }
        // Transient failure (offline, outbox full): keep the dump and let the
        // next boot retry.
    }

    heap_caps_free(data);
}

void cloud_msg_send_claim_if_needed() {
    g_needs_claim = true;

    int64_t now = esp_timer_get_time() / 1000;
    if (g_last_claim_ms > 0 && (now - g_last_claim_ms) < CLAIM_RETRY_MS) {
        return;
    }

    auto* token = static_cast<uint8_t*>(heap_caps_malloc(CLAIM_TOKEN_MAX, MALLOC_CAP_SPIRAM));
    if (token == nullptr) return;

    size_t token_len = CLAIM_TOKEN_MAX;
    if (kd_common_get_claim_token(reinterpret_cast<char*>(token), &token_len) != ESP_OK || token_len == 0) {
        heap_caps_free(token);
        return;
    }

    Kd__V1__ClaimDevice claim = KD__V1__CLAIM_DEVICE__INIT;
    claim.claim_token.data = token;
    claim.claim_token.len = token_len;

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_CLAIM_DEVICE;
    msg.claim_device = &claim;

    cloud_msg_queue(&msg);
    g_last_claim_ms = now;
    heap_caps_free(token);

    ESP_LOGI(TAG, "Sent claim request");
}

void cloud_msg_send_cert_report() {
    size_t cert_len = 0;
    if (kd_common_get_device_cert(nullptr, &cert_len) != ESP_OK || cert_len == 0) {
        ESP_LOGW(TAG, "No device certificate to report");
        return;
    }

    auto* cert_buf = static_cast<uint8_t*>(heap_caps_malloc(cert_len, MALLOC_CAP_SPIRAM));
    if (cert_buf == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate cert buffer");
        return;
    }

    if (kd_common_get_device_cert(reinterpret_cast<char*>(cert_buf), &cert_len) != ESP_OK) {
        heap_caps_free(cert_buf);
        return;
    }

    Kd__V1__CertReport report = KD__V1__CERT_REPORT__INIT;
    report.current_cert.data = cert_buf;
    report.current_cert.len = cert_len;

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_CERT_REPORT;
    msg.cert_report = &report;

    cloud_msg_queue(&msg);
    heap_caps_free(cert_buf);

    ESP_LOGI(TAG, "Sent cert report (%zu bytes)", cert_len);
}

void cloud_msg_send_cert_renew_request(const char* csr, size_t csr_len) {
    if (csr == nullptr || csr_len == 0) {
        ESP_LOGE(TAG, "Invalid CSR");
        return;
    }

    Kd__V1__CertRenewRequest req = KD__V1__CERT_RENEW_REQUEST__INIT;
    req.csr.data = reinterpret_cast<uint8_t*>(const_cast<char*>(csr));
    req.csr.len = csr_len;

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_CERT_RENEW_REQUEST;
    msg.cert_renew_request = &req;

    cloud_msg_queue(&msg);
    ESP_LOGI(TAG, "Sent cert renew request");
}

void cloud_msg_send_license_request() {
    Kd__V1__LicenseRequest req = KD__V1__LICENSE_REQUEST__INIT;

    // Set device ID from certificate CN
    req.device_id = kd_common_get_device_name();

    // Get current license hash if we have a license
    static uint8_t license_hash[32] = { 0 };
    drm_license_info_t info;
    if (drm_license_get_info(&info) == ESP_OK) {
        // Hash the current license info as a freshness check
        // In production, this would be the hash of the actual license file
        size_t hash_len = 0;
        psa_hash_compute(PSA_ALG_SHA_256, reinterpret_cast<const uint8_t*>(&info),
            sizeof(info), license_hash, sizeof(license_hash), &hash_len);
        req.current_license_hash.data = license_hash;
        req.current_license_hash.len = sizeof(license_hash);
    }

    req.device_timestamp = static_cast<int64_t>(time(nullptr));

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_LICENSE_REQUEST;
    msg.license_request = &req;

    cloud_msg_queue(&msg);
    ESP_LOGI(TAG, "Sent license request");
}

void cloud_msg_send_pattern_download_request(const char* pattern_uuid) {
    if (pattern_uuid == nullptr) {
        ESP_LOGE(TAG, "Invalid pattern UUID");
        return;
    }

    // Check license before requesting download
    if (!drm_license_can_download()) {
        drm_license_status_t status = drm_license_get_status();
        if (status == DRM_LICENSE_NOT_FOUND) {
            ESP_LOGE(TAG, "No valid license, cannot request pattern download");
        }
        else if (status == DRM_LICENSE_PATTERN_LIMIT_REACHED) {
            ESP_LOGE(TAG, "Pattern limit reached, cannot request download");
        }
        else {
            ESP_LOGE(TAG, "License invalid (status=%d), cannot request download", status);
        }
        return;
    }

    Kd__V1__RequestPatternDownload req = KD__V1__REQUEST_PATTERN_DOWNLOAD__INIT;
    req.pattern_uuid = const_cast<char*>(pattern_uuid);

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_REQUEST_PATTERN_DOWNLOAD;
    msg.request_pattern_download = &req;

    cloud_msg_queue(&msg);
    ESP_LOGI(TAG, "Sent pattern download request for %s", pattern_uuid);
}

void cloud_msg_send_sync_purchases_request() {
    Kd__V1__SyncPurchasesRequest req = KD__V1__SYNC_PURCHASES_REQUEST__INIT;

    Kd__V1__TranquilMessage msg = KD__V1__TRANQUIL_MESSAGE__INIT;
    msg.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_SYNC_PURCHASES_REQUEST;
    msg.sync_purchases_request = &req;

    cloud_msg_queue(&msg);
    ESP_LOGI(TAG, "Sent sync purchases request");
}
