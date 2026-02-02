#include "drm_purchase.h"
#include "drm_license.h"

#include <kd_common.h>
#include <esp_log.h>
#include <mbedtls/x509_crt.h>
#include <mbedtls/platform_util.h>

#include <kd/v1/tranquil.pb-c.h>

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

static const char* TAG = "drm_purchase";

// Purchase receipt file header structure (same format as license)
#pragma pack(push, 1)
struct PurchaseFileHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;
    uint32_t payload_len;
    uint32_t signature_len;
};
#pragma pack(pop)

// Get the receipt file path for a pattern UUID
static void get_receipt_path(const char* pattern_uuid, char* path, size_t path_len) {
    snprintf(path, path_len, "/sd/patterns/%s.licdat", pattern_uuid);
}

// Verify that receipt is for this device
static bool verify_device_binding(const char* for_device) {
    size_t cert_len = 0;
    if (kd_common_get_device_cert(nullptr, &cert_len) != ESP_OK || cert_len == 0) {
        ESP_LOGW(TAG, "No device certificate available");
        return false;
    }

    constexpr size_t MAX_CERT_SIZE = 2048;
    if (cert_len >= MAX_CERT_SIZE) {
        ESP_LOGE(TAG, "Certificate too large: %zu", cert_len);
        return false;
    }

    char cert_pem[MAX_CERT_SIZE];
    if (kd_common_get_device_cert(cert_pem, &cert_len) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device certificate");
        return false;
    }

    // Parse certificate to extract CN
    mbedtls_x509_crt crt;
    mbedtls_x509_crt_init(&crt);

    bool match = false;
    if (mbedtls_x509_crt_parse(&crt, reinterpret_cast<const unsigned char*>(cert_pem),
        cert_len + 1) == 0) {
        // Extract CN from subject
        char cn[128] = { 0 };
        int cn_ret = mbedtls_x509_dn_gets(cn, sizeof(cn), &crt.subject);
        if (cn_ret > 0) {
            // The CN format is: CN=TRANQUIL-XXXXX.iotdevices.koiosdigital.net
            char* cn_start = strstr(cn, "CN=");
            if (cn_start) {
                cn_start += 3;  // Skip "CN="
                char* cn_end = strchr(cn_start, ',');
                if (cn_end) *cn_end = '\0';

                match = (strcmp(cn_start, for_device) == 0);
                if (!match) {
                    ESP_LOGW(TAG, "Device mismatch: cert='%s', receipt='%s'",
                        cn_start, for_device);
                }
            }
        }
    }

    mbedtls_x509_crt_free(&crt);
    return match;
}

bool drm_purchase_is_valid(const char* pattern_uuid) {
    return drm_purchase_verify(pattern_uuid, nullptr) == ESP_OK;
}

esp_err_t drm_purchase_verify(const char* pattern_uuid, drm_purchase_info_t* info) {
    if (!pattern_uuid) {
        return ESP_ERR_INVALID_ARG;
    }

    char path[128];
    get_receipt_path(pattern_uuid, path, sizeof(path));

    struct stat st;
    if (stat(path, &st) != 0) {
        // Receipt doesn't exist - not an error, just not purchased
        return ESP_ERR_NOT_FOUND;
    }

    FILE* f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open receipt file: %s", path);
        return ESP_FAIL;
    }

    esp_err_t result = ESP_FAIL;

    // Read header
    PurchaseFileHeader header;
    if (fread(&header, sizeof(header), 1, f) != 1) {
        ESP_LOGE(TAG, "Failed to read receipt header");
        goto cleanup;
    }

    // Validate header
    if (header.magic != DRM_PURCHASE_MAGIC) {
        ESP_LOGE(TAG, "Invalid receipt magic: 0x%08lX", (unsigned long)header.magic);
        result = ESP_ERR_INVALID_ARG;
        goto cleanup;
    }

    if (header.version != DRM_PURCHASE_VERSION) {
        ESP_LOGE(TAG, "Unsupported receipt version: 0x%04X", header.version);
        result = ESP_ERR_INVALID_ARG;
        goto cleanup;
    }

    // Sanity check lengths
    if (header.payload_len > 4096 || header.signature_len > 512) {
        ESP_LOGE(TAG, "Invalid receipt lengths: payload=%lu, sig=%lu",
            (unsigned long)header.payload_len, (unsigned long)header.signature_len);
        result = ESP_ERR_INVALID_ARG;
        goto cleanup;
    }

    {
        // Use stack allocation for small files (hot path during playback)
        uint8_t payload[512];
        uint8_t signature[256];

        if (header.payload_len > sizeof(payload) || header.signature_len > sizeof(signature)) {
            ESP_LOGE(TAG, "Receipt too large for stack buffer");
            result = ESP_ERR_NO_MEM;
            goto cleanup;
        }

        if (fread(payload, 1, header.payload_len, f) != header.payload_len) {
            ESP_LOGE(TAG, "Failed to read receipt payload");
            goto cleanup;
        }

        if (fread(signature, 1, header.signature_len, f) != header.signature_len) {
            ESP_LOGE(TAG, "Failed to read receipt signature");
            goto cleanup;
        }

        // Verify signature using license module's function
        esp_err_t sig_ret = drm_license_verify_signature(payload, header.payload_len,
            signature, header.signature_len);
        if (sig_ret != ESP_OK) {
            ESP_LOGE(TAG, "Receipt signature invalid for: %s", pattern_uuid);
            result = ESP_ERR_INVALID_ARG;
            goto cleanup;
        }

        // Parse protobuf
        Kd__V1__PurchaseReceipt* pb = kd__v1__purchase_receipt__unpack(
            nullptr, header.payload_len, payload);

        if (!pb) {
            ESP_LOGE(TAG, "Failed to parse receipt protobuf");
            result = ESP_ERR_INVALID_ARG;
            goto cleanup;
        }

        // Verify pattern UUID matches
        if (!pb->pattern_uuid || strcmp(pb->pattern_uuid, pattern_uuid) != 0) {
            ESP_LOGE(TAG, "Receipt UUID mismatch: expected='%s', got='%s'",
                pattern_uuid, pb->pattern_uuid ? pb->pattern_uuid : "(null)");
            kd__v1__purchase_receipt__free_unpacked(pb, nullptr);
            result = ESP_ERR_INVALID_ARG;
            goto cleanup;
        }

        // Verify device binding
        if (!pb->for_device || !verify_device_binding(pb->for_device)) {
            ESP_LOGE(TAG, "Receipt not for this device");
            kd__v1__purchase_receipt__free_unpacked(pb, nullptr);
            result = ESP_ERR_INVALID_ARG;
            goto cleanup;
        }

        // Copy to output struct if provided
        if (info) {
            memset(info, 0, sizeof(*info));
            if (pb->pattern_uuid) {
                strncpy(info->pattern_uuid, pb->pattern_uuid, sizeof(info->pattern_uuid) - 1);
            }
            if (pb->for_device) {
                strncpy(info->for_device, pb->for_device, sizeof(info->for_device) - 1);
            }
            info->purchased_at = pb->purchased_at;
            if (pb->receipt_id) {
                strncpy(info->receipt_id, pb->receipt_id, sizeof(info->receipt_id) - 1);
            }
            if (pb->pattern_name) {
                strncpy(info->pattern_name, pb->pattern_name, sizeof(info->pattern_name) - 1);
            }
        }

        kd__v1__purchase_receipt__free_unpacked(pb, nullptr);
        result = ESP_OK;
    }

cleanup:
    fclose(f);
    return result;
}

esp_err_t drm_purchase_save(const char* pattern_uuid,
    const uint8_t* payload, size_t payload_len,
    const uint8_t* signature, size_t signature_len) {
    if (!pattern_uuid || !payload || !signature) {
        return ESP_ERR_INVALID_ARG;
    }

    // Verify signature before saving
    esp_err_t ret = drm_license_verify_signature(payload, payload_len, signature, signature_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot save receipt: invalid signature");
        return ret;
    }

    char path[128];
    get_receipt_path(pattern_uuid, path, sizeof(path));

    FILE* f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to create receipt file: %s", path);
        return ESP_FAIL;
    }

    // Write header
    PurchaseFileHeader header = {
        .magic = DRM_PURCHASE_MAGIC,
        .version = DRM_PURCHASE_VERSION,
        .flags = 0,
        .payload_len = static_cast<uint32_t>(payload_len),
        .signature_len = static_cast<uint32_t>(signature_len),
    };

    bool success = true;
    if (fwrite(&header, sizeof(header), 1, f) != 1) {
        success = false;
    }
    if (success && fwrite(payload, 1, payload_len, f) != payload_len) {
        success = false;
    }
    if (success && fwrite(signature, 1, signature_len, f) != signature_len) {
        success = false;
    }

    fclose(f);

    if (!success) {
        ESP_LOGE(TAG, "Failed to write receipt file");
        remove(path);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Saved purchase receipt for: %s", pattern_uuid);
    return ESP_OK;
}

esp_err_t drm_purchase_delete(const char* pattern_uuid) {
    if (!pattern_uuid) {
        return ESP_ERR_INVALID_ARG;
    }

    char path[128];
    get_receipt_path(pattern_uuid, path, sizeof(path));

    if (remove(path) != 0) {
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Deleted purchase receipt for: %s", pattern_uuid);
    return ESP_OK;
}
