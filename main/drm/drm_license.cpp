#include "drm_license.h"

#include <kd_common.h>
#include <esp_log.h>
#include <mbedtls/pk.h>
#include <mbedtls/sha256.h>
#include <mbedtls/x509_crt.h>
#include <mbedtls/platform_util.h>

#include <kd/v1/tranquil.pb-c.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>

#include "storage/ManifestDatabase.h"

static const char* TAG = "drm_license";

// Server's RSA-2048 public key for license signature verification
static const char* SERVER_PUBLIC_KEY_PEM = R"(
-----BEGIN PUBLIC KEY-----
MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEA36eG1Y0pMReZTtz4T+5e
pdWXYllk6zKexdJWLrKuxAo1NtM5hEdGdZ+9FpQ8iCn6hiOQw5sAZ1wt3JeIpl+Q
uqPTXc4nrtFf7LgwBN9tLIMI/ke5lyF9ZvO+xbXyV5W5SWalXtMwWghGh1ylYs5e
/AV5EfL+xhNm1Zst9HwgtiTV4btIGvqi35KeyzaRt6iiXBciYNwCsyxD5etDW3hy
+eClDxFGcB9RDxnIqYA+WthOVIIG2f4SU6yWToDdyEdfUXLaggcG85MIW+o3PZHU
qU+eL4afQ7RybUaA1thwZ01fJjYrFQqO+KtlWBBliiYWuq465jKBUgt+D+8BiNgg
HQIDAQAB
-----END PUBLIC KEY-----
)";

// License file header structure
#pragma pack(push, 1)
struct LicenseFileHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;
    uint32_t payload_len;
    uint32_t signature_len;
};
#pragma pack(pop)

// Module state
static struct {
    SemaphoreHandle_t mutex;
    bool initialized;
    bool loaded;
    drm_license_status_t status;
    drm_license_info_t info;
    uint8_t* payload;
    size_t payload_len;
    uint8_t* signature;
    size_t signature_len;
} license_state = {
    .mutex = nullptr,
    .initialized = false,
    .loaded = false,
    .status = DRM_LICENSE_NOT_FOUND,
    .info = {},
    .payload = nullptr,
    .payload_len = 0,
    .signature = nullptr,
    .signature_len = 0,
};

static esp_err_t load_license_file(void);
static esp_err_t parse_license_payload(void);
static drm_license_status_t validate_license(void);
static void free_license_data(void);

esp_err_t drm_license_init(void) {
    if (license_state.initialized) {
        return ESP_OK;
    }

    license_state.mutex = xSemaphoreCreateMutex();
    if (license_state.mutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    license_state.initialized = true;

    // Try to load license (may fail if no license file)
    esp_err_t ret = load_license_file();
    if (ret == ESP_OK) {
        license_state.status = validate_license();
    }

    ESP_LOGI(TAG, "License manager initialized, status: %d", license_state.status);
    return ESP_OK;
}

bool drm_license_is_valid(void) {
    if (!license_state.initialized) {
        return false;
    }

    xSemaphoreTake(license_state.mutex, portMAX_DELAY);
    bool valid = (license_state.status == DRM_LICENSE_VALID);
    xSemaphoreGive(license_state.mutex);

    return valid;
}

drm_license_status_t drm_license_get_status(void) {
    if (!license_state.initialized) {
        return DRM_LICENSE_NOT_FOUND;
    }

    xSemaphoreTake(license_state.mutex, portMAX_DELAY);
    drm_license_status_t status = license_state.status;
    xSemaphoreGive(license_state.mutex);

    return status;
}

uint32_t drm_license_get_max_patterns(void) {
    if (!license_state.initialized || license_state.status != DRM_LICENSE_VALID) {
        return 0;
    }

    xSemaphoreTake(license_state.mutex, portMAX_DELAY);
    uint32_t max = license_state.info.max_patterns;
    xSemaphoreGive(license_state.mutex);

    return max;
}

bool drm_license_can_download(void) {
    if (!drm_license_is_valid()) {
        return false;
    }

    uint32_t max_patterns = drm_license_get_max_patterns();
    if (max_patterns == 0) {
        return false;  // No license or no limit info
    }

    // Check current pattern count
    size_t current_count = ManifestDatabase::instance().getPatternCount();

    return current_count < max_patterns;
}

esp_err_t drm_license_get_info(drm_license_info_t* info) {
    if (!license_state.initialized || !license_state.loaded) {
        return ESP_ERR_NOT_FOUND;
    }

    if (info != nullptr) {
        xSemaphoreTake(license_state.mutex, portMAX_DELAY);
        memcpy(info, &license_state.info, sizeof(drm_license_info_t));
        xSemaphoreGive(license_state.mutex);
    }

    return ESP_OK;
}

esp_err_t drm_license_save(const uint8_t* payload, size_t payload_len,
    const uint8_t* signature, size_t signature_len) {
    if (!license_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Verify signature first
    esp_err_t ret = drm_license_verify_signature(payload, payload_len, signature, signature_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "License signature verification failed");
        return ret;
    }

    // Write to file
    FILE* f = fopen(DRM_LICENSE_PATH, "wb");
    if (f == nullptr) {
        ESP_LOGE(TAG, "Failed to open license file for writing");
        return ESP_FAIL;
    }

    LicenseFileHeader header = {
        .magic = DRM_LICENSE_MAGIC,
        .version = DRM_LICENSE_VERSION,
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
        ESP_LOGE(TAG, "Failed to write license file");
        return ESP_FAIL;
    }

    // Reload the license
    return drm_license_reload();
}

esp_err_t drm_license_verify_signature(const uint8_t* payload, size_t payload_len,
    const uint8_t* signature, size_t signature_len) {
    mbedtls_pk_context pk;
    mbedtls_pk_init(&pk);

    int ret = mbedtls_pk_parse_public_key(&pk,
        reinterpret_cast<const unsigned char*>(SERVER_PUBLIC_KEY_PEM),
        strlen(SERVER_PUBLIC_KEY_PEM) + 1);

    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to parse server public key: -0x%04X", -ret);
        mbedtls_pk_free(&pk);
        return ESP_FAIL;
    }

    // Calculate SHA-256 hash of payload
    uint8_t hash[32];
    mbedtls_sha256(payload, payload_len, hash, 0);

    // Verify signature
    ret = mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, hash, 32, signature, signature_len);
    mbedtls_pk_free(&pk);

    if (ret != 0) {
        ESP_LOGE(TAG, "Signature verification failed: -0x%04X", -ret);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t drm_license_reload(void) {
    if (!license_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(license_state.mutex, portMAX_DELAY);

    free_license_data();
    esp_err_t ret = load_license_file();
    if (ret == ESP_OK) {
        license_state.status = validate_license();
    }
    else {
        license_state.status = DRM_LICENSE_NOT_FOUND;
    }

    xSemaphoreGive(license_state.mutex);

    ESP_LOGI(TAG, "License reloaded, status: %d", license_state.status);
    return ret;
}

static esp_err_t load_license_file(void) {
    struct stat st;
    if (stat(DRM_LICENSE_PATH, &st) != 0) {
        ESP_LOGW(TAG, "License file not found: %s", DRM_LICENSE_PATH);
        license_state.loaded = false;
        return ESP_ERR_NOT_FOUND;
    }

    FILE* f = fopen(DRM_LICENSE_PATH, "rb");
    if (f == nullptr) {
        ESP_LOGE(TAG, "Failed to open license file");
        license_state.loaded = false;
        return ESP_FAIL;
    }

    // Read header
    LicenseFileHeader header;
    if (fread(&header, sizeof(header), 1, f) != 1) {
        ESP_LOGE(TAG, "Failed to read license header");
        fclose(f);
        license_state.loaded = false;
        return ESP_FAIL;
    }

    // Validate header
    if (header.magic != DRM_LICENSE_MAGIC) {
        ESP_LOGE(TAG, "Invalid license magic: 0x%08lX", (unsigned long)header.magic);
        fclose(f);
        license_state.loaded = false;
        return ESP_ERR_INVALID_ARG;
    }

    if (header.version != DRM_LICENSE_VERSION) {
        ESP_LOGE(TAG, "Unsupported license version: %u", header.version);
        fclose(f);
        license_state.loaded = false;
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Allocate and read payload
    license_state.payload = static_cast<uint8_t*>(malloc(header.payload_len));
    if (license_state.payload == nullptr) {
        fclose(f);
        license_state.loaded = false;
        return ESP_ERR_NO_MEM;
    }
    license_state.payload_len = header.payload_len;

    if (fread(license_state.payload, 1, header.payload_len, f) != header.payload_len) {
        ESP_LOGE(TAG, "Failed to read license payload");
        free_license_data();
        fclose(f);
        license_state.loaded = false;
        return ESP_FAIL;
    }

    // Allocate and read signature
    license_state.signature = static_cast<uint8_t*>(malloc(header.signature_len));
    if (license_state.signature == nullptr) {
        free_license_data();
        fclose(f);
        license_state.loaded = false;
        return ESP_ERR_NO_MEM;
    }
    license_state.signature_len = header.signature_len;

    if (fread(license_state.signature, 1, header.signature_len, f) != header.signature_len) {
        ESP_LOGE(TAG, "Failed to read license signature");
        free_license_data();
        fclose(f);
        license_state.loaded = false;
        return ESP_FAIL;
    }

    fclose(f);
    license_state.loaded = true;

    // Parse the payload
    return parse_license_payload();
}

static esp_err_t parse_license_payload(void) {
    if (!license_state.loaded || license_state.payload == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    // Deserialize protobuf
    Kd__V1__LicensePayload* pb = kd__v1__license_payload__unpack(
        nullptr, license_state.payload_len, license_state.payload);

    if (pb == nullptr) {
        ESP_LOGE(TAG, "Failed to parse license payload protobuf");
        return ESP_ERR_INVALID_ARG;
    }

    // Copy to info struct
    memset(&license_state.info, 0, sizeof(license_state.info));

    if (pb->for_device) {
        strncpy(license_state.info.for_device, pb->for_device,
            sizeof(license_state.info.for_device) - 1);
    }
    license_state.info.max_patterns = pb->max_patterns;
    license_state.info.valid_from = pb->valid_from;
    license_state.info.valid_to = pb->valid_to;
    if (pb->license_id) {
        strncpy(license_state.info.license_id, pb->license_id,
            sizeof(license_state.info.license_id) - 1);
    }
    license_state.info.issued_at = pb->issued_at;

    kd__v1__license_payload__free_unpacked(pb, nullptr);

    return ESP_OK;
}

static drm_license_status_t validate_license(void) {
    if (!license_state.loaded) {
        return DRM_LICENSE_NOT_FOUND;
    }

    // Verify signature
    esp_err_t ret = drm_license_verify_signature(
        license_state.payload, license_state.payload_len,
        license_state.signature, license_state.signature_len);

    if (ret != ESP_OK) {
        return DRM_LICENSE_SIGNATURE_INVALID;
    }

    // Check device binding
    size_t cert_len = 0;
    if (kd_common_get_device_cert(nullptr, &cert_len) == ESP_OK && cert_len > 0) {
        char* cert_pem = static_cast<char*>(malloc(cert_len + 1));
        if (cert_pem && kd_common_get_device_cert(cert_pem, &cert_len) == ESP_OK) {
            // Parse certificate to extract CN
            mbedtls_x509_crt crt;
            mbedtls_x509_crt_init(&crt);

            if (mbedtls_x509_crt_parse(&crt, reinterpret_cast<const unsigned char*>(cert_pem),
                cert_len + 1) == 0) {
                // Extract CN from subject
                char cn[128] = { 0 };
                int cn_ret = mbedtls_x509_dn_gets(cn, sizeof(cn), &crt.subject);
                if (cn_ret > 0) {
                    // Check if license for_device matches certificate CN
                    // The CN format is: CN=TRANQUIL-XXXXX.iotdevices.koiosdigital.net
                    char* cn_start = strstr(cn, "CN=");
                    if (cn_start) {
                        cn_start += 3;  // Skip "CN="
                        char* cn_end = strchr(cn_start, ',');
                        if (cn_end) *cn_end = '\0';

                        if (strcmp(cn_start, license_state.info.for_device) != 0) {
                            ESP_LOGE(TAG, "Device mismatch: cert='%s', license='%s'",
                                cn_start, license_state.info.for_device);
                            mbedtls_x509_crt_free(&crt);
                            free(cert_pem);
                            return DRM_LICENSE_DEVICE_MISMATCH;
                        }
                    }
                }
            }
            mbedtls_x509_crt_free(&crt);
        }
        free(cert_pem);
    }

    // Check time validity
    time_t now = time(nullptr);

    // Only validate time if we have a reasonable timestamp (after 2024)
    if (now > 1704067200) {  // Jan 1, 2024
        if (now < license_state.info.valid_from) {
            ESP_LOGW(TAG, "License not yet valid");
            return DRM_LICENSE_NOT_YET_VALID;
        }

        if (now > license_state.info.valid_to) {
            ESP_LOGW(TAG, "License expired");
            return DRM_LICENSE_EXPIRED;
        }
    }
    else {
        ESP_LOGW(TAG, "System time not set, skipping time validation");
    }

    return DRM_LICENSE_VALID;
}

static void free_license_data(void) {
    if (license_state.payload) {
        mbedtls_platform_zeroize(license_state.payload, license_state.payload_len);
        free(license_state.payload);
        license_state.payload = nullptr;
    }
    license_state.payload_len = 0;

    if (license_state.signature) {
        free(license_state.signature);
        license_state.signature = nullptr;
    }
    license_state.signature_len = 0;

    memset(&license_state.info, 0, sizeof(license_state.info));
    license_state.loaded = false;
}
