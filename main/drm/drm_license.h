#pragma once

#include <esp_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief License file magic number "KDLC"
 */
#define DRM_LICENSE_MAGIC 0x4B444C43

/**
 * @brief License file format version
 */
#define DRM_LICENSE_VERSION 0x0001

/**
 * @brief License file path on SD card
 */
#define DRM_LICENSE_PATH "/sd/license.dat"

/**
 * @brief License validation result
 */
typedef enum {
    DRM_LICENSE_VALID = 0,
    DRM_LICENSE_NOT_FOUND,
    DRM_LICENSE_INVALID_FORMAT,
    DRM_LICENSE_SIGNATURE_INVALID,
    DRM_LICENSE_EXPIRED,
    DRM_LICENSE_NOT_YET_VALID,
    DRM_LICENSE_DEVICE_MISMATCH,
    DRM_LICENSE_PATTERN_LIMIT_REACHED,
} drm_license_status_t;

/**
 * @brief License information (decoded from file)
 */
typedef struct {
    char for_device[128];       // Device CN
    uint32_t max_patterns;      // Maximum patterns allowed
    int64_t valid_from;         // Unix timestamp UTC
    int64_t valid_to;           // Unix timestamp UTC
    char license_id[64];        // Unique license ID
    int64_t issued_at;          // Issue timestamp
} drm_license_info_t;

/**
 * @brief Initialize the license manager
 *
 * Loads and validates the license file from SD card.
 * Should be called during system startup.
 *
 * @return ESP_OK if license loaded (may be invalid), error code on I/O failure
 */
esp_err_t drm_license_init(void);

/**
 * @brief Check if a valid license is loaded
 *
 * @return true if license is valid and not expired
 */
bool drm_license_is_valid(void);

/**
 * @brief Get detailed license validation status
 *
 * @return Status code indicating why license is valid/invalid
 */
drm_license_status_t drm_license_get_status(void);

/**
 * @brief Get maximum patterns allowed by license
 *
 * @return Maximum pattern count, or 0 if no valid license
 */
uint32_t drm_license_get_max_patterns(void);

/**
 * @brief Check if device can download another pattern
 *
 * Checks current pattern count against license limit.
 *
 * @return true if download is allowed
 */
bool drm_license_can_download(void);

/**
 * @brief Get license information
 *
 * @param info Output structure for license info (may be NULL to check if loaded)
 * @return ESP_OK if license info available, ESP_ERR_NOT_FOUND if no license
 */
esp_err_t drm_license_get_info(drm_license_info_t* info);

/**
 * @brief Save a new license to SD card
 *
 * @param payload Serialized LicensePayload protobuf
 * @param payload_len Length of payload
 * @param signature Server RSA signature over payload
 * @param signature_len Length of signature
 * @return ESP_OK on success, error code on failure
 */
esp_err_t drm_license_save(const uint8_t* payload, size_t payload_len,
                           const uint8_t* signature, size_t signature_len);

/**
 * @brief Verify signature over license payload
 *
 * Uses pinned server public key to verify RSA signature.
 *
 * @param payload License payload data
 * @param payload_len Length of payload
 * @param signature RSA signature
 * @param signature_len Length of signature
 * @return ESP_OK if signature valid, ESP_ERR_INVALID_ARG if invalid
 */
esp_err_t drm_license_verify_signature(const uint8_t* payload, size_t payload_len,
                                       const uint8_t* signature, size_t signature_len);

/**
 * @brief Reload license from SD card
 *
 * Re-reads and validates the license file.
 *
 * @return ESP_OK on success
 */
esp_err_t drm_license_reload(void);

#ifdef __cplusplus
}
#endif
