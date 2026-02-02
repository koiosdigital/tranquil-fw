#pragma once

#include <esp_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Purchase receipt file magic number "KDPR"
 */
#define DRM_PURCHASE_MAGIC 0x5250444B

/**
 * @brief Purchase receipt file format version
 */
#define DRM_PURCHASE_VERSION 0x0001

/**
 * @brief Purchase receipt information (decoded from file)
 */
typedef struct {
    char pattern_uuid[40];      // Pattern this receipt authorizes
    char for_device[128];       // Device CN
    int64_t purchased_at;       // Unix timestamp UTC
    char receipt_id[64];        // Unique receipt ID for support
    char pattern_name[128];     // Human-readable name
} drm_purchase_info_t;

/**
 * @brief Check if a purchase receipt exists and is valid for this device
 *
 * Validates the receipt file at /sd/patterns/{uuid}.licdat
 *
 * @param pattern_uuid Pattern UUID to check
 * @return true if valid purchase receipt exists for this device
 */
bool drm_purchase_is_valid(const char* pattern_uuid);

/**
 * @brief Load and verify a purchase receipt
 *
 * @param pattern_uuid Pattern UUID
 * @param info Output structure for receipt info (may be NULL)
 * @return ESP_OK if valid receipt, ESP_ERR_NOT_FOUND if no receipt,
 *         ESP_ERR_INVALID_ARG if invalid receipt
 */
esp_err_t drm_purchase_verify(const char* pattern_uuid, drm_purchase_info_t* info);

/**
 * @brief Save a new purchase receipt (from server response)
 *
 * @param pattern_uuid Pattern UUID
 * @param payload Serialized PurchaseReceipt protobuf
 * @param payload_len Length of payload
 * @param signature Server RSA signature over payload
 * @param signature_len Length of signature
 * @return ESP_OK on success, error code on failure
 */
esp_err_t drm_purchase_save(const char* pattern_uuid,
                            const uint8_t* payload, size_t payload_len,
                            const uint8_t* signature, size_t signature_len);

/**
 * @brief Delete a purchase receipt
 *
 * @param pattern_uuid Pattern UUID
 * @return ESP_OK if deleted, ESP_ERR_NOT_FOUND if no receipt
 */
esp_err_t drm_purchase_delete(const char* pattern_uuid);

#ifdef __cplusplus
}
#endif
