#include "drm_crypto.h"

#include <kd_common.h>
#include <esp_log.h>
#include <esp_ds.h>
#include <esp_heap_caps.h>
#include <hal/hmac_types.h>

#include <mbedtls/sha256.h>
#include <mbedtls/platform_util.h>

#include <string.h>

static const char* TAG = "drm_crypto";

// SHA-256 hash of empty label for OAEP (SHA256("") = e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855)
static const uint8_t OAEP_LHASH_SHA256[32] = {
    0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14,
    0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f, 0xb9, 0x24,
    0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c,
    0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55
};

void drm_mgf1_sha256(const uint8_t* seed, size_t seed_len,
                     uint8_t* mask, size_t mask_len) {
    // MGF1 with SHA-256: mask = H(seed || counter_0) || H(seed || counter_1) || ...
    uint8_t hash[32];
    uint8_t counter[4] = {0, 0, 0, 0};
    size_t offset = 0;

    while (offset < mask_len) {
        mbedtls_sha256_context ctx;
        mbedtls_sha256_init(&ctx);
        mbedtls_sha256_starts(&ctx, 0);  // 0 = SHA-256 (not SHA-224)
        mbedtls_sha256_update(&ctx, seed, seed_len);
        mbedtls_sha256_update(&ctx, counter, 4);
        mbedtls_sha256_finish(&ctx, hash);
        mbedtls_sha256_free(&ctx);

        // Copy as much as needed from this hash block
        size_t to_copy = mask_len - offset;
        if (to_copy > 32) {
            to_copy = 32;
        }
        memcpy(mask + offset, hash, to_copy);
        offset += to_copy;

        // Increment counter (big-endian)
        for (int i = 3; i >= 0; i--) {
            if (++counter[i] != 0) break;
        }
    }

    mbedtls_platform_zeroize(hash, sizeof(hash));
}

esp_err_t drm_remove_oaep_sha256(const uint8_t* decrypted, size_t decrypted_len,
                                  uint8_t* output, size_t* output_len) {
    // For RSA-4096: decrypted_len = 512 bytes
    // OAEP structure: EM = 0x00 || maskedSeed (32) || maskedDB (479)
    // Total = 1 + 32 + 479 = 512 bytes

    if (decrypted_len != DRM_RSA_KEY_BYTES) {
        ESP_LOGE(TAG, "OAEP: Invalid decrypted length %zu (expected %d)",
                 decrypted_len, DRM_RSA_KEY_BYTES);
        return ESP_ERR_INVALID_ARG;
    }

    // Check leading byte
    if (decrypted[0] != 0x00) {
        ESP_LOGE(TAG, "OAEP: Invalid leading byte 0x%02x", decrypted[0]);
        return ESP_ERR_INVALID_ARG;
    }

    constexpr size_t hash_len = DRM_SHA256_BYTES;  // 32 for SHA-256
    const size_t db_len = decrypted_len - 1 - hash_len;  // 479 for RSA-4096

    const uint8_t* masked_seed = &decrypted[1];
    const uint8_t* masked_db = &decrypted[1 + hash_len];

    // Use stack buffers for OAEP working arrays (small, cold-path)
    constexpr size_t MAX_DB_LEN = DRM_RSA_KEY_BYTES - 1 - DRM_SHA256_BYTES;  // 479 for RSA-4096
    uint8_t seed_mask[hash_len] = {0};
    uint8_t seed[hash_len] = {0};
    uint8_t db_mask[MAX_DB_LEN] = {0};
    uint8_t db[MAX_DB_LEN] = {0};

    esp_err_t ret = ESP_OK;

    // Step 1: seedMask = MGF1(maskedDB, hash_len)
    drm_mgf1_sha256(masked_db, db_len, seed_mask, hash_len);

    // Step 2: seed = maskedSeed XOR seedMask
    for (size_t i = 0; i < hash_len; i++) {
        seed[i] = masked_seed[i] ^ seed_mask[i];
    }

    // Step 3: dbMask = MGF1(seed, db_len)
    drm_mgf1_sha256(seed, hash_len, db_mask, db_len);

    // Step 4: DB = maskedDB XOR dbMask
    for (size_t i = 0; i < db_len; i++) {
        db[i] = masked_db[i] ^ db_mask[i];
    }

    // Step 5: Verify lHash = SHA256("")
    // DB structure: lHash (32) || PS (0x00 bytes) || 0x01 || M
    if (memcmp(db, OAEP_LHASH_SHA256, hash_len) != 0) {
        ESP_LOGE(TAG, "OAEP: lHash mismatch");
        ret = ESP_ERR_INVALID_ARG;
        goto cleanup;
    }

    // Step 6: Find 0x01 separator after padding zeros
    {
        int msg_start = -1;
        for (size_t i = hash_len; i < db_len; i++) {
            if (db[i] == 0x01) {
                msg_start = i + 1;
                break;
            } else if (db[i] != 0x00) {
                ESP_LOGE(TAG, "OAEP: Invalid padding byte 0x%02x at position %zu", db[i], i);
                ret = ESP_ERR_INVALID_ARG;
                goto cleanup;
            }
        }

        if (msg_start < 0) {
            ESP_LOGE(TAG, "OAEP: No 0x01 separator found");
            ret = ESP_ERR_INVALID_ARG;
            goto cleanup;
        }

        size_t msg_len = db_len - msg_start;

        if (*output_len < msg_len) {
            ESP_LOGE(TAG, "OAEP: Output buffer too small (%zu < %zu)", *output_len, msg_len);
            ret = ESP_ERR_INVALID_SIZE;
            goto cleanup;
        }

        memcpy(output, &db[msg_start], msg_len);
        *output_len = msg_len;
    }

cleanup:
    mbedtls_platform_zeroize(seed_mask, hash_len);
    mbedtls_platform_zeroize(seed, hash_len);
    mbedtls_platform_zeroize(db_mask, db_len);
    mbedtls_platform_zeroize(db, db_len);

    return ret;
}

esp_err_t drm_rsa_decrypt(const uint8_t* ciphertext, uint8_t* plaintext) {
    // Get DS context from kd_common
    esp_ds_data_ctx_t* ds_ctx = kd_common_crypto_get_ctx();
    if (ds_ctx == nullptr) {
        ESP_LOGE(TAG, "Failed to get DS context");
        return ESP_FAIL;
    }

    size_t rsa_bytes = ds_ctx->rsa_length_bits / 8;
    if (rsa_bytes != DRM_RSA_KEY_BYTES) {
        ESP_LOGE(TAG, "Unexpected RSA key size: %zu bits (expected %d)",
                 ds_ctx->rsa_length_bits, DRM_RSA_KEY_BYTES * 8);
        free(ds_ctx->esp_ds_data);
        free(ds_ctx);
        return ESP_ERR_INVALID_ARG;
    }

    // Allocate DMA-capable buffers
    uint8_t* input = static_cast<uint8_t*>(heap_caps_calloc(rsa_bytes, 1, MALLOC_CAP_DMA));
    uint8_t* output = static_cast<uint8_t*>(heap_caps_calloc(rsa_bytes, 1, MALLOC_CAP_DMA));
    if (!input || !output) {
        heap_caps_free(input);
        heap_caps_free(output);
        free(ds_ctx->esp_ds_data);
        free(ds_ctx);
        return ESP_ERR_NO_MEM;
    }

    // Copy ciphertext and reverse for DS peripheral (little-endian)
    memcpy(input, ciphertext, rsa_bytes);
    kd_common_reverse_bytes(input, rsa_bytes);

    // Perform RSA decryption: m^d mod n
    // (mathematically identical to signing operation)
    esp_ds_context_t* ds_sign_ctx = nullptr;
    hmac_key_id_t hmac_key = static_cast<hmac_key_id_t>(ds_ctx->efuse_key_id);

    esp_err_t ret = esp_ds_start_sign(input, ds_ctx->esp_ds_data, hmac_key, &ds_sign_ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_ds_start_sign failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = esp_ds_finish_sign(output, ds_sign_ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_ds_finish_sign failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Reverse back to big-endian
    kd_common_reverse_bytes(output, rsa_bytes);

    // Copy to caller's buffer
    memcpy(plaintext, output, rsa_bytes);

cleanup:
    mbedtls_platform_zeroize(input, rsa_bytes);
    mbedtls_platform_zeroize(output, rsa_bytes);
    heap_caps_free(input);
    heap_caps_free(output);
    free(ds_ctx->esp_ds_data);
    free(ds_ctx);

    return ret;
}

esp_err_t drm_recover_aes_key(const uint8_t* encrypted_key, uint8_t aes_key[DRM_AES_KEY_BYTES]) {
    // Step 1: RSA decrypt the encrypted key (use stack buffer - 512 bytes, cold path)
    uint8_t decrypted[DRM_RSA_KEY_BYTES] = {0};

    esp_err_t ret = drm_rsa_decrypt(encrypted_key, decrypted);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RSA decryption failed: %s", esp_err_to_name(ret));
        mbedtls_platform_zeroize(decrypted, DRM_RSA_KEY_BYTES);
        return ret;
    }

    // Step 2: Remove OAEP padding to recover AES key
    size_t key_len = DRM_AES_KEY_BYTES;
    ret = drm_remove_oaep_sha256(decrypted, DRM_RSA_KEY_BYTES, aes_key, &key_len);

    // Zeroize decrypted data immediately
    mbedtls_platform_zeroize(decrypted, DRM_RSA_KEY_BYTES);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OAEP unpadding failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (key_len != DRM_AES_KEY_BYTES) {
        ESP_LOGE(TAG, "Unexpected key length after OAEP: %zu (expected %d)",
                 key_len, DRM_AES_KEY_BYTES);
        mbedtls_platform_zeroize(aes_key, DRM_AES_KEY_BYTES);
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "Successfully recovered AES-256 key");
    return ESP_OK;
}
