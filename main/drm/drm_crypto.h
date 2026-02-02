#pragma once

#include <esp_err.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @brief RSA-4096 key size in bytes
     */
#define DRM_RSA_KEY_BYTES 512

     /**
      * @brief AES-256 key size in bytes
      */
#define DRM_AES_KEY_BYTES 32

      /**
       * @brief AES-GCM IV size in bytes
       */
#define DRM_AES_IV_BYTES 12

       /**
        * @brief AES-GCM tag size in bytes
        */
#define DRM_AES_TAG_BYTES 16

        /**
         * @brief SHA-256 hash size in bytes
         */
#define DRM_SHA256_BYTES 32

         /**
          * @brief Recover AES-256 key from RSA-OAEP encrypted ciphertext
          *
          * Uses the DS peripheral to perform RSA decryption (m^d mod n),
          * then removes OAEP-SHA256 padding to extract the AES key.
          *
          * @param encrypted_key RSA-OAEP encrypted AES key (512 bytes for RSA-4096)
          * @param aes_key Output buffer for recovered AES key (32 bytes)
          * @return ESP_OK on success, error code on failure
          */
    esp_err_t drm_recover_aes_key(const uint8_t* encrypted_key, uint8_t aes_key[DRM_AES_KEY_BYTES]);

    /**
     * @brief Perform raw RSA decryption using DS peripheral
     *
     * Computes m^d mod n using the device's RSA private key via DS peripheral.
     * The private key never leaves the DS peripheral.
     *
     * @param ciphertext Input ciphertext (512 bytes for RSA-4096)
     * @param plaintext Output buffer for decrypted data (512 bytes)
     * @return ESP_OK on success, error code on failure
     */
    esp_err_t drm_rsa_decrypt(const uint8_t* ciphertext, uint8_t* plaintext);

    /**
     * @brief Remove OAEP-SHA256 padding from RSA-decrypted data
     *
     * Decodes OAEP padding according to RFC 8017 (PKCS#1 v2.2).
     * Uses SHA-256 for both the hash function and MGF1.
     *
     * @param decrypted RSA-decrypted data (512 bytes for RSA-4096)
     * @param decrypted_len Length of decrypted data
     * @param output Output buffer for unpadded message
     * @param output_len Input: output buffer size. Output: actual message length
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG on padding error
     */
    esp_err_t drm_remove_oaep_sha256(const uint8_t* decrypted, size_t decrypted_len,
        uint8_t* output, size_t* output_len);

    /**
     * @brief MGF1 mask generation function using SHA-256
     *
     * Generates a mask of arbitrary length from a seed using
     * iterative SHA-256 hashing (PKCS#1 MGF1).
     *
     * @param seed Input seed data
     * @param seed_len Length of seed
     * @param mask Output buffer for generated mask
     * @param mask_len Desired mask length
     */
    void drm_mgf1_sha256(const uint8_t* seed, size_t seed_len,
        uint8_t* mask, size_t mask_len);

#ifdef __cplusplus
}
#endif
