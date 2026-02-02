#include "EncryptedPatternReader.h"
#include "drm/drm_crypto.h"
#include "drm/drm_license.h"
#include "drm/drm_purchase.h"

#include <esp_log.h>
#include <mbedtls/platform_util.h>

#include <stdio.h>
#include <string.h>

static const char* TAG = "encrypted_reader";

EncryptedPatternReader::EncryptedPatternReader() {
    memset(aes_key_, 0, sizeof(aes_key_));
    memset(&header_, 0, sizeof(header_));
    memset(base_iv_, 0, sizeof(base_iv_));
}

EncryptedPatternReader::~EncryptedPatternReader() {
    close();
}

void EncryptedPatternReader::getFilePath(const char* uuid, char* path, size_t path_size) {
    snprintf(path, path_size, "%s/%s.dat", PATTERNS_PATH, uuid);
}

void EncryptedPatternReader::clearKey() {
    mbedtls_platform_zeroize(aes_key_, sizeof(aes_key_));
    has_key_ = false;
}

esp_err_t EncryptedPatternReader::importAesKey() {
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_DECRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_CTR);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 256);
    psa_set_key_lifetime(&attributes, PSA_KEY_LIFETIME_VOLATILE);

    psa_status_t status = psa_import_key(&attributes, aes_key_,
        DRM_AES_KEY_BYTES, &aes_key_id_);
    psa_reset_key_attributes(&attributes);

    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to import AES key to PSA: %d", status);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t EncryptedPatternReader::startCipherAtPosition(size_t byte_position) {
    // Abort any active operation
    if (cipher_active_) {
        psa_cipher_abort(&cipher_op_);
        cipher_active_ = false;
    }

    // Calculate counter value for this position
    // CTR counter = base_IV + (position / 16)
    uint8_t iv[16];
    memcpy(iv, base_iv_, 16);

    // Add position/16 to counter (big-endian, last 8 bytes used as counter)
    uint64_t block_num = byte_position / 16;
    for (int i = 15; i >= 8 && block_num > 0; i--) {
        uint16_t sum = iv[i] + (block_num & 0xFF);
        iv[i] = sum & 0xFF;
        block_num = (block_num >> 8) + (sum >> 8);  // carry
    }

    // Setup new cipher operation
    cipher_op_ = PSA_CIPHER_OPERATION_INIT;
    psa_status_t status = psa_cipher_decrypt_setup(&cipher_op_, aes_key_id_, PSA_ALG_CTR);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_cipher_decrypt_setup failed: %d", status);
        return ESP_FAIL;
    }

    status = psa_cipher_set_iv(&cipher_op_, iv, sizeof(iv));
    if (status != PSA_SUCCESS) {
        psa_cipher_abort(&cipher_op_);
        ESP_LOGE(TAG, "psa_cipher_set_iv failed: %d", status);
        return ESP_FAIL;
    }

    cipher_active_ = true;
    decrypt_position_ = byte_position;

    // Handle partial block: skip bytes within current block
    size_t skip = byte_position % 16;
    if (skip > 0) {
        // Decrypt dummy bytes to advance keystream
        uint8_t dummy_in[16] = {0};
        uint8_t dummy_out[16];
        size_t out_len = 0;
        status = psa_cipher_update(&cipher_op_, dummy_in, skip, dummy_out, sizeof(dummy_out), &out_len);
        if (status != PSA_SUCCESS) {
            psa_cipher_abort(&cipher_op_);
            cipher_active_ = false;
            ESP_LOGE(TAG, "psa_cipher_update (skip) failed: %d", status);
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

esp_err_t EncryptedPatternReader::open(const char* uuid) {
    if (file_ != nullptr) {
        close();
    }

    // Check authorization: purchase receipt OR valid subscription
    bool authorized = false; //RESET TO FALSE

    // First, check for purchase receipt (permanent ownership)
    if (drm_purchase_is_valid(uuid)) {
        ESP_LOGI(TAG, "Pattern authorized via purchase receipt: %s", uuid);
        authorized = true;
    }
    // Fall back to subscription license
    else if (drm_license_is_valid()) {
        ESP_LOGI(TAG, "Pattern authorized via subscription: %s", uuid);
        authorized = true;
    }

    if (!authorized) {
        ESP_LOGE(TAG, "Cannot decrypt pattern: no valid authorization");
        return ESP_ERR_NOT_ALLOWED;
    }

    char file_path[256];
    getFilePath(uuid, file_path, sizeof(file_path));

    return openFile(file_path);
}

esp_err_t EncryptedPatternReader::openFile(const char* path) {
    // Authorization already checked in open() - openFile is internal only
    file_ = fopen(path, "rb");
    if (file_ == nullptr) {
        ESP_LOGE(TAG, "Failed to open encrypted pattern: %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    // Read header
    if (fread(&header_, sizeof(header_), 1, file_) != 1) {
        ESP_LOGE(TAG, "Failed to read pattern header");
        fclose(file_);
        file_ = nullptr;
        return ESP_FAIL;
    }

    // Validate header
    if (header_.magic != ENCRYPTED_PATTERN_MAGIC) {
        ESP_LOGE(TAG, "Invalid pattern magic: 0x%08lX", (unsigned long)header_.magic);
        fclose(file_);
        file_ = nullptr;
        return ESP_ERR_INVALID_ARG;
    }

    if (header_.version != ENCRYPTED_PATTERN_VERSION) {
        ESP_LOGE(TAG, "Unsupported pattern version: %u", header_.version);
        fclose(file_);
        file_ = nullptr;
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (header_.scheme != ENCRYPTION_SCHEME_RSA_OAEP_AES_CTR) {
        ESP_LOGE(TAG, "Unsupported encryption scheme: %u", header_.scheme);
        fclose(file_);
        file_ = nullptr;
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Verify binary format flag
    if (!(header_.flags & ENCRYPTED_PATTERN_FLAG_BINARY)) {
        ESP_LOGE(TAG, "Non-binary format not supported");
        fclose(file_);
        file_ = nullptr;
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Recover AES key using DS peripheral
    esp_err_t ret = drm_recover_aes_key(header_.encrypted_key, aes_key_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to recover AES key: %s", esp_err_to_name(ret));
        fclose(file_);
        file_ = nullptr;
        return ret;
    }
    has_key_ = true;

    // Initialize decryption context
    ret = initDecryption();
    if (ret != ESP_OK) {
        clearKey();
        fclose(file_);
        file_ = nullptr;
        return ret;
    }

    current_point_ = 0;
    has_peeked_ = false;

    ESP_LOGI(TAG, "Successfully opened encrypted pattern (bufferless)");
    return ESP_OK;
}

esp_err_t EncryptedPatternReader::initDecryption() {
    // Copy IV from header for rewind/seek
    memcpy(base_iv_, header_.iv, sizeof(base_iv_));

    // Import key to PSA
    esp_err_t ret = importAesKey();
    if (ret != ESP_OK) {
        return ret;
    }

    // Start cipher at position 0
    return startCipherAtPosition(0);
}

void EncryptedPatternReader::close() {
    // Abort cipher operation
    if (cipher_active_) {
        psa_cipher_abort(&cipher_op_);
        cipher_active_ = false;
    }

    // Destroy PSA key
    if (aes_key_id_ != PSA_KEY_ID_NULL) {
        psa_destroy_key(aes_key_id_);
        aes_key_id_ = PSA_KEY_ID_NULL;
    }

    // Zeroize sensitive data
    clearKey();
    mbedtls_platform_zeroize(base_iv_, sizeof(base_iv_));

    if (file_) {
        fclose(file_);
        file_ = nullptr;
    }

    memset(&header_, 0, sizeof(header_));
    decrypt_position_ = 0;
    current_point_ = 0;
    has_peeked_ = false;
}

bool EncryptedPatternReader::isOpen() const {
    return file_ != nullptr && cipher_active_ && aes_key_id_ != PSA_KEY_ID_NULL;
}

size_t EncryptedPatternReader::getTotalLines() {
    return header_.point_count;
}

size_t EncryptedPatternReader::getCurrentLine() const {
    return current_point_;
}

PatternPoint EncryptedPatternReader::readBinaryPoint() {
    if (!isOpen() || current_point_ >= header_.point_count) {
        return PatternPoint();
    }

    // Read encrypted point directly from file
    BinaryPoint bp;
    if (fread(&bp, sizeof(bp), 1, file_) != 1) {
        ESP_LOGE(TAG, "Failed to read encrypted point at index %zu", current_point_);
        return PatternPoint();
    }

    // Decrypt using PSA cipher
    uint8_t decrypted[sizeof(BinaryPoint)];
    size_t out_len = 0;
    psa_status_t status = psa_cipher_update(&cipher_op_,
        reinterpret_cast<uint8_t*>(&bp), sizeof(bp),
        decrypted, sizeof(decrypted), &out_len);

    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_cipher_update failed: %d", status);
        return PatternPoint();
    }

    decrypt_position_ += sizeof(bp);

    // Parse decrypted point
    BinaryPoint* dbp = reinterpret_cast<BinaryPoint*>(decrypted);

    // Convert uint16 rho to float 0.0-1.0
    double rho = static_cast<double>(dbp->rho) / 65535.0;

    return PatternPoint(static_cast<double>(dbp->theta), rho);
}

PatternPoint EncryptedPatternReader::readNext() {
    if (has_peeked_) {
        has_peeked_ = false;
        current_point_++;
        return peeked_point_;
    }

    PatternPoint point = readBinaryPoint();
    if (point.valid) {
        current_point_++;
    }
    return point;
}

PatternPoint EncryptedPatternReader::peekNext() {
    if (has_peeked_) {
        return peeked_point_;
    }

    if (!isOpen()) {
        return PatternPoint();
    }

    // Save state
    long saved_file_pos = ftell(file_);
    size_t saved_decrypt_pos = decrypt_position_;
    size_t saved_point = current_point_;

    // Read next point (advances cipher state)
    peeked_point_ = readBinaryPoint();

    // Restore file position
    fseek(file_, saved_file_pos, SEEK_SET);

    // Restart cipher at saved position
    startCipherAtPosition(saved_decrypt_pos);
    current_point_ = saved_point;

    if (peeked_point_.valid) {
        has_peeked_ = true;
    }

    return peeked_point_;
}

bool EncryptedPatternReader::hasMore() const {
    if (has_peeked_) {
        return peeked_point_.valid;
    }
    return isOpen() && current_point_ < header_.point_count;
}

esp_err_t EncryptedPatternReader::rewind() {
    if (!isOpen()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Seek file back to start of ciphertext (after header)
    fseek(file_, sizeof(header_), SEEK_SET);

    // Restart cipher at position 0
    esp_err_t ret = startCipherAtPosition(0);
    if (ret != ESP_OK) {
        return ret;
    }

    current_point_ = 0;
    has_peeked_ = false;

    return ESP_OK;
}
