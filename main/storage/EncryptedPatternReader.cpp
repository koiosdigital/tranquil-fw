#include "EncryptedPatternReader.h"
#include "drm/drm_crypto.h"
#include "drm/drm_license.h"
#include "drm/drm_purchase.h"

#include <esp_log.h>
#include <mbedtls/aes.h>
#include <mbedtls/platform_util.h>

#include <stdio.h>
#include <string.h>

static const char* TAG = "encrypted_reader";

EncryptedPatternReader::EncryptedPatternReader() {
    mbedtls_aes_init(&aes_);
    memset(aes_key_, 0, sizeof(aes_key_));
    memset(&header_, 0, sizeof(header_));
    memset(nonce_counter_, 0, sizeof(nonce_counter_));
    memset(stream_block_, 0, sizeof(stream_block_));
    memset(saved_nonce_counter_, 0, sizeof(saved_nonce_counter_));
    memset(saved_stream_block_, 0, sizeof(saved_stream_block_));
}

EncryptedPatternReader::~EncryptedPatternReader() {
    close();
    mbedtls_aes_free(&aes_);
}

void EncryptedPatternReader::getFilePath(const char* uuid, char* path, size_t path_size) {
    snprintf(path, path_size, "%s/%s.dat", PATTERNS_PATH, uuid);
}

void EncryptedPatternReader::clearKey() {
    mbedtls_platform_zeroize(aes_key_, sizeof(aes_key_));
    has_key_ = false;
}

void EncryptedPatternReader::saveCtrState() {
    saved_nc_off_ = nc_off_;
    memcpy(saved_nonce_counter_, nonce_counter_, sizeof(nonce_counter_));
    memcpy(saved_stream_block_, stream_block_, sizeof(stream_block_));
}

void EncryptedPatternReader::restoreCtrState() {
    nc_off_ = saved_nc_off_;
    memcpy(nonce_counter_, saved_nonce_counter_, sizeof(nonce_counter_));
    memcpy(stream_block_, saved_stream_block_, sizeof(stream_block_));
}

esp_err_t EncryptedPatternReader::open(const char* uuid) {
    if (file_ != nullptr) {
        close();
    }

    // Check authorization: purchase receipt OR valid subscription
    bool authorized = false;

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

    ESP_LOGI(TAG, "Opening encrypted pattern, %lu points",
             (unsigned long)header_.point_count);

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
    if (aes_initialized_) {
        mbedtls_aes_free(&aes_);
        mbedtls_aes_init(&aes_);
    }

    // Set up AES key for encryption (CTR mode uses encryption for both directions)
    int ret = mbedtls_aes_setkey_enc(&aes_, aes_key_, 256);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set AES key: -0x%04X", -ret);
        return ESP_FAIL;
    }

    // Initialize counter from IV
    memcpy(nonce_counter_, header_.iv, sizeof(nonce_counter_));
    nc_off_ = 0;
    memset(stream_block_, 0, sizeof(stream_block_));

    aes_initialized_ = true;
    return ESP_OK;
}

void EncryptedPatternReader::close() {
    if (aes_initialized_) {
        mbedtls_aes_free(&aes_);
        mbedtls_aes_init(&aes_);
        aes_initialized_ = false;
    }

    clearKey();

    if (file_) {
        fclose(file_);
        file_ = nullptr;
    }

    memset(&header_, 0, sizeof(header_));
    memset(nonce_counter_, 0, sizeof(nonce_counter_));
    memset(stream_block_, 0, sizeof(stream_block_));
    memset(saved_nonce_counter_, 0, sizeof(saved_nonce_counter_));
    memset(saved_stream_block_, 0, sizeof(saved_stream_block_));
    nc_off_ = 0;
    saved_nc_off_ = 0;
    current_point_ = 0;
    has_peeked_ = false;
}

bool EncryptedPatternReader::isOpen() const {
    return file_ != nullptr && aes_initialized_;
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

    // Decrypt in place using AES-CTR (handles partial blocks via nc_off_/stream_block_)
    int ret = mbedtls_aes_crypt_ctr(&aes_,
                                     sizeof(bp),
                                     &nc_off_,
                                     nonce_counter_,
                                     stream_block_,
                                     reinterpret_cast<uint8_t*>(&bp),
                                     reinterpret_cast<uint8_t*>(&bp));
    if (ret != 0) {
        ESP_LOGE(TAG, "AES-CTR decryption failed: -0x%04X", -ret);
        return PatternPoint();
    }

    // Convert uint16 rho to float 0.0-1.0
    double rho = static_cast<double>(bp.rho) / 65535.0;

    return PatternPoint(static_cast<double>(bp.theta), rho);
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

    // Save file position and CTR state
    long prev_pos = ftell(file_);
    saveCtrState();

    // Read next point
    peeked_point_ = readBinaryPoint();

    // Restore file position and CTR state
    fseek(file_, prev_pos, SEEK_SET);
    restoreCtrState();

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

    // Re-initialize AES-CTR context with original IV
    esp_err_t ret = initDecryption();
    if (ret != ESP_OK) {
        return ret;
    }

    current_point_ = 0;
    has_peeked_ = false;

    return ESP_OK;
}
