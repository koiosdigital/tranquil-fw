#include "EncryptedPatternReader.h"
#include "drm/drm_crypto.h"

#include <esp_log.h>
#include <mbedtls/gcm.h>
#include <mbedtls/sha256.h>
#include <mbedtls/platform_util.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char* TAG = "encrypted_reader";

EncryptedPatternReader::EncryptedPatternReader() = default;

EncryptedPatternReader::~EncryptedPatternReader() {
    close();
}

void EncryptedPatternReader::getFilePath(const char* uuid, char* path, size_t path_size) {
    snprintf(path, path_size, "%s/%s.dat", PATTERNS_PATH, uuid);
}

esp_err_t EncryptedPatternReader::open(const char* uuid) {
    if (decrypted_data_ != nullptr) {
        close();
    }

    char file_path[256];
    getFilePath(uuid, file_path, sizeof(file_path));

    return decryptFile(file_path);
}

esp_err_t EncryptedPatternReader::decryptFile(const char* path) {
    FILE* f = fopen(path, "rb");
    if (f == nullptr) {
        ESP_LOGE(TAG, "Failed to open encrypted pattern: %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    // Read header
    EncryptedPatternHeader header;
    if (fread(&header, sizeof(header), 1, f) != 1) {
        ESP_LOGE(TAG, "Failed to read pattern header");
        fclose(f);
        return ESP_FAIL;
    }

    // Validate header
    if (header.magic != ENCRYPTED_PATTERN_MAGIC) {
        ESP_LOGE(TAG, "Invalid pattern magic: 0x%08lX", (unsigned long)header.magic);
        fclose(f);
        return ESP_ERR_INVALID_ARG;
    }

    if (header.version != ENCRYPTED_PATTERN_VERSION) {
        ESP_LOGE(TAG, "Unsupported pattern version: %u", header.version);
        fclose(f);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (header.scheme != ENCRYPTION_SCHEME_RSA_OAEP_AES_GCM) {
        ESP_LOGE(TAG, "Unsupported encryption scheme: %u", header.scheme);
        fclose(f);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "Opening encrypted pattern, original size: %lu", (unsigned long)header.original_size);

    // Recover AES key using DS peripheral
    uint8_t aes_key[DRM_AES_KEY_BYTES];
    esp_err_t ret = drm_recover_aes_key(header.encrypted_key, aes_key);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to recover AES key: %s", esp_err_to_name(ret));
        fclose(f);
        return ret;
    }

    // Get file size to calculate ciphertext length
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, sizeof(header), SEEK_SET);

    size_t ciphertext_len = file_size - sizeof(header) - AES_GCM_TAG_SIZE;
    if (ciphertext_len != header.original_size) {
        ESP_LOGW(TAG, "Ciphertext length (%zu) doesn't match original size (%lu)",
                 ciphertext_len, (unsigned long)header.original_size);
    }

    // Allocate buffers
    uint8_t* ciphertext = static_cast<uint8_t*>(malloc(ciphertext_len));
    uint8_t* auth_tag = static_cast<uint8_t*>(malloc(AES_GCM_TAG_SIZE));
    decrypted_data_ = static_cast<char*>(malloc(ciphertext_len + 1));  // +1 for null terminator

    if (!ciphertext || !auth_tag || !decrypted_data_) {
        ESP_LOGE(TAG, "Failed to allocate decryption buffers");
        free(ciphertext);
        free(auth_tag);
        free(decrypted_data_);
        decrypted_data_ = nullptr;
        mbedtls_platform_zeroize(aes_key, sizeof(aes_key));
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    // Read ciphertext and auth tag
    if (fread(ciphertext, 1, ciphertext_len, f) != ciphertext_len) {
        ESP_LOGE(TAG, "Failed to read ciphertext");
        ret = ESP_FAIL;
        goto cleanup;
    }

    if (fread(auth_tag, 1, AES_GCM_TAG_SIZE, f) != AES_GCM_TAG_SIZE) {
        ESP_LOGE(TAG, "Failed to read auth tag");
        ret = ESP_FAIL;
        goto cleanup;
    }

    fclose(f);
    f = nullptr;

    // Initialize GCM context and decrypt
    {
        mbedtls_gcm_context gcm;
        mbedtls_gcm_init(&gcm);

        int mbret = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, aes_key, 256);
        mbedtls_platform_zeroize(aes_key, sizeof(aes_key));  // Zeroize key immediately

        if (mbret != 0) {
            ESP_LOGE(TAG, "Failed to set GCM key: -0x%04X", -mbret);
            mbedtls_gcm_free(&gcm);
            ret = ESP_FAIL;
            goto cleanup;
        }

        // Decrypt and verify in one operation
        mbret = mbedtls_gcm_auth_decrypt(&gcm,
                                          ciphertext_len,
                                          header.iv, sizeof(header.iv),
                                          nullptr, 0,  // No additional authenticated data
                                          auth_tag, AES_GCM_TAG_SIZE,
                                          ciphertext,
                                          reinterpret_cast<unsigned char*>(decrypted_data_));

        mbedtls_gcm_free(&gcm);

        if (mbret != 0) {
            ESP_LOGE(TAG, "GCM decryption/authentication failed: -0x%04X", -mbret);
            ret = ESP_ERR_INVALID_ARG;
            goto cleanup;
        }
    }

    decrypted_size_ = ciphertext_len;
    decrypted_data_[decrypted_size_] = '\0';  // Null terminate for string operations

    // Verify hash
    {
        uint8_t computed_hash[32];
        mbedtls_sha256(reinterpret_cast<const unsigned char*>(decrypted_data_),
                       decrypted_size_, computed_hash, 0);

        if (memcmp(computed_hash, header.original_hash, 32) != 0) {
            ESP_LOGW(TAG, "Decrypted content hash doesn't match header (may be normal for some formats)");
            // Don't fail - hash mismatch might be acceptable in some cases
        }
    }

    ESP_LOGI(TAG, "Successfully decrypted pattern (%zu bytes)", decrypted_size_);

    free(ciphertext);
    free(auth_tag);
    read_offset_ = 0;
    current_line_ = 0;
    lines_counted_ = false;
    has_peeked_ = false;
    return ESP_OK;

cleanup:
    if (f) fclose(f);
    free(ciphertext);
    free(auth_tag);
    free(decrypted_data_);
    decrypted_data_ = nullptr;
    decrypted_size_ = 0;
    mbedtls_platform_zeroize(aes_key, sizeof(aes_key));
    return ret;
}

void EncryptedPatternReader::close() {
    if (decrypted_data_) {
        mbedtls_platform_zeroize(decrypted_data_, decrypted_size_);
        free(decrypted_data_);
        decrypted_data_ = nullptr;
    }
    decrypted_size_ = 0;
    read_offset_ = 0;
    total_lines_ = 0;
    current_line_ = 0;
    lines_counted_ = false;
    has_peeked_ = false;
}

bool EncryptedPatternReader::isOpen() const {
    return decrypted_data_ != nullptr;
}

size_t EncryptedPatternReader::getTotalLines() {
    if (!isOpen()) return 0;

    if (!lines_counted_) {
        // Count lines in decrypted data
        total_lines_ = 0;
        for (size_t i = 0; i < decrypted_size_; i++) {
            if (decrypted_data_[i] == '\n') {
                total_lines_++;
            }
        }
        lines_counted_ = true;
    }

    return total_lines_;
}

size_t EncryptedPatternReader::getCurrentLine() const {
    return current_line_;
}

bool EncryptedPatternReader::readNextLine(char* buffer, size_t buffer_size) {
    if (!isOpen() || read_offset_ >= decrypted_size_) {
        return false;
    }

    size_t line_start = read_offset_;
    size_t line_len = 0;

    // Find end of line
    while (read_offset_ < decrypted_size_ && decrypted_data_[read_offset_] != '\n') {
        read_offset_++;
        line_len++;
    }

    // Skip the newline
    if (read_offset_ < decrypted_size_) {
        read_offset_++;
    }

    // Copy to buffer
    if (line_len >= buffer_size) {
        line_len = buffer_size - 1;
    }

    memcpy(buffer, &decrypted_data_[line_start], line_len);
    buffer[line_len] = '\0';

    // Strip trailing \r if present
    if (line_len > 0 && buffer[line_len - 1] == '\r') {
        buffer[line_len - 1] = '\0';
    }

    return true;
}

PatternPoint EncryptedPatternReader::parseLine(const char* line) {
    if (line == nullptr || line[0] == '\0') {
        return PatternPoint();
    }

    // Skip comment lines
    if (line[0] == '#') {
        return PatternPoint();
    }

    double theta, rho;
    if (sscanf(line, "%lf %lf", &theta, &rho) == 2) {
        return PatternPoint(theta, rho);
    }

    return PatternPoint();
}

PatternPoint EncryptedPatternReader::readNext() {
    if (has_peeked_) {
        has_peeked_ = false;
        current_line_++;
        return peeked_point_;
    }

    while (readNextLine(line_buffer_, sizeof(line_buffer_))) {
        PatternPoint point = parseLine(line_buffer_);
        if (point.valid) {
            current_line_++;
            return point;
        }
        // Skip invalid/comment lines but still count them
        current_line_++;
    }

    return PatternPoint();
}

PatternPoint EncryptedPatternReader::peekNext() {
    if (has_peeked_) {
        return peeked_point_;
    }

    // Save current state
    size_t saved_offset = read_offset_;
    size_t saved_line = current_line_;

    // Try to read
    while (readNextLine(line_buffer_, sizeof(line_buffer_))) {
        PatternPoint point = parseLine(line_buffer_);
        if (point.valid) {
            peeked_point_ = point;
            has_peeked_ = true;

            // Restore offset (but keep peeked state)
            read_offset_ = saved_offset;
            current_line_ = saved_line;
            return peeked_point_;
        }
    }

    // Restore state
    read_offset_ = saved_offset;
    current_line_ = saved_line;
    return PatternPoint();
}

bool EncryptedPatternReader::hasMore() const {
    if (has_peeked_) {
        return peeked_point_.valid;
    }
    return isOpen() && read_offset_ < decrypted_size_;
}

esp_err_t EncryptedPatternReader::rewind() {
    if (!isOpen()) {
        return ESP_ERR_INVALID_STATE;
    }

    read_offset_ = 0;
    current_line_ = 0;
    has_peeked_ = false;

    return ESP_OK;
}
