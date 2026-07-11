#include "EncryptedPatternReader.h"
#include "drm/drm_crypto.h"
#include "drm/drm_license.h"
#include "drm/drm_purchase.h"

#include <esp_log.h>
#include <mbedtls/platform_util.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cmath>
#include <stdio.h>
#include <string.h>

static const char* TAG = "encrypted_reader";

// -----------------------------------------------------------------------------
// Single-entry AES key cache
//
// drm_recover_aes_key() costs ~1.7s of RSA-4096 on the DS peripheral, and the
// player re-opens the pattern on every playlist advance/loop. Cache the last
// successfully recovered key by pattern UUID so repeat opens skip the RSA step.
// The cache only bypasses key recovery — the authorization check in open()
// still runs on every open. Lifetime: zeroized by invalidateKeyCache() (called
// on license reload), otherwise lives until overwritten by another pattern.
// -----------------------------------------------------------------------------
namespace {

struct AesKeyCache {
    char uuid[64];
    uint8_t key[DRM_AES_KEY_BYTES];
    bool valid;
};

AesKeyCache s_key_cache = {};

SemaphoreHandle_t keyCacheMutex() {
    // Thread-safe magic static; may be nullptr if allocation fails (cache
    // is then simply bypassed)
    static SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
    return mutex;
}

bool keyCacheLookup(const char* uuid, uint8_t key[DRM_AES_KEY_BYTES]) {
    SemaphoreHandle_t mutex = keyCacheMutex();
    if (uuid == nullptr || mutex == nullptr) {
        return false;
    }

    bool hit = false;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (s_key_cache.valid &&
        strncmp(s_key_cache.uuid, uuid, sizeof(s_key_cache.uuid)) == 0) {
        memcpy(key, s_key_cache.key, DRM_AES_KEY_BYTES);
        hit = true;
    }
    xSemaphoreGive(mutex);
    return hit;
}

void keyCacheStore(const char* uuid, const uint8_t key[DRM_AES_KEY_BYTES]) {
    SemaphoreHandle_t mutex = keyCacheMutex();
    if (uuid == nullptr || mutex == nullptr ||
        strlen(uuid) >= sizeof(s_key_cache.uuid)) {
        return;
    }

    xSemaphoreTake(mutex, portMAX_DELAY);
    strncpy(s_key_cache.uuid, uuid, sizeof(s_key_cache.uuid) - 1);
    s_key_cache.uuid[sizeof(s_key_cache.uuid) - 1] = '\0';
    memcpy(s_key_cache.key, key, DRM_AES_KEY_BYTES);
    s_key_cache.valid = true;
    xSemaphoreGive(mutex);
}

}  // namespace

void EncryptedPatternReader::invalidateKeyCache() {
    SemaphoreHandle_t mutex = keyCacheMutex();
    if (mutex == nullptr) {
        return;
    }
    xSemaphoreTake(mutex, portMAX_DELAY);
    mbedtls_platform_zeroize(&s_key_cache, sizeof(s_key_cache));
    xSemaphoreGive(mutex);
}

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

    // Add position/16 to the big-endian 128-bit counter. The carry must
    // propagate through all 16 bytes (not just the low 8): a random base IV
    // near 0xFF..FF in byte 8 would otherwise drop the carry and desync the
    // keystream on seek.
    uint64_t block_num = byte_position / 16;
    unsigned int carry = 0;
    for (int i = 15; i >= 0; i--) {
        unsigned int sum = iv[i] + static_cast<unsigned int>(block_num & 0xFF) + carry;
        iv[i] = sum & 0xFF;
        carry = sum >> 8;
        block_num >>= 8;
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

    return openFile(file_path, uuid);
}

esp_err_t EncryptedPatternReader::openFile(const char* path, const char* uuid) {
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

    // Validate point_count against actual file size so a truncated file
    // can't wedge playback (hasMore() true forever past EOF). Done before
    // key recovery so a corrupt file fails fast, not after ~1.7s of RSA.
    long file_size = -1;
    if (fseek(file_, 0, SEEK_END) == 0) {
        file_size = ftell(file_);
    }
    if (file_size < 0 || fseek(file_, sizeof(header_), SEEK_SET) != 0) {
        ESP_LOGE(TAG, "Failed to determine pattern file size");
        fclose(file_);
        file_ = nullptr;
        return ESP_FAIL;
    }
    size_t available_points =
        (static_cast<size_t>(file_size) - sizeof(header_)) / sizeof(BinaryPoint);
    if (header_.point_count > available_points) {
        ESP_LOGW(TAG, "Pattern truncated: header claims %lu points, file holds %zu",
                 (unsigned long)header_.point_count, available_points);
        if (available_points == 0) {
            fclose(file_);
            file_ = nullptr;
            return ESP_ERR_INVALID_SIZE;
        }
        header_.point_count = available_points;
    }

    // Recover AES key, skipping the ~1.7s RSA-4096 operation when the
    // single-entry cache already holds this pattern's key
    esp_err_t ret;
    if (keyCacheLookup(uuid, aes_key_)) {
        ESP_LOGD(TAG, "AES key cache hit for %s", uuid);
        has_key_ = true;
    }
    else {
        ret = drm_recover_aes_key(header_.encrypted_key, aes_key_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to recover AES key: %s", esp_err_to_name(ret));
            fclose(file_);
            file_ = nullptr;
            return ret;
        }
        has_key_ = true;
        keyCacheStore(uuid, aes_key_);
    }

    // Initialize decryption context (zeroizes aes_key_ after PSA import)
    ret = initDecryption();
    if (ret != ESP_OK) {
        clearKey();
        fclose(file_);
        file_ = nullptr;
        return ret;
    }

    current_point_ = 0;
    has_peeked_ = false;
    read_failed_ = false;

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

    // The key now lives in the PSA keystore (aes_key_id_); rewind/seek go
    // through the key id, so drop the raw copy immediately
    clearKey();

    // Start cipher at position 0
    ret = startCipherAtPosition(0);
    if (ret != ESP_OK) {
        // Don't leak the volatile PSA key slot on the error path
        psa_destroy_key(aes_key_id_);
        aes_key_id_ = PSA_KEY_ID_NULL;
        return ret;
    }

    return ESP_OK;
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
    read_failed_ = false;
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
    if (!isOpen() || read_failed_ || current_point_ >= header_.point_count) {
        return PatternPoint();
    }

    // Read encrypted point directly from file. A short read is terminal:
    // the file offset and CTR keystream would be desynced from here on.
    BinaryPoint bp;
    if (fread(&bp, sizeof(bp), 1, file_) != 1) {
        ESP_LOGE(TAG, "Failed to read encrypted point at index %zu", current_point_);
        read_failed_ = true;
        return PatternPoint();
    }

    // Decrypt using PSA cipher
    uint8_t decrypted[sizeof(BinaryPoint)];
    size_t out_len = 0;
    psa_status_t status = psa_cipher_update(&cipher_op_,
        reinterpret_cast<uint8_t*>(&bp), sizeof(bp),
        decrypted, sizeof(decrypted), &out_len);

    if (status != PSA_SUCCESS || out_len != sizeof(decrypted)) {
        ESP_LOGE(TAG, "psa_cipher_update failed: %d (out_len %zu)", status, out_len);
        // Terminal: abort the cipher so nothing keeps decrypting a
        // desynced keystream
        read_failed_ = true;
        psa_cipher_abort(&cipher_op_);
        cipher_active_ = false;
        return PatternPoint();
    }

    decrypt_position_ += sizeof(bp);

    // Parse decrypted point
    BinaryPoint* dbp = reinterpret_cast<BinaryPoint*>(decrypted);

    // Reject garbage motion targets: theta must be a finite, sane angle
    // (|theta| < 10000 rad is generous for multi-rotation patterns).
    // Terminal: CTR garbage here means the rest of the stream is garbage
    // too (wrong key or corrupt ciphertext).
    if (!std::isfinite(dbp->theta) || std::fabs(dbp->theta) >= 10000.0f) {
        ESP_LOGE(TAG, "Invalid theta at point %zu (corrupt stream)", current_point_);
        read_failed_ = true;
        return PatternPoint();
    }

    // Convert uint16 rho to float 0.0-1.0 (in [0,1] by construction)
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

    // Read the point and leave the file/cipher advanced past it:
    // readNext()'s cached branch returns peeked_point_ WITHOUT touching
    // the stream, so restoring state here would desync the ciphertext
    // position from current_point_ — every peek+read cycle decrypts the
    // same bytes and playback replays one point until the index runs out.
    // (This also avoids a full CTR cipher restart per peeked point.)
    peeked_point_ = readBinaryPoint();

    if (peeked_point_.valid) {
        has_peeked_ = true;
    }

    return peeked_point_;
}

bool EncryptedPatternReader::hasMore() const {
    if (read_failed_) {
        return false;
    }
    if (has_peeked_) {
        return peeked_point_.valid;
    }
    return isOpen() && current_point_ < header_.point_count;
}

esp_err_t EncryptedPatternReader::rewind() {
    if (!isOpen()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Seek file back to start of ciphertext (after header). If the seek
    // fails, do NOT restart the cipher against a mispositioned file — that
    // would silently desync keystream and ciphertext.
    if (fseek(file_, sizeof(header_), SEEK_SET) != 0) {
        ESP_LOGE(TAG, "Rewind seek failed");
        read_failed_ = true;
        return ESP_FAIL;
    }

    // Restart cipher at position 0
    esp_err_t ret = startCipherAtPosition(0);
    if (ret != ESP_OK) {
        read_failed_ = true;
        return ret;
    }

    current_point_ = 0;
    has_peeked_ = false;
    read_failed_ = false;

    return ESP_OK;
}
