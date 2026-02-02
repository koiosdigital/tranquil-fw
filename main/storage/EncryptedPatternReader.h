#pragma once

#include "PatternReader.h"
#include "drm/drm_crypto.h"
#include <mbedtls/aes.h>
#include <stdint.h>

/**
 * @brief Encrypted pattern file magic number "KDEP"
 */
#define ENCRYPTED_PATTERN_MAGIC 0x5045444B

/**
 * @brief Encrypted pattern file format version
 */
#define ENCRYPTED_PATTERN_VERSION 0x0001

/**
 * @brief Encryption scheme: RSA-OAEP + AES-256-CTR
 */
#define ENCRYPTION_SCHEME_RSA_OAEP_AES_CTR 1

/**
 * @brief Encrypted pattern header flags
 */
#define ENCRYPTED_PATTERN_FLAG_BINARY 0x01

/**
 * @brief Encrypted pattern file header (576 bytes total)
 *
 * Uses AES-256-CTR for streaming decryption with random seek support.
 * Integrity verified on download only (SHA-256 in original_hash).
 */
#pragma pack(push, 1)
struct EncryptedPatternHeader {
    uint32_t magic;              // 0x0000: "KDEP" (0x4B444550)
    uint16_t version;            // 0x0004: Format version (0x0001)
    uint8_t scheme;              // 0x0006: Encryption scheme (1 = RSA-OAEP + AES-CTR)
    uint8_t flags;               // 0x0007: Flags (0x01 = binary format)
    uint32_t original_size;      // 0x0008: Unencrypted pattern size in bytes
    uint32_t point_count;        // 0x000C: Number of points
    uint8_t original_hash[32];   // 0x0010: SHA-256 of plaintext (verified on download)
    uint8_t encrypted_key[512];  // 0x0030: RSA-4096-OAEP encrypted AES key
    uint8_t iv[16];              // 0x0230: AES-CTR IV (128-bit)
};
#pragma pack(pop)

static_assert(sizeof(EncryptedPatternHeader) == 576,
    "EncryptedPatternHeader size mismatch");

/**
 * @brief Encrypted pattern reader with streaming AES-CTR decryption
 *
 * Opens encrypted .dat pattern files (KDEP magic), recovers the AES key
 * using the DS peripheral, and provides streaming decryption access.
 *
 * Bufferless design: decrypts one point (4 bytes) at a time directly from file.
 * AES-CTR maintains partial block state, allowing byte-granular decryption.
 * Total memory footprint: ~1.2KB (dominated by mbedtls_aes_context).
 *
 * Note: Integrity (SHA-256) is verified on download only, not during playback.
 */
class EncryptedPatternReader : public IPatternReader {
public:
    EncryptedPatternReader();
    ~EncryptedPatternReader() override;

    // Prevent copying
    EncryptedPatternReader(const EncryptedPatternReader&) = delete;
    EncryptedPatternReader& operator=(const EncryptedPatternReader&) = delete;

    esp_err_t open(const char* uuid) override;
    void close() override;
    bool isOpen() const override;
    size_t getTotalLines() override;
    size_t getCurrentLine() const override;
    PatternPoint readNext() override;
    PatternPoint peekNext() override;
    bool hasMore() const override;
    esp_err_t rewind() override;

private:
    static constexpr const char* PATTERNS_PATH = "/sd/patterns";

    // File handle kept open for streaming
    FILE* file_ = nullptr;

    // Header info (stored for rewind)
    EncryptedPatternHeader header_;

    // AES key kept in memory for lifetime of reader
    uint8_t aes_key_[DRM_AES_KEY_BYTES];
    bool has_key_ = false;

    // AES-CTR context for streaming decryption
    mbedtls_aes_context aes_;
    bool aes_initialized_ = false;
    size_t nc_off_ = 0;              // Offset within current block
    uint8_t nonce_counter_[16];      // Current counter value
    uint8_t stream_block_[16];       // Cached keystream block

    // Point tracking
    size_t current_point_ = 0;

    // Peek support
    bool has_peeked_ = false;
    PatternPoint peeked_point_;

    // Saved CTR state for peek restore
    size_t saved_nc_off_ = 0;
    uint8_t saved_nonce_counter_[16];
    uint8_t saved_stream_block_[16];

    // Helper functions
    void getFilePath(const char* uuid, char* path, size_t path_size);
    PatternPoint readBinaryPoint();

    // Streaming helpers
    esp_err_t openFile(const char* path);
    esp_err_t initDecryption();
    void saveCtrState();
    void restoreCtrState();
    void clearKey();
};
