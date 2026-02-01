#pragma once

#include "PatternReader.h"
#include <mbedtls/gcm.h>
#include <stdint.h>

/**
 * @brief Encrypted pattern file magic number "KDEP"
 */
#define ENCRYPTED_PATTERN_MAGIC 0x4B444550

/**
 * @brief Encrypted pattern file format version
 */
#define ENCRYPTED_PATTERN_VERSION 0x0001

/**
 * @brief Encryption scheme: RSA-OAEP + AES-256-GCM
 */
#define ENCRYPTION_SCHEME_RSA_OAEP_AES_GCM 1

/**
 * @brief Encrypted pattern file header (568 bytes total)
 */
#pragma pack(push, 1)
struct EncryptedPatternHeader {
    uint32_t magic;              // 0x0000: "KDEP" (0x4B444550)
    uint16_t version;            // 0x0004: Format version (0x0001)
    uint8_t scheme;              // 0x0006: Encryption scheme (1 = RSA-OAEP + AES-GCM)
    uint8_t reserved;            // 0x0007: Reserved (0x00)
    uint32_t original_size;      // 0x0008: Unencrypted pattern size
    uint8_t original_hash[32];   // 0x000C: SHA-256 of plaintext
    uint8_t encrypted_key[512];  // 0x002C: RSA-4096-OAEP encrypted AES key
    uint8_t iv[12];              // 0x022C: AES-GCM nonce
};
#pragma pack(pop)

static_assert(sizeof(EncryptedPatternHeader) == 568,
              "EncryptedPatternHeader size mismatch");

/**
 * @brief Encrypted pattern reader implementing IPatternReader
 *
 * Opens encrypted .dat pattern files, recovers the AES key using the DS
 * peripheral, decrypts the content, and provides streaming access.
 *
 * The entire pattern is decrypted into memory on open() for simplicity
 * and to verify the GCM authentication tag upfront.
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
    static constexpr size_t LINE_BUFFER_SIZE = 512;
    static constexpr const char* PATTERNS_PATH = "/sd/patterns";
    static constexpr size_t AES_GCM_TAG_SIZE = 16;

    // Decrypted pattern data (in memory)
    char* decrypted_data_ = nullptr;
    size_t decrypted_size_ = 0;
    size_t read_offset_ = 0;

    // Line parsing state
    size_t total_lines_ = 0;
    size_t current_line_ = 0;
    bool lines_counted_ = false;
    char line_buffer_[LINE_BUFFER_SIZE];

    // Peek support
    bool has_peeked_ = false;
    PatternPoint peeked_point_;

    // Helper functions
    void getFilePath(const char* uuid, char* path, size_t path_size);
    PatternPoint parseLine(const char* line);
    bool readNextLine(char* buffer, size_t buffer_size);
    esp_err_t decryptFile(const char* path);
};
