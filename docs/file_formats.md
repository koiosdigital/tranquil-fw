# Tranquil Binary File Formats

This document describes the binary file formats used by Tranquil firmware for patterns and licensing.

## Table of Contents

1. [License Files (.license)](#license-files)
2. [THRB Files (Unencrypted Patterns)](#thrb-files)
3. [KDEP Files (Encrypted Patterns)](#kdep-files)
4. [Common Constants](#common-constants)

---

## License Files

**File Location:** `/sd/license.dat`

### Magic Number & Version

| Field   | Value        | ASCII  |
|---------|--------------|--------|
| Magic   | `0x4B444C43` | "KDLC" |
| Version | `0x0001`     | -      |

### File Header Structure (14 bytes)

```c
#pragma pack(push, 1)
struct LicenseFileHeader {
    uint32_t magic;         // 0x0000: 0x4B444C43 ("KDLC")
    uint16_t version;       // 0x0004: 0x0001
    uint16_t flags;         // 0x0006: Reserved (0x0000)
    uint32_t payload_len;   // 0x0008: Length of protobuf payload
    uint32_t signature_len; // 0x000C: Length of RSA signature
};
#pragma pack(pop)
```

### File Layout

| Offset | Size     | Description                    |
|--------|----------|--------------------------------|
| 0x0000 | 14       | Header                         |
| 0x000E | variable | Protobuf-encoded LicensePayload |
| -      | variable | RSA-2048 signature (typically 256 bytes) |

### Payload Structure (Protobuf)

```protobuf
message LicensePayload {
    string for_device = 1;     // Device certificate CN (binding)
    uint32 max_patterns = 2;   // Maximum stored patterns allowed
    int64 valid_from = 3;      // Unix timestamp UTC
    int64 valid_to = 4;        // Unix timestamp UTC
    string license_id = 5;     // Unique ID for revocation
    int64 issued_at = 6;       // Issue timestamp (Unix UTC)
    string store_token = 7;    // JWT token (max 384 bytes)
}
```

### Security

- **Signature:** RSA-2048 with SHA-256 over the payload
- **Device Binding:** License `for_device` must match device certificate CN
- **Time Validation:** Current time must be within `valid_from` to `valid_to`

---

## THRB Files

**File Extension:** `.dat`
**File Location:** `/sd/patterns/<uuid>.dat`
**Format Name:** THeta-Rho Binary

### Magic Number

| Field | Value        | ASCII  |
|-------|--------------|--------|
| Magic | `0x42524854` | "THRB" |

### File Header Structure (8 bytes)

```c
#pragma pack(push, 1)
struct UnencryptedPatternHeader {
    uint32_t magic;        // 0x0000: 0x42524854 ("THRB")
    uint32_t point_count;  // 0x0004: Number of points in pattern
};
#pragma pack(pop)
```

### Point Structure (6 bytes per point)

```c
struct BinaryPoint {
    float theta;     // 4 bytes: Angle in radians (IEEE 754 float)
    uint16_t rho;    // 2 bytes: Radius (0-65535 maps to 0.0-1.0)
};
```

### File Layout

| Offset | Size              | Description        |
|--------|-------------------|--------------------|
| 0x0000 | 8                 | Header             |
| 0x0008 | 6 × point_count   | Point data array   |

### Data Encoding

- **theta:** IEEE 754 single-precision float in radians. Values can exceed ±2π for multi-rotation patterns.
- **rho:** 16-bit unsigned integer normalized to [0.0, 1.0] by dividing by 65535.0

### Example File Size

For a pattern with 10,000 points:
```
8 + (6 × 10,000) = 60,008 bytes
```

---

## KDEP Files

**File Extension:** `.dat`
**File Location:** `/sd/patterns/<uuid>.dat`
**Format Name:** KD Encrypted Pattern

### Magic Number & Version

| Field   | Value        | ASCII  |
|---------|--------------|--------|
| Magic   | `0x5045444B` | "KDEP" |
| Version | `0x0001`     | -      |
| Scheme  | `0x01`       | RSA-OAEP + AES-256-CTR |

### File Header Structure (576 bytes)

```c
#pragma pack(push, 1)
struct EncryptedPatternHeader {
    uint32_t magic;              // 0x0000: 0x5045444B ("KDEP")
    uint16_t version;            // 0x0004: 0x0001
    uint8_t scheme;              // 0x0006: 0x01 (RSA-OAEP + AES-CTR)
    uint8_t flags;               // 0x0007: 0x01 = binary format
    uint32_t original_size;      // 0x0008: Unencrypted pattern size
    uint32_t point_count;        // 0x000C: Number of points
    uint8_t original_hash[32];   // 0x0010: SHA-256 of plaintext
    uint8_t encrypted_key[512];  // 0x0030: RSA-4096-OAEP encrypted AES key
    uint8_t iv[16];              // 0x0230: AES-CTR initialization vector
};
#pragma pack(pop)
```

### Header Field Details

| Offset | Name          | Type         | Size | Description                       |
|--------|---------------|--------------|------|-----------------------------------|
| 0x0000 | magic         | uint32_t     | 4    | "KDEP" magic number               |
| 0x0004 | version       | uint16_t     | 2    | Format version (0x0001)           |
| 0x0006 | scheme        | uint8_t      | 1    | Encryption scheme ID (0x01)       |
| 0x0007 | flags         | uint8_t      | 1    | Format flags (0x01 = binary)      |
| 0x0008 | original_size | uint32_t     | 4    | Size of decrypted data in bytes   |
| 0x000C | point_count   | uint32_t     | 4    | Number of points in pattern       |
| 0x0010 | original_hash | uint8_t[32]  | 32   | SHA-256 hash of plaintext         |
| 0x0030 | encrypted_key | uint8_t[512] | 512  | RSA-4096-OAEP encrypted AES key   |
| 0x0230 | iv            | uint8_t[16]  | 16   | AES-CTR initialization vector     |

### File Layout

| Offset | Size     | Description                     |
|--------|----------|---------------------------------|
| 0x0000 | 576      | Header                          |
| 0x0240 | variable | AES-256-CTR encrypted point data |

### Encryption Scheme (0x01)

**Key Encryption:**
- Algorithm: RSA-4096-OAEP with SHA-256
- Input: 32-byte AES-256 key
- Output: 512-byte encrypted key blob

**Data Encryption:**
- Algorithm: AES-256-CTR
- Key: Decrypted from `encrypted_key` field
- IV: 16-byte value from `iv` field
- Plaintext: THRB-format point data (6 bytes × point_count)

### Decryption Process

1. Read and validate header (magic, version, scheme, flags)
2. Verify valid DRM license exists
3. Use ESP32 DS peripheral to RSA-OAEP decrypt the AES key
4. Initialize AES-256-CTR with decrypted key and IV
5. Stream-decrypt ciphertext in 64KB chunks
6. Optionally verify SHA-256 hash against `original_hash`

### Decrypted Data Format

Once decrypted, the data is identical to THRB point data:
```c
struct BinaryPoint {
    float theta;     // 4 bytes
    uint16_t rho;    // 2 bytes
};
// Repeated point_count times
```

---

## Common Constants

### Key Sizes

```c
#define DRM_RSA_KEY_BYTES   512   // RSA-4096 (4096 bits / 8)
#define DRM_AES_KEY_BYTES   32    // AES-256 (256 bits / 8)
#define DRM_SHA256_BYTES    32    // SHA-256 output
```

### File Paths

| Purpose          | Path                        |
|------------------|-----------------------------|
| License file     | `/sd/license.dat`           |
| Pattern storage  | `/sd/patterns/<uuid>.dat`   |

### Magic Number Summary

| Format  | Magic (hex)  | Magic (ASCII) | Header Size |
|---------|--------------|---------------|-------------|
| License | `0x4B444C43` | "KDLC"        | 14 bytes    |
| THRB    | `0x42524854` | "THRB"        | 8 bytes     |
| KDEP    | `0x5045444B` | "KDEP"        | 576 bytes   |

---

## Source Files

- **License:** [drm_license.h](../main/drm/drm_license.h), [drm_license.cpp](../main/drm/drm_license.cpp)
- **THRB:** [PatternReader.h](../main/storage/PatternReader.h), [PatternReader.cpp](../main/storage/PatternReader.cpp)
- **KDEP:** [EncryptedPatternReader.h](../main/storage/EncryptedPatternReader.h), [EncryptedPatternReader.cpp](../main/storage/EncryptedPatternReader.cpp)
- **Protobuf:** [tranquil.proto](../components/protobufs/proto/kd/v1/tranquil.proto)
