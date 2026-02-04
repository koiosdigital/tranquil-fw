# TEE Signing Handler Documentation

Position-independent assembly functions for cryptographic operations in the TEE (World 0).

## Overview

The `tee_signing_handler.S` file provides modular assembly functions for:
- DS (Digital Signature) peripheral operations
- HMAC peripheral operations
- SHA256 peripheral operations
- Memory utilities (copy, compare, XOR, zero)

All code is **position-independent** - addresses are built using `MOVI/SLLI/OR` to avoid literal pools. Functions can be copied to RTC FAST memory and executed in World 0.

## Peripheral Register Map

### Base Addresses

| Peripheral | Base Address |
|------------|-------------|
| DS | `0x6003D000` |
| HMAC | `0x6003E000` |
| SHA | `0x6003B000` |
| SYSTEM | `0x600C0000` |

### DS Registers

| Register | Offset | Address | Purpose |
|----------|--------|---------|---------|
| DS_C_Y | 0x0000 | 0x6003D000 | Encrypted Y param (512B) |
| DS_C_M | 0x0200 | 0x6003D200 | Encrypted M param (512B) |
| DS_C_RB | 0x0400 | 0x6003D400 | Encrypted RB param (512B) |
| DS_C_BOX | 0x0600 | 0x6003D600 | Encrypted BOX param |
| DS_IV | 0x0630 | 0x6003D630 | IV (16 bytes) |
| DS_X | 0x0800 | 0x6003D800 | Input message (512B) |
| DS_Z | 0x0A00 | 0x6003DA00 | Output signature (512B) |
| DS_SET_START | 0x0E00 | 0x6003DE00 | Start HMAC→DS key derivation |
| DS_SET_ME | 0x0E04 | 0x6003DE04 | Start signing operation |
| DS_SET_FINISH | 0x0E08 | 0x6003DE08 | Cleanup |
| DS_QUERY_BUSY | 0x0E0C | 0x6003DE0C | Poll: 0=idle |
| DS_QUERY_KEY_WRONG | 0x0E10 | 0x6003DE10 | Key error check |
| DS_QUERY_CHECK | 0x0E14 | 0x6003DE14 | Result status |

### HMAC Registers

| Register | Offset | Address | Purpose |
|----------|--------|---------|---------|
| HMAC_SET_START | 0x0040 | 0x6003E040 | Start HMAC |
| HMAC_SET_PARA_PURPOSE | 0x0044 | 0x6003E044 | Purpose (7=DS) |
| HMAC_SET_PARA_KEY | 0x0048 | 0x6003E048 | Key slot (0-5) |
| HMAC_SET_PARA_FINISH | 0x004C | 0x6003E04C | Apply config |
| HMAC_QUERY_BUSY | 0x006C | 0x6003E06C | Poll: 0=idle |
| HMAC_SET_INVALIDATE_DS | 0x0064 | 0x6003E064 | Invalidate DS key |

### SHA Registers

| Register | Offset | Address | Purpose |
|----------|--------|---------|---------|
| SHA_MODE | 0x0000 | 0x6003B000 | Algorithm (2=SHA256) |
| SHA_START | 0x0010 | 0x6003B010 | Start new hash |
| SHA_CONTINUE | 0x0014 | 0x6003B014 | Continue hash |
| SHA_BUSY | 0x0018 | 0x6003B018 | Poll: 0=idle |
| SHA_H_MEM | 0x0040 | 0x6003B040 | Hash state (8 words) |
| SHA_M_MEM | 0x0080 | 0x6003B080 | Message block (16 words) |

### System Clock/Reset

| Register | Address | Bits |
|----------|---------|------|
| PERIP_CLK_EN1 | 0x600C0010 | Bit 2=SHA, Bit 4=DS, Bit 5=HMAC |
| PERIP_RST_EN1 | 0x600C0028 | Same bits for reset |

## Function Reference

### Clock/Reset Functions

#### `tee_crypto_clocks_enable`

Enable DS, HMAC, and SHA peripheral clocks.

```
Input:  none
Output: none
Clobbers: a8, a9, a10
```

#### `tee_crypto_clocks_disable`

Disable DS, HMAC, and SHA peripheral clocks.

```
Input:  none
Output: none
Clobbers: a8, a9, a10
```

#### `tee_crypto_reset`

Reset and release DS, HMAC, and SHA peripherals.

```
Input:  none
Output: none
Clobbers: a8, a9, a10
```

### DS Functions

#### `tee_ds_hmac_init`

Initialize HMAC for DS key derivation. Configures HMAC to derive the decryption key from the specified eFuse key block and pass it to the DS peripheral.

```
Input:  a2 = key_id (0-5, eFuse key block)
Output: a2 = 0 on success, TEE_ERR_KEY_FAIL (-4) on error
Clobbers: a8, a9, a10, a11
```

#### `tee_ds_load_message`

Copy 512-byte message/ciphertext to DS_X_BASE register.

```
Input:  a2 = pointer to 512-byte source buffer
Output: none
Clobbers: a8, a9, a10, a11, a3
```

#### `tee_ds_execute`

Execute DS signing/decryption operation and wait for completion.

```
Input:  none (assumes params and message already loaded)
Output: a2 = 0 on success, TEE_ERR_CRYPTO (-2) on error
Clobbers: a8, a9, a10
```

**Note:** This function blocks for ~100-500ms depending on RSA key size.

#### `tee_ds_read_result`

Copy 512-byte result from DS_Z_BASE register to buffer.

```
Input:  a2 = pointer to 512-byte destination buffer
Output: none
Clobbers: a8, a9, a10, a3
```

#### `tee_ds_cleanup`

Cleanup DS peripheral state. Finishes operation and invalidates the derived key.

```
Input:  none
Output: none
Clobbers: a8, a9, a10
```

### SHA256 Functions

#### `tee_sha256_init`

Initialize SHA peripheral for SHA256 mode.

```
Input:  none
Output: none
Clobbers: a8, a9, a10
```

#### `tee_sha256_block`

Process one 64-byte SHA256 block.

```
Input:  a2 = pointer to 64-byte block
        a3 = 0 for first block (SHA_START), nonzero for continuation (SHA_CONTINUE)
Output: none
Clobbers: a8, a9, a10, a11, a4
```

#### `tee_sha256_read_hash`

Read 32-byte hash result from SHA_H_MEM.

```
Input:  a2 = pointer to 32-byte output buffer
Output: none
Clobbers: a8, a9, a10, a3
```

### Memory Utilities

#### `tee_memcpy32`

Copy words (word-aligned, word count).

```
Input:  a2 = destination pointer
        a3 = source pointer
        a4 = word count
Output: none
Clobbers: a8, a9
```

#### `tee_memcmp32`

Compare words.

```
Input:  a2 = pointer 1
        a3 = pointer 2
        a4 = word count
Output: a2 = 0 if equal, nonzero if different
Clobbers: a8, a9, a10
```

#### `tee_xor_bytes`

XOR byte arrays (word-aligned, word count).

```
Input:  a2 = destination pointer (also source 1)
        a3 = source 2 pointer
        a4 = word count
Output: destination = destination XOR source2
Clobbers: a8, a9, a10
```

#### `tee_memzero32`

Zero memory (word-aligned).

```
Input:  a2 = pointer
        a3 = word count
Output: none
Clobbers: a8
```

## DS Operation Sequence

Complete sequence for RSA private key operation:

```
1. tee_crypto_clocks_enable()      // Enable peripheral clocks
2. tee_crypto_reset()              // Reset peripherals
3. tee_ds_hmac_init(key_id)        // Configure HMAC, derive key
4. [Load encrypted params to DS_C_* registers]  // Not yet implemented
5. [Load IV to DS_IV register]                  // Not yet implemented
6. tee_ds_load_message(ciphertext) // Load input
7. tee_ds_execute()                // Execute RSA operation (~100-500ms)
8. tee_ds_read_result(output)      // Read result
9. tee_ds_cleanup()                // Cleanup state
10. tee_crypto_clocks_disable()    // Disable clocks
```

## SHA256 Operation Sequence

For hashing a message:

```
1. tee_crypto_clocks_enable()      // Enable SHA clock
2. tee_sha256_init()               // Set SHA256 mode
3. tee_sha256_block(block1, 0)     // First block (START)
4. tee_sha256_block(block2, 1)     // Subsequent blocks (CONTINUE)
5. ... repeat for all blocks ...
6. tee_sha256_read_hash(output)    // Read 32-byte result
7. tee_crypto_clocks_disable()     // Disable clocks
```

**Note:** Caller is responsible for SHA256 padding (0x80, zeros, 64-bit length).

## MGF1-SHA256 Algorithm (for OAEP)

MGF1 generates mask bytes from a seed:

```
mask = SHA256(seed || 0x00000000) || SHA256(seed || 0x00000001) || ...
```

Implementation requires:
1. Concatenate seed with 4-byte big-endian counter
2. Pad to 64 bytes for SHA256
3. Hash and append to output
4. Increment counter and repeat until output length reached

For RSA-4096 OAEP with SHA256:
- `seedMask` = MGF1(maskedDB, 32) → ~15 SHA256 calls
- `dbMask` = MGF1(seed, 479) → ~15 SHA256 calls

## OAEP Decoding (for Pattern Key)

After RSA decryption, the 512-byte output has structure:

```
[0]      = 0x00           (must be zero)
[1-32]   = maskedSeed     (32 bytes)
[33-511] = maskedDB       (479 bytes)
```

Unmasking sequence:
1. `seedMask = MGF1(maskedDB, 32)`
2. `seed = maskedSeed XOR seedMask`
3. `dbMask = MGF1(seed, 479)`
4. `DB = maskedDB XOR dbMask`

DB structure after unmasking:
```
[0-31]   = lHash          (SHA256 of label)
[32-N]   = 0x00...        (padding zeros)
[N+1]    = 0x01           (separator)
[N+2-...]= message        (the AES key)
```

### Custom OAEP Label for Pattern Keys

For TEE gating, pattern keys use custom label `"koios:pattern:v1"`:

```
lHash = SHA256("koios:pattern:v1")
      = f13d32f9d29adfdf19855fc6c3121e3a0ab3f0a31d0785d3eea02d84b129ef39
```

As byte array:
```c
static const uint8_t PATTERN_KEY_LHASH[32] = {
    0xf1, 0x3d, 0x32, 0xf9, 0xd2, 0x9a, 0xdf, 0xdf,
    0x19, 0x85, 0x5f, 0xc6, 0xc3, 0x12, 0x1e, 0x3a,
    0x0a, 0xb3, 0xf0, 0xa3, 0x1d, 0x07, 0x85, 0xd3,
    0xee, 0xa0, 0x2d, 0x84, 0xb1, 0x29, 0xef, 0x39
};
```

TEE validates this lHash after OAEP decoding to ensure the ciphertext was encrypted with the correct label.

## TEE Gating Logic

### Attestation Level

The TEE maintains an `attestation_level` flag in **TEE private memory** (W0-only, not accessible from W1):

```
TEE Private Memory Layout (0x600FE000):
  0x600FEF00 = tee_state_t
    +0x00: attestation_level (uint32_t) - 0=not attested, 1=attested
```

### Operation Flow

```
1. Normal signing (TLS CertificateVerify):
   - Always allowed
   - Execute DS operation
   - Return signature to W1

2. OAEP decryption (pattern key):
   - Execute DS operation (raw RSA decrypt)
   - Unmask OAEP, validate lHash
   - Check attestation_level from TEE private memory:
     - If attestation_level == 0: FAIL, zero output, return error
     - If attestation_level == 1: Copy AES key to shared memory, return OK
```

### Why This Works

- `attestation_level` is in W0-only memory - W1 app cannot modify it
- Only the TEE (or bootloader during init) can set attestation_level
- Pattern decryption is gated on device attestation state
- TLS signing is unaffected (always works)

### Handler Pseudo-code

```c
int tee_ds_handler(void) {
    uint32_t request_type = TEE_API()->request;

    // Enable crypto clocks, init HMAC/DS
    tee_crypto_clocks_enable();
    tee_ds_hmac_init(key_id);

    // Load params and execute DS operation
    tee_ds_load_params(...);
    tee_ds_load_message(ciphertext);
    int result = tee_ds_execute();
    if (result != 0) goto cleanup;

    tee_ds_read_result(output);

    if (request_type == TEE_REQ_SIGN) {
        // Normal signing - always allowed
        // Copy signature to shared memory
        memcpy(TEE_API()->output, output, 512);
        result = TEE_OK;
    }
    else if (request_type == TEE_REQ_PATTERN_KEY) {
        // OAEP decryption - check attestation
        tee_oaep_unmask(output);
        tee_oaep_validate_lhash(output + 1 + 32, PATTERN_KEY_LHASH);

        // ** GATING CHECK **
        if (TEE_STATE()->attestation_level == 0) {
            // Not attested - FAIL
            tee_memzero32(output, 128);  // Zero sensitive data
            result = TEE_ERR_UNAUTHORIZED;
        } else {
            // Attested - extract and return key
            tee_oaep_extract_key(output, aes_key);
            memcpy(TEE_API()->output, aes_key, 32);
            result = TEE_OK;
        }
    }

cleanup:
    tee_ds_cleanup();
    tee_crypto_clocks_disable();
    TEE_API()->last_result = result;
    return result;
}
```

## Error Codes

| Code | Value | Description |
|------|-------|-------------|
| TEE_OK | 0 | Success |
| TEE_ERR_CRYPTO | -2 | Cryptographic operation failed |
| TEE_ERR_KEY_FAIL | -4 | Key derivation failed |
| TEE_ERR_INVALID_LABEL | -8 | OAEP lHash validation failed |

## Integration Guide

### Required Changes to `tee_config.h`

Add attestation state to `tee_state_t`:

```c
typedef struct {
    uint32_t attestation_level;          /* 0=not attested, 1=attested */
    uint8_t wcl_msg_seq_core0;
    uint8_t wcl_msg_seq_core1;
    uint8_t reserved[57];                /* Padding to 64 bytes */
} __attribute__((packed, aligned(4))) tee_state_t;

#define TEE_STATE() ((volatile tee_state_t*)TEE_STATE_ADDR)
```

Add request type constants:

```c
#define TEE_REQ_NONE           0x00000000
#define TEE_REQ_PATTERN_KEY    0x504B4559  /* "PKEY" */
#define TEE_REQ_SIGN           0x5349474E  /* "SIGN" */
```

Add function declarations:

```c
/* tee_signing_handler.S */
void tee_crypto_clocks_enable(void);
void tee_crypto_clocks_disable(void);
void tee_crypto_reset(void);
int tee_ds_hmac_init(uint32_t key_id);
void tee_ds_load_message(const uint8_t* src);
int tee_ds_execute(void);
void tee_ds_read_result(uint8_t* dst);
void tee_ds_cleanup(void);
void tee_sha256_init(void);
void tee_sha256_block(const uint8_t* block, int is_continue);
void tee_sha256_read_hash(uint8_t* dst);
void tee_memcpy32(uint32_t* dst, const uint32_t* src, size_t words);
int tee_memcmp32(const uint32_t* p1, const uint32_t* p2, size_t words);
void tee_xor_bytes(uint32_t* dst, const uint32_t* src, size_t words);
void tee_memzero32(uint32_t* ptr, size_t words);
extern void tee_signing_handler_end(void);
```

### Required Changes to `tee_init.c`

Copy handler to RTC memory:

```c
/* Copy signing handler functions */
size_t handler_size = (size_t)((uint8_t*)tee_signing_handler_end -
                               (uint8_t*)tee_crypto_clocks_enable);
memcpy((void*)TEE_ENTRY_SIGN, (const void*)tee_crypto_clocks_enable, handler_size);
```

### Required Changes to `tee_api.h`

Extend shared memory structure for DS operations:

```c
typedef struct tee_ds_request {
    uint8_t ciphertext[512];      /* RSA-4096 input */
    uint8_t ds_params_c[2040];    /* Encrypted DS params */
    uint8_t ds_iv[16];            /* IV */
    uint8_t output[32];           /* AES key output */
    uint8_t key_id;               /* eFuse key block (0-5) */
    uint8_t padding[3];           /* Alignment */
} tee_ds_request_t;
```

### Cloud/Tooling Changes

When encrypting pattern keys, use OAEP label:

```python
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import hashes

ciphertext = public_key.encrypt(
    aes_key,
    padding.OAEP(
        mgf=padding.MGF1(algorithm=hashes.SHA256()),
        algorithm=hashes.SHA256(),
        label=b"koios:pattern:v1"  # Custom label for TEE validation
    )
)
```

## Size Estimates

| Component | Approximate Size |
|-----------|-----------------|
| Clock/reset functions | ~150 bytes |
| DS functions | ~400 bytes |
| SHA256 functions | ~250 bytes |
| Memory utilities | ~200 bytes |
| **Total** | ~1000 bytes |

All functions fit comfortably in TEE private region (4KB).
