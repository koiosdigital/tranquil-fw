/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_signing_handler.S
 * @brief TEE DS/HMAC/SHA256 handlers - position-independent assembly
 *
 * This file contains modular functions for cryptographic operations in the TEE.
 * All code is position-independent (addresses built with MOVI/SLLI/OR).
 * Functions can be copied to RTC FAST memory and called from W0.
 *
 * Peripheral Base Addresses:
 *   DS_BASE     = 0x6003D000
 *   HMAC_BASE   = 0x6003E000
 *   SHA_BASE    = 0x6003B000
 *   SYSTEM_BASE = 0x600C0000
 *
 * Register Convention:
 *   a2-a5: Function arguments / return values
 *   a8-a11: Scratch registers (caller-saved)
 *   a0: Return address (for windowed calls)
 *   a1: Stack pointer
 */

    .section .text

/* ============================================================================
 * Peripheral Register Offsets
 * ============================================================================ */

/* System registers (relative to 0x600C0000) */
#define SYS_PERIP_CLK_EN1_OFF       0x0010
#define SYS_PERIP_RST_EN1_OFF       0x0028

/* Clock/Reset bits */
#define CLK_SHA_BIT                 0x04    /* Bit 2 */
#define CLK_DS_BIT                  0x10    /* Bit 4 */
#define CLK_HMAC_BIT                0x20    /* Bit 5 */
#define CLK_ALL_CRYPTO              0x34    /* Bits 2,4,5 */

/* DS registers (relative to 0x6003D000) */
#define DS_C_Y_OFF                  0x0000
#define DS_C_M_OFF                  0x0200
#define DS_C_RB_OFF                 0x0400
#define DS_C_BOX_OFF                0x0600
#define DS_IV_OFF                   0x0630
#define DS_X_OFF                    0x0800
#define DS_Z_OFF                    0x0A00
#define DS_SET_START_OFF            0x0E00
#define DS_SET_ME_OFF               0x0E04
#define DS_SET_FINISH_OFF           0x0E08
#define DS_QUERY_BUSY_OFF           0x0E0C
#define DS_QUERY_KEY_WRONG_OFF      0x0E10
#define DS_QUERY_CHECK_OFF          0x0E14

/* HMAC registers (relative to 0x6003E000) */
#define HMAC_SET_START_OFF          0x0040
#define HMAC_SET_PARA_PURPOSE_OFF   0x0044
#define HMAC_SET_PARA_KEY_OFF       0x0048
#define HMAC_SET_PARA_FINISH_OFF    0x004C
#define HMAC_QUERY_BUSY_OFF         0x006C
#define HMAC_SET_INVALIDATE_DS_OFF  0x0064

/* SHA registers (relative to 0x6003B000) */
#define SHA_MODE_OFF                0x0000
#define SHA_START_OFF               0x0010
#define SHA_CONTINUE_OFF            0x0014
#define SHA_BUSY_OFF                0x0018
#define SHA_H_MEM_OFF               0x0040
#define SHA_M_MEM_OFF               0x0080

/* SHA mode values */
#define SHA_MODE_SHA256             2

/* HMAC purpose for DS */
#define HMAC_PURPOSE_DS             7

/* Error codes */
#define TEE_OK                      0
#define TEE_ERR_KEY_FAIL            -4
#define TEE_ERR_CRYPTO              -2
#define TEE_ERR_INVALID_LABEL       -8
#define TEE_ERR_UNAUTHORIZED        -9

/* TEE State structure offset (in TEE private memory at 0x600FEF00) */
#define TEE_STATE_ATTESTATION_OFF   0       /* uint32_t attestation_level */

/* RSA-4096 sizes */
#define RSA_BYTES                   512
#define RSA_WORDS                   128

/* OAEP sizes (RSA-4096 with SHA256) */
#define OAEP_HASH_LEN               32
#define OAEP_DB_LEN                 479     /* 512 - 1 - 32 */

/* ============================================================================
 * Helper Macros for Building Addresses
 * ============================================================================
 * Since we can't use literal pools, we build addresses with MOVI/SLLI/OR.
 * These macros assume the result register and a scratch register.
 */

/*
 * Build SYSTEM_BASE (0x600C0000) in register
 * 0x600C0000 = 0x600 << 20 | 0x0C << 16
 */
.macro BUILD_SYSTEM_BASE reg, scratch
    movi    \reg, 0x600
    slli    \reg, \reg, 20          /* 0x60000000 */
    movi    \scratch, 0x0C
    slli    \scratch, \scratch, 16  /* 0x000C0000 */
    or      \reg, \reg, \scratch    /* 0x600C0000 */
.endm

/*
 * Build DS_BASE (0x6003D000) in register
 * 0x6003D000 = 0x600 << 20 | 0x3D << 12
 */
.macro BUILD_DS_BASE reg, scratch
    movi    \reg, 0x600
    slli    \reg, \reg, 20          /* 0x60000000 */
    movi    \scratch, 0x3D
    slli    \scratch, \scratch, 12  /* 0x0003D000 */
    or      \reg, \reg, \scratch    /* 0x6003D000 */
.endm

/*
 * Build HMAC_BASE (0x6003E000) in register
 * 0x6003E000 = 0x600 << 20 | 0x3E << 12
 */
.macro BUILD_HMAC_BASE reg, scratch
    movi    \reg, 0x600
    slli    \reg, \reg, 20          /* 0x60000000 */
    movi    \scratch, 0x3E
    slli    \scratch, \scratch, 12  /* 0x0003E000 */
    or      \reg, \reg, \scratch    /* 0x6003E000 */
.endm

/*
 * Build SHA_BASE (0x6003B000) in register
 * 0x6003B000 = 0x600 << 20 | 0x3B << 12
 */
.macro BUILD_SHA_BASE reg, scratch
    movi    \reg, 0x600
    slli    \reg, \reg, 20          /* 0x60000000 */
    movi    \scratch, 0x3B
    slli    \scratch, \scratch, 12  /* 0x0003B000 */
    or      \reg, \reg, \scratch    /* 0x6003B000 */
.endm

/*
 * Build TEE_API_ADDR (0x600FF000) in register
 * 0x600FF000 = 0x600 << 20 | 0xFF << 12
 */
.macro BUILD_API_ADDR reg, scratch
    movi    \reg, 0x600
    slli    \reg, \reg, 20          /* 0x60000000 */
    movi    \scratch, 0xFF
    slli    \scratch, \scratch, 12  /* 0x000FF000 */
    or      \reg, \reg, \scratch    /* 0x600FF000 */
.endm

/*
 * Build TEE_STATE_ADDR (0x600FEF00) in register
 * 0x600FEF00 = 0x600 << 20 | 0xFE << 12 | 0xF00
 */
.macro BUILD_STATE_ADDR reg, scratch
    movi    \reg, 0x600
    slli    \reg, \reg, 20          /* 0x60000000 */
    movi    \scratch, 0xFE
    slli    \scratch, \scratch, 12  /* 0x000FE000 */
    or      \reg, \reg, \scratch    /* 0x600FE000 */
    movi    \scratch, 0xF00
    or      \reg, \reg, \scratch    /* 0x600FEF00 */
.endm

/* ============================================================================
 * tee_crypto_clocks_enable
 * ============================================================================
 * Enable DS, HMAC, SHA peripheral clocks.
 *
 * Input:  none
 * Output: none
 * Clobbers: a8, a9, a10
 */
    .global tee_crypto_clocks_enable
    .type   tee_crypto_clocks_enable, @function
    .align  4

tee_crypto_clocks_enable:
    /* Build SYSTEM_BASE in a8 */
    BUILD_SYSTEM_BASE a8, a9

    /* Read PERIP_CLK_EN1, set bits 2,4,5, write back */
    l32i    a10, a8, SYS_PERIP_CLK_EN1_OFF
    movi    a9, CLK_ALL_CRYPTO
    or      a10, a10, a9
    s32i    a10, a8, SYS_PERIP_CLK_EN1_OFF
    memw

    ret

    .size   tee_crypto_clocks_enable, . - tee_crypto_clocks_enable

/* ============================================================================
 * tee_crypto_clocks_disable
 * ============================================================================
 * Disable DS, HMAC, SHA peripheral clocks.
 *
 * Input:  none
 * Output: none
 * Clobbers: a8, a9, a10
 */
    .global tee_crypto_clocks_disable
    .type   tee_crypto_clocks_disable, @function
    .align  4

tee_crypto_clocks_disable:
    /* Build SYSTEM_BASE in a8 */
    BUILD_SYSTEM_BASE a8, a9

    /* Read PERIP_CLK_EN1, clear bits 2,4,5, write back */
    l32i    a10, a8, SYS_PERIP_CLK_EN1_OFF
    movi    a9, CLK_ALL_CRYPTO
    movi    a8, -1
    xor     a9, a9, a8              /* Invert to get mask */
    and     a10, a10, a9
    BUILD_SYSTEM_BASE a8, a9
    s32i    a10, a8, SYS_PERIP_CLK_EN1_OFF
    memw

    ret

    .size   tee_crypto_clocks_disable, . - tee_crypto_clocks_disable

/* ============================================================================
 * tee_crypto_reset
 * ============================================================================
 * Reset and release DS, HMAC, SHA peripherals.
 *
 * Input:  none
 * Output: none
 * Clobbers: a8, a9, a10
 */
    .global tee_crypto_reset
    .type   tee_crypto_reset, @function
    .align  4

tee_crypto_reset:
    /* Build SYSTEM_BASE in a8 */
    BUILD_SYSTEM_BASE a8, a9

    /* Set reset bits */
    l32i    a10, a8, SYS_PERIP_RST_EN1_OFF
    movi    a9, CLK_ALL_CRYPTO
    or      a10, a10, a9
    s32i    a10, a8, SYS_PERIP_RST_EN1_OFF
    memw

    /* Clear reset bits */
    l32i    a10, a8, SYS_PERIP_RST_EN1_OFF
    movi    a9, CLK_ALL_CRYPTO
    movi    a8, -1
    xor     a9, a9, a8              /* Invert */
    BUILD_SYSTEM_BASE a8, a10
    l32i    a10, a8, SYS_PERIP_RST_EN1_OFF
    and     a10, a10, a9
    s32i    a10, a8, SYS_PERIP_RST_EN1_OFF
    memw

    ret

    .size   tee_crypto_reset, . - tee_crypto_reset

/* ============================================================================
 * tee_ds_hmac_init
 * ============================================================================
 * Initialize HMAC for DS key derivation.
 *
 * Input:  a2 = key_id (0-5, eFuse key block)
 * Output: a2 = 0 on success, negative on error
 * Clobbers: a8, a9, a10, a11
 */
    .global tee_ds_hmac_init
    .type   tee_ds_hmac_init, @function
    .align  4

tee_ds_hmac_init:
    /* Build HMAC_BASE in a8 */
    BUILD_HMAC_BASE a8, a9

    /* Start HMAC: write 1 to SET_START */
    movi    a10, 1
    s32i    a10, a8, HMAC_SET_START_OFF
    memw

    /* Wait for HMAC idle */
.Lhmac_init_wait1:
    l32i    a10, a8, HMAC_QUERY_BUSY_OFF
    bnez    a10, .Lhmac_init_wait1

    /* Set purpose = 7 (DS key derivation) */
    movi    a10, HMAC_PURPOSE_DS
    s32i    a10, a8, HMAC_SET_PARA_PURPOSE_OFF

    /* Set key slot from a2 */
    s32i    a2, a8, HMAC_SET_PARA_KEY_OFF

    /* Apply configuration */
    movi    a10, 1
    s32i    a10, a8, HMAC_SET_PARA_FINISH_OFF
    memw

    /* Wait for HMAC idle */
.Lhmac_init_wait2:
    l32i    a10, a8, HMAC_QUERY_BUSY_OFF
    bnez    a10, .Lhmac_init_wait2

    /* Build DS_BASE in a8 */
    BUILD_DS_BASE a8, a9

    /* Trigger HMAC->DS key derivation */
    movi    a10, 1
    s32i    a10, a8, DS_SET_START_OFF
    memw

    /* Wait for DS idle (key derivation complete) */
.Lds_key_wait:
    l32i    a10, a8, DS_QUERY_BUSY_OFF
    bnez    a10, .Lds_key_wait

    /* Check for key errors */
    l32i    a10, a8, DS_QUERY_KEY_WRONG_OFF
    bnez    a10, .Lds_key_error

    /* Success */
    movi    a2, TEE_OK
    ret

.Lds_key_error:
    movi    a2, TEE_ERR_KEY_FAIL
    ret

    .size   tee_ds_hmac_init, . - tee_ds_hmac_init

/* ============================================================================
 * tee_ds_load_message
 * ============================================================================
 * Copy 512-byte message/ciphertext to DS_X_BASE.
 *
 * Input:  a2 = pointer to 512-byte source buffer
 * Output: none
 * Clobbers: a8, a9, a10, a11, a3
 */
    .global tee_ds_load_message
    .type   tee_ds_load_message, @function
    .align  4

tee_ds_load_message:
    /* Build DS_BASE + DS_X_OFF in a8 */
    BUILD_DS_BASE a8, a9
    movi    a9, DS_X_OFF
    add     a8, a8, a9              /* a8 = DS_X_BASE */

    /* Copy 128 words (512 bytes) */
    movi    a3, RSA_WORDS

.Lds_load_msg_loop:
    l32i    a10, a2, 0              /* Load from source */
    s32i    a10, a8, 0              /* Store to DS_X */
    addi    a2, a2, 4
    addi    a8, a8, 4
    addi    a3, a3, -1
    bnez    a3, .Lds_load_msg_loop

    memw
    ret

    .size   tee_ds_load_message, . - tee_ds_load_message

/* ============================================================================
 * tee_ds_execute
 * ============================================================================
 * Execute DS signing operation and wait for completion.
 *
 * Input:  none (assumes params and message already loaded)
 * Output: a2 = 0 on success, negative on error
 * Clobbers: a8, a9, a10
 */
    .global tee_ds_execute
    .type   tee_ds_execute, @function
    .align  4

tee_ds_execute:
    /* Build DS_BASE in a8 */
    BUILD_DS_BASE a8, a9

    /* Start signing: write 1 to DS_SET_ME */
    movi    a10, 1
    s32i    a10, a8, DS_SET_ME_OFF
    memw

    /* Wait for completion (this is the long wait ~100-500ms) */
.Lds_exec_wait:
    l32i    a10, a8, DS_QUERY_BUSY_OFF
    bnez    a10, .Lds_exec_wait

    /* Check result: 0=OK, 1=MD_FAIL, 2=PAD_FAIL, 3=BOTH */
    l32i    a10, a8, DS_QUERY_CHECK_OFF
    beqi    a10, 1, .Lds_exec_error     /* MD fail */
    beqi    a10, 3, .Lds_exec_error     /* Both fail */

    /* Success (0 or 2=padding fail is still readable) */
    movi    a2, TEE_OK
    ret

.Lds_exec_error:
    movi    a2, TEE_ERR_CRYPTO
    ret

    .size   tee_ds_execute, . - tee_ds_execute

/* ============================================================================
 * tee_ds_read_result
 * ============================================================================
 * Copy 512-byte result from DS_Z_BASE to buffer.
 *
 * Input:  a2 = pointer to 512-byte destination buffer
 * Output: none
 * Clobbers: a8, a9, a10, a3
 */
    .global tee_ds_read_result
    .type   tee_ds_read_result, @function
    .align  4

tee_ds_read_result:
    /* Build DS_BASE + DS_Z_OFF in a8 */
    BUILD_DS_BASE a8, a9
    movi    a9, DS_Z_OFF
    add     a8, a8, a9              /* a8 = DS_Z_BASE */

    /* Copy 128 words (512 bytes) */
    movi    a3, RSA_WORDS

.Lds_read_loop:
    l32i    a10, a8, 0              /* Load from DS_Z */
    s32i    a10, a2, 0              /* Store to dest */
    addi    a8, a8, 4
    addi    a2, a2, 4
    addi    a3, a3, -1
    bnez    a3, .Lds_read_loop

    ret

    .size   tee_ds_read_result, . - tee_ds_read_result

/* ============================================================================
 * tee_ds_cleanup
 * ============================================================================
 * Cleanup DS peripheral state.
 *
 * Input:  none
 * Output: none
 * Clobbers: a8, a9, a10
 */
    .global tee_ds_cleanup
    .type   tee_ds_cleanup, @function
    .align  4

tee_ds_cleanup:
    /* Build DS_BASE in a8 */
    BUILD_DS_BASE a8, a9

    /* Finish DS operation */
    movi    a10, 1
    s32i    a10, a8, DS_SET_FINISH_OFF
    memw

    /* Wait for idle */
.Lds_cleanup_wait:
    l32i    a10, a8, DS_QUERY_BUSY_OFF
    bnez    a10, .Lds_cleanup_wait

    /* Invalidate HMAC->DS key */
    BUILD_HMAC_BASE a8, a9
    movi    a10, 1
    s32i    a10, a8, HMAC_SET_INVALIDATE_DS_OFF
    memw

    ret

    .size   tee_ds_cleanup, . - tee_ds_cleanup

/* ============================================================================
 * tee_sha256_init
 * ============================================================================
 * Initialize SHA peripheral for SHA256.
 *
 * Input:  none
 * Output: none
 * Clobbers: a8, a9, a10
 */
    .global tee_sha256_init
    .type   tee_sha256_init, @function
    .align  4

tee_sha256_init:
    /* Build SHA_BASE in a8 */
    BUILD_SHA_BASE a8, a9

    /* Set mode = SHA256 (2) */
    movi    a10, SHA_MODE_SHA256
    s32i    a10, a8, SHA_MODE_OFF
    memw

    ret

    .size   tee_sha256_init, . - tee_sha256_init

/* ============================================================================
 * tee_sha256_block
 * ============================================================================
 * Process one 64-byte SHA256 block.
 *
 * Input:  a2 = pointer to 64-byte block
 *         a3 = 0 for first block (START), nonzero for continuation (CONTINUE)
 * Output: none
 * Clobbers: a8, a9, a10, a11, a4
 */
    .global tee_sha256_block
    .type   tee_sha256_block, @function
    .align  4

tee_sha256_block:
    /* Build SHA_BASE in a8 */
    BUILD_SHA_BASE a8, a9

    /* Copy 16 words (64 bytes) to SHA_M_MEM */
    movi    a9, SHA_M_MEM_OFF
    add     a9, a8, a9              /* a9 = SHA_M_MEM address */
    movi    a4, 16                  /* 16 words */

.Lsha_block_copy:
    l32i    a10, a2, 0
    s32i    a10, a9, 0
    addi    a2, a2, 4
    addi    a9, a9, 4
    addi    a4, a4, -1
    bnez    a4, .Lsha_block_copy

    /* Start or Continue based on a3 */
    beqz    a3, .Lsha_first_block

    /* Continuation block */
    movi    a10, 1
    s32i    a10, a8, SHA_CONTINUE_OFF
    j       .Lsha_block_wait

.Lsha_first_block:
    /* First block */
    movi    a10, 1
    s32i    a10, a8, SHA_START_OFF

.Lsha_block_wait:
    memw
    /* Wait for completion */
.Lsha_busy_wait:
    l32i    a10, a8, SHA_BUSY_OFF
    bnez    a10, .Lsha_busy_wait

    ret

    .size   tee_sha256_block, . - tee_sha256_block

/* ============================================================================
 * tee_sha256_read_hash
 * ============================================================================
 * Read 32-byte hash result from SHA_H_MEM.
 *
 * Input:  a2 = pointer to 32-byte output buffer
 * Output: none
 * Clobbers: a8, a9, a10, a3
 */
    .global tee_sha256_read_hash
    .type   tee_sha256_read_hash, @function
    .align  4

tee_sha256_read_hash:
    /* Build SHA_BASE + SHA_H_MEM_OFF in a8 */
    BUILD_SHA_BASE a8, a9
    movi    a9, SHA_H_MEM_OFF
    add     a8, a8, a9              /* a8 = SHA_H_MEM address */

    /* Copy 8 words (32 bytes) */
    movi    a3, 8

.Lsha_read_loop:
    l32i    a10, a8, 0
    s32i    a10, a2, 0
    addi    a8, a8, 4
    addi    a2, a2, 4
    addi    a3, a3, -1
    bnez    a3, .Lsha_read_loop

    ret

    .size   tee_sha256_read_hash, . - tee_sha256_read_hash

/* ============================================================================
 * tee_memcpy32
 * ============================================================================
 * Copy words (must be word-aligned, word count).
 *
 * Input:  a2 = destination pointer
 *         a3 = source pointer
 *         a4 = word count
 * Output: none
 * Clobbers: a8, a9
 */
    .global tee_memcpy32
    .type   tee_memcpy32, @function
    .align  4

tee_memcpy32:
    beqz    a4, .Lmemcpy32_done

.Lmemcpy32_loop:
    l32i    a8, a3, 0
    s32i    a8, a2, 0
    addi    a2, a2, 4
    addi    a3, a3, 4
    addi    a4, a4, -1
    bnez    a4, .Lmemcpy32_loop

.Lmemcpy32_done:
    ret

    .size   tee_memcpy32, . - tee_memcpy32

/* ============================================================================
 * tee_memcmp32
 * ============================================================================
 * Compare words.
 *
 * Input:  a2 = pointer 1
 *         a3 = pointer 2
 *         a4 = word count
 * Output: a2 = 0 if equal, nonzero if different
 * Clobbers: a8, a9, a10
 */
    .global tee_memcmp32
    .type   tee_memcmp32, @function
    .align  4

tee_memcmp32:
    beqz    a4, .Lmemcmp32_equal

.Lmemcmp32_loop:
    l32i    a8, a2, 0
    l32i    a9, a3, 0
    bne     a8, a9, .Lmemcmp32_diff
    addi    a2, a2, 4
    addi    a3, a3, 4
    addi    a4, a4, -1
    bnez    a4, .Lmemcmp32_loop

.Lmemcmp32_equal:
    movi    a2, 0
    ret

.Lmemcmp32_diff:
    movi    a2, 1
    ret

    .size   tee_memcmp32, . - tee_memcmp32

/* ============================================================================
 * tee_xor_bytes
 * ============================================================================
 * XOR byte arrays (word-aligned, word count for efficiency).
 *
 * Input:  a2 = destination pointer (also source 1)
 *         a3 = source 2 pointer
 *         a4 = word count
 * Output: destination = destination XOR source2
 * Clobbers: a8, a9, a10
 */
    .global tee_xor_bytes
    .type   tee_xor_bytes, @function
    .align  4

tee_xor_bytes:
    beqz    a4, .Lxor_done

.Lxor_loop:
    l32i    a8, a2, 0               /* Load dest word */
    l32i    a9, a3, 0               /* Load src2 word */
    xor     a8, a8, a9              /* XOR */
    s32i    a8, a2, 0               /* Store back */
    addi    a2, a2, 4
    addi    a3, a3, 4
    addi    a4, a4, -1
    bnez    a4, .Lxor_loop

.Lxor_done:
    ret

    .size   tee_xor_bytes, . - tee_xor_bytes

/* ============================================================================
 * tee_memzero32
 * ============================================================================
 * Zero memory (word-aligned).
 *
 * Input:  a2 = pointer
 *         a3 = word count
 * Output: none
 * Clobbers: a8
 */
    .global tee_memzero32
    .type   tee_memzero32, @function
    .align  4

tee_memzero32:
    beqz    a3, .Lmemzero_done
    movi    a8, 0

.Lmemzero_loop:
    s32i    a8, a2, 0
    addi    a2, a2, 4
    addi    a3, a3, -1
    bnez    a3, .Lmemzero_loop

.Lmemzero_done:
    ret

    .size   tee_memzero32, . - tee_memzero32

/* ============================================================================
 * tee_check_attestation
 * ============================================================================
 * Check attestation_level in TEE private memory.
 * Used to gate OAEP decryption operations - if not attested, fail.
 *
 * Input:  none
 * Output: a2 = 0 (TEE_OK) if attestation_level != 0
 *         a2 = TEE_ERR_UNAUTHORIZED (-9) if attestation_level == 0
 * Clobbers: a8, a9, a10
 */
    .global tee_check_attestation
    .type   tee_check_attestation, @function
    .align  4

tee_check_attestation:
    /* Build TEE_STATE_ADDR (0x600FEF00) in a8 */
    BUILD_STATE_ADDR a8, a9

    /* Read attestation_level at offset 0 */
    memw
    l32i    a10, a8, TEE_STATE_ATTESTATION_OFF

    /* If attestation_level == 0, unauthorized */
    beqz    a10, .Lattest_unauthorized

    /* Authorized */
    movi    a2, TEE_OK
    ret

.Lattest_unauthorized:
    movi    a2, TEE_ERR_UNAUTHORIZED
    ret

    .size   tee_check_attestation, . - tee_check_attestation

/* ============================================================================
 * End Markers for Size Calculation
 * ============================================================================ */

    .global tee_signing_handler_end
tee_signing_handler_end:

