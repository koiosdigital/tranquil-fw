/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_api.h
 * @brief TEE API structures and definitions for app client
 *
 * This header mirrors the bootloader's tee_config.h API structures.
 * Keep in sync with bootloader_components/koios_tee/include/tee_config.h
 */

#ifndef TEE_API_H
#define TEE_API_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Memory Addresses
 * ============================================================================ */

#define TEE_API_ADDR            0x600FF000  /* Shared API structure */
#define TEE_ENTRY_STUB          0x600FE100  /* W1->W0 entry (WCL monitored) */

/* ============================================================================
 * Service IDs
 * ============================================================================ */

typedef enum {
    TEE_SVC_NOP         = 0,    /* No operation / health check */
    TEE_SVC_TEST        = 1,    /* Test service (increment counter) */
    TEE_SVC_GET_INFO    = 2,    /* Get TEE version/capabilities */
    /* Crypto services (future) */
    TEE_SVC_SIGN        = 10,   /* DS signing */
    TEE_SVC_DECRYPT     = 11,   /* OAEP decryption */
    TEE_SVC_HASH        = 12,   /* SHA256 hashing */
    TEE_SVC_MAX         = 32    /* Maximum service ID */
} tee_service_id_t;

/* ============================================================================
 * Error Codes
 * ============================================================================ */

typedef enum {
    TEE_OK              =  0,   /* Success */
    TEE_ERR_INVALID_SVC = -1,   /* Invalid service ID */
    TEE_ERR_INVALID_ARG = -2,   /* Invalid argument */
    TEE_ERR_CRYPTO      = -3,   /* Cryptographic operation failed */
    TEE_ERR_KEY_FAIL    = -4,   /* Key operation failed */
    TEE_ERR_BUSY        = -5,   /* TEE is busy */
    TEE_ERR_NO_MEM      = -6,   /* Out of memory */
    TEE_ERR_NOT_INIT    = -7,   /* TEE not initialized */
    TEE_ERR_UNAUTHORIZED= -9,   /* Unauthorized operation */
} tee_error_t;

/* ============================================================================
 * API Structure (128 bytes, at 0x600FF000)
 * ============================================================================ */

#define TEE_API_MAGIC       0x54454521  /* "TEE!" */
#define TEE_API_VERSION     2

typedef struct {
    /* Header (16 bytes) */
    uint32_t magic;                         /* TEE_API_MAGIC */
    uint32_t version;                       /* TEE_API_VERSION */
    uint32_t flags;                         /* Feature/status flags */
    uint32_t capabilities;                  /* Supported services bitmask */

    /* Call state (16 bytes) */
    volatile uint32_t service_id;           /* Service to invoke */
    volatile int32_t  result;               /* Return value from handler */
    volatile uint32_t call_count;           /* Incremented on each call */
    volatile uint32_t return_addr;          /* App return address */

    /* Parameters (32 bytes) */
    volatile uint32_t param[8];             /* Generic parameters */

    /* Buffer info (16 bytes) */
    volatile uint32_t in_buf_addr;          /* Input buffer address */
    volatile uint32_t in_buf_len;           /* Input buffer length */
    volatile uint32_t out_buf_addr;         /* Output buffer address */
    volatile uint32_t out_buf_len;          /* Output buffer length */

    /* Debug/state (16 bytes) */
    volatile uint32_t debug_stage;          /* Debug stage counter */
    volatile uint32_t last_error;           /* Last error code */
    volatile uint32_t core_id;              /* Core that made the call */
    volatile uint32_t reserved_dbg;

    /* Reserved (32 bytes) */
    uint32_t reserved[8];
} __attribute__((packed, aligned(4))) tee_api_t;

_Static_assert(sizeof(tee_api_t) == 128, "tee_api_t must be 128 bytes");

/* Access macro */
#define TEE_API() ((volatile tee_api_t*)TEE_API_ADDR)

#ifdef __cplusplus
}
#endif

#endif /* TEE_API_H */
