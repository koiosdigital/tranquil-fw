/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

 /**
  * @file tee_api.h
  * @brief TEE API - Shared memory structure for app<->TEE communication
  *
  * Phase 1: World Controller Test
  *
  * This header defines the shared memory structure used for communication
  * between the app (World 1) and TEE handlers (World 0).
  *
  * Memory Layout:
  *   TEE private:  0x600FE000-0x600FF000 (4KB) - World 0 only
  *   Shared mem:   0x600FF000-0x60100000 (4KB) - Both worlds
  */

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

    /* ============================================================================
     * TEE API Magic and Version
     * ============================================================================ */

#define TEE_API_MAGIC       0x54454521  /* "TEE!" */
#define TEE_API_VERSION     1

     /* ============================================================================
      * Error Codes
      * ============================================================================ */

#define TEE_OK              0
#define TEE_ERR_INVALID    -1
#define TEE_ERR_CRYPTO     -2
#define TEE_ERR_NOT_INIT   -3
#define TEE_ERR_KEY_FAIL   -4
#define TEE_ERR_UNSIGNED   -5
#define TEE_ERR_NO_API     -6
#define TEE_ERR_WORLD      -7   /* World switch error */

      /* ============================================================================
       * Trust Level Constants
       * ============================================================================ */

#define APP_TRUST_SIGNED    0x5349474E  /* "SIGN" */
#define APP_TRUST_UNSIGNED  0x554E5349  /* "UNSI" */

       /* ============================================================================
        * Memory Addresses
        * ============================================================================ */

#define TEE_PRIVATE_BASE    0x600FE000  /* TEE private memory */
#define TEE_PRIVATE_SIZE    0x1000      /* 4KB */
#define TEE_API_ADDR        0x600FF000  /* Shared memory / API table */
#define TEE_SHARED_SIZE     0x1000      /* 4KB */

        /* TEE entry points (fixed addresses in TEE private memory) */
#define TEE_ENTRY_TEST       0x600FE100
#define TEE_ENTRY_TRAMPOLINE 0x600FE200

/* ============================================================================
 * World Controller Registers (for app-side W0->W1 switch)
 * ============================================================================ */

#define WORLD_TEE            0x01   /* World 0 - Secure */
#define WORLD_REE            0x02   /* World 1 - Non-secure */

 /* ============================================================================
  * TEE API Structure (shared memory)
  * ============================================================================
  *
  * Phase 1: Simple structure for testing world switching.
  * Future phases will add function pointers for crypto services.
  */
    typedef struct tee_api {
        /* Header */
        uint32_t magic;                      /* TEE_API_MAGIC */
        uint32_t version;                    /* TEE_API_VERSION */
        volatile uint32_t call_count;        /* Incremented by TEE on each call */
        volatile int32_t last_result;        /* Return value from last call */

        /* Request/Response */
        volatile uint32_t request;           /* Input from app */
        volatile uint32_t response;          /* Output from TEE */
        volatile uint32_t return_addr;       /* Return address for world switch */
        volatile uint32_t world_switched;    /* Set to 1 after W0->W1 switch done */

        /* Trust/Status (Phase 2+) */
        volatile uint32_t app_trust_level;   /* APP_TRUST_SIGNED or APP_TRUST_UNSIGNED */
        volatile uint32_t provisioned;       /* Non-zero if device is provisioned */

        /* Debug counter - tracks execution progress through handler */
        volatile uint32_t debug_stage;       /* Incremented at each stage */

        /* Padding to 64 bytes */
        uint32_t reserved[5];

    } __attribute__((packed, aligned(4))) tee_api_t;

    /* Compile-time check */
    _Static_assert(sizeof(tee_api_t) == 64, "tee_api_t should be 64 bytes");

    /* ============================================================================
     * API Access Macros
     * ============================================================================ */

     /**
      * @brief Get pointer to TEE API table
      */
#define TEE_API()   ((volatile tee_api_t *)TEE_API_ADDR)

      /**
       * @brief Check if TEE API is valid
       */
    static inline int tee_api_valid(void)
    {
        volatile tee_api_t* api = TEE_API();
        return (api->magic == TEE_API_MAGIC && api->version >= TEE_API_VERSION);
    }

    /**
     * @brief Direct register access (for world switching)
     */
#define REG(addr)  (*(volatile uint32_t*)(addr))

#ifdef __cplusplus
}
#endif
