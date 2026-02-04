/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

 /**
  * @file tee_config.h
  * @brief TEE configuration - memory addresses and World Controller registers
  */

#ifndef TEE_CONFIG_H
#define TEE_CONFIG_H

#include <stdint.h>


  /* ============================================================================
   * Memory Layout
   * ============================================================================
   *
   * RTC FAST Memory: 0x600FE000 - 0x60100000 (8KB total)
   *
   * 0x600FE000-0x600FF000 (4KB): TEE Private (World 0 only)
   *   - 0x600FE000: Handler code copied here
   *   - 0x600FE100: TEE_ENTRY_TEST - WCL monitored entry point
   *   - 0x600FE200: TEE_ENTRY_TRAMPOLINE - return trampoline
   *   - 0x600FEF00: TEE state variables
   *   - 0x600FEF80: Stack (128 bytes, grows down)
   *
   * 0x600FF000-0x60100000 (4KB): Shared Memory (both worlds)
   *   - tee_api_t structure at 0x600FF000
   */

#define TEE_PRIVATE_BASE     0x600FE000
#define TEE_PRIVATE_SIZE     0x1000      /* 4KB */
#define TEE_API_ADDR         0x600FF000
#define TEE_SHARED_SIZE      0x1000      /* 4KB */

   /* Fixed entry point addresses in TEE private memory */
#define TEE_EXIT_TO_W1       0x600FE080  /* W0→W1 exit stub for bootloader */
#define TEE_ENTRY_TEST       0x600FE100  /* W1→W0 entry for app TEE calls */
#define TEE_ENTRY_TRAMPOLINE 0x600FE200  /* W0→W1 return trampoline */

/* TEE state area */
#define TEE_STATE_ADDR       0x600FEF00

/* ============================================================================
 * TEE Private State Structure (World 0 only)
 * ============================================================================ */

typedef struct {
    uint8_t wcl_msg_seq_core0;           /* WCL message sequence address for Core 0 */
    uint8_t wcl_msg_seq_core1;           /* WCL message sequence address for Core 1 */
    uint8_t reserved[62];                /* Padding to 64 bytes */
} __attribute__((packed, aligned(4))) tee_state_t;

#define TEE_STATE() ((volatile tee_state_t*)TEE_STATE_ADDR)

/* ============================================================================
 * TEE API Structure (shared memory)
 * ============================================================================ */

#define TEE_API_MAGIC        0x54454521  /* "TEE!" */
#define TEE_API_VERSION      1

typedef struct {
    uint32_t magic;                      /* TEE_API_MAGIC */
    uint32_t version;                    /* TEE_API_VERSION */
    volatile uint32_t call_count;        /* Incremented by TEE on each call */
    volatile int32_t last_result;        /* Return value from last call */
    volatile uint32_t request;           /* Input from app */
    volatile uint32_t response;          /* Output from TEE */
    volatile uint32_t return_addr;       /* Return address for world switch */
    uint32_t reserved[9];                /* Padding to 64 bytes */
} __attribute__((packed, aligned(4))) tee_api_t;

#define TEE_API() ((volatile tee_api_t*)TEE_API_ADDR)

/* ============================================================================
 * Function Declarations
 * ============================================================================ */

 /* tee_wcl.c */
void tee_wcl_configure(uint32_t core, uint32_t address);
void tee_wcl_setup_world_return(uint32_t return_addr, uint32_t core);
void tee_wcl_clear_write_buffer(uint32_t core);

/* tee_handlers.c */
void tee_test_handler(void);

/* tee_pms.c */
void tee_configure_pms(void);

/* tee_exit_stub.S */
void tee_exit_stub(void);
extern void tee_exit_stub_end(void);

/* tee_trampoline.S */
void tee_return_trampoline(void);

#endif /* TEE_CONFIG_H */
