/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_config.h
 * @brief TEE configuration - memory addresses, API structures, service definitions
 *
 * This header is shared between the bootloader TEE component and the app client.
 * Keep structures and definitions in sync.
 */

#ifndef TEE_CONFIG_H
#define TEE_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Memory Layout
 * ============================================================================
 *
 * RTC FAST Memory: 0x600FE000 - 0x60100000 (8KB total)
 *
 * 0x600FE000-0x600FF000 (4KB): TEE Private (World 0 only)
 *   0x600FE000-0x600FE080: [reserved]
 *   0x600FE080: TEE_EXIT_STUB - W0->W1 exit stub for bootloader
 *   0x600FE100: TEE_ENTRY_STUB - W1->W0 entry (WCL monitored)
 *   0x600FE300-0x600FEF00: C handlers (linked at this address)
 *   0x600FEF00-0x600FF000: State variables, stack
 *
 * 0x600FF000-0x60100000 (4KB): Shared Memory (both worlds)
 *   0x600FF000: tee_api_t structure (128 bytes)
 *   0x600FF080: TEE_RETURN_TRAMPOLINE - W0->W1 return (must be in shared so W1 can execute)
 *   0x600FF0C0-0x60100000: Request/response buffers
 */

/* Fixed memory addresses */
#define TEE_PRIVATE_BASE        0x600FE000
#define TEE_PRIVATE_SIZE        0x1000      /* 4KB */

#define TEE_EXIT_STUB           0x600FE080  /* W0->W1 exit stub */
#define TEE_ENTRY_STUB          0x600FE100  /* W1->W0 entry (WCL monitored) */
#define TEE_RETURN_TRAMPOLINE   0x600FF080  /* W0->W1 return trampoline (in shared memory so W1 can execute) */

#define TEE_HANDLERS_ADDR       0x600FE300  /* C handlers copied here (PIC) */
#define TEE_HANDLERS_SIZE       0x500       /* 1.25KB max for handlers */

#define TEE_STATE_ADDR          0x600FEF00  /* TEE state variables */
#define TEE_STACK_TOP           0x600FEF00  /* Stack grows down from state */

#define TEE_API_ADDR            0x600FF000  /* Shared API structure */
#define TEE_SHARED_SIZE         0x1000      /* 4KB */
#define TEE_BUFFER_ADDR         0x600FF0C0  /* Start of buffer area (after trampoline) */
#define TEE_BUFFER_SIZE         0x0F80      /* ~4KB for buffers */

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

/* ============================================================================
 * TEE Private State Structure (at 0x600FEF00, World 0 only)
 * ============================================================================ */

typedef struct {
    uint8_t wcl_msg_seq_core0;              /* WCL message sequence for Core 0 */
    uint8_t wcl_msg_seq_core1;              /* WCL message sequence for Core 1 */
    uint8_t initialized;                    /* TEE initialized flag */
    uint8_t attestation_level;              /* Current attestation level */
    uint32_t reserved[15];                  /* Padding to 64 bytes */
} __attribute__((packed, aligned(4))) tee_state_t;

_Static_assert(sizeof(tee_state_t) == 64, "tee_state_t must be 64 bytes");

#define TEE_STATE() ((volatile tee_state_t*)TEE_STATE_ADDR)

/* ============================================================================
 * Handler Function Type
 * ============================================================================ */

typedef int32_t (*tee_handler_fn)(volatile tee_api_t* api);

/* ============================================================================
 * API Struct Field Offsets (for assembly)
 * ============================================================================ */

#define API_MAGIC_OFF           0
#define API_VERSION_OFF         4
#define API_FLAGS_OFF           8
#define API_CAPABILITIES_OFF    12
#define API_SERVICE_ID_OFF      16
#define API_RESULT_OFF          20
#define API_CALL_COUNT_OFF      24
#define API_RETURN_ADDR_OFF     28
#define API_PARAM_OFF           32
#define API_IN_BUF_ADDR_OFF     64
#define API_IN_BUF_LEN_OFF      68
#define API_OUT_BUF_ADDR_OFF    72
#define API_OUT_BUF_LEN_OFF     76
#define API_DEBUG_STAGE_OFF     80
#define API_LAST_ERROR_OFF      84
#define API_CORE_ID_OFF         88

/* ============================================================================
 * WCL Register Offsets (for assembly)
 * ============================================================================
 * Core 0 base: 0x600D0000
 * Core 1 base: 0x600D0400 (offset +0x400)
 */

#define WCL_BASE_CORE0          0x600D0000
#define WCL_BASE_CORE1          0x600D0400
#define WCL_CORE_OFFSET         0x400

#define WCL_WORLD_TRIGGER_OFF   0x140
#define WCL_WORLD_PREPARE_OFF   0x144
#define WCL_WORLD_UPDATE_OFF    0x148
#define WCL_WORLD_IRAM0_OFF     0x150

/* ============================================================================
 * Function Declarations (Bootloader)
 * ============================================================================ */

/* tee_wcl.c - World Controller configuration (DO NOT MODIFY) */
void tee_wcl_configure(uint32_t core, uint32_t address);
void tee_wcl_setup_world_return(uint32_t return_addr, uint32_t core);
void tee_wcl_clear_write_buffer(uint32_t core);

/* tee_pms.c - Permission Management System */
void tee_configure_pms(void);

/* tee_handlers.c - C handlers */
int32_t tee_dispatch(uint32_t service_id, volatile tee_api_t* api);
int32_t tee_handle_nop(volatile tee_api_t* api);
int32_t tee_handle_test(volatile tee_api_t* api);
int32_t tee_handle_get_info(volatile tee_api_t* api);

/* Assembly stubs - symbols for copying */
void tee_entry_stub_start(void);
extern void tee_entry_stub_end(void);

void tee_exit_stub_start(void);
extern void tee_exit_stub_end(void);

void tee_return_trampoline_start(void);
extern void tee_return_trampoline_end(void);

/* Handler section symbols */
extern char _tee_handlers_start[];
extern char _tee_handlers_end[];

#ifdef __cplusplus
}
#endif

#endif /* TEE_CONFIG_H */
