/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_entry_stub.S
 * @brief TEE entry stub - minimal position-independent handler
 *
 * This stub is copied to TEE_ENTRY_STUB (0x600FE100) in RTC FAST memory.
 * When the app (World 1) executes this address, WCL switches to World 0.
 *
 * Position Independence:
 *   All addresses are built using MOVI/SLLI/OR to avoid literal pools.
 *   This allows the code to be copied to any location.
 *
 * Flow:
 *   1. App (W1) jumps here -> WCL switches to W0
 *   2. Entry: 2x NOP (TRM requirement for world switch completion)
 *   3. Detect core ID from PRID register
 *   4. Build API and WCL addresses
 *   5. Execute handler logic (inline for position-independence)
 *   6. Configure WCL for W0->W1 return
 *   7. Jump to trampoline -> WCL switches to W1 -> returns to app
 *
 * Register Allocation:
 *   a2  - Return address (from api->return_addr)
 *   a3  - Core ID (0 or 1)
 *   a4  - Service ID
 *   a8  - TEE_API_ADDR (0x600FF000), preserved
 *   a9  - Scratch register
 *   a10 - WCL per-core base, preserved
 *   a11 - Scratch for address building
 *   a12 - World status / scratch
 *   a13 - Debug stage counter (check in crash dump)
 */

    .section .text
    .global tee_entry_stub_start
    .type   tee_entry_stub_start, @function
    .align  4

/*
 * API field offsets (from tee_config.h, 128-byte struct)
 */
#define API_SERVICE_ID_OFF      16
#define API_RESULT_OFF          20
#define API_CALL_COUNT_OFF      24
#define API_RETURN_ADDR_OFF     28
#define API_DEBUG_STAGE_OFF     80
#define API_CORE_ID_OFF         88

/*
 * WCL register offsets from per-core base
 */
#define WCL_WORLD_TRIGGER_OFF   0x140
#define WCL_WORLD_PREPARE_OFF   0x144
#define WCL_WORLD_UPDATE_OFF    0x148
#define WCL_WORLD_IRAM0_OFF     0x150

/*
 * Service IDs
 */
#define TEE_SVC_NOP             0
#define TEE_SVC_TEST            1

tee_entry_stub_start:
    /* ========================================================================
     * STAGE 1: World Switch Completion
     * ======================================================================== */
    nop.n                           /* Required by TRM 16.5.4.1 */
    nop.n

    movi    a13, 1                  /* Debug: Stage 1 - Entry */

    /* ========================================================================
     * STAGE 2: Core Detection
     * ======================================================================== */
    rsr.prid a3                     /* Read processor ID */
    extui   a3, a3, 13, 1           /* Extract core ID (bit 13) */

    movi    a13, 2                  /* Debug: Stage 2 - Core detected */

    /* ========================================================================
     * STAGE 3: Build TEE_API_ADDR (0x600FF000)
     * ======================================================================== */
    movi    a8, 0x600               /* a8 = 0x600 */
    slli    a8, a8, 20              /* a8 = 0x60000000 */
    movi    a9, 0xFF                /* a9 = 0xFF */
    slli    a9, a9, 12              /* a9 = 0x000FF000 */
    or      a8, a8, a9              /* a8 = 0x600FF000 (API address) */

    movi    a13, 3                  /* Debug: Stage 3 - API addr built */

    /* ========================================================================
     * STAGE 4: Build WCL Per-Core Base Address
     * ======================================================================== */
    movi    a10, 0x600              /* a10 = 0x600 */
    slli    a10, a10, 20            /* a10 = 0x60000000 */
    movi    a11, 0x0D               /* a11 = 0x0D */
    slli    a11, a11, 16            /* a11 = 0x000D0000 */
    or      a10, a10, a11           /* a10 = 0x600D0000 (Core 0 base) */
    slli    a11, a3, 10             /* a11 = core_id * 0x400 */
    add     a10, a10, a11           /* a10 = per-core WCL base */

    movi    a13, 4                  /* Debug: Stage 4 - WCL addr built */

    /* ========================================================================
     * STAGE 5: Read Service ID and Verify World
     * ======================================================================== */
    memw
    l32i    a4, a8, API_SERVICE_ID_OFF   /* a4 = service_id */

    /* Read world status for debug/verification */
    l32i    a12, a10, WCL_WORLD_IRAM0_OFF
    extui   a12, a12, 0, 1          /* a12 = secure bit (1=W0, 0=W1) */

    /* Store core ID in API for debug */
    s32i    a3, a8, API_CORE_ID_OFF

    movi    a13, 5                  /* Debug: Stage 5 - Service read */

    /* ========================================================================
     * STAGE 6: Handler Dispatch (inline for position-independence)
     * ======================================================================== */

    /* Check service ID */
    beqi    a4, TEE_SVC_NOP, .Lhandle_nop
    beqi    a4, TEE_SVC_TEST, .Lhandle_test

    /* Unknown service - return error */
    movi    a9, -1                  /* TEE_ERR_INVALID_SVC */
    s32i    a9, a8, API_RESULT_OFF
    j       .Lsetup_return

.Lhandle_nop:
    /* NOP handler: just return success */
    movi    a9, 0                   /* TEE_OK */
    s32i    a9, a8, API_RESULT_OFF
    j       .Lsetup_return

.Lhandle_test:
    /* TEST handler: increment call_count, return success */
    l32i    a9, a8, API_CALL_COUNT_OFF
    addi    a9, a9, 1
    s32i    a9, a8, API_CALL_COUNT_OFF

    movi    a9, 0                   /* TEE_OK */
    s32i    a9, a8, API_RESULT_OFF
    /* Fall through to setup_return */

    /* ========================================================================
     * STAGE 7: Setup WCL Return (W0 -> W1)
     * ======================================================================== */
.Lsetup_return:
    memw

    movi    a13, 7                  /* Debug: Stage 7 - Setup return */

    /* Load return address from API */
    l32i    a2, a8, API_RETURN_ADDR_OFF

    /* Configure WORLD_PREPARE = 0x2 (World 1 / REE) */
    movi    a9, 2
    memw
    s32i    a9, a10, WCL_WORLD_PREPARE_OFF

    movi    a13, 8                  /* Debug: Stage 8 - PREPARE set */

    /* Build trampoline address (0x600FE200) */
    movi    a9, 0x600               /* a9 = 0x600 */
    slli    a9, a9, 20              /* a9 = 0x60000000 */
    movi    a11, 0xFE               /* a11 = 0xFE */
    slli    a11, a11, 12            /* a11 = 0x000FE000 */
    or      a9, a9, a11             /* a9 = 0x600FE000 */
    movi    a11, 0x200              /* a11 = 0x200 */
    or      a9, a9, a11             /* a9 = 0x600FE200 (trampoline) */

    /* Set WORLD_TRIGGER = trampoline address */
    memw
    s32i    a9, a10, WCL_WORLD_TRIGGER_OFF

    movi    a13, 9                  /* Debug: Stage 9 - TRIGGER set */

    /* Set WORLD_UPDATE = 1 to commit changes */
    movi    a11, 1
    memw
    s32i    a11, a10, WCL_WORLD_UPDATE_OFF

    movi    a13, 10                 /* Debug: Stage 10 - UPDATE set */

    /* Store debug stage in API */
    s32i    a13, a8, API_DEBUG_STAGE_OFF

    /* Memory barrier before jump */
    memw

    movi    a13, 11                 /* Debug: Stage 11 - Jumping */

    /* ========================================================================
     * STAGE 8: Jump to Trampoline
     * ======================================================================== */
    /* a9 already contains trampoline address (0x600FE200) */
    /* a2 contains return address for trampoline to use */
    jx      a9

    .size   tee_entry_stub_start, . - tee_entry_stub_start

/*
 * End marker for size calculation during copy
 */
    .global tee_entry_stub_end
tee_entry_stub_end:
