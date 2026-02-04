/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_entry_stub.S
 * @brief TEE entry stub - calls C dispatcher for handler logic
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
 *   3. Set up stack pointer
 *   4. Build API address, read service_id
 *   5. Build dispatcher address (TEE_HANDLERS_ADDR = 0x600FE300)
 *   6. CALLX0 to tee_dispatch(service_id, api)
 *   7. Store result to api->result
 *   8. Configure WCL for W0->W1 return
 *   9. Jump to trampoline -> WCL switches to W1 -> returns to app
 *
 * Register Allocation:
 *   a0  - Return address (used by CALLX0)
 *   a1  - Stack pointer (TEE_STACK_TOP = 0x600FEF00)
 *   a2  - Arg 1: service_id / Return value from dispatcher
 *   a3  - Arg 2: api pointer / Core ID after rebuild
 *   a8  - TEE_API_ADDR (0x600FF000), rebuilt after call
 *   a9  - Scratch register
 *   a10 - WCL per-core base, built after call
 *   a11 - Scratch for address building
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
#define API_RETURN_ADDR_OFF     28

/*
 * WCL register offsets from per-core base
 */
#define WCL_WORLD_TRIGGER_OFF   0x140
#define WCL_WORLD_PREPARE_OFF   0x144
#define WCL_WORLD_UPDATE_OFF    0x148

tee_entry_stub_start:
    /* ========================================================================
     * STAGE 1: World Switch Completion
     * ======================================================================== */
    nop.n                           /* Required by TRM 16.5.4.1 */
    nop.n

    /* ========================================================================
     * STAGE 2: Set Up Stack Pointer (0x600FEF00)
     * ======================================================================== */
    movi    a1, 0x600               /* a1 = 0x600 */
    slli    a1, a1, 20              /* a1 = 0x60000000 */
    movi    a9, 0xFE                /* a9 = 0xFE */
    slli    a9, a9, 12              /* a9 = 0x000FE000 */
    or      a1, a1, a9              /* a1 = 0x600FE000 */
    movi    a9, 0xF00               /* a9 = 0xF00 */
    or      a1, a1, a9              /* a1 = 0x600FEF00 (stack pointer) */

    /* ========================================================================
     * STAGE 3: Build TEE_API_ADDR (0x600FF000) and Read Service ID
     * ======================================================================== */
    movi    a8, 0x600               /* a8 = 0x600 */
    slli    a8, a8, 20              /* a8 = 0x60000000 */
    movi    a9, 0xFF                /* a9 = 0xFF */
    slli    a9, a9, 12              /* a9 = 0x000FF000 */
    or      a8, a8, a9              /* a8 = 0x600FF000 (API address) */

    memw
    l32i    a2, a8, API_SERVICE_ID_OFF   /* a2 = service_id (arg 1) */
    mov     a3, a8                       /* a3 = api pointer (arg 2) */

    /* ========================================================================
     * STAGE 4: Build Dispatcher Address (TEE_HANDLERS_ADDR = 0x600FE300)
     * ======================================================================== */
    movi    a9, 0x600               /* a9 = 0x600 */
    slli    a9, a9, 20              /* a9 = 0x60000000 */
    movi    a10, 0xFE               /* a10 = 0xFE */
    slli    a10, a10, 12            /* a10 = 0x000FE000 */
    or      a9, a9, a10             /* a9 = 0x600FE000 */
    movi    a10, 0x300              /* a10 = 0x300 */
    or      a9, a9, a10             /* a9 = 0x600FE300 (tee_dispatch) */

    /* ========================================================================
     * STAGE 5: Call C Dispatcher
     * ======================================================================== */
    /* a2 = service_id, a3 = api pointer, a9 = dispatcher address */
    callx0  a9                      /* Call tee_dispatch(service_id, api) */
                                    /* Result returned in a2 */

    /* ========================================================================
     * STAGE 6: Rebuild Addresses (clobbered by C call)
     * ======================================================================== */
    /* Rebuild API address (0x600FF000) */
    movi    a8, 0x600
    slli    a8, a8, 20
    movi    a9, 0xFF
    slli    a9, a9, 12
    or      a8, a8, a9              /* a8 = 0x600FF000 */

    /* Store result from C dispatcher */
    s32i    a2, a8, API_RESULT_OFF  /* api->result = return value */

    /* Get core ID for WCL */
    rsr.prid a3
    extui   a3, a3, 13, 1           /* a3 = core ID (0 or 1) */

    /* Build WCL per-core base (0x600D0000 + core * 0x400) */
    movi    a10, 0x600
    slli    a10, a10, 20
    movi    a11, 0x0D
    slli    a11, a11, 16
    or      a10, a10, a11           /* a10 = 0x600D0000 (Core 0 base) */
    slli    a11, a3, 10             /* a11 = core_id * 0x400 */
    add     a10, a10, a11           /* a10 = per-core WCL base */

    /* ========================================================================
     * STAGE 7: Setup WCL Return (W0 -> W1)
     * ======================================================================== */
    memw

    /* Load return address from API */
    l32i    a2, a8, API_RETURN_ADDR_OFF

    /* Configure WORLD_PREPARE = 0x2 (World 1 / REE) */
    movi    a9, 2
    memw
    s32i    a9, a10, WCL_WORLD_PREPARE_OFF

    /* Build trampoline address (0x600FF080 - in shared memory so W1 can execute) */
    movi    a9, 0x600
    slli    a9, a9, 20
    movi    a11, 0xFF
    slli    a11, a11, 12
    or      a9, a9, a11             /* a9 = 0x600FF000 */
    movi    a11, 0x080
    or      a9, a9, a11             /* a9 = 0x600FF080 (trampoline) */

    /* Set WORLD_TRIGGER = trampoline address */
    memw
    s32i    a9, a10, WCL_WORLD_TRIGGER_OFF

    /* Set WORLD_UPDATE = 1 to commit changes */
    movi    a11, 1
    memw
    s32i    a11, a10, WCL_WORLD_UPDATE_OFF

    /* Memory barrier before jump */
    memw

    /* ========================================================================
     * STAGE 8: Jump to Trampoline
     * ======================================================================== */
    /* a9 already contains trampoline address (0x600FF080) */
    /* a2 contains return address for trampoline to use */
    jx      a9

    .size   tee_entry_stub_start, . - tee_entry_stub_start

/*
 * End marker for size calculation during copy
 */
    .global tee_entry_stub_end
tee_entry_stub_end:
