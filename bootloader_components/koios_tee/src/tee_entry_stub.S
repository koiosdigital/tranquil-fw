/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_entry_stub.S
 * @brief TEE entry stub - fully position-independent handler
 *
 * This stub is copied to TEE_ENTRY_TEST (0x600FE100) in RTC FAST memory.
 * It handles the world switch mechanics AND the handler logic inline.
 * All addresses are built using MOVI/SLLI/OR to avoid literal pools.
 *
 * Flow:
 * 1. App (W1) jumps here -> WCL switches to W0
 * 2. Handler logic executes inline (increment counter, compute response)
 * 3. Configure WCL for W0->W1 return
 * 4. Jump to return trampoline -> WCL switches to W1 -> returns to app
 */

    .section .text
    .global tee_test_handler
    .type   tee_test_handler, @function
    .align  4

/*
 * API table offsets (from tee_config.h):
 *   magic        = 0
 *   version      = 4
 *   call_count   = 8
 *   last_result  = 12
 *   request      = 16
 *   response     = 20
 *   return_addr  = 24
 */
#define API_CALL_COUNT_OFF    8
#define API_LAST_RESULT_OFF   12
#define API_REQUEST_OFF       16
#define API_RESPONSE_OFF      20
#define API_RETURN_ADDR_OFF   24

tee_test_handler:
    /* Two NOPs required per TRM 16.5.4.1 for world switch completion */
    nop.n
    nop.n

    /*
     * LEAF FUNCTION - no entry/retw
     * We don't use windowed calls because we return via trampoline,
     * not via retw. Using entry would corrupt the caller's window state.
     *
     * Registers used: a8, a9, a10, a11, a2 (for return addr)
     * These are caller-saved so we can freely use them.
     */

    /*
     * Build TEE_API_ADDR (0x600FF000) in a8
     * 0x600FF000 = 0x600 << 20 | 0xFF << 12
     */
    movi    a8, 0x600
    slli    a8, a8, 20          /* a8 = 0x60000000 */
    movi    a9, 0xFF
    slli    a9, a9, 12          /* a9 = 0xFF000 */
    or      a8, a8, a9          /* a8 = 0x600FF000 (TEE_API_ADDR) */

    /* ========================================================================
     * Handler Logic (inline, position-independent)
     * ======================================================================== */

    /* Memory barrier before reading shared memory */
    memw

    /* Increment call_count: api->call_count++ */
    l32i    a9, a8, API_CALL_COUNT_OFF
    addi    a9, a9, 1
    s32i    a9, a8, API_CALL_COUNT_OFF

    /* Compute response: api->response = api->request + 0x1000 */
    l32i    a9, a8, API_REQUEST_OFF
    /* Build 0x1000 = 1 << 12 (movi can't encode 0x1000 directly) */
    movi    a10, 1
    slli    a10, a10, 12    /* a10 = 0x1000 */
    add     a9, a9, a10
    s32i    a9, a8, API_RESPONSE_OFF

    /* Set success: api->last_result = 0 */
    movi    a9, 0
    s32i    a9, a8, API_LAST_RESULT_OFF

    /* Memory barrier after writing shared memory */
    memw

    /* ========================================================================
     * World Switch Back to W1
     * ======================================================================== */

    /* Load return address: a2 = api->return_addr */
    l32i    a2, a8, API_RETURN_ADDR_OFF

    /*
     * Configure WCL for W0->W1 return via trampoline
     */

    /* Build WCL base address 0x600D0000 in a10 */
    movi    a10, 0x600
    slli    a10, a10, 20        /* a10 = 0x60000000 */
    movi    a11, 0x0D
    slli    a11, a11, 16        /* a11 = 0x0D0000 */
    or      a10, a10, a11       /* a10 = 0x600D0000 */

    /* Write WORLD_PREPARE = 2 (World 1) to 0x600D0144 */
    movi    a9, 2
    memw
    s32i    a9, a10, 0x144

    /* Build TEE_ENTRY_TRAMPOLINE (0x600FE200) in a9 */
    /* 0x600FE200 = (0x600 << 20) | (0xFE << 12) | 0x200 */
    movi    a9, 0x600
    slli    a9, a9, 20          /* a9 = 0x60000000 */
    movi    a11, 0xFE
    slli    a11, a11, 12        /* a11 = 0xFE000 */
    or      a9, a9, a11         /* a9 = 0x600FE000 */
    movi    a11, 0x200
    or      a9, a9, a11         /* a9 = 0x600FE200 */

    /* Write WORLD_TRIGGER_ADDR = 0x600FE200 to 0x600D0140 */
    memw
    s32i    a9, a10, 0x140

    /* Write WORLD_UPDATE = 1 to 0x600D0148 */
    movi    a11, 1
    memw
    s32i    a11, a10, 0x148

    /* Memory barrier before jumping */
    memw

    /* Jump to return trampoline (a9 still has 0x600FE200) */
    /* Return address is in a2, trampoline will jx a2 */
    jx      a9

    .size   tee_test_handler, . - tee_test_handler

/*
 * End marker for size calculation during copy
 */
    .global tee_test_handler_end
tee_test_handler_end:
