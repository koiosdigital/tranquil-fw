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
 * 2. Detect core ID and verify secure world execution
 * 3. Handler logic executes inline (increment counter, compute response)
 * 4. Configure WCL for W0->W1 return (using correct core's registers)
 * 5. Jump to return trampoline -> WCL switches to W1 -> returns to app
 *
 * Register Allocation (fixed, no reuse of critical values):
 *   a2  - Return address (loaded from api->return_addr)
 *   a3  - Core ID (0 or 1), kept throughout
 *   a8  - TEE_API_ADDR (0x600FF000), kept throughout
 *   a9  - Temporary, freely reused
 *   a10 - WCL per-core base (0x600D0000 or 0x600D0400), kept throughout
 *   a11 - Temporary for address building
 *   a12 - World status from WORLD_IRAM0
 */

    .section .text
    .global tee_test_handler
    .type   tee_test_handler, @function
    .align  4

/*
 * API table offsets (from tee_api.h):
 *   magic        = 0
 *   version      = 4
 *   call_count   = 8
 *   last_result  = 12
 *   request      = 16
 *   response     = 20
 *   return_addr  = 24
 *   ...
 *   debug_stage  = 40
 */
#define API_CALL_COUNT_OFF    8
#define API_LAST_RESULT_OFF   12
#define API_REQUEST_OFF       16
#define API_RESPONSE_OFF      20
#define API_RETURN_ADDR_OFF   24
#define API_DEBUG_STAGE_OFF   40

/*
 * WCL register offsets from per-core base:
 *   Core 0 base: 0x600D0000
 *   Core 1 base: 0x600D0400 (offset +0x400 from Core 0)
 */
#define WCL_WORLD_TRIGGER_OFF 0x140
#define WCL_WORLD_PREPARE_OFF 0x144
#define WCL_WORLD_UPDATE_OFF  0x148
#define WCL_WORLD_IRAM0_OFF   0x150

tee_test_handler:
    /* Two NOPs required per TRM 16.5.4.1 for world switch completion */
    nop.n
    nop.n

    /*
     * LEAF FUNCTION - no entry/retw
     * DEBUG: A13 used as stage counter (check in crash dump)
     */

    movi    a13, 1              /* STAGE 1: Entry */

    /* Get core ID from PRID special register (bit 13 = core ID) */
    rsr.prid a3
    extui   a3, a3, 13, 1       /* a3 = core_id (0 or 1) */

    movi    a13, 2              /* STAGE 2: After core detect */

    /* Build TEE_API_ADDR (0x600FF000) in a8 */
    movi    a8, 0x600
    slli    a8, a8, 20
    movi    a9, 0xFF
    slli    a9, a9, 12
    or      a8, a8, a9          /* a8 = 0x600FF000 */

    movi    a13, 3              /* STAGE 3: After API addr */

    /* Build WCL Per-Core Base Address in a10 */
    movi    a10, 0x600
    slli    a10, a10, 20
    movi    a11, 0x0D
    slli    a11, a11, 16
    or      a10, a10, a11       /* a10 = 0x600D0000 */
    slli    a11, a3, 10         /* a11 = core_id * 0x400 */
    add     a10, a10, a11       /* a10 = per-core WCL base */

    movi    a13, 4              /* STAGE 4: After WCL addr */

    memw
    l32i    a12, a10, WCL_WORLD_IRAM0_OFF
    extui   a12, a12, 0, 1      /* a12 = secure bit */

    movi    a13, 5              /* STAGE 5: After world read */

    /* Handler logic */
    l32i    a9, a8, API_CALL_COUNT_OFF
    addi    a9, a9, 1
    s32i    a9, a8, API_CALL_COUNT_OFF

    l32i    a9, a8, API_REQUEST_OFF
    movi    a11, 1
    slli    a11, a11, 12
    add     a9, a9, a11
    add     a9, a9, a12
    s32i    a9, a8, API_RESPONSE_OFF

    movi    a9, 0
    s32i    a9, a8, API_LAST_RESULT_OFF
    memw

    movi    a13, 6              /* STAGE 6: After handler logic */

    /* Load return address */
    l32i    a2, a8, API_RETURN_ADDR_OFF

    movi    a13, 7              /* STAGE 7: After load ret addr */

    /* Configure WCL for W0->W1 return */
    movi    a9, 2
    memw
    s32i    a9, a10, WCL_WORLD_PREPARE_OFF

    movi    a13, 8              /* STAGE 8: After WORLD_PREPARE */

    /* Build trampoline address 0x600FE200 */
    movi    a9, 0x600
    slli    a9, a9, 20
    movi    a11, 0xFE
    slli    a11, a11, 12
    or      a9, a9, a11
    movi    a11, 0x200
    or      a9, a9, a11         /* a9 = 0x600FE200 */

    memw
    s32i    a9, a10, WCL_WORLD_TRIGGER_OFF

    movi    a13, 9              /* STAGE 9: After TRIGGER_ADDR */

    movi    a11, 1
    memw
    s32i    a11, a10, WCL_WORLD_UPDATE_OFF

    movi    a13, 10             /* STAGE 10: After WORLD_UPDATE */

    memw

    movi    a13, 11             /* STAGE 11: Before jx trampoline */

    /* Jump to trampoline */
    jx      a9

    .size   tee_test_handler, . - tee_test_handler

/*
 * End marker for size calculation during copy
 */
    .global tee_test_handler_end
tee_test_handler_end:
