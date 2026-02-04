/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_trampoline.S
 * @brief Return trampoline for World 0 -> World 1 transition
 *
 * This code is copied to TEE_RETURN_TRAMPOLINE (0x600FE200) in RTC FAST memory.
 * When executed, WCL detects the address and switches to World 1, then the
 * code jumps to the return address passed in a2.
 *
 * Position Independence:
 *   This code is minimal and uses only the a2 register passed from the handler.
 *
 * Prerequisites:
 *   1. a2 contains the return address
 *   2. WCL configured: WORLD_PREPARE=0x2, WORLD_TRIGGER=this address, WORLD_UPDATE=1
 *
 * Flow:
 *   1. Handler sets up WCL for W0->W1 transition
 *   2. Handler loads return address into a2
 *   3. Handler jumps here
 *   4. WCL detects this address and switches to W1
 *   5. Jump to a2 (return address) - now in W1
 */

    .section .text
    .global tee_return_trampoline_start
    .type   tee_return_trampoline_start, @function
    .align  4

/*
 * Debug: API debug_stage offset = 80 (0x50)
 */
#define API_DEBUG_STAGE_OFF     80

tee_return_trampoline_start:
    /* World switch completion (required by TRM 16.5.4.1) - MUST BE FIRST! */
    nop.n
    nop.n

    /* DEBUG: Build API address in a8 (0x600FF000) - AFTER NOPs! */
    movi    a8, 0x600
    slli    a8, a8, 20              /* a8 = 0x60000000 */
    movi    a9, 0xFF
    slli    a9, a9, 12              /* a9 = 0x000FF000 */
    or      a8, a8, a9              /* a8 = 0x600FF000 */

    /* DEBUG: Stage 0x100 = in trampoline after world switch */
    movi    a9, 0x100
    s32i    a9, a8, API_DEBUG_STAGE_OFF

    /* Memory barrier after world switch */
    memw

    /* DEBUG: Stage 0x101 = after memw, about to return */
    movi    a9, 0x101
    s32i    a9, a8, API_DEBUG_STAGE_OFF

    /* Sync instruction stream before jumping to flash */
    isync

    /* DEBUG: Stage 0x102 = after isync */
    movi    a9, 0x102
    s32i    a9, a8, API_DEBUG_STAGE_OFF

    /* Jump to return address (in a2) - now in World 1 */
    jx      a2

    .size   tee_return_trampoline_start, . - tee_return_trampoline_start

/*
 * End marker for size calculation during copy
 */
    .global tee_return_trampoline_end
tee_return_trampoline_end:
