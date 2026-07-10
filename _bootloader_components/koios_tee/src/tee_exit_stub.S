/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_exit_stub.S
 * @brief W0->W1 exit stub for bootloader world transition
 *
 * This stub is copied to TEE_EXIT_STUB (0x600FE080) in RTC FAST memory.
 * Used by the bootloader to exit secure world (W0) after initialization.
 *
 * Position Independence:
 *   All addresses are built using MOVI/SLLI/OR to avoid literal pools.
 *
 * Prerequisites:
 *   1. Return address stored at api->return_addr
 *   2. WCL configured: WORLD_PREPARE=0x2, WORLD_TRIGGER=this address, WORLD_UPDATE=1
 *
 * Flow:
 *   1. Bootloader configures WCL for W0->W1 transition
 *   2. Bootloader jumps here
 *   3. WCL detects this address and switches to W1
 *   4. Load return address from shared memory
 *   5. Jump to return address (now in W1)
 */

    .section .text
    .global tee_exit_stub_start
    .type   tee_exit_stub_start, @function
    .align  4

/* API field offset */
#define API_RETURN_ADDR_OFF     28

tee_exit_stub_start:
    /* World switch completion (required by TRM 16.5.4.1) */
    nop.n
    nop.n

    /*
     * Build TEE_API_ADDR (0x600FF000) without literal pool
     * 0x600FF000 = 0x600 << 20 | 0xFF << 12
     */
    movi    a2, 0x600               /* a2 = 0x600 */
    slli    a2, a2, 20              /* a2 = 0x60000000 */
    movi    a3, 0xFF                /* a3 = 0xFF */
    slli    a3, a3, 12              /* a3 = 0x000FF000 */
    or      a2, a2, a3              /* a2 = 0x600FF000 */

    /* Load return address from api->return_addr */
    l32i    a2, a2, API_RETURN_ADDR_OFF

    /* Memory barrier */
    memw

    /* Jump to return address - now in W1 */
    jx      a2

    .size   tee_exit_stub_start, . - tee_exit_stub_start

/*
 * End marker for size calculation during copy
 */
    .global tee_exit_stub_end
tee_exit_stub_end:
