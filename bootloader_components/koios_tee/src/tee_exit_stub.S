/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_exit_stub.S
 * @brief W0→W1 exit stub for bootloader world transition
 *
 * This stub is copied to TEE_EXIT_TO_W1 (0x600FE080) in RTC FAST memory.
 * When executed with WCL configured for W0→W1 transition, it triggers
 * the world switch and jumps to the return address.
 *
 * Used by bootloader to exit secure world after PMS configuration.
 */

    .section .text
    .global tee_exit_stub
    .type   tee_exit_stub, @function
    .align  4

/**
 * tee_exit_stub - World 0 to World 1 exit point
 *
 * Entry (in W0):
 *   Return address stored at TEE_API_ADDR + 24 (return_addr field)
 *   WCL configured: WORLD_PREPARE=0x2, WORLD_TRIGGER=this address
 *
 * Exit:
 *   Jumps to return_addr in World 1 (non-secure)
 *
 * The caller must:
 *   1. Store return address in TEE_API()->return_addr
 *   2. Configure WCL: WORLD_PREPARE=0x2, WORLD_TRIGGER=TEE_EXIT_TO_W1, WORLD_UPDATE=1
 *   3. Jump here
 */
#define TEE_API_ADDR        0x600FF000
#define RETURN_ADDR_OFFSET  24

tee_exit_stub:
    /* Two NOPs required per TRM 16.5.4.1 for world switch completion */
    nop.n
    nop.n

    /*
     * Load return address from shared memory (TEE_API_ADDR + 24)
     * Build address without literal pool (position-independent):
     * 0x600FF000 = 0x600 << 20 | 0xFF << 12 | 0x000
     */
    movi    a2, 0x600       /* a2 = 0x600 */
    slli    a2, a2, 20      /* a2 = 0x60000000 */
    movi    a3, 0xFF        /* a3 = 0xFF */
    slli    a3, a3, 12      /* a3 = 0xFF000 */
    or      a2, a2, a3      /* a2 = 0x600FF000 */
    l32i    a2, a2, RETURN_ADDR_OFFSET  /* a2 = *(0x600FF000 + 24) */

    /* Memory barrier before jump */
    memw

    /* Jump to return address - now in W1 */
    jx      a2

    .size   tee_exit_stub, . - tee_exit_stub

/**
 * End marker for size calculation during copy
 */
    .global tee_exit_stub_end
tee_exit_stub_end:
