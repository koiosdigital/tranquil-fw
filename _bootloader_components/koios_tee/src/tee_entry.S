/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_entry.S
 * @brief Minimal TEE entry stub
 *
 * This is the entry point for TEE calls. WCL monitors this address
 * (0x600FE300) for W1->W0 transitions.
 *
 * Flow:
 *   1. App (W1) jumps here -> WCL switches to W0
 *   2. NOPs (TRM 16.5.4.1 requirement)
 *   3. Stack setup
 *   4. Call tee_handler_main (C function that does dispatch + WCL setup)
 *   5. Jump to trampoline (address returned in a2)
 */

    .section .text.tee_entry
    .global tee_entry
    .type   tee_entry, @function
    .align  4

tee_entry:
    /* TRM 16.5.4.1 - Required after world switch */
    nop.n
    nop.n

    /* Set up TEE stack pointer (0x600FEF00) */
    movi    a1, 0x600
    slli    a1, a1, 20              /* a1 = 0x60000000 */
    movi    a9, 0xFE
    slli    a9, a9, 12              /* a9 = 0x000FE000 */
    or      a1, a1, a9              /* a1 = 0x600FE000 */
    movi    a9, 0xF                 /* Build 0xF00 without literal */
    slli    a9, a9, 8               /* a9 = 0xF00 */
    or      a1, a1, a9              /* a1 = 0x600FEF00 (stack) */

    /* Build API address (0x600FF000) */
    movi    a8, 0x600
    slli    a8, a8, 20              /* a8 = 0x60000000 */
    movi    a9, 0xFF
    slli    a9, a9, 12              /* a9 = 0x000FF000 */
    or      a8, a8, a9              /* a8 = 0x600FF000 (API) */

    /* Load args for C handler */
    memw
    l32i    a2, a8, 16              /* a2 = api->service_id (offset 16) */
    mov     a3, a8                  /* a3 = api pointer */

    /* Call C handler (handles dispatch + WCL setup, returns trampoline addr) */
    call0   tee_handler_main

    /* a2 now contains trampoline address - jump to it */
    jx      a2

    .size   tee_entry, . - tee_entry
