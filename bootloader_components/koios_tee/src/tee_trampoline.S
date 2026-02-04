/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_trampoline.S
 * @brief Return trampoline for World 0 -> World 1 transition
 *
 * This code is copied to TEE_ENTRY_TRAMPOLINE (0x600FE200) in RTC FAST memory.
 * When executed, WCL detects the address and switches to World 1, then the
 * code jumps to the return address passed in a2.
 *
 * The world switch happens when the CPU's PC matches the configured
 * WORLD_TRIGGER_ADDR register.
 */

    .section .text
    .global tee_return_trampoline
    .type   tee_return_trampoline, @function
    .align  4

/*
 * tee_return_trampoline
 *
 * Entry:
 *   a2 = return address (set by handler before jumping here)
 *   WCL configured to switch to World 1 when this address is executed
 *
 * Exit:
 *   Jumps to a2 in World 1
 *
 * The handler sets up WCL registers:
 *   WORLD_PREPARE = WORLD_REE (0x02)
 *   WORLD_TRIGGER = TEE_ENTRY_TRAMPOLINE
 *   WORLD_UPDATE  = 1
 *
 * When CPU executes this address:
 *   - WCL triggers world switch to World 1
 *   - Execution continues in World 1
 *   - Jump to a2 returns to caller
 */
tee_return_trampoline:
    /* Memory barrier before world switch */
    memw

    /* Jump to return address - now in World 1 */
    jx      a2

    .size   tee_return_trampoline, . - tee_return_trampoline


/*
 * Trampoline end marker for size calculation
 */
    .global tee_return_trampoline_end
tee_return_trampoline_end:
