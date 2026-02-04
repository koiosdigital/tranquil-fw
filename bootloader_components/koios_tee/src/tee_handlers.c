/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

 /**
  * @file tee_handlers.c
  * @brief TEE service handlers
  *
  * These handlers run in World 0 (secure). They are copied to fixed
  * addresses in RTC FAST memory during bootloader initialization.
  *
  * Handler flow:
  * 1. App (W1) jumps to entry address -> WCL switches to W0
  * 2. Handler executes in W0
  * 3. Handler configures return world switch
  * 4. Handler jumps to trampoline -> WCL switches to W1 -> returns to app
  */

#include "tee_config.h"
#include "soc/world_controller_reg.h"
#include "soc/world_controller_struct.h"

  /**
   * @brief Test service handler
   *
   * Simple handler that increments call counter and echoes request with
   * modification. Used to verify world switching works correctly.
   *
   * Protocol:
   *   Input:  api->request = value to echo
   *           api->return_addr = address to return to
   *   Output: api->response = request + 0x1000
   *           api->last_result = 0 (success)
   *           api->call_count incremented
   *
   * @note This function is copied to TEE_ENTRY_TEST (0x600FE100).
   *       It must be position-independent or use absolute addresses.
   */
__attribute__((noinline, used))
void tee_test_handler(void)
{
    /*
     * Two NOPs required for nested interrupt handling per TRM 16.5.4.1
     * This gives the hardware time to complete the world switch.
     */
    __asm__ volatile("nop.n");
    __asm__ volatile("nop.n");

    /* Access shared memory */
    volatile tee_api_t* api = TEE_API();

    /* Increment call counter */
    api->call_count++;

    /* Process request: echo with modification */
    api->response = api->request + 0x1000;
    api->last_result = 0;  /* Success */

    /* Get return address from shared memory */
    uint32_t ret_addr = api->return_addr;

    /* Configure world switch: when we hit trampoline, switch to W1 */
    REG_WRITE(WCL_CORE_0_WORLD_PREPARE_REG, 2);
    REG_WRITE(WCL_CORE_0_WORLD_TRIGGER_ADDR_REG, TEE_ENTRY_TRAMPOLINE);
    REG_WRITE(WCL_CORE_0_WORLD_UPDATE_REG, 1);

    /* Memory barrier */
    __asm__ volatile("memw" ::: "memory");

    /*
     * Jump to trampoline with return address in a2.
     * The trampoline will:
     * 1. Execute at TEE_ENTRY_TRAMPOLINE -> WCL switches to W1
     * 2. Jump to ret_addr (now in W1)
     */
    register uint32_t a2_ret __asm__("a2") = ret_addr;
    __asm__ volatile(
        "j tee_return_trampoline\n"
        :: "r"(a2_ret)
        : "memory"
        );

    __builtin_unreachable();
}

/*
 * Symbols for handler size calculation (defined in assembly or linker)
 * Used by tee_init.c to copy handlers to RTC memory
 */
extern void tee_return_trampoline(void);
