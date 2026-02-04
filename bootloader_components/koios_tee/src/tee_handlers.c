/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

 /**
  * @file tee_handlers.c
  * @brief TEE service handlers and dispatch logic
  *
  * This file contains:
  * 1. C handlers compiled with PIC (-fPIC -fno-jump-tables)
  * 2. Bootloader-time initialization helpers
  *
  * All functions in .tee_handlers section are copied to RTC FAST memory
  * at boot time. The entry stub calls tee_dispatch() which must be the
  * FIRST function in this section (at offset 0 from TEE_HANDLERS_ADDR).
  *
  * Constraints for handlers:
  * - No global/static variables (use api struct for state)
  * - No switch statements (use if-else to avoid jump tables)
  * - Compiled with -fPIC -fno-jump-tables
  */

#include "tee_config.h"
#include <string.h>

  /*
   * Handlers are compiled separately and linked at 0x600FE300.
   * No section attributes needed - the linker script handles placement.
   * tee_dispatch MUST be first (entry point specified in linker).
   */

   /* ============================================================================
    * C Handlers - linked at 0x600FE300 (RTC FAST)
    * ============================================================================ */

    /**
     * @brief Service dispatcher - entry point called from assembly stub
     *
     * Routes service calls to appropriate handlers.
     * This is the ENTRY POINT - must be at 0x600FE300.
     *
     * @param service_id Service to invoke
     * @param api Pointer to shared API structure
     * @return Result code (TEE_OK or error)
     */
int32_t tee_dispatch(uint32_t service_id, volatile tee_api_t* api)
{
    /* TEMP: Write marker - small value fits in movi (max 2047) */

    /* DEBUG: Copy return_addr to reserved[0] so we can see it after reset */
    api->reserved[0] = api->return_addr;

    /* Build WCL address without literal pool (avoids l32r placing data before function) */
    uint32_t wcl_addr;
    __asm__ volatile(
        "movi   %0, 0x600\n"
        "slli   %0, %0, 20\n"        /* 0x60000000 */
        "movi   a9, 0x0D\n"
        "slli   a9, a9, 16\n"        /* 0x000D0000 */
        "or     %0, %0, a9\n"        /* 0x600D0000 */
        "movi   a9, 0x150\n"
        "or     %0, %0, a9\n"        /* 0x600D0150 */
        : "=a"(wcl_addr)
        :
        : "a9"
    );
    uint32_t wcl0_val = *(volatile uint32_t*)wcl_addr;

    /* Per TRM: bit 0 indicates IRAM world - 0=non-secure(W1), 1=secure(W0) */
    api->call_count = (wcl0_val & 0x1);

    (void)service_id;
    return 0;
#if 0
    /* Use if-else instead of switch to avoid jump tables */
    if (service_id == TEE_SVC_NOP) {
        return tee_handle_nop(api);
    }
    else if (service_id == TEE_SVC_TEST) {
        return tee_handle_test(api);
    }
    else if (service_id == TEE_SVC_GET_INFO) {
        return tee_handle_get_info(api);
    }
    else {
        api->last_error = TEE_ERR_INVALID_SVC;
        return TEE_ERR_INVALID_SVC;
    }
#endif
}

/**
 * @brief NOP handler - health check
 */
int32_t tee_handle_nop(volatile tee_api_t* api)
{
    (void)api;
    return TEE_OK;
}

/**
 * @brief Test handler - increment counter
 */
int32_t tee_handle_test(volatile tee_api_t* api)
{
    api->call_count++;
    return TEE_OK;
}

/**
 * @brief Get TEE info handler
 */
int32_t tee_handle_get_info(volatile tee_api_t* api)
{
    api->param[0] = TEE_API_VERSION;
    api->param[1] = api->capabilities;
    return TEE_OK;
}

/*
 * NOTE: Bootloader-time init functions (tee_api_init, tee_state_init, etc.)
 * are in tee_init.c, NOT here. This file is compiled separately and linked
 * at 0x600FE300 - it only contains the runtime handlers.
 */
