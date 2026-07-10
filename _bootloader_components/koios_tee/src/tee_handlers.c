/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_handlers.c
 * @brief TEE service handlers
 *
 * This file contains:
 * 1. tee_handler_main() - Main C handler called from assembly entry stub
 * 2. Service dispatcher and individual handlers
 *
 * The assembly entry stub (tee_entry.S) handles:
 * - NOPs for TRM compliance after world switch
 * - Stack setup
 * - Calling tee_handler_main(service_id, api)
 * - Jumping to the trampoline address returned by tee_handler_main
 *
 * All functions are compiled with CALL0 ABI (-mabi=call0) and linked
 * after the assembly entry stub at 0x600FE300 (RTC FAST).
 *
 * Constraints:
 * - No global/static variables (use api struct for state)
 * - No switch statements (use if-else to avoid jump tables)
 */

#include "tee_config.h"

/* ============================================================================
 * Main Handler - called from assembly entry stub
 * ============================================================================ */

/**
 * @brief Main TEE handler - dispatches service and configures return
 *
 * Called from the assembly entry stub with:
 *   a2 = service_id
 *   a3 = api pointer
 *
 * Returns the trampoline address in a2 for the entry stub to jump to.
 *
 * @param service_id Service to invoke
 * @param api Pointer to shared API structure
 * @return Trampoline address (0x600FF080)
 */
uint32_t tee_handler_main(uint32_t service_id, volatile tee_api_t* api)
{
    int32_t result;

    /* Dispatch to appropriate handler */
    if (service_id == TEE_SVC_NOP) {
        result = tee_handle_nop(api);
    }
    else if (service_id == TEE_SVC_TEST) {
        result = tee_handle_test(api);
    }
    else if (service_id == TEE_SVC_GET_INFO) {
        result = tee_handle_get_info(api);
    }
    else {
        api->last_error = TEE_ERR_INVALID_SVC;
        result = TEE_ERR_INVALID_SVC;
    }

    /* Store result in API structure */
    api->result = result;

    /* Configure WCL for W0->W1 return */
    uint32_t core;
    __asm__ volatile("rsr.prid %0" : "=r"(core));
    core = (core >> 13) & 0x1;

    /* Build WCL per-core base (0x600D0000 + core * 0x400) */
    volatile uint32_t* wcl_base = (volatile uint32_t*)(0x600D0000 + (core * 0x400));

    /* WORLD_PREPARE = 0x2 (World 1 / non-secure) */
    wcl_base[0x144 / 4] = 0x2;

    /* WORLD_TRIGGER = trampoline address (0x600FF080) */
    wcl_base[0x140 / 4] = TEE_RETURN_TRAMPOLINE;

    /* WORLD_UPDATE = 1 to commit configuration */
    wcl_base[0x148 / 4] = 1;

    __asm__ volatile("memw" ::: "memory");

    /* Return trampoline address for entry stub to jump to */
    return TEE_RETURN_TRAMPOLINE;
}

/* ============================================================================
 * Service Handlers
 * ============================================================================ */

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
