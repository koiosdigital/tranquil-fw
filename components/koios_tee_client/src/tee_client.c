/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_client.c
 * @brief TEE client implementation for application code
 *
 * Provides functions to call TEE services from World 1 (non-secure).
 * The actual world switch is performed by jumping to the TEE entry stub
 * in RTC FAST memory, which WCL monitors to trigger the W1->W0 transition.
 */

#include "tee_client.h"
#include "soc/world_controller_reg.h"
#include <string.h>

/**
 * @brief Internal function to perform TEE call
 *
 * Stores return address in API, then jumps to TEE entry stub.
 * WCL triggers world switch when entry stub address is executed.
 *
 * MUST be in IRAM - return point needs to be in IRAM, not flash,
 * because flash cache access after world switch may not work.
 */
static int32_t __attribute__((noinline, section(".iram1"))) tee_invoke(void)
{
    volatile tee_api_t* api = TEE_API();
    int32_t result;

    /* Memory barrier before call */
    __asm__ volatile("memw" ::: "memory");

    /*
     * Inline assembly to:
     * 1. Get address of return label
     * 2. Store it in api->return_addr
     * 3. Jump to TEE entry stub
     * 4. Return label: read result
     *
     * The entry stub is at TEE_ENTRY_STUB (0x600FE100).
     * When CPU executes this address, WCL switches to World 0.
     * After handler completes, WCL switches back to World 1
     * and execution continues at the return label.
     */
    __asm__ volatile(
        /* Save a0 (return address) and a1 (stack pointer) before TEE call */
        /* Use api->reserved[1] (offset 100) for a1, reserved[2] (offset 104) for a0 */
        "s32i   a0, %[api], 104\n"          /* api->reserved[2] = a0 (return addr) */
        "s32i   a1, %[api], 100\n"          /* api->reserved[1] = a1 (stack ptr) */

        /* Build return address and store in API */
        "movi   a2, .Ltee_return_%=\n"
        "s32i   a2, %[api], 28\n"           /* api->return_addr offset */
        "memw\n"

        /* Build TEE entry address (0x600FE100) without literal pool */
        "movi   a2, 0x600\n"
        "slli   a2, a2, 20\n"               /* a2 = 0x60000000 */
        "movi   a3, 0xFE\n"
        "slli   a3, a3, 12\n"               /* a3 = 0x000FE000 */
        "or     a2, a2, a3\n"               /* a2 = 0x600FE000 */
        "movi   a3, 0x100\n"
        "or     a2, a2, a3\n"               /* a2 = 0x600FE100 */

        /* Jump to TEE entry - WCL will switch to W0 */
        "jx     a2\n"

        /* Return point - execution resumes here after TEE returns */
        ".Ltee_return_%=:\n"
        /* Rebuild API address (0x600FF000) - can't trust any registers after TEE */
        "movi   a2, 0x600\n"
        "slli   a2, a2, 20\n"
        "movi   a3, 0xFF\n"
        "slli   a3, a3, 12\n"
        "or     a2, a2, a3\n"               /* a2 = 0x600FF000 (API address) */
        /* Restore a0 and a1 from api->reserved */
        "l32i   a0, a2, 104\n"              /* Restore a0 (return addr) */
        "l32i   a1, a2, 100\n"              /* Restore a1 (stack ptr) */
        "memw\n"
        :
        : [api] "a"(api)
        : "a2", "a3", "memory"
    );

    /* Read result from API */
    result = api->result;

    return result;
}

tee_error_t tee_client_init(void)
{
    volatile tee_api_t* api = TEE_API();

    /* Verify magic */
    if (api->magic != TEE_API_MAGIC) {
        return TEE_ERR_NOT_INIT;
    }

    /* Verify version compatibility */
    if (api->version < 1) {
        return TEE_ERR_NOT_INIT;
    }

    return TEE_OK;
}

bool tee_client_available(void)
{
    volatile tee_api_t* api = TEE_API();
    return (api->magic == TEE_API_MAGIC);
}

int32_t tee_call(tee_service_id_t service_id)
{
    volatile tee_api_t* api = TEE_API();

    /* Set service ID */
    api->service_id = service_id;

    /* Invoke TEE */
    return tee_invoke();
}

int32_t tee_call_params(tee_service_id_t service_id,
                        const uint32_t* params,
                        size_t num_params)
{
    volatile tee_api_t* api = TEE_API();

    /* Set service ID */
    api->service_id = service_id;

    /* Copy parameters */
    if (params && num_params > 0) {
        if (num_params > 8) {
            num_params = 8;
        }
        for (size_t i = 0; i < num_params; i++) {
            api->param[i] = params[i];
        }
    }

    /* Invoke TEE */
    return tee_invoke();
}

uint32_t tee_get_call_count(void)
{
    return TEE_API()->call_count;
}

int32_t tee_get_last_result(void)
{
    return TEE_API()->result;
}

uint32_t tee_get_debug_stage(void)
{
    return TEE_API()->debug_stage;
}

bool tee_is_world1(void)
{
    /* Read WCL_CORE_0_WORLD_IRAM0_REG - bit 0: 1=W0, 0=W1 */
    uint32_t wcl_val = *(volatile uint32_t*)WCL_CORE_0_WORLD_IRAM0_REG;
    return (wcl_val & 0x1) == 0;
}
