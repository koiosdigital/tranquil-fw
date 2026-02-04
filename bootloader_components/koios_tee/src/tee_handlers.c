/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tee_handlers.c
 * @brief TEE service handlers and dispatch logic
 *
 * This file contains:
 * 1. Bootloader-time initialization helpers
 * 2. Handler function definitions (for future PIC compilation)
 *
 * Note: Current implementation keeps handler logic inline in tee_entry_stub.S
 * for position-independence. This file provides structure for future expansion
 * when proper PIC compilation or separate TEE binary support is added.
 */

#include "tee_config.h"
#include <string.h>

/* ============================================================================
 * Bootloader-Time API Table Initialization
 * ============================================================================
 * These functions run during bootloader init (in W0) before world switch.
 * They set up the shared API table that both worlds use.
 */

/**
 * @brief Initialize the shared API table
 *
 * Called from bootloader during TEE init. Sets up the API structure
 * at TEE_API_ADDR (0x600FF000).
 */
void tee_api_init(void)
{
    volatile tee_api_t* api = TEE_API();

    /* Zero the entire structure */
    memset((void*)api, 0, sizeof(tee_api_t));

    /* Set header fields */
    api->magic = TEE_API_MAGIC;
    api->version = TEE_API_VERSION;
    api->flags = 0;

    /* Set capabilities - bitmask of supported services */
    api->capabilities = (1 << TEE_SVC_NOP) |
                       (1 << TEE_SVC_TEST) |
                       (1 << TEE_SVC_GET_INFO);

    /* Initialize call state */
    api->service_id = 0;
    api->result = 0;
    api->call_count = 0;
    api->return_addr = 0;

    /* Memory barrier */
    __asm__ volatile("memw" ::: "memory");
}

/**
 * @brief Initialize the TEE private state
 *
 * Called from bootloader during TEE init. Sets up the state structure
 * at TEE_STATE_ADDR (0x600FEF00).
 */
void tee_state_init(void)
{
    volatile tee_state_t* state = TEE_STATE();

    /* Zero the state */
    memset((void*)state, 0, sizeof(tee_state_t));

    /* Mark as initialized */
    state->initialized = 1;
    state->attestation_level = 0;

    /* Memory barrier */
    __asm__ volatile("memw" ::: "memory");
}

/**
 * @brief Check if TEE is properly initialized
 *
 * @return true if TEE API magic and version are valid
 */
bool tee_is_initialized(void)
{
    volatile tee_api_t* api = TEE_API();
    return (api->magic == TEE_API_MAGIC && api->version >= 1);
}

/* ============================================================================
 * Handler Prototypes (for future PIC compilation)
 * ============================================================================
 * These handlers would be compiled with -fPIC -fno-jump-tables and copied
 * to RTC FAST memory. For now, handler logic is inline in tee_entry_stub.S.
 */

#if 0  /* Future: Enable when PIC compilation is set up */

/**
 * @brief NOP handler - health check
 */
__attribute__((section(".tee_text"), noinline))
int32_t tee_handle_nop(volatile tee_api_t* api)
{
    (void)api;
    return TEE_OK;
}

/**
 * @brief Test handler - increment counter
 */
__attribute__((section(".tee_text"), noinline))
int32_t tee_handle_test(volatile tee_api_t* api)
{
    api->call_count++;
    api->result = TEE_OK;
    return TEE_OK;
}

/**
 * @brief Get TEE info handler
 */
__attribute__((section(".tee_text"), noinline))
int32_t tee_handle_get_info(volatile tee_api_t* api)
{
    api->param[0] = TEE_API_VERSION;
    api->param[1] = api->capabilities;
    api->result = TEE_OK;
    return TEE_OK;
}

/**
 * @brief Service dispatcher
 *
 * Routes service calls to appropriate handlers.
 */
__attribute__((section(".tee_text"), noinline))
int32_t tee_dispatch(uint32_t service_id, volatile tee_api_t* api)
{
    switch (service_id) {
        case TEE_SVC_NOP:
            return tee_handle_nop(api);
        case TEE_SVC_TEST:
            return tee_handle_test(api);
        case TEE_SVC_GET_INFO:
            return tee_handle_get_info(api);
        default:
            api->result = TEE_ERR_INVALID_SVC;
            return TEE_ERR_INVALID_SVC;
    }
}

#endif /* Future PIC handlers */
