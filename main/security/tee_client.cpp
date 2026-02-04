/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

 /**
  * @file tee_client.cpp
  * @brief TEE Client - Phase 1 World Controller Test
  *
  * Implements the client-side interface for testing World Controller
  * switching between app (World 1) and TEE (World 0).
  *
  * World Switching Flow:
  *
  * tee_init() - verifies TEE loaded:
  *   App starts in W1 (from bootloader)

  * tee_test_call() - W1->W0->W1 round-trip:
  *   App in W1
  *   -> Set request in shared memory
  *   -> Jump to TEE_ENTRY_TEST (WCL entry point)
  *   -> WCL switches to W0
  *   -> TEE handler executes
  *   -> Handler configures return switch, jumps to trampoline
  *   -> WCL switches to W1
  *   -> Returns to app with result
  */

#include "tee_client.h"

#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char* TAG = "tee_client";

/* ============================================================================
 * State
 * ============================================================================ */

 /* Mutex for thread-safe TEE access */
static SemaphoreHandle_t s_tee_mutex = nullptr;

/* Cached API pointer */
static volatile tee_api_t* s_api = nullptr;

/* ============================================================================
 * Internal Helpers
 * ============================================================================ */

static inline bool tee_lock(uint32_t timeout_ms)
{
    if (s_tee_mutex == nullptr) {
        return false;
    }
    return xSemaphoreTake(s_tee_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

static inline void tee_unlock(void)
{
    if (s_tee_mutex != nullptr) {
        xSemaphoreGive(s_tee_mutex);
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

int tee_init(void)
{
    ESP_LOGI(TAG, "Initializing TEE client...");

    /* Create mutex */
    if (s_tee_mutex == nullptr) {
        s_tee_mutex = xSemaphoreCreateMutex();
        if (s_tee_mutex == nullptr) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return TEE_ERR_INVALID;
        }
    }

    /* Validate API table */
    volatile tee_api_t* api = TEE_API();

    if (api->magic != TEE_API_MAGIC) {
        ESP_LOGE(TAG, "TEE API not initialized (magic=0x%08lx, expected=0x%08lx)",
            (unsigned long)api->magic, (unsigned long)TEE_API_MAGIC);
        return TEE_ERR_NO_API;
    }

    if (api->version < TEE_API_VERSION) {
        ESP_LOGE(TAG, "TEE API version mismatch: got %lu, need >= %d",
            (unsigned long)api->version, TEE_API_VERSION);
        return TEE_ERR_INVALID;
    }

    s_api = api;

    ESP_LOGI(TAG, "API validated: magic=0x%08lx version=%lu call_count=%lu",
        (unsigned long)api->magic,
        (unsigned long)api->version,
        (unsigned long)api->call_count);

    return TEE_OK;
}

bool tee_is_world1(void)
{
    volatile tee_api_t* api = TEE_API();
    return api->world_switched != 0;
}

/**
 * @brief Call TEE entry point and return here
 *
 * This function:
 * 1. Stores the return label address in shared memory
 * 2. Jumps to TEE entry (triggering W1->W0 switch)
 * 3. TEE handler processes, configures W0->W1 switch
 * 4. Trampoline jumps to return_addr, resuming here
 */
static void __attribute__((noinline, used)) tee_invoke_entry(void)
{
    volatile tee_api_t* api = TEE_API();

    /*
     * Get the address of our return label using inline assembly.
     * The trampoline will jump here after the world switch completes.
     * We use a label instead of A0 because A0 contains windowed return
     * format which can't be used with a simple jx instruction.
     */
    uint32_t ret_addr;
    __asm__ volatile(
        "movi %0, .Ltee_return_label\n"
        : "=r"(ret_addr)
    );
    api->return_addr = ret_addr;

    /* Memory barrier */
    __asm__ volatile("memw" ::: "memory");

    /*
     * Jump to TEE entry point.
     * WCL will detect PC == TEE_ENTRY_TEST and switch to World 0.
     */
    __asm__ volatile(
        "movi a8, %0\n"
        "jx a8\n"
        ".Ltee_return_label:\n"
        :
        : "i"(TEE_ENTRY_TEST)
        : "a8", "memory"
    );

    /* Execution resumes here after world switch back to W1 */
}

int32_t tee_test_call(uint32_t input)
{
    if (s_api == nullptr) {
        ESP_LOGE(TAG, "TEE client not initialized");
        return TEE_ERR_NOT_INIT;
    }

    if (!tee_lock(5000)) {
        ESP_LOGE(TAG, "Failed to acquire mutex");
        return TEE_ERR_INVALID;
    }

    volatile tee_api_t* api = s_api;

    /* Set request */
    api->request = input;
    api->last_result = TEE_ERR_INVALID;  /* Will be set by handler */

    /* Memory barrier before calling TEE */
    __asm__ volatile("memw" ::: "memory");

    /* Call the TEE - this will return via the world switch trampoline */
    tee_invoke_entry();

    /* Memory barrier after world switch */
    __asm__ volatile("memw" ::: "memory");

    /* Read result */
    int32_t result = api->last_result;

    tee_unlock();

    return result;
}

uint32_t tee_get_call_count(void)
{
    volatile tee_api_t* api = TEE_API();
    if (api->magic != TEE_API_MAGIC) {
        return 0;
    }
    return api->call_count;
}

int32_t tee_get_last_result(void)
{
    volatile tee_api_t* api = TEE_API();
    if (api->magic != TEE_API_MAGIC) {
        return TEE_ERR_NO_API;
    }
    return api->last_result;
}
