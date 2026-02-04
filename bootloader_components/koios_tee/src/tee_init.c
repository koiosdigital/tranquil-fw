/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

 /**
  * @file tee_init.c
  * @brief TEE initialization in bootloader
  *
  * Uses ESP-IDF bootloader hooks to initialize the TEE before the
  * application starts. Sets up:
  * - Handler code in RTC FAST memory
  * - World Controller entry points
  * - Shared memory API table
  */

#include <string.h>
#include "esp_log.h"
#include "tee_config.h"
#include "soc/world_controller_reg.h"
#include "bootloader_common.h"
#include "bootloader_utility.h"
#include "esp_image_format.h"

static const char* TAG = "tee";

static void print_world(const char* tag) {
    // Read WCL_CORE_0_World_IRam0_REG
    uint32_t wcl0_val = *(volatile uint32_t*)WCL_CORE_0_WORLD_IRAM0_REG;
    uint32_t world0_bit = wcl0_val & 0x1;  // Extract bit 0

    uint32_t wcl1_val = *(volatile uint32_t*)WCL_CORE_1_WORLD_IRAM0_REG;
    uint32_t world1_bit = wcl1_val & 0x1;  // Extract bit 0

    // Per TRM: bit 0 indicates IRAM world - 0=non-secure(W1), 1=secure(W0)
    // Note: W0=secure, W1=non-secure
    const char* world0_name = world0_bit ? "secure" : "non-secure";
    const char* world1_name = world1_bit ? "secure" : "non-secure";
    ESP_LOGE(TAG, "%s: core 0: %s, core 1: %s", tag, world0_name, world1_name);
}

/**
 * @brief Copy a function to a fixed address in RTC memory
 *
 * This is a simple copy that works for small, position-independent code.
 * For larger handlers, we'd need to handle relocations.
 *
 * @param dest Destination address in RTC memory
 * @param src Source function address
 * @param max_size Maximum bytes to copy
 * @return Number of bytes copied, or 0 on error
 */
static size_t copy_handler_to_rtc(void* dest, const void* src, const void* src_end, size_t max_size)
{
    /*
     * Calculate actual size from the end marker.
     * The assembly files define foo_end labels for size calculation.
     */
    size_t size = (size_t)((const uint8_t*)src_end - (const uint8_t*)src);
    if (size > max_size) {
        ESP_LOGW(TAG, "Handler size %u exceeds max %u, truncating", (unsigned)size, (unsigned)max_size);
        size = max_size;
    }

    /* Word-aligned copy for IRAM compatibility - round UP to include partial words */
    const uint32_t* s = (const uint32_t*)src;
    uint32_t* d = (uint32_t*)dest;
    size_t words = (size + 3) / 4;  /* Round up to next word */
    for (size_t i = 0; i < words; i++) {
        d[i] = s[i];
    }

    return size;
}

/**
 * @brief Initialize the shared API table
 */
static void init_api_table(void)
{
    volatile tee_api_t* api = TEE_API();

    /* Zero the shared memory region */
    memset((void*)api, 0, TEE_SHARED_SIZE);

    /* Initialize API table */
    api->magic = TEE_API_MAGIC;
    api->version = TEE_API_VERSION;
    api->call_count = 0;
    api->last_result = 0;
    api->request = 0;
    api->response = 0;
    api->return_addr = 0;
    api->handler_addr = 0;  /* Not used - handler logic is inline in entry stub */

    /* Memory barrier */
    __asm__ volatile("memw" ::: "memory");

    ESP_LOGI(TAG, "API table initialized at 0x%08x", TEE_API_ADDR);
}

/* ============================================================================
 * Bootloader Hooks
 * ============================================================================ */

 /**
  * @brief Called before bootloader hardware initialization
  */
void bootloader_before_init(void)
{
    /* Nothing needed */
}

/**
 * @brief Continue boot after world switch to W1
 *
 * This function is called after the W0->W1 transition completes.
 * It must be noinline so we have a stable function address to jump to.
 */
__attribute__((noinline, used))
static void continue_boot_in_w1(void)
{
    print_world("continue_boot_in_w1");

    /* Copy test handler for app TEE calls */
    size_t handler_size = copy_handler_to_rtc(
        (void*)TEE_ENTRY_TEST,
        (const void*)tee_test_handler,
        (const void*)tee_test_handler_end,
        0x100  /* Max 256 bytes */
    );
    ESP_LOGI(TAG, "Copied test handler to 0x%08x (%u bytes)",
        TEE_ENTRY_TEST, (unsigned)handler_size);

    /* Sync instruction cache */
    __asm__ volatile("memw\nisync" ::: "memory");

    /* Configure WCL entry points for app W1→W0 calls */
    tee_wcl_configure(0, TEE_ENTRY_TEST);
    tee_wcl_configure(1, TEE_ENTRY_TEST);
    ESP_LOGI(TAG, "WCL configured for app: entry at 0x%08x", TEE_ENTRY_TEST);

    /* Initialize shared memory API table */
    init_api_table();

    ESP_LOGI(TAG, "TEE initialization complete");

    /* ========================================================================
     * Load and launch the application
     * ======================================================================== */

     /* Load partition table */
    bootloader_state_t bs;
    if (!bootloader_utility_load_partition_table(&bs)) {
        ESP_LOGE(TAG, "Failed to load partition table");
        abort();
    }

    /* Get the selected boot partition (handles OTA selection) */
    int boot_index = bootloader_utility_get_selected_boot_partition(&bs);
    if (boot_index == INVALID_INDEX) {
        ESP_LOGE(TAG, "No bootable partition found");
        abort();
    }

    print_world("before_load_boot_image");
    bootloader_utility_load_boot_image(&bs, boot_index);
}

/**
 * @brief Called after bootloader hardware initialization
 *
 * TEE initialization sequence (running in W0/secure by default):
 * 1. Configure PMS (memory permissions)
 * 2. Copy stubs to RTC FAST memory
 * 3. Exit to W1 (non-secure) via exit stub
 */
void bootloader_after_init(void)
{
    print_world("after_init_start");

    /* 1. Configure PMS while in secure world */
    tee_configure_pms();

    /* 2. Zero TEE private region and copy stubs */
    ESP_LOGI(TAG, "Initializing TEE memory at 0x%08x", TEE_PRIVATE_BASE);
    memset((void*)TEE_PRIVATE_BASE, 0, TEE_PRIVATE_SIZE);

    /* Copy exit stub for W0→W1 transition */
    size_t exit_size = copy_handler_to_rtc(
        (void*)TEE_EXIT_TO_W1,
        (const void*)tee_exit_stub,
        (const void*)tee_exit_stub_end,
        0x40  /* Max 64 bytes - stub builds address without literal pool */
    );
    ESP_LOGI(TAG, "Copied exit stub to 0x%08x (%u bytes)",
        TEE_EXIT_TO_W1, (unsigned)exit_size);

    /* Copy trampoline for W0→W1 return (used by TEE handlers later) */
    size_t tramp_size = copy_handler_to_rtc(
        (void*)TEE_ENTRY_TRAMPOLINE,
        (const void*)tee_return_trampoline,
        (const void*)tee_return_trampoline_end,
        0x40  /* Max 64 bytes */
    );
    ESP_LOGI(TAG, "Copied trampoline to 0x%08x (%u bytes)",
        TEE_ENTRY_TRAMPOLINE, (unsigned)tramp_size);

    /* Sync instruction cache */
    __asm__ volatile("memw\nisync" ::: "memory");

    /* 3. Configure WCL to switch to W1 when exit stub executes */
    REG_WRITE(WCL_CORE_0_WORLD_PREPARE_REG, 0x2);  /* WORLD_REE (W1) */
    REG_WRITE(WCL_CORE_0_WORLD_TRIGGER_ADDR_REG, TEE_EXIT_TO_W1);
    REG_WRITE(WCL_CORE_0_WORLD_UPDATE_REG, 1);
    __asm__ volatile("memw" ::: "memory");

    /* Store continuation function address in shared memory */
    uint32_t cont_addr = (uint32_t)continue_boot_in_w1;
    TEE_API()->return_addr = cont_addr;
    __asm__ volatile("memw" ::: "memory");

    ESP_LOGI(TAG, "Jumping to exit stub, continuation at 0x%08x", cont_addr);

    /* Jump to exit stub - it reads return_addr from shared memory and jumps there */
    {
        uint32_t target = TEE_EXIT_TO_W1;
        __asm__ volatile(
            "jx %0\n"
            :
        : "r"(target)
            : "memory"
            );
    }

    /* Should never reach here - exit stub jumps to continue_boot_in_w1 */
    __builtin_unreachable();
}

/* Force linker to include this module (hooks are weak symbols) */
void bootloader_hooks_include(void) {}
