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
  *
  * Initialization Flow:
  * 1. bootloader_after_init() runs in W0 (secure world)
  * 2. Configure PMS (placeholder)
  * 3. Copy stubs to RTC FAST memory
  * 4. Configure WCL for W0->W1 exit
  * 5. Jump to exit stub -> switches to W1
  * 6. continue_boot_in_w1() runs in W1
  * 7. Copy entry stub, configure WCL for app calls
  * 8. Initialize API table
  * 9. Load and launch application
  */

#include <string.h>
#include "esp_log.h"
#include "tee_config.h"
#include "soc/world_controller_reg.h"
#include "bootloader_common.h"
#include "bootloader_utility.h"
#include "esp_image_format.h"

static const char* TAG = "tee";

/* Embedded handlers binary (generated from tee_handlers.c, linked at 0x600FE300) */
extern const uint8_t _tee_handlers_bin[];
extern const size_t _tee_handlers_bin_len;

/* ============================================================================
 * Bootloader-Time API Table Initialization
 * ============================================================================ */

 /**
  * @brief Initialize the shared API table
  */
static void tee_api_init(void)
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
 */
static void tee_state_init(void)
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
 * @brief Print current world status for debugging
 */
static void print_world(const char* context)
{
    uint32_t wcl0_val = *(volatile uint32_t*)WCL_CORE_0_WORLD_IRAM0_REG;
    uint32_t wcl1_val = *(volatile uint32_t*)WCL_CORE_1_WORLD_IRAM0_REG;

    /* Per TRM: bit 0 indicates IRAM world - 0=non-secure(W1), 1=secure(W0) */
    const char* world0 = (wcl0_val & 0x1) ? "W0/secure" : "W1/non-secure";
    const char* world1 = (wcl1_val & 0x1) ? "W0/secure" : "W1/non-secure";

    ESP_LOGI(TAG, "[%s] Core0: %s, Core1: %s", context, world0, world1);
}

/**
 * @brief Copy code to RTC FAST memory
 *
 * Performs word-aligned copy for IRAM compatibility.
 *
 * @param dest Destination address in RTC FAST memory
 * @param src Source function start address
 * @param src_end Source function end marker
 * @param max_size Maximum bytes allowed
 * @return Number of bytes copied
 */
static size_t copy_to_rtc(void* dest, const void* src, const void* src_end, size_t max_size)
{
    size_t size = (size_t)((const uint8_t*)src_end - (const uint8_t*)src);

    if (size > max_size) {
        ESP_LOGW(TAG, "Code size %u exceeds max %u, truncating",
            (unsigned)size, (unsigned)max_size);
        size = max_size;
    }

    /* Word-aligned copy (round up to include partial words) */
    const uint32_t* s = (const uint32_t*)src;
    uint32_t* d = (uint32_t*)dest;
    size_t words = (size + 3) / 4;

    for (size_t i = 0; i < words; i++) {
        d[i] = s[i];
    }

    return size;
}

/**
 * @brief Sync instruction cache after copying code
 */
static void sync_icache(void)
{
    __asm__ volatile("memw\nisync" ::: "memory");
}

/* ============================================================================
 * Bootloader Hooks
 * ============================================================================ */

 /**
  * @brief Called before bootloader hardware initialization
  */
void bootloader_before_init(void)
{

}

/**
 * @brief Continue boot after world switch to W1
 *
 * This function is called after the W0->W1 transition completes.
 * Must be noinline to have a stable function address.
 */
__attribute__((noinline, used))
static void continue_boot_in_w1(void)
{
    print_world("continue_boot_in_w1");

    /* Copy C handlers binary to RTC FAST (linked at 0x600FE300) */
    size_t handlers_size = _tee_handlers_bin_len;
    if (handlers_size > TEE_HANDLERS_SIZE) {
        ESP_LOGW(TAG, "Handlers size %u exceeds max %u, truncating",
            (unsigned)handlers_size, TEE_HANDLERS_SIZE);
        handlers_size = TEE_HANDLERS_SIZE;
    }

    /* Word-aligned copy */
    const uint32_t* src = (const uint32_t*)_tee_handlers_bin;
    uint32_t* dst = (uint32_t*)TEE_HANDLERS_ADDR;
    size_t words = (handlers_size + 3) / 4;
    for (size_t i = 0; i < words; i++) {
        dst[i] = src[i];
    }
    ESP_LOGI(TAG, "Handlers binary copied to 0x%08x (%u bytes)",
        TEE_HANDLERS_ADDR, (unsigned)handlers_size);

    /* Copy entry stub for app TEE calls */
    size_t entry_size = copy_to_rtc(
        (void*)TEE_ENTRY_STUB,
        (const void*)tee_entry_stub_start,
        (const void*)tee_entry_stub_end,
        0x100  /* Max 256 bytes */
    );
    ESP_LOGI(TAG, "Entry stub copied to 0x%08x (%u bytes)",
        TEE_ENTRY_STUB, (unsigned)entry_size);

    sync_icache();

    /* Configure WCL entry points for app W1->W0 calls */
    tee_wcl_configure(0, TEE_ENTRY_STUB);
    tee_wcl_configure(1, TEE_ENTRY_STUB);
    ESP_LOGI(TAG, "WCL configured: entry at 0x%08x", TEE_ENTRY_STUB);

    /* Initialize shared memory API table and state */
    tee_api_init();
    tee_state_init();
    ESP_LOGI(TAG, "API initialized at 0x%08x", TEE_API_ADDR);

    ESP_LOGI(TAG, "TEE initialization complete");

    /* ========================================================================
     * Load and launch the application
     * ======================================================================== */

    bootloader_state_t bs;
    if (!bootloader_utility_load_partition_table(&bs)) {
        ESP_LOGE(TAG, "Failed to load partition table");
        abort();
    }

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
 * 2. Zero TEE private region
 * 3. Copy exit stub and trampoline to RTC FAST
 * 4. Configure WCL for W0->W1 exit
 * 5. Jump to exit stub -> transitions to W1
 */
void bootloader_after_init(void)
{
    print_world("bootloader_after_init");

    volatile tee_api_t* api = TEE_API();
    uint32_t debug_stage = api->debug_stage;
    uint32_t call_count = api->call_count;
    uint32_t magic = api->magic;

    /* Use early log (works before full init) */
    ESP_LOGE(TAG, "RTC FAST post-reset: magic=0x%08lx debug_stage=%lu call_count=%lu",
        (unsigned long)magic, (unsigned long)debug_stage, (unsigned long)call_count);

    /* Log the captured return_addr (stored by handler in reserved[0]) */
    ESP_LOGE(TAG, "Captured return_addr=0x%08lx  current return_addr=0x%08lx",
        (unsigned long)api->reserved[0], (unsigned long)api->return_addr);

    /* Also dump first words of handler area */
    uint32_t* handlers = (uint32_t*)TEE_HANDLERS_ADDR;
    ESP_LOGE(TAG, "Handlers @ 0x%08x: %08lx %08lx",
        TEE_HANDLERS_ADDR, (unsigned long)handlers[0], (unsigned long)handlers[1]);

    /* 1. Configure PMS while in secure world */
    tee_configure_pms();

    /* 2. Zero TEE private region */
    ESP_LOGI(TAG, "Initializing TEE memory at 0x%08x", TEE_PRIVATE_BASE);
    memset((void*)TEE_PRIVATE_BASE, 0, TEE_PRIVATE_SIZE);

    /* 3. Copy exit stub for W0->W1 transition */
    size_t exit_size = copy_to_rtc(
        (void*)TEE_EXIT_STUB,
        (const void*)tee_exit_stub_start,
        (const void*)tee_exit_stub_end,
        0x40  /* Max 64 bytes */
    );
    ESP_LOGI(TAG, "Exit stub copied to 0x%08x (%u bytes)",
        TEE_EXIT_STUB, (unsigned)exit_size);

    /* Copy return trampoline */
    size_t tramp_size = copy_to_rtc(
        (void*)TEE_RETURN_TRAMPOLINE,
        (const void*)tee_return_trampoline_start,
        (const void*)tee_return_trampoline_end,
        0x40  /* Max 64 bytes */
    );
    ESP_LOGI(TAG, "Trampoline copied to 0x%08x (%u bytes)",
        TEE_RETURN_TRAMPOLINE, (unsigned)tramp_size);

    sync_icache();

    /* 4. Configure WCL to switch to W1 when exit stub executes */
    REG_WRITE(WCL_CORE_0_WORLD_PREPARE_REG, 0x2);  /* WORLD_REE (W1) */
    REG_WRITE(WCL_CORE_0_WORLD_TRIGGER_ADDR_REG, TEE_EXIT_STUB);
    REG_WRITE(WCL_CORE_0_WORLD_UPDATE_REG, 1);
    __asm__ volatile("memw" ::: "memory");

    /* Store continuation function address in API (for exit stub to read) */
    uint32_t cont_addr = (uint32_t)continue_boot_in_w1;
    TEE_API()->return_addr = cont_addr;
    __asm__ volatile("memw" ::: "memory");

    ESP_LOGI(TAG, "Jumping to exit stub, continuation at 0x%08x", cont_addr);

    /* 5. Jump to exit stub */
    {
        uint32_t target = TEE_EXIT_STUB;
        __asm__ volatile(
            "jx %0\n"
            :
        : "r"(target)
            : "memory"
            );
    }

    /* Should never reach here */
    __builtin_unreachable();
}

/* Force linker to include this module (hooks are weak symbols) */
void bootloader_hooks_include(void) {}
