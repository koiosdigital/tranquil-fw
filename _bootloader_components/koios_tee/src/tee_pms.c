/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

 /**
  * @file tee_pms.c
  * @brief Permission Management System configuration for TEE
  *
  * Configures the PMS peripheral to enforce memory access permissions
  * between secure (W0) and non-secure (W1) worlds.
  *
  * Called from bootloader while still in secure world (W0).
  */

#include "esp_log.h"
#include "tee_config.h"

static const char* TAG = "pms";

/**
 * @brief Configure Permission Management System
 *
 * Sets up memory region permissions:
 * - TEE private region (0x600FE000-0x600FF000): W0 only
 * - Shared region (0x600FF000-0x60100000): Both worlds
 * - Flash/IRAM regions: Based on app requirements
 *
 * Called from bootloader_after_init() while in W0.
 */
void tee_configure_pms(void)
{
    ESP_LOGI(TAG, "Configuring PMS (placeholder)");

    /*
     * TODO: Configure PMS registers here
     *
     * Example regions to configure:
     * - RTC FAST private: 0x600FE000-0x600FF000 (W0 only)
     * - RTC FAST shared:  0x600FF000-0x60100000 (both worlds)
     * - IRAM TEE code:    TBD (W0 only)
     * - DRAM TEE data:    TBD (W0 only)
     */
}
