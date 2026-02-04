/*
 * SPDX-FileCopyrightText: 2025 Koios
 * SPDX-License-Identifier: Apache-2.0
 */

 /**
  * @file tee_wcl.c
  * @brief World Controller configuration for ESP32-S3 TEE
  *
  * Configures the WCL hardware to:
  * - Monitor entry addresses for W1->W0 transitions (app calling TEE)
  * - Configure W0->W1 transitions (TEE returning to app)
  */

#include "tee_config.h"
#include "soc/world_controller_reg.h"
#include "soc/world_controller_struct.h"

  /**
   * @brief Configure World Controller entry points
   *
   * Sets up WCL to monitor the TEE entry address. When CPU (in World 1)
   * executes this address, it automatically switches to World 0.
   *
   * @param core The core to enable this switch for, should be the core that called TEE
   * @param address The address at which we switch from Non-Secure World to Secure World
   *
   * We operate in non-secure world by default. So, i
   */
void tee_wcl_configure(uint32_t core, uint32_t address)
{
    if (core == 0) {
        /* Configure WCL message address for data bus switch sequence (Core 0) */
        wcl_core_0_message_addr_reg_t core_0_message_addr_reg = {
            .core_0_message_addr = (uint32_t)&TEE_STATE()->wcl_msg_seq_core0
        };
        REG_WRITE(WCL_CORE_0_MESSAGE_ADDR_REG, core_0_message_addr_reg.val);

        /* Z=8: sequence 0,1,2,3,4,5,6,7,8 must be written to trigger data bus switch */
        wcl_core_0_message_max_reg_t wcl_core_0_message_max_reg = {
            .core_0_message_max = 8
        };
        REG_WRITE(WCL_CORE_0_MESSAGE_MAX_REG, wcl_core_0_message_max_reg.val);

        /* Configure entry point 1 for W1->W0 transition */
        wcl_core_0_entry_1_addr_reg_t core_0_entry_1_addr_reg = {
            .core_0_entry_1_addr = address
        };
        REG_WRITE(WCL_CORE_0_ENTRY_1_ADDR_REG, core_0_entry_1_addr_reg.val);

        /* Enable monitoring for entry 1 (bit 1) */
        wcl_core_0_entry_check_reg_t core_0_entry_check_reg = {
            .core_0_entry_check = (1 << 1)
        };
        REG_WRITE(WCL_CORE_0_ENTRY_CHECK_REG, core_0_entry_check_reg.val);

        /* Enable NMI mask (required for world switching per TRM 16.5.4.1) */
        wcl_core_0_nmi_mask_enable_reg_t core_0_nmi_mask_enable_reg = {
            .core_0_nmi_mask_enable = 1
        };
        REG_WRITE(WCL_CORE_0_NMI_MASK_ENABLE_REG, core_0_nmi_mask_enable_reg.val);
    }
    else {
        /* Configure WCL message address for data bus switch sequence (Core 1) */
        wcl_core_1_message_addr_reg_t core_1_message_addr_reg = {
            .core_1_message_addr = (uint32_t)&TEE_STATE()->wcl_msg_seq_core1
        };
        REG_WRITE(WCL_CORE_1_MESSAGE_ADDR_REG, core_1_message_addr_reg.val);
        /* Z=8: sequence 0,1,2,3,4,5,6,7,8 must be written to trigger data bus switch */
        wcl_core_1_message_max_reg_t wcl_core_1_message_max_reg = {
            .core_1_message_max = 8
        };
        REG_WRITE(WCL_CORE_1_MESSAGE_MAX_REG, wcl_core_1_message_max_reg.val);

        /* Configure entry point 1 for W1->W0 transition */
        wcl_core_1_entry_1_addr_reg_t core_1_entry_1_addr_reg = {
            .core_1_entry_1_addr = address
        };
        REG_WRITE(WCL_CORE_1_ENTRY_1_ADDR_REG, core_1_entry_1_addr_reg.val);

        /* Enable monitoring for entry 1 (bit 1) */
        wcl_core_1_entry_check_reg_t core_1_entry_check_reg = {
            .core_1_entry_check = (1 << 1)
        };
        REG_WRITE(WCL_CORE_1_ENTRY_CHECK_REG, core_1_entry_check_reg.val);

        /* Enable NMI mask (required for world switching per TRM 16.5.4.1) */
        wcl_core_1_nmi_mask_enable_reg_t core_1_nmi_mask_enable_reg = {
            .core_1_nmi_mask_enable = 1
        };
        REG_WRITE(WCL_CORE_1_NMI_MASK_ENABLE_REG, core_1_nmi_mask_enable_reg.val);
    }
}

/**
 * @brief Configure World Controller for W0->W1 return
 *
 * Sets up WCL to switch to World 1 when the CPU executes the
 * trampoline address. The actual return to the caller happens
 * via the trampoline code.
 *
 * @param return_addr The address in World 1 to return to after trampoline
 * @param core The core to enable this switch for, should be the core that called TEE
 *
 * @note This is called by service handlers before returning.
 *       The handler then jumps to TEE_RETURN_TRAMPOLINE, which
 *       triggers the world switch and jumps to return_addr.
 */
void tee_wcl_setup_world_return(uint32_t return_addr, uint32_t core)
{
    (void)return_addr;  /* Return addr is passed via shared memory/register, black magic! */

    if (core == 0) {
        /* Indicate that CPU needs to prepare to switch to World 1 */
        wcl_core_0_world_prepare_reg_t core_0_world_prepare_reg = {
            .core_0_world_prepare = 0x2
        };
        REG_WRITE(WCL_CORE_0_WORLD_PREPARE_REG, core_0_world_prepare_reg.val);

        /* Indicate that it will happen at the trampoline */
        wcl_core_0_world_trigger_addr_reg_t core_0_world_trigger_addr = {
            .core_0_world_trigger_addr = TEE_RETURN_TRAMPOLINE
        };
        REG_WRITE(WCL_CORE_0_WORLD_TRIGGER_ADDR_REG, core_0_world_trigger_addr.val);

        /* Indicate that the world configuration change is complete, and can now take affect */
        wcl_core_0_world_update_reg_t core_0_world_update = {
            .core_0_update = 1
        };
        REG_WRITE(WCL_CORE_0_WORLD_UPDATE_REG, core_0_world_update.val);
    }
    else {
        /* Indicate that CPU needs to prepare to switch to World 1 */
        wcl_core_1_world_prepare_reg_t core_1_world_prepare_reg = {
            .core_1_world_prepare = 0x2
        };
        REG_WRITE(WCL_CORE_1_WORLD_PREPARE_REG, core_1_world_prepare_reg.val);

        /* Indicate that it will happen at the trampoline */
        wcl_core_1_world_trigger_addr_reg_t core_1_world_trigger_addr = {
            .core_1_world_trigger_addr = TEE_RETURN_TRAMPOLINE
        };
        REG_WRITE(WCL_CORE_1_WORLD_TRIGGER_ADDR_REG, core_1_world_trigger_addr.val);

        /* Indicate that the world configuration change is complete, and can now take affect */
        wcl_core_1_world_update_reg_t core_1_world_update = {
            .core_1_update = 1
        };
        REG_WRITE(WCL_CORE_1_WORLD_UPDATE_REG, core_1_world_update.val);
    }

    /* Clear write_buffer before switching worlds, helps when op. is immediate */
    __asm__ volatile("memw" ::: "memory");
}

/**
 * @brief Clear CPU data bus write buffer before switching to Secure World
 *
 * Per ESP32-S3 TRM 16.5.4.1: To switch the CPU data bus from Non-Secure to Secure
 * World, write the sequence 0, 1, 2, ..., Z to the address configured in
 * WCL_CORE_m_MESSAGE_ADDR_REG. Z is configured via WCL_CORE_m_MESSAGE_MAX_REG (8).
 *
 * @param core CPU core number (0 or 1)
 */
void tee_wcl_clear_write_buffer(uint32_t core)
{
    volatile uint8_t* msg_addr;

    if (core == 0) {
        msg_addr = &TEE_STATE()->wcl_msg_seq_core0;
    }
    else {
        msg_addr = &TEE_STATE()->wcl_msg_seq_core1;
    }

    /* Suppress unused variable warning - function intentionally not fully implemented */
    (void)msg_addr;

    /* Write sequence 0-8 to trigger data bus switch to Non-secure World */

    /*
    __asm__ volatile(
        "movi    a8, 0\n"
        "s8i     a8, %0, 0\n"
        "movi    a8, 1\n"
        "s8i     a8, %0, 0\n"
        "movi    a8, 2\n"
        "s8i     a8, %0, 0\n"
        "movi    a8, 3\n"
        "s8i     a8, %0, 0\n"
        "movi    a8, 4\n"
        "s8i     a8, %0, 0\n"
        "movi    a8, 5\n"
        "s8i     a8, %0, 0\n"
        "movi    a8, 6\n"
        "s8i     a8, %0, 0\n"
        "movi    a8, 7\n"
        "s8i     a8, %0, 0\n"
        "movi    a8, 8\n"
        "s8i     a8, %0, 0\n"
        "memw\n"
        :
    : "a"(msg_addr)
        : "a8", "memory"
        );
    */

    //unused in practice
}
