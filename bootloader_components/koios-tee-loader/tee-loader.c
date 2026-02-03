// bootloader_components/tee_loader/tee_loader.c

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_efuse.h"
#include "esp_secure_boot.h"
#include "esp_flash_partitions.h"
#include "esp_image_format.h"
#include "bootloader_flash_priv.h"
#include "bootloader_common.h"
#include "bootloader_utility.h"
#include "soc/sensitive_reg.h"
#include "soc/world_controller_reg.h"
#include "soc/rtc_cntl_reg.h"

static const char* TAG = "tee_loader";

// ════════════════════════════════════════════════════════════════════════════
// Memory Layout
// ════════════════════════════════════════════════════════════════════════════
#define TEE_IRAM_START      0x40370000
#define TEE_IRAM_SIZE       0x8000      // 32KB
#define TEE_DRAM_START      0x3FC88000  
#define TEE_DRAM_SIZE       0x8000      // 32KB
#define SHARED_BUF_ADDR     0x3FC90000
#define SHARED_BUF_SIZE     0x1000      // 4KB

#define RTC_BOOT_MODE_REG   RTC_CNTL_STORE0_REG
#define BOOT_MODE_PROV      0x50524F56
#define BOOT_MODE_SECURE    0x5345435F

// ════════════════════════════════════════════════════════════════════════════
// Provisioning Check
// ════════════════════════════════════════════════════════════════════════════
static bool is_provisioned(void)
{
    esp_efuse_purpose_t purpose;
    if (esp_efuse_get_key_purpose(EFUSE_BLK_KEY0, &purpose) != ESP_OK) {
        return false;
    }
    return (purpose == ESP_EFUSE_KEY_PURPOSE_HMAC_DOWN_DIGITAL_SIGNATURE);
}

// ════════════════════════════════════════════════════════════════════════════
// TEE Partition Loading with Signature Verification
// ════════════════════════════════════════════════════════════════════════════
static esp_err_t load_and_verify_tee(void)
{
    ESP_LOGI(TAG, "Loading TEE partition...");

    // Find TEE partition
    const esp_partition_pos_t* tee_part = NULL;
    esp_partition_info_t part_info;

    // Scan partition table for "tee" partition
    for (int i = 0; i < ESP_PARTITION_TABLE_MAX_ENTRIES; i++) {
        esp_err_t err = bootloader_flash_read(
            ESP_PARTITION_TABLE_OFFSET + i * sizeof(esp_partition_info_t),
            &part_info, sizeof(part_info), true);

        if (err != ESP_OK || part_info.magic != ESP_PARTITION_MAGIC) {
            break;
        }

        if (strcmp((char*)part_info.label, "tee") == 0) {
            tee_part = &part_info.pos;
            break;
        }
    }

    if (!tee_part) {
        ESP_LOGE(TAG, "TEE partition not found!");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "TEE partition: offset=0x%x, size=0x%x",
        tee_part->offset, tee_part->size);

    // ─────────────────────────────────────────────────────────────────────────
    // CRITICAL: Verify TEE signature using Secure Boot V2
    // ─────────────────────────────────────────────────────────────────────────
    esp_image_metadata_t tee_meta;
    esp_err_t err = esp_image_verify(
        ESP_IMAGE_VERIFY,           // Full verification including signature
        tee_part,
        &tee_meta
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TEE signature verification FAILED: %s", esp_err_to_name(err));
        return ESP_ERR_IMAGE_INVALID;
    }

    ESP_LOGI(TAG, "TEE signature verified OK");

    // ─────────────────────────────────────────────────────────────────────────
    // Copy TEE segments to protected memory
    // ─────────────────────────────────────────────────────────────────────────

    // TEE image has segments like any ESP-IDF app
    // Segment 0: .text → copy to TEE_IRAM_START
    // Segment 1: .data → copy to TEE_DRAM_START

    for (int i = 0; i < tee_meta.image.segment_count; i++) {
        esp_image_segment_header_t* seg = &tee_meta.segments[i];
        uint32_t load_addr = seg->load_addr;
        uint32_t data_len = seg->data_len;
        uint32_t src_offset = tee_meta.segment_data[i];  // Offset in flash

        ESP_LOGI(TAG, "TEE segment %d: load=0x%08x, len=%u", i, load_addr, data_len);

        // Validate load address is in TEE region
        bool valid = false;
        if (load_addr >= TEE_IRAM_START &&
            load_addr + data_len <= TEE_IRAM_START + TEE_IRAM_SIZE) {
            valid = true;
        }
        if (load_addr >= TEE_DRAM_START &&
            load_addr + data_len <= TEE_DRAM_START + TEE_DRAM_SIZE) {
            valid = true;
        }

        if (!valid) {
            ESP_LOGE(TAG, "TEE segment load addr 0x%08x outside TEE region!", load_addr);
            return ESP_ERR_INVALID_ARG;
        }

        // Copy from flash to RAM
        err = bootloader_flash_read(
            tee_part->offset + src_offset,
            (void*)load_addr,
            data_len,
            true  // Decrypt if flash encryption enabled
        );

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to load TEE segment %d", i);
            return err;
        }
    }

    ESP_LOGI(TAG, "TEE loaded to protected memory");
    return ESP_OK;
}

// ════════════════════════════════════════════════════════════════════════════
// PMS Configuration
// ════════════════════════════════════════════════════════════════════════════
static void configure_pms(void)
{
    ESP_LOGI(TAG, "Configuring PMS...");

    // ─────────────────────────────────────────────────────────────────────────
    // Peripheral PMS: Block DS/HMAC/eFuse from World 1
    // ─────────────────────────────────────────────────────────────────────────
    uint32_t val;

    // Core 0
    val = REG_READ(SENSITIVE_CORE_0_PIF_PMS_CONSTRAIN_4_REG);
    val = (val & ~0xF) | 0x3;  // DS: W0=RW, W1=NONE
    REG_WRITE(SENSITIVE_CORE_0_PIF_PMS_CONSTRAIN_4_REG, val);

    val = REG_READ(SENSITIVE_CORE_0_PIF_PMS_CONSTRAIN_5_REG);
    val = (val & ~0xF) | 0x3;  // HMAC: W0=RW, W1=NONE
    REG_WRITE(SENSITIVE_CORE_0_PIF_PMS_CONSTRAIN_5_REG, val);

    // Core 1 (same)
    val = REG_READ(SENSITIVE_CORE_1_PIF_PMS_CONSTRAIN_4_REG);
    val = (val & ~0xF) | 0x3;
    REG_WRITE(SENSITIVE_CORE_1_PIF_PMS_CONSTRAIN_4_REG, val);

    val = REG_READ(SENSITIVE_CORE_1_PIF_PMS_CONSTRAIN_5_REG);
    val = (val & ~0xF) | 0x3;
    REG_WRITE(SENSITIVE_CORE_1_PIF_PMS_CONSTRAIN_5_REG, val);

    // ─────────────────────────────────────────────────────────────────────────
    // Memory PMS: Protect TEE IRAM/DRAM from World 1
    // ─────────────────────────────────────────────────────────────────────────
    uint32_t tee_iram_end = (TEE_IRAM_START + TEE_IRAM_SIZE) >> 2;
    uint32_t tee_dram_end = (TEE_DRAM_START + TEE_DRAM_SIZE) >> 2;
    uint32_t shared_end = (SHARED_BUF_ADDR + SHARED_BUF_SIZE) >> 2;

    // IRAM split
    REG_WRITE(SENSITIVE_CORE_0_IRAM0_SRAM_LINE_0_SPLITADDR_REG, tee_iram_end);
    REG_WRITE(SENSITIVE_CORE_1_IRAM0_SRAM_LINE_0_SPLITADDR_REG, tee_iram_end);

    // IRAM permissions: region0(TEE)=W0:RX,W1:NONE  region1(app)=both:RX
    uint32_t iram_pms = (0x5 << 0) | (0x0 << 3) | (0x5 << 6) | (0x5 << 9);
    REG_WRITE(SENSITIVE_CORE_0_IRAM0_PMS_CONSTRAIN_2_REG, iram_pms);
    REG_WRITE(SENSITIVE_CORE_1_IRAM0_PMS_CONSTRAIN_2_REG, iram_pms);

    // DRAM splits
    REG_WRITE(SENSITIVE_CORE_0_DRAM0_SRAM_LINE_0_SPLITADDR_REG, tee_dram_end);
    REG_WRITE(SENSITIVE_CORE_0_DRAM0_SRAM_LINE_1_SPLITADDR_REG, shared_end);
    REG_WRITE(SENSITIVE_CORE_1_DRAM0_SRAM_LINE_0_SPLITADDR_REG, tee_dram_end);
    REG_WRITE(SENSITIVE_CORE_1_DRAM0_SRAM_LINE_1_SPLITADDR_REG, shared_end);

    // DRAM permissions: TEE=W0only, shared=both, app=both
    uint32_t dram_pms = (0x3 << 0) | (0x0 << 2) | (0x3 << 4) | (0x3 << 6) | (0x3 << 8) | (0x3 << 10);
    REG_WRITE(SENSITIVE_CORE_0_DRAM0_PMS_CONSTRAIN_1_REG, dram_pms);
    REG_WRITE(SENSITIVE_CORE_1_DRAM0_PMS_CONSTRAIN_1_REG, dram_pms);

    ESP_LOGI(TAG, "PMS configured");
}

static void lock_pms(void)
{
    ESP_LOGI(TAG, "Locking PMS (irreversible until reset)...");

    REG_SET_BIT(SENSITIVE_CORE_0_PIF_PMS_CONSTRAIN_0_REG, BIT(0));
    REG_SET_BIT(SENSITIVE_CORE_1_PIF_PMS_CONSTRAIN_0_REG, BIT(0));
    REG_SET_BIT(SENSITIVE_CORE_0_IRAM0_PMS_CONSTRAIN_0_REG, BIT(0));
    REG_SET_BIT(SENSITIVE_CORE_1_IRAM0_PMS_CONSTRAIN_0_REG, BIT(0));
    REG_SET_BIT(SENSITIVE_CORE_0_DRAM0_PMS_CONSTRAIN_0_REG, BIT(0));
    REG_SET_BIT(SENSITIVE_CORE_1_DRAM0_PMS_CONSTRAIN_0_REG, BIT(0));

    ESP_LOGI(TAG, "PMS LOCKED");
}

// ════════════════════════════════════════════════════════════════════════════
// World Controller Setup
// ════════════════════════════════════════════════════════════════════════════
static void configure_world_controller(void)
{
    // TEE entry point is at start of TEE IRAM
    uint32_t tee_entry = TEE_IRAM_START;

    // Configure World 0 → World 1 switch trigger
    // When CPU executes from app entry point, switch to World 1

    // For now, we start in World 0 (bootloader/TEE context)
    // App will be launched into World 1

    REG_WRITE(WORLD_CONTROLLER_WCL_CORE_0_WORLD_PREPARE_REG, 0x2);  // Target = World 1
    REG_WRITE(WORLD_CONTROLLER_WCL_CORE_0_WORLD_UPDATE_REG, 0x1);

    REG_WRITE(WORLD_CONTROLLER_WCL_CORE_1_WORLD_PREPARE_REG, 0x2);
    REG_WRITE(WORLD_CONTROLLER_WCL_CORE_1_WORLD_UPDATE_REG, 0x1);

    ESP_LOGI(TAG, "World Controller: App will run in World 1");
}

// ════════════════════════════════════════════════════════════════════════════
// Bootloader Hooks
// ════════════════════════════════════════════════════════════════════════════

void bootloader_before_init(void)
{
    // Nothing here
}

void bootloader_after_init(void)
{
    ESP_LOGI(TAG, "════════════════════════════════════════════");
    ESP_LOGI(TAG, "  Koios TEE Loader v1.0");
    ESP_LOGI(TAG, "════════════════════════════════════════════");

    // ─────────────────────────────────────────────────────────────────────────
    // Check provisioning state
    // ─────────────────────────────────────────────────────────────────────────
    if (!is_provisioned()) {
        ESP_LOGW(TAG, "╔═══════════════════════════════════════╗");
        ESP_LOGW(TAG, "║     PROVISIONING MODE                 ║");
        ESP_LOGW(TAG, "║     PMS disabled, full DS access      ║");
        ESP_LOGW(TAG, "╚═══════════════════════════════════════╝");

        REG_WRITE(RTC_BOOT_MODE_REG, BOOT_MODE_PROV);
        return;  // Boot normally, no TEE
    }

    // ─────────────────────────────────────────────────────────────────────────
    // SECURE MODE: Load and verify TEE
    // ─────────────────────────────────────────────────────────────────────────
    ESP_LOGI(TAG, "SECURE MODE - Loading TEE...");

    esp_err_t err = load_and_verify_tee();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TEE load failed! Halting.");
        while (1) {}  // Security halt - don't boot without valid TEE
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Configure isolation
    // ─────────────────────────────────────────────────────────────────────────
    configure_pms();
    configure_world_controller();
    lock_pms();

    REG_WRITE(RTC_BOOT_MODE_REG, BOOT_MODE_SECURE);

    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║     SECURE MODE ACTIVE                ║");
    ESP_LOGI(TAG, "║     TEE loaded, PMS locked            ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");

    // Bootloader continues to load app...
}