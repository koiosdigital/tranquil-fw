#include "sd.h"

static const char* TAG = "sdfs";
static const char mount_point[] = MOUNT_POINT;

sdmmc_card_t* card;

void init_sd() {
    ESP_LOGI(TAG, "Initializing SD card");
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED; // Set to maximum frequency supported by the host

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;

    slot_config.clk = (gpio_num_t)PIN_CLK;
    slot_config.cmd = (gpio_num_t)PIN_CMD;
    slot_config.d0 = (gpio_num_t)PIN_D0;
    slot_config.d1 = (gpio_num_t)PIN_D1;
    slot_config.d2 = (gpio_num_t)PIN_D2;
    slot_config.d3 = (gpio_num_t)PIN_D3;
    slot_config.cd = (gpio_num_t)PIN_CD;

    ESP_LOGI(TAG, "Mounting to %s", mount_point);
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem.");
        }
        else {
            ESP_LOGE(TAG, "Failed to initialize the card: %s", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "FS mounted: %s", mount_point);

    // Dump card information
    ESP_LOGI(TAG, "Card name: %s", card->cid.name);
    ESP_LOGI(TAG, "Card speed: %dkHz", card->real_freq_khz);
    ESP_LOGI(TAG, "Card size: %" PRIu64 "MB", ((uint64_t)card->csd.capacity) * card->csd.sector_size / (1024 * 1024));
}

void deinit_sd() {
    ESP_LOGI(TAG, "Unmounting SD card");
    esp_err_t ret = esp_vfs_fat_sdcard_unmount(mount_point, card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Unmount failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Unmounted successfully");
}
