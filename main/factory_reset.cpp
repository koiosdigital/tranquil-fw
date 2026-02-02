#include "factory_reset.h"

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>

#include "esp_log.h"
#include "nvs_flash.h"

#include "kd_common.h"
#include "config_manager.h"
#include "ManifestDatabase.h"
#include "SandTablePlayer.h"

static const char* TAG = "factory_reset";

// Helper to recursively delete directory contents
static esp_err_t delete_directory_contents(const char* path) {
    DIR* dir = opendir(path);
    if (!dir) {
        // Directory doesn't exist, that's fine
        return ESP_OK;
    }

    struct dirent* entry;
    char filepath[348];

    while ((entry = readdir(dir)) != nullptr) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        snprintf(filepath, sizeof(filepath), "%s/%s", path, entry->d_name);

        struct stat st;
        if (stat(filepath, &st) == 0) {
            if (S_ISDIR(st.st_mode)) {
                // Recursively delete subdirectory
                delete_directory_contents(filepath);
                rmdir(filepath);
            }
            else {
                // Delete file
                if (unlink(filepath) != 0) {
                    ESP_LOGW(TAG, "Failed to delete %s", filepath);
                }
            }
        }
    }

    closedir(dir);
    return ESP_OK;
}

// Helper to erase an NVS namespace completely
static esp_err_t erase_nvs_namespace(const char* ns) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // Namespace doesn't exist, nothing to clear
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS namespace '%s': %s", ns, esp_err_to_name(err));
        return err;
    }

    err = nvs_erase_all(handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to erase NVS namespace '%s': %s", ns, esp_err_to_name(err));
    }

    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Cleared NVS namespace: %s", ns);
    return err;
}

esp_err_t tranquil_factory_reset() {
    ESP_LOGW(TAG, "=== FACTORY RESET STARTING ===");

    // Stop playback first
    SandTablePlayer::stop();

    // Close database to ensure clean state
    ManifestDatabase::instance().shutdown();

    // -------------------------------------------------------------------------
    // Clear NVS settings (WiFi, NTP, claim token, etc.)
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Clearing WiFi credentials...");
    kd_common_clear_wifi_credentials();

    ESP_LOGI(TAG, "Clearing claim token...");
    kd_common_clear_claim_token();

    ESP_LOGI(TAG, "Clearing NTP configuration...");
    erase_nvs_namespace("ntp_cfg2");

    ESP_LOGI(TAG, "Clearing PixelDriver configuration...");
    erase_nvs_namespace("pixdriver");

    ESP_LOGI(TAG, "Clearing schedule configuration...");
    erase_nvs_namespace("tranquil_sched");

    // -------------------------------------------------------------------------
    // Clear calibration and robot config via ConfigManager
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Clearing calibration data...");
    sand_table::ConfigManager::instance().clear_calibration();

    ESP_LOGI(TAG, "Resetting robot configuration to defaults...");
    sand_table::ConfigManager::instance().reset_to_defaults();

    // -------------------------------------------------------------------------
    // Clear SD card contents
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Deleting SD contents...");
    delete_directory_contents("/sd");

    ESP_LOGW(TAG, "=== FACTORY RESET COMPLETE ===");
    return ESP_OK;
}
