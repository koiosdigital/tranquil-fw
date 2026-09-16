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
#include "drm/drm_purchase.h"
#include "sockets.h"
#include "storage/jobs/job_processor.h"

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

    // Quiesce every SD-card writer BEFORE the wipe below, or a job worker /
    // cloud handler keeps creating files under the delete (and a pattern the
    // player still has open would be unlinked out from under it — FATFS has
    // no file locking here).
    //  1. Player: releases the open pattern file.
    //  2. Cloud: drop inbound messages (license / receipt / download
    //     responses all write to the card) and refuse new download jobs.
    //     A soft quiesce, not a deinit — this may run inside cloudlink's own
    //     message callback (cloud-initiated reset), where tearing the client
    //     down would be use-after-free.
    //  3. Job processor: joins the running conversion/thumbnail/download
    //     workers (bounded wait) so nothing is mid-write.
    SandTablePlayer::stop();
    cloud_sockets_quiesce();
    jobs::JobProcessor::instance().stop();

    // Close database to ensure clean state
    ManifestDatabase::instance().shutdown();

    // The receipt files are about to disappear; drop the cached verdicts.
    drm_purchase_cache_clear();

    // -------------------------------------------------------------------------
    // Clear NVS settings (WiFi, NTP, claim token, etc.)
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Clearing WiFi credentials...");
    kd_common_clear_wifi_credentials();

    ESP_LOGI(TAG, "Clearing claim token...");
    kd_common_clear_claim_token();

    // kd_common owns the NTP namespace name; erasing a guessed name here
    // used to silently miss it.
    ESP_LOGI(TAG, "Clearing NTP configuration...");
    kd_common_ntp_reset();

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
