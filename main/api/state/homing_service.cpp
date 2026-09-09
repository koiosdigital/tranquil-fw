// Asynchronous homing implementation
#include "homing_service.h"
#include "library_notifier.h"
#include "SandTablePlayer.h"
#include "motion_controller.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>

static const char* TAG = "homing_service";

namespace {

std::atomic<bool> s_running{ false };

void homing_task(void* arg) {
    const bool force_full = reinterpret_cast<intptr_t>(arg) != 0;

    auto* mc = SandTablePlayer::getMotionController();
    if (mc) {
        auto result = mc->home(force_full);
        if (result.is_ok()) {
            ESP_LOGI(TAG, "Homing complete (force_full=%d)", force_full);
        }
        else {
            ESP_LOGE(TAG, "Homing failed (error=%d)", static_cast<int>(result.error()));
        }
    }
    else {
        ESP_LOGE(TAG, "Motion controller not initialized");
    }

    s_running.store(false, std::memory_order_release);
    // Let everyone (LAN + cloud) see is_homed/is_homing flip.
    library_notifier::system_info_changed();
    vTaskDelete(nullptr);
}

}  // namespace

namespace homing_service {

esp_err_t start(bool force_full_calibration) {
    bool expected = false;
    if (!s_running.compare_exchange_strong(expected, true)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!SandTablePlayer::getMotionController()) {
        s_running.store(false);
        return ESP_ERR_INVALID_STATE;
    }

    // 6 KB: home_all() logs with floats and runs the seek loops.
    if (xTaskCreate(homing_task, "homing", 6144,
                    reinterpret_cast<void*>(static_cast<intptr_t>(force_full_calibration ? 1 : 0)),
                    5, nullptr) != pdPASS) {
        s_running.store(false);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Homing started (force_full=%d)", force_full_calibration);
    library_notifier::system_info_changed();
    return ESP_OK;
}

bool is_homing() {
    return s_running.load(std::memory_order_acquire);
}

}  // namespace homing_service
