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

// How long the homing task waits for the player to reach STOPPED after
// asking it to stop (the service task drains the motion queue first).
constexpr TickType_t kStopWaitTicks = pdMS_TO_TICKS(5000);
constexpr TickType_t kStopPollTicks = pdMS_TO_TICKS(20);

// MotionController::home() refuses to run while playback is active. Stop
// the player here (on the homing task, so the request path never blocks)
// and wait until it reports STOPPED before seeking.
bool stop_player_and_wait() {
    if (SandTablePlayer::getPlaybackState() == PlaybackState::STOPPED) return true;

    ESP_LOGI(TAG, "Stopping playback before homing");
    SandTablePlayer::stop();

    const TickType_t deadline = xTaskGetTickCount() + kStopWaitTicks;
    while (SandTablePlayer::getPlaybackState() != PlaybackState::STOPPED) {
        if (static_cast<int32_t>(xTaskGetTickCount() - deadline) >= 0) return false;
        vTaskDelay(kStopPollTicks);
    }
    return true;
}

void homing_task(void* arg) {
    const bool force_full = reinterpret_cast<intptr_t>(arg) != 0;

    auto* mc = SandTablePlayer::getMotionController();
    if (!mc) {
        ESP_LOGE(TAG, "Motion controller not initialized");
    }
    else if (!stop_player_and_wait()) {
        ESP_LOGE(TAG, "Player did not stop; homing aborted");
    }
    else {
        auto result = mc->home(force_full);
        if (result.is_ok()) {
            ESP_LOGI(TAG, "Homing complete (force_full=%d)", force_full);
        }
        else {
            ESP_LOGE(TAG, "Homing failed (error=%d)", static_cast<int>(result.error()));
        }
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
        return ESP_FAIL;
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
