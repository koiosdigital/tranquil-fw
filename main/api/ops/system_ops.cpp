// System operations implementation
#include "system_ops.h"
#include "system_info.h"
#include "homing_service.h"
#include "SandTablePlayer.h"
#include "factory_reset.h"

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>

static const char* TAG = "system_ops";

namespace system_ops {

namespace {

void restart_timer_callback(void* arg) {
    (void)arg;
    ESP_LOGW(TAG, "Restarting device...");
    esp_restart();
}

}  // namespace

void schedule_restart(uint32_t delay_ms) {
    esp_timer_handle_t restart_timer;
    esp_timer_create_args_t timer_args = {
        .callback = restart_timer_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "restart",
    };
    if (esp_timer_create(&timer_args, &restart_timer) == ESP_OK) {
        esp_timer_start_once(restart_timer, static_cast<uint64_t>(delay_ms) * 1000);
        ESP_LOGI(TAG, "Device will restart in %lums", static_cast<unsigned long>(delay_ms));
    }
    else {
        ESP_LOGW(TAG, "Failed to create restart timer, restarting immediately");
        esp_restart();
    }
}

OpResult home(bool force_full_calibration) {
    ESP_LOGI(TAG, "Home request (force_full=%d)", force_full_calibration);
    esp_err_t err = homing_service::start(force_full_calibration);
    if (err == ESP_OK) return OpResult::success();
    if (err == ESP_ERR_INVALID_STATE) return OpResult::fail(err, "Homing already in progress");
    return OpResult::fail(err, "Failed to start homing");
}

OpResult reboot(const char* reason) {
    ESP_LOGW(TAG, "Reboot requested: %s", (reason && reason[0]) ? reason : "requested");
    // Stop motion first so the arm is not mid-move when power drops.
    SandTablePlayer::stop();
    schedule_restart(500);
    return OpResult::success();
}

OpResult factory_reset(const char* reason) {
    ESP_LOGW(TAG, "Factory reset requested: %s", (reason && reason[0]) ? reason : "no reason");
    esp_err_t err = tranquil_factory_reset();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Factory reset failed: %s", esp_err_to_name(err));
        return OpResult::fail(err, "Factory reset failed");
    }
    schedule_restart(300);
    return OpResult::success();
}

void info_to_json(api_common::ChunkBuf* cb) {
    using namespace api_common;
    system_info::Snapshot s;
    system_info::snapshot(s);

    chunk_buf_write_str(cb, "{\"firmware_version\":");
    chunk_buf_write_json_string(cb, s.firmware_version);
    chunk_buf_write_str(cb, ",\"hardware_model\":");
    chunk_buf_write_json_string(cb, s.hardware_model);
    chunk_buf_write_str(cb, ",\"device_id\":");
    chunk_buf_write_json_string(cb, s.device_id);
    chunk_buf_write_str(cb, ",\"hostname\":");
    chunk_buf_write_json_string(cb, s.hostname);
    chunk_buf_write_str(cb, ",\"ip_address\":");
    chunk_buf_write_json_string(cb, s.ip_address);
    chunk_buf_printf(cb, ",\"is_homed\":%s,\"is_homing\":%s,\"free_heap\":%lu,\"free_psram\":%lu,",
        s.is_homed ? "true" : "false", s.is_homing ? "true" : "false",
        (unsigned long)s.free_heap, (unsigned long)s.free_psram);
    chunk_buf_printf(cb, "\"uptime_s\":%lu,\"wifi_rssi\":%ld,\"quiet_hours_active\":%s,\"timezone\":",
        (unsigned long)s.uptime_s, (long)s.wifi_rssi, s.quiet_hours_active ? "true" : "false");
    chunk_buf_write_json_string(cb, s.timezone);
    chunk_buf_write(cb, "}", 1);
}

}  // namespace system_ops
