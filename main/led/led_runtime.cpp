// LED strip runtime implementation
#include "led_runtime.h"
#include "led_state.h"
#include "kd_pixdriver.h"
#include "sdkconfig.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

static const char* TAG = "led_runtime";

namespace led_runtime {

namespace {

constexpr int32_t kCurrentLimitMa = 1750;

SemaphoreHandle_t s_mutex = nullptr;
int32_t s_channel_id = -1;
sand_table::RuntimeLEDConfig s_active;

// Map the NVS-persisted strip config (small int codes) to the driver enums.
// Values are validated on entry, but clamp defensively.
PixelFormat to_format(uint8_t code) {
    return (code >= 3 && code <= 5) ? static_cast<PixelFormat>(code) : PixelFormat::RGB;
}
LedICType to_ic(uint8_t code) {
    return (code <= 2) ? static_cast<LedICType>(code) : LedICType::WS2812;
}
ColorOrder to_order(uint8_t code) {
    return (code <= 5) ? static_cast<ColorOrder>(code) : ColorOrder::GRB;
}

bool hardware_equal(const sand_table::RuntimeLEDConfig& a, const sand_table::RuntimeLEDConfig& b) {
    return a.has_leds == b.has_leds && a.led_count == b.led_count &&
           a.ic_type == b.ic_type && a.format == b.format &&
           a.color_order == b.color_order && a.white_swap == b.white_swap;
}

void bring_up_locked(const sand_table::RuntimeLEDConfig& cfg) {
    if (!cfg.has_leds || cfg.led_count == 0) {
        ESP_LOGI(TAG, "LEDs disabled");
        return;
    }
    // LED pin stays in sdkconfig (hardware config); strip type/layout is
    // runtime config from NVS.
    s_channel_id = PixelDriver::addChannel(ChannelConfig(
        (gpio_num_t)CONFIG_LED_PIN, cfg.led_count, to_format(cfg.format), "",
        to_ic(cfg.ic_type), to_order(cfg.color_order), cfg.white_swap));
    if (s_channel_id < 0) {
        ESP_LOGE(TAG, "Failed to add LED channel");
        return;
    }
    PixelDriver::setCurrentLimit(kCurrentLimitMa);
    PixelDriver::start();
    ESP_LOGI(TAG, "LED strip up: %u px, ic=%u fmt=%u order=%u wswap=%d",
        cfg.led_count, cfg.ic_type, cfg.format, cfg.color_order, cfg.white_swap);
}

void tear_down_locked() {
    if (s_channel_id < 0) return;
    // Stop the driver task before touching the channel list: driverTask
    // iterates channels_ every tick. stop() deletes the task outright, so
    // give an in-flight transmit a moment to finish first.
    PixelDriver::stop();
    vTaskDelay(pdMS_TO_TICKS(50));
    PixelDriver::removeChannel(s_channel_id);
    s_channel_id = -1;
    ESP_LOGI(TAG, "LED strip torn down");
}

}  // namespace

void init() {
    if (!s_mutex) s_mutex = xSemaphoreCreateMutex();

    // Always initialise the driver (effect engine) so the LED API can answer
    // and LEDs can be enabled later without a reboot.
    PixelDriver::initialize(60);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_active = sand_table::ConfigManager::instance().led_config();
    bring_up_locked(s_active);
    xSemaphoreGive(s_mutex);
}

void apply(const sand_table::RuntimeLEDConfig& cfg) {
    if (!s_mutex) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (hardware_equal(cfg, s_active)) {
        xSemaphoreGive(s_mutex);
        return;
    }
    ESP_LOGI(TAG, "Applying new LED hardware config");
    tear_down_locked();
    s_active = cfg;
    bring_up_locked(s_active);
    xSemaphoreGive(s_mutex);

    // Channel state (and channel count) changed for every client.
    led_state::notify_changed();
}

}  // namespace led_runtime
