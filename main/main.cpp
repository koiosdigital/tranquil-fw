#include <stdio.h>
#include <string.h>
#include <cmath>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_heap_caps.h"

#include "esp_event.h"
#include "esp_log.h"

#include "kd_common.h"
#include "kd_pixdriver.h"

#include "sand_table.h"
#include "config_manager.h"
#include "console_commands.h"
#include "ManifestDatabase.h"
#include "SandTablePlayer.h"
#include "types.h"

#include "api.h"
#include "usb_pd.h"

static const char* TAG = "main";

static sand_table::MotionController* g_motion_controller = nullptr;

// Log current heap usage
static void log_heap_stats(const char* label) {
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    size_t external_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t external_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);

    ESP_LOGI(TAG, "HEAP [%s] Internal: %zu free (%zu largest) | External: %zu free (%zu largest)",
        label, internal_free, internal_largest, external_free, external_largest);
}

// Draw a circle at the given radius using 8 segments of pi/4 each
// direction: +1 for CCW, -1 for CW
static void draw_circle(double& current_theta, float rho, int direction, float feedrate) {
    constexpr double QUARTER_PI = M_PI / 4.0;
    sand_table::PolarPosition pos;
    pos.rho = rho;

    for (int i = 0; i < 8; i++) {
        current_theta += direction * QUARTER_PI;
        pos.theta = current_theta;
        (void)g_motion_controller->move_to(pos, feedrate);
    }
}

// Draw a triangle at the given radius
// Vertices are evenly spaced at 120 degrees apart
static void draw_triangle(double start_theta, float rho, float feedrate) {
    constexpr double TWO_PI_THIRDS = 2.0 * M_PI / 3.0;
    sand_table::PolarPosition pos;
    pos.rho = rho;

    // Draw three sides connecting vertices at 0, 120, 240 degrees from start
    for (int i = 0; i < 3; i++) {
        pos.theta = start_theta + (i + 1) * TWO_PI_THIRDS;
        (void)g_motion_controller->move_to(pos, feedrate);
    }
}

extern "C" void app_main(void)
{
    esp_event_loop_create_default();

    stusb_init();

    kd_common_set_provisioning_pop_token_format(ProvisioningPOPTokenFormat_t::NONE);
    kd_common_init();

    // Initialize ConfigManager early (before components that need config)
    auto cfg_err = sand_table::ConfigManager::instance().init();
    if (cfg_err != ESP_OK) {
        ESP_LOGW(TAG, "ConfigManager init failed: %s, using defaults", esp_err_to_name(cfg_err));
    }

    ManifestDatabase::instance().initialize();

    // Initialize motion controller
    bool motion_ok = false;
    g_motion_controller = new sand_table::MotionController();
    auto init_result = g_motion_controller->init();
    if (init_result.is_err()) {
        ESP_LOGE(TAG, "Failed to initialize motion controller");
    }
    else {
        auto start_result = g_motion_controller->start();
        if (start_result.is_err()) {
            ESP_LOGE(TAG, "Failed to start motion controller");
        }
        else {
            motion_ok = true;
            sand_table::console_init(g_motion_controller);
        }
    }

    SandTablePlayer::initialize(g_motion_controller);

    // Initialize LED driver using runtime config from NVS
    const auto& led_config = sand_table::ConfigManager::instance().led_config();

    if (led_config.has_leds && led_config.led_count > 0) {
        PixelDriver::initialize(60);

        PixelFormat format = led_config.is_rgbw ? PixelFormat::RGBW : PixelFormat::RGB;
        // LED pin stays in sdkconfig (hardware config)
        PixelDriver::addChannel(ChannelConfig((gpio_num_t)CONFIG_LED_PIN, led_config.led_count, format));
        PixelDriver::setCurrentLimit(1750);
        PixelDriver::start();

        ESP_LOGI(TAG, "LED driver initialized: %d LEDs, %s format",
            led_config.led_count, led_config.is_rgbw ? "RGBW" : "RGB");
    }
    else {
        ESP_LOGI(TAG, "LEDs disabled in configuration");
    }

    tranquil_api_init();

    g_motion_controller->home();

    while (!g_motion_controller->is_homed()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
