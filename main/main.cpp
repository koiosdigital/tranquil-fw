#include <stdio.h>
#include <string.h>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_event.h"
#include "esp_log.h"

#include "kd_common.h"
#include "kd_pixdriver.h"

#include "sand_table.h"
#include "ManifestManager.h"
#include "SandTablePlayer.h"
#include "types.h"

#include "api.h"
#include "usb_pd.h"

static const char* TAG = "main";

static sand_table::MotionController* g_motion_controller = nullptr;

extern "C" void app_main(void)
{
    //event loop
    esp_event_loop_create_default();

    stusb_init();

    //use protocomm security version 0
    kd_common_set_provisioning_pop_token_format(ProvisioningPOPTokenFormat_t::NONE);
    kd_common_init();

    ManifestManager::initialize();

    // Initialize motion controller
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
    }

    // Initialize player with motion controller
    SandTablePlayer::initialize(g_motion_controller);

    PixelDriver::initialize(60); // 60Hz update rate

#if defined(CONFIG_LED_TYPE_RGB) && CONFIG_LED_TYPE_RGB
    PixelFormat format = PixelFormat::RGB;
#else
    PixelFormat format = PixelFormat::RGBW;
#endif

    PixelDriver::addChannel(ChannelConfig((gpio_num_t)CONFIG_LED_PIN, CONFIG_LED_NUM_LEDS, format));
    PixelDriver::setCurrentLimit(1750); // Set current limit to 1750mA (1.75A)
    PixelDriver::start();

    api_init();

    // Home the motion system
    auto home_result = g_motion_controller->home();
    if (home_result.is_err()) {
        ESP_LOGE(TAG, "Homing failed");
    }
    else {
        ESP_LOGI(TAG, "Homing succeeded");
    }

    // Draw a square pattern at constant rho, 4 positions at 90° intervals
    constexpr float rho = 50.0f;       // mm (midpoint of 5-150mm range)
    constexpr float feedrate = 50.0f;  // mm/s

    ESP_LOGI(TAG, "Starting square demo at rho=%.1fmm, feedrate=%.1fmm/s", rho, feedrate);

    sand_table::PolarPosition pos;
    pos.rho = rho;

    // Queue all moves - they execute sequentially via the motion planner
    pos.theta = 0.0f;
    ESP_LOGI(TAG, "Queuing move to theta=%.1f°, rho=%.1fmm", pos.theta, pos.rho);
    g_motion_controller->move_to(pos, feedrate);

    pos.theta = 270.0f;
    ESP_LOGI(TAG, "Queuing move to theta=%.1f°, rho=%.1fmm", pos.theta, pos.rho);
    g_motion_controller->move_to(pos, feedrate);

    pos.theta = 180.0f;
    ESP_LOGI(TAG, "Queuing move to theta=%.1f°, rho=%.1fmm", pos.theta, pos.rho);
    g_motion_controller->move_to(pos, feedrate);

    pos.theta = 270.0f;
    ESP_LOGI(TAG, "Queuing move to theta=%.1f°, rho=%.1fmm", pos.theta, pos.rho);
    g_motion_controller->move_to(pos, feedrate);

    pos.theta = 360.0f;
    ESP_LOGI(TAG, "Queuing move to theta=%.1f°, rho=%.1fmm", pos.theta, pos.rho);
    g_motion_controller->move_to(pos, feedrate);

    ESP_LOGI(TAG, "All moves queued. Waiting for completion...");

    // Wait for all motions to complete
    while (g_motion_controller->get_state() == sand_table::SystemState::Running ||
        g_motion_controller->get_status().queue_depth > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "Square demo complete!");
}