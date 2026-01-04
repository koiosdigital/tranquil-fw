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
    } else {
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

    vTaskSuspend(NULL);
}