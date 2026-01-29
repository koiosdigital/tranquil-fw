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

    // Development: Skip homing, use estimated calibration values
    // theta_steps_per_rot = EFFECTIVE_STEPS_PER_REV * THETA_GEAR_RATIO
    //                     = (200 * 16) * 4 = 12800 steps per rotation
    // rho_max_steps = estimated ~3 rotations of travel = 200 * 16 * 3 = 9600 steps
    constexpr int32_t theta_steps_per_rot = static_cast<int32_t>(
        sand_table::MechanicalConfig::EFFECTIVE_STEPS_PER_REV *
        sand_table::MechanicalConfig::THETA_GEAR_RATIO
        );
    constexpr int32_t rho_max_steps = 200 * 16 * 3;  // ~3 rotations of travel (estimate)
    g_motion_controller->_set_homed(theta_steps_per_rot, rho_max_steps);
    ESP_LOGI(TAG, "Development mode: Homing skipped (theta=%ld steps/rot, rho=%ld max)",
        theta_steps_per_rot, rho_max_steps);

    // Draw 3 squares using linear interpolation in Cartesian space
    // Square corners at 45°, 135°, 225°, 315°
    constexpr float feedrate = 25.0f;   // RPM

    ESP_LOGI(TAG, "Starting 3-square demo, feedrate=%.1f RPM", feedrate);

    sand_table::PolarPosition pos;

    // === Square 1: rho=0.4, forward direction (CCW) ===
    constexpr double rho1 = 0.4;
    ESP_LOGI(TAG, "Square 1: rho=%.2f, forward", rho1);

    pos.theta = M_PI / 4.0;
    pos.rho = rho1;
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 3.0 * M_PI / 4.0;
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 5.0 * M_PI / 4.0;
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 7.0 * M_PI / 4.0;
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = M_PI / 4.0;
    (void)g_motion_controller->move_to(pos, feedrate);

    // === Square 2: rho=0.2, reversed direction (CW) ===
    constexpr double rho2 = 0.2;
    ESP_LOGI(TAG, "Square 2: rho=%.2f, reversed", rho2);

    pos.theta = M_PI / 4.0;
    pos.rho = rho2;
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 7.0 * M_PI / 4.0;
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 5.0 * M_PI / 4.0;
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 3.0 * M_PI / 4.0;
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = M_PI / 4.0;
    (void)g_motion_controller->move_to(pos, feedrate);

    // === Square 3: rho=0.6, forward direction (CCW) ===
    constexpr double rho3 = 0.6;
    ESP_LOGI(TAG, "Square 3: rho=%.2f, forward", rho3);

    pos.theta = M_PI / 4.0;
    pos.rho = rho3;
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 3.0 * M_PI / 4.0;
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 5.0 * M_PI / 4.0;
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 7.0 * M_PI / 4.0;
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = M_PI / 4.0;
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.rho = 0;
    (void)g_motion_controller->move_to(pos, feedrate);

    ESP_LOGI(TAG, "All moves queued. Waiting for completion...");

    // Wait for all motions to complete
    while (g_motion_controller->get_state() == sand_table::SystemState::Running ||
        g_motion_controller->get_status().queue_depth > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "3-square demo complete!");
}