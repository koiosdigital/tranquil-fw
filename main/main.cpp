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

    // Draw shapes using linear interpolation in Cartesian space
    // Test pattern: 3 squares, 2 triangles, 1 hexagon, return to center
    constexpr float feedrate = 25.0f;   // RPM

    ESP_LOGI(TAG, "Starting shape demo: 3 squares + 2 triangles + hexagon, feedrate=%.1f RPM", feedrate);

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

    // === Triangle 1: rho=0.35, CCW (vertices at 0°, 120°, 240°) ===
    constexpr double tri_rho1 = 0.35;
    ESP_LOGI(TAG, "Triangle 1: rho=%.2f, CCW", tri_rho1);

    pos.theta = 0.0;
    pos.rho = tri_rho1;
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 2.0 * M_PI / 3.0;  // 120°
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 4.0 * M_PI / 3.0;  // 240°
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 2.0 * M_PI;  // 360° (back to 0°, but accumulated)
    (void)g_motion_controller->move_to(pos, feedrate);

    // === Triangle 2: rho=0.55, CW (reversed) ===
    constexpr double tri_rho2 = 0.55;
    ESP_LOGI(TAG, "Triangle 2: rho=%.2f, CW", tri_rho2);

    pos.rho = tri_rho2;
    // Already at 360°, go CW: 360°→240°→120°→0°
    pos.theta = 4.0 * M_PI / 3.0;  // 240°
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 2.0 * M_PI / 3.0;  // 120°
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 0.0;  // 0°
    (void)g_motion_controller->move_to(pos, feedrate);

    // === Hexagon: rho=0.45, CCW (vertices at 0°, 60°, 120°, 180°, 240°, 300°) ===
    constexpr double hex_rho = 0.45;
    ESP_LOGI(TAG, "Hexagon: rho=%.2f, CCW", hex_rho);

    pos.rho = hex_rho;
    // Starting at 0°
    pos.theta = M_PI / 3.0;  // 60°
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 2.0 * M_PI / 3.0;  // 120°
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = M_PI;  // 180°
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 4.0 * M_PI / 3.0;  // 240°
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 5.0 * M_PI / 3.0;  // 300°
    (void)g_motion_controller->move_to(pos, feedrate);

    pos.theta = 2.0 * M_PI;  // 360° (back to 0°)
    (void)g_motion_controller->move_to(pos, feedrate);

    // === Return to center ===
    ESP_LOGI(TAG, "Returning to center");
    pos.rho = 0.0;
    (void)g_motion_controller->move_to(pos, feedrate);

    ESP_LOGI(TAG, "All moves queued. Waiting for completion...");

    // Wait for all motions to complete
    while (g_motion_controller->get_state() == sand_table::SystemState::Running ||
        g_motion_controller->get_status().queue_depth > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Get final status for position comparison
    auto status = g_motion_controller->get_status();

    // Expected final position:
    // - Rho = 0 (center)
    // - Theta accumulated through: 3 squares + 2 triangles + hexagon
    //   Net theta depends on CW vs CCW rotations and unwrapping
    //   Last explicit theta before center was 2π (hexagon endpoint)
    //
    // For drift detection, the key check is rho should be exactly 0,
    // and there should be no "Theta drift detected" warnings during execution.
    constexpr int32_t expected_rho_steps = 0;

    ESP_LOGI(TAG, "=== FINAL POSITION ===");
    ESP_LOGI(TAG, "Motor theta:  %ld steps (%.2f rotations)",
        status.theta.position_steps,
        static_cast<float>(status.theta.position_steps) / theta_steps_per_rot);
    ESP_LOGI(TAG, "Motor rho:    %ld steps (expected: %ld, error: %ld)",
        status.rho.position_steps, expected_rho_steps,
        status.rho.position_steps - expected_rho_steps);
    ESP_LOGI(TAG, "Tracked pos:  theta=%.4f rad (%.1f°), rho=%.4f",
        status.position.theta,
        status.position.theta * 180.0 / M_PI,
        status.position.rho);
    ESP_LOGI(TAG, "======================");

    ESP_LOGI(TAG, "Shape demo complete!");
}