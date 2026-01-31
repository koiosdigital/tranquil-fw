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
#include "config_manager.h"
#include "ManifestManager.h"
#include "SandTablePlayer.h"
#include "types.h"

#include "api.h"
#include "usb_pd.h"

static const char* TAG = "main";

static sand_table::MotionController* g_motion_controller = nullptr;

// Wait for all queued motion to complete
static void wait_for_motion() {
    while (g_motion_controller->is_moving() || g_motion_controller->queue_depth() > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(50));  // Brief settle time
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
        wait_for_motion();
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
        wait_for_motion();
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
    } else {
        ESP_LOGI(TAG, "LEDs disabled in configuration");
    }

    tranquil_api_init();

    // Development: Skip homing, use estimated calibration values
    constexpr int32_t theta_steps_per_rot = sand_table::MechanicalConfig::STEPS_PER_THETA_ROTATION;
    constexpr int32_t rho_max_steps = 200 * 16 * 3;  // ~3 rotations of travel
    g_motion_controller->home();

    while (!g_motion_controller->is_homed()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "Homing complete");

    // ==========================================================================
    // DEMO PATTERN: 3 circles, 2 triangles, return to center
    // ==========================================================================

    constexpr float feedrate = 15.0f;  // RPM
    sand_table::PolarPosition pos;
    double current_theta = 0.0;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== DEMO PATTERN ===");
    ESP_LOGI(TAG, "3 circles, 2 triangles, return to center");
    ESP_LOGI(TAG, "Feedrate: %.1f RPM", feedrate);

    // --- CIRCLE 1: CCW at rho=0.3 ---
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Circle 1: CCW at rho=0.3");
    pos.theta = 0.0;
    pos.rho = 0.3f;
    (void)g_motion_controller->move_to(pos, feedrate);
    wait_for_motion();
    draw_circle(current_theta, 0.3f, +1, feedrate);

    // --- CIRCLE 2: CCW at rho=0.5 ---
    ESP_LOGI(TAG, "Circle 2: CCW at rho=0.5");
    pos.theta = current_theta;
    pos.rho = 0.5f;
    (void)g_motion_controller->move_to(pos, feedrate);
    wait_for_motion();
    draw_circle(current_theta, 0.5f, +1, feedrate);

    // --- CIRCLE 3: CCW at rho=0.7 ---
    ESP_LOGI(TAG, "Circle 3: CCW at rho=0.7");
    pos.theta = current_theta;
    pos.rho = 0.7f;
    (void)g_motion_controller->move_to(pos, feedrate);
    wait_for_motion();
    draw_circle(current_theta, 0.7f, +1, feedrate);

    // --- TRIANGLE 1: at rho=0.6 ---
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Triangle 1: at rho=0.6");
    pos.theta = current_theta;
    pos.rho = 0.6f;
    (void)g_motion_controller->move_to(pos, feedrate);
    wait_for_motion();
    draw_triangle(current_theta, 0.6f, feedrate);
    current_theta += 2.0 * M_PI;  // Triangle completes a full rotation

    // --- TRIANGLE 2: at rho=0.4 ---
    ESP_LOGI(TAG, "Triangle 2: at rho=0.4");
    pos.theta = current_theta;
    pos.rho = 0.4f;
    (void)g_motion_controller->move_to(pos, feedrate);
    wait_for_motion();
    draw_triangle(current_theta, 0.4f, feedrate);
    current_theta += 2.0 * M_PI;

    // --- RETURN TO CENTER ---
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Returning to center");
    pos.theta = current_theta;
    pos.rho = 0.0f;
    (void)g_motion_controller->move_to(pos, feedrate);
    wait_for_motion();

    // --- FINAL STATUS ---
    auto status = g_motion_controller->get_status();
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== DEMO COMPLETE ===");
    ESP_LOGI(TAG, "Final position: theta=%ld steps, rho=%ld steps",
        status.theta.position_steps, status.rho.position_steps);
    ESP_LOGI(TAG, "Polar: theta=%.4f rad, rho=%.4f",
        status.position.theta, status.position.rho);
}
