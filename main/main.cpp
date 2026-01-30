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
    // theta_steps_per_rot = STEPS_PER_THETA_ROTATION (integer constant, no float math)
    //                     = (200 * 16 * 800) / 100 = 25600 steps per rotation
    // rho_max_steps = estimated ~3 rotations of travel = 200 * 16 * 3 = 9600 steps
    constexpr int32_t theta_steps_per_rot = sand_table::MechanicalConfig::STEPS_PER_THETA_ROTATION;
    constexpr int32_t rho_max_steps = 200 * 16 * 3;  // ~3 rotations of travel (estimate)
    g_motion_controller->_set_homed(theta_steps_per_rot, rho_max_steps);
    ESP_LOGI(TAG, "Development mode: Homing skipped (theta=%ld steps/rot, rho=%ld max)",
        theta_steps_per_rot, rho_max_steps);

    // ==========================================================================
    // CIRCLE TEST PATTERN
    // ==========================================================================
    // Draws 3 circles CCW at different radii, then 3 circles CW, then returns to center.
    // Each circle is split into 8 segments of π/4 to avoid wrap-around issues.

    constexpr float feedrate = 15.0f;   // RPM
    constexpr double QUARTER_PI = M_PI / 4.0;
    sand_table::PolarPosition pos;

    ESP_LOGI(TAG, "=== CIRCLE TEST PATTERN ===");
    ESP_LOGI(TAG, "Feedrate: %.1f RPM", feedrate);
    ESP_LOGI(TAG, "Steps/theta_rot: %ld", theta_steps_per_rot);
    ESP_LOGI(TAG, "Rho max steps: %ld", rho_max_steps);

    // Helper lambda to wait for motion completion
    auto wait_for_motion = []() {
        while (g_motion_controller->is_moving() || g_motion_controller->queue_depth() > 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // Brief settle time
    };

    // Helper lambda to draw a circle in π/4 segments
    // direction: +1 for CCW, -1 for CW
    auto draw_circle = [&](double& current_theta, float rho, int direction) {
        pos.rho = rho;
        for (int i = 0; i < 8; i++) {
            current_theta += direction * QUARTER_PI;
            pos.theta = current_theta;
            (void)g_motion_controller->move_to(pos, feedrate);
            wait_for_motion();
        }
    };

    double current_theta = 0.0;

    // Move to starting radius
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Moving to starting position ===");
    pos.theta = 0.0;
    pos.rho = 0.3;
    (void)g_motion_controller->move_to(pos, feedrate);
    wait_for_motion();

    // ========== CIRCLE 1: CCW at rho=0.3 ==========
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== CIRCLE 1: CCW at rho=0.3 ===");
    draw_circle(current_theta, 0.3f, +1);

    auto status1 = g_motion_controller->get_status();
    ESP_LOGI(TAG, "After circle 1: theta=%ld steps, rho=%ld steps, displayed rho=%.3f",
        status1.theta.position_steps, status1.rho.position_steps, status1.position.rho);

    // ========== CIRCLE 2: CCW at rho=0.5 ==========
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== CIRCLE 2: CCW at rho=0.5 ===");
    pos.theta = current_theta;
    pos.rho = 0.5;
    (void)g_motion_controller->move_to(pos, feedrate);
    wait_for_motion();
    draw_circle(current_theta, 0.5f, +1);

    auto status2 = g_motion_controller->get_status();
    ESP_LOGI(TAG, "After circle 2: theta=%ld steps, rho=%ld steps, displayed rho=%.3f",
        status2.theta.position_steps, status2.rho.position_steps, status2.position.rho);

    // ========== CIRCLE 3: CCW at rho=0.7 ==========
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== CIRCLE 3: CCW at rho=0.7 ===");
    pos.theta = current_theta;
    pos.rho = 0.7;
    (void)g_motion_controller->move_to(pos, feedrate);
    wait_for_motion();
    draw_circle(current_theta, 0.7f, +1);

    auto status3 = g_motion_controller->get_status();
    ESP_LOGI(TAG, "After circle 3: theta=%ld steps, rho=%ld steps, displayed rho=%.3f",
        status3.theta.position_steps, status3.rho.position_steps, status3.position.rho);

    // ========== CIRCLE 4: CW at rho=0.6 ==========
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== CIRCLE 4: CW (reverse) at rho=0.6 ===");
    pos.theta = current_theta;
    pos.rho = 0.6;
    (void)g_motion_controller->move_to(pos, feedrate);
    wait_for_motion();
    draw_circle(current_theta, 0.6f, -1);

    auto status4 = g_motion_controller->get_status();
    ESP_LOGI(TAG, "After circle 4: theta=%ld steps, rho=%ld steps, displayed rho=%.3f",
        status4.theta.position_steps, status4.rho.position_steps, status4.position.rho);

    // ========== CIRCLE 5: CW at rho=0.4 ==========
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== CIRCLE 5: CW (reverse) at rho=0.4 ===");
    pos.theta = current_theta;
    pos.rho = 0.4;
    (void)g_motion_controller->move_to(pos, feedrate);
    wait_for_motion();
    draw_circle(current_theta, 0.4f, -1);

    auto status5 = g_motion_controller->get_status();
    ESP_LOGI(TAG, "After circle 5: theta=%ld steps, rho=%ld steps, displayed rho=%.3f",
        status5.theta.position_steps, status5.rho.position_steps, status5.position.rho);

    // ========== CIRCLE 6: CW at rho=0.2 ==========
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== CIRCLE 6: CW (reverse) at rho=0.2 ===");
    pos.theta = current_theta;
    pos.rho = 0.2;
    (void)g_motion_controller->move_to(pos, feedrate);
    wait_for_motion();
    draw_circle(current_theta, 0.2f, -1);

    auto status6 = g_motion_controller->get_status();
    ESP_LOGI(TAG, "After circle 6: theta=%ld steps, rho=%ld steps, displayed rho=%.3f",
        status6.theta.position_steps, status6.rho.position_steps, status6.position.rho);

    // ========== RETURN TO CENTER ==========
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== RETURNING TO CENTER ===");
    pos.theta = current_theta;
    pos.rho = 0.0;
    (void)g_motion_controller->move_to(pos, feedrate);
    wait_for_motion();

    auto status_final = g_motion_controller->get_status();

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "=== CIRCLE TEST COMPLETE ===");
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "Final motor position:");
    ESP_LOGI(TAG, "  Theta: %ld steps (%.2f rotations)",
        status_final.theta.position_steps,
        static_cast<float>(status_final.theta.position_steps) / theta_steps_per_rot);
    ESP_LOGI(TAG, "  Rho:   %ld steps", status_final.rho.position_steps);
    ESP_LOGI(TAG, "Final displayed position:");
    ESP_LOGI(TAG, "  Theta: %.4f rad (%.1f deg)",
        status_final.position.theta, status_final.position.theta * 180.0 / M_PI);
    ESP_LOGI(TAG, "  Rho:   %.4f (expected: 0.0)", status_final.position.rho);
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Net theta rotations: 3 CCW - 3 CW = 0 (should return to start)");
    ESP_LOGI(TAG, "Expected final theta: %.4f rad", current_theta);
    ESP_LOGI(TAG, "If rho drifted, coupling compensation may have issues.");
}