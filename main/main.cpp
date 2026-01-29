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
    // DIAGNOSTIC TEST PATTERN - Simplified to isolate motion issues
    // ==========================================================================
    // This test verifies each axis independently, then combined motion.
    // Watch the serial output to see if commanded steps match actual position.

    constexpr float feedrate = 10.0f;   // RPM (slow for testing)
    sand_table::PolarPosition pos;

    ESP_LOGI(TAG, "=== DIAGNOSTIC MOTION TEST ===");
    ESP_LOGI(TAG, "Feedrate: %.1f RPM", feedrate);
    ESP_LOGI(TAG, "Gear ratio: %.2f", sand_table::MechanicalConfig::THETA_GEAR_RATIO);
    ESP_LOGI(TAG, "Steps/theta_rot: %ld", theta_steps_per_rot);
    ESP_LOGI(TAG, "Rho max steps: %ld", rho_max_steps);

    // ========== TEST 1: RHO ONLY (no theta change) ==========
    // Move from center (0,0) to (0, 0.5) - pure radial move
    // Expected: theta motor = 0, rho motor = 0.5 * rho_max = 4800 steps
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 1: RHO ONLY (center to rho=0.5) ===");
    ESP_LOGI(TAG, "Expected: theta=0, rho=+4800 steps");
    pos.theta = 0.0;
    pos.rho = 0.5;
    (void)g_motion_controller->move_to(pos, feedrate);

    // Wait for completion
    while (g_motion_controller->is_moving() || g_motion_controller->queue_depth() > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(500));  // Extra settle time

    auto status1 = g_motion_controller->get_status();
    ESP_LOGI(TAG, "RESULT: theta=%ld, rho=%ld (expected: 0, 4800)",
        status1.theta.position_steps, status1.rho.position_steps);

    // ========== TEST 2: THETA ONLY (rho held constant) ==========
    // Move from (0, 0.5) to (π, 0.5) - 180° rotation at constant rho
    // Expected: theta motor += 12800 steps (half rotation)
    //           rho motor += 1600 steps (coupling compensation = 12800/8)
    // Final: theta=12800, rho=4800+1600=6400
    // Displayed rho should still be ~0.5 (6400 - 12800/8 = 4800 effective)
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 2: THETA ONLY (rotate 180° at rho=0.5) ===");
    ESP_LOGI(TAG, "Expected: theta=+12800 (total 12800), rho=+1600 (total 6400, coupling)");
    pos.theta = M_PI;
    pos.rho = 0.5;
    (void)g_motion_controller->move_to(pos, feedrate);

    while (g_motion_controller->is_moving() || g_motion_controller->queue_depth() > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    auto status2 = g_motion_controller->get_status();
    ESP_LOGI(TAG, "RESULT: theta=%ld, rho=%ld (expected: 12800, 6400)",
        status2.theta.position_steps, status2.rho.position_steps);
    ESP_LOGI(TAG, "Displayed rho: %.4f (expected: ~0.5)", status2.position.rho);

    // ========== TEST 3: BOTH AXES (combined motion) ==========
    // Move from (π, 0.5) to (2π, 0.8) - 180° more + rho increase
    // Expected: theta motor += 12800 steps (another half rotation, total 25600)
    //           rho motor: base = (0.8-0.5) * 9600 = 2880
    //                      coupling = 12800/8 = 1600
    //                      delta = 2880 + 1600 = 4480
    // Final: theta=25600, rho=6400+4480=10880
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 3: BOTH AXES (rotate 180° more + rho to 0.8) ===");
    ESP_LOGI(TAG, "Expected: theta=+12800 (total 25600), rho=+4480 (total 10880)");
    pos.theta = 2.0 * M_PI;
    pos.rho = 0.8;
    (void)g_motion_controller->move_to(pos, feedrate);

    while (g_motion_controller->is_moving() || g_motion_controller->queue_depth() > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    auto status3 = g_motion_controller->get_status();
    // Note: position overflow may have wrapped theta from 25600 to 0
    ESP_LOGI(TAG, "RESULT: theta=%ld, rho=%ld",
        status3.theta.position_steps, status3.rho.position_steps);
    ESP_LOGI(TAG, "Displayed: theta=%.4f rad, rho=%.4f (expected: ~0 or ~2π, 0.8)",
        status3.position.theta, status3.position.rho);

    // ========== TEST 4: REVERSE THETA (test negative direction) ==========
    // Move from current to (π, 0.8) - rotate backwards 180°
    // Expected: theta motor -= 12800 (or if wrapped, from 0 to -12800, then wrap to 12800)
    //           rho motor -= 1600 (coupling compensation for negative theta)
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 4: REVERSE THETA (rotate -180° at rho=0.8) ===");
    pos.theta = M_PI;
    pos.rho = 0.8;
    (void)g_motion_controller->move_to(pos, feedrate);

    while (g_motion_controller->is_moving() || g_motion_controller->queue_depth() > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    auto status4 = g_motion_controller->get_status();
    ESP_LOGI(TAG, "RESULT: theta=%ld, rho=%ld",
        status4.theta.position_steps, status4.rho.position_steps);
    ESP_LOGI(TAG, "Displayed: theta=%.4f rad (%.1f°), rho=%.4f",
        status4.position.theta, status4.position.theta * 180.0 / M_PI, status4.position.rho);

    // ========== TEST 5: RETURN TO CENTER ==========
    // Move to (π, 0) - retract to center (rho=0)
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 5: RETURN TO CENTER (rho to 0) ===");
    pos.theta = M_PI;  // Keep theta same
    pos.rho = 0.0;
    (void)g_motion_controller->move_to(pos, feedrate);

    // Wait for final move
    while (g_motion_controller->is_moving() || g_motion_controller->queue_depth() > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    auto status_final = g_motion_controller->get_status();

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "=== DIAGNOSTIC TEST COMPLETE ===");
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "Final motor position:");
    ESP_LOGI(TAG, "  Theta: %ld steps (%.2f rotations)",
        status_final.theta.position_steps,
        static_cast<float>(status_final.theta.position_steps) / theta_steps_per_rot);
    ESP_LOGI(TAG, "  Rho:   %ld steps", status_final.rho.position_steps);
    ESP_LOGI(TAG, "Final displayed position:");
    ESP_LOGI(TAG, "  Theta: %.4f rad (%.1f°)",
        status_final.position.theta, status_final.position.theta * 180.0 / M_PI);
    ESP_LOGI(TAG, "  Rho:   %.4f (expected: ~0.0)", status_final.position.rho);
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "INTERPRETATION:");
    ESP_LOGI(TAG, "- If rho drifts over time, check coupling compensation");
    ESP_LOGI(TAG, "- If theta doesn't match expected, check step counting");
    ESP_LOGI(TAG, "- If motors move wrong direction, check DIR pin polarity");
    ESP_LOGI(TAG, "- If motion is jerky/slow, check step rate calculation");
}