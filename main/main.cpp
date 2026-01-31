#include <stdio.h>
#include <string.h>
#include <cmath>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_event.h"
#include "esp_log.h"

#include "kd_common.h"
#include "kd_pixdriver.h"

#include "sand_table.h"
#include "config_manager.h"
#include "ManifestDatabase.h"
#include "SandTablePlayer.h"
#include "types.h"

#include "api.h"
#include "usb_pd.h"

static const char* TAG = "main";

static sand_table::MotionController* g_motion_controller = nullptr;

// SQLite stress test - performs CRUD operations and reports ops/sec
static void sqlite_stress_test() {
    constexpr int NUM_OPS = 100;
    auto& db = ManifestDatabase::instance();

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== SQLITE STRESS TEST ===");
    ESP_LOGI(TAG, "Operations per test: %d", NUM_OPS);

    std::vector<std::string> uuids;
    uuids.reserve(NUM_OPS);

    // CREATE test
    int64_t start = esp_timer_get_time();
    for (int i = 0; i < NUM_OPS; i++) {
        Pattern p;
        p.uuid = ManifestDatabase::generateUUID();
        p.name = "StressTest_" + std::to_string(i);
        p.creator = "test";
        p.date = ManifestDatabase::currentTimestamp();
        p.popularity = i;
        p.reversible = (i % 2 == 0);
        p.size_bytes = 1024 * (i + 1);
        db.addPattern(p);
        uuids.push_back(p.uuid);
    }
    int64_t create_us = esp_timer_get_time() - start;
    float create_ops = (NUM_OPS * 1000000.0f) / create_us;
    ESP_LOGI(TAG, "CREATE: %d ops in %lld us = %.1f ops/sec", NUM_OPS, create_us, create_ops);

    // READ test
    start = esp_timer_get_time();
    for (int i = 0; i < NUM_OPS; i++) {
        auto p = db.getPattern(uuids[i]);
        if (!p.has_value()) {
            ESP_LOGE(TAG, "READ failed for uuid %s", uuids[i].c_str());
        }
    }
    int64_t read_us = esp_timer_get_time() - start;
    float read_ops = (NUM_OPS * 1000000.0f) / read_us;
    ESP_LOGI(TAG, "READ:   %d ops in %lld us = %.1f ops/sec", NUM_OPS, read_us, read_ops);

    // UPDATE test
    start = esp_timer_get_time();
    for (int i = 0; i < NUM_OPS; i++) {
        Pattern p;
        p.uuid = uuids[i];
        p.name = "Updated_" + std::to_string(i);
        p.creator = "test_updated";
        p.date = ManifestDatabase::currentTimestamp();
        p.popularity = i * 10;
        p.reversible = (i % 2 != 0);
        p.size_bytes = 2048 * (i + 1);
        db.updatePattern(uuids[i], p);
    }
    int64_t update_us = esp_timer_get_time() - start;
    float update_ops = (NUM_OPS * 1000000.0f) / update_us;
    ESP_LOGI(TAG, "UPDATE: %d ops in %lld us = %.1f ops/sec", NUM_OPS, update_us, update_ops);

    // DELETE test
    start = esp_timer_get_time();
    for (int i = 0; i < NUM_OPS; i++) {
        db.deletePattern(uuids[i]);
    }
    int64_t delete_us = esp_timer_get_time() - start;
    float delete_ops = (NUM_OPS * 1000000.0f) / delete_us;
    ESP_LOGI(TAG, "DELETE: %d ops in %lld us = %.1f ops/sec", NUM_OPS, delete_us, delete_ops);

    // Report DB file size
    struct stat st;
    if (stat("/data/manifest.db", &st) == 0) {
        ESP_LOGI(TAG, "Database file size: %ld bytes", st.st_size);
    } else {
        ESP_LOGW(TAG, "Could not stat database file");
    }

    // Verify database is empty
    size_t remaining = db.getPatternCount();
    ESP_LOGI(TAG, "Patterns remaining after test: %zu", remaining);

    // Summary
    float total_us = create_us + read_us + update_us + delete_us;
    float avg_ops = (NUM_OPS * 4 * 1000000.0f) / total_us;
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "TOTAL: %d ops in %.1f ms = %.1f avg ops/sec", NUM_OPS * 4, total_us / 1000.0f, avg_ops);
    ESP_LOGI(TAG, "=== STRESS TEST COMPLETE ===");
    ESP_LOGI(TAG, "");
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

    // Stress test SQLite before motion operations
    sqlite_stress_test();

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
    }
    else {
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
    draw_circle(current_theta, 0.3f, +1, feedrate);

    // --- CIRCLE 2: CCW at rho=0.5 ---
    ESP_LOGI(TAG, "Circle 2: CCW at rho=0.5");
    pos.theta = current_theta;
    pos.rho = 0.5f;
    (void)g_motion_controller->move_to(pos, feedrate);
    draw_circle(current_theta, 0.5f, +1, feedrate);

    // --- CIRCLE 3: CCW at rho=0.7 ---
    ESP_LOGI(TAG, "Circle 3: CCW at rho=0.7");
    pos.theta = current_theta;
    pos.rho = 0.7f;
    (void)g_motion_controller->move_to(pos, feedrate);
    draw_circle(current_theta, 0.7f, +1, feedrate);

    // --- TRIANGLE 1: at rho=0.6 ---
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Triangle 1: at rho=0.6");
    pos.theta = current_theta;
    pos.rho = 0.6f;
    (void)g_motion_controller->move_to(pos, feedrate);
    draw_triangle(current_theta, 0.6f, feedrate);
    current_theta += 2.0 * M_PI;  // Triangle completes a full rotation

    // --- TRIANGLE 2: at rho=0.4 ---
    ESP_LOGI(TAG, "Triangle 2: at rho=0.4");
    pos.theta = current_theta;
    pos.rho = 0.4f;
    (void)g_motion_controller->move_to(pos, feedrate);
    draw_triangle(current_theta, 0.4f, feedrate);
    current_theta += 2.0 * M_PI;

    // --- RETURN TO CENTER ---
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Returning to center");
    pos.theta = current_theta;
    pos.rho = 0.0f;
    (void)g_motion_controller->move_to(pos, feedrate);

    // --- FINAL STATUS ---
    auto status = g_motion_controller->get_status();
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== DEMO COMPLETE ===");
    ESP_LOGI(TAG, "Final position: theta=%ld steps, rho=%ld steps",
        status.theta.position_steps, status.rho.position_steps);
    ESP_LOGI(TAG, "Polar: theta=%.4f rad, rho=%.4f",
        status.position.theta, status.position.rho);
}
