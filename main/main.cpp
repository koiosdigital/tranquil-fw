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
#include "esp_task_wdt.h"

#include "kd_common.h"
#include "kdc_heap_tracing.h"
#include "kd_pixdriver.h"

#include "sand_table.h"
#include "config_manager.h"
#include "console_commands.h"
#include "ManifestDatabase.h"
#include "SandTablePlayer.h"
#include "types.h"

#include "api.h"
#include "usb_pd.h"
#include "drm/drm_license.h"
#include "sockets.h"
#include "storage/jobs/job_queue.h"
#include "storage/jobs/job_processor.h"
#include "storage/jobs/conversion_executor.h"
#include "storage/jobs/thumbnail_executor.h"
#include "storage/jobs/download_executor.h"

#include "security/tee_client.h"

static const char* TAG = "main";

static sand_table::MotionController* g_motion_controller = nullptr;

static void print_world(const char* tag) {
    // Read WCL_CORE_0_World_IRam0_REG
    uint32_t wcl0_val = *(volatile uint32_t*)0x600D0000 + 0x150;
    uint32_t world0_bit = wcl0_val & 0x1;  // Extract bit 0

    uint32_t wcl1_val = *(volatile uint32_t*)0x600D0000 + 0x550;
    uint32_t world1_bit = wcl1_val & 0x1;  // Extract bit 0

    // Per TRM: bit 0 indicates IRAM world - 0=non-secure(W1), 1=secure(W0)
    // Note: W0=secure, W1=non-secure
    const char* world0_name = world0_bit ? "secure" : "non-secure";
    const char* world1_name = world1_bit ? "secure" : "non-secure";
    ESP_LOGE(TAG, "%s: core 0: %s, core 1: %s", tag, world0_name, world1_name);
}

extern "C" void app_main(void)
{
    print_world("app_main_start");

    // Dump what's at the hardcoded TEE_ENTRY_ADDR (0x600FE100)
    uint32_t* tee_ws_entry = (uint32_t*)0x600FE100;
    ESP_LOGI(TAG, "TEE world switch entry @ 0x600FE100:");
    ESP_LOGI(TAG, "  [0x00] %08x %08x %08x %08x",
        tee_ws_entry[0], tee_ws_entry[1], tee_ws_entry[2], tee_ws_entry[3]);

    // Initialize TEE and switch to World 1
    int tee_ret = tee_init();
    ESP_LOGI(TAG, "tee_init() returned: %d", tee_ret);

    vTaskDelay(pdMS_TO_TICKS(3000));

    if (tee_ret == TEE_OK) {
        // Test World 1 -> World 0 -> World 1 round-trip
        ESP_LOGI(TAG, "Testing TEE call...");
        int32_t test_result = tee_test_call(0x42);
        ESP_LOGI(TAG, "tee_test_call(0x42) returned: %ld", (long)test_result);
        ESP_LOGI(TAG, "TEE call_count: %lu", (unsigned long)tee_get_call_count());

        volatile tee_api_t* api = TEE_API();
        ESP_LOGI(TAG, "api->response: 0x%lx (expected 0x1042)",
            (unsigned long)api->response);
    }

    print_world("post_tee_call");

    vTaskSuspend(NULL);

    // Disable watchdogs for slow integrity checking
    esp_task_wdt_deinit();

    esp_event_loop_create_default();

    stusb_init();

    kd_common_set_provisioning_srp_password_format(PROVISIONING_SRP_FORMAT_STATIC);
    kd_common_init();

    // Initialize ConfigManager early (before components that need config)
    auto cfg_err = sand_table::ConfigManager::instance().init();
    if (cfg_err != ESP_OK) {
        ESP_LOGW(TAG, "ConfigManager init failed: %s, using defaults", esp_err_to_name(cfg_err));
    }

    ManifestDatabase::instance().initialize();
    ManifestDatabase::instance().selfTest();


    /*

    // Initialize job processing system
    jobs::JobQueue::instance().initialize();
    std::unordered_map<jobs::JobType, std::unique_ptr<jobs::IJobExecutor>> executors;
    executors[jobs::JobType::Conversion] = std::make_unique<jobs::ConversionExecutor>();
    executors[jobs::JobType::Thumbnail] = std::make_unique<jobs::ThumbnailExecutor>();
    executors[jobs::JobType::Download] = std::make_unique<jobs::DownloadExecutor>();
    jobs::JobProcessor::instance().init(std::move(executors));
    ESP_LOGI(TAG, "Job processing system initialized");

    */

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
        else {
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

        kd_common_api_register_handlers(PixelDriver::attach_api);
    }

    tranquil_api_init();
    //cloud_sockets_init();

    auto ret = g_motion_controller->home();
    if (ret.is_err()) {
        ESP_LOGE(TAG, "Failed to home motion controller");
    }
}
