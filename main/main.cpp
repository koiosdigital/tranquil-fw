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
#include "kd_api.h"
#include "kdc_heap_tracing.h"
#include "kd_pixdriver.h"

#include "sand_table.h"
#include "config_manager.h"
#include "console_commands.h"
#include "ManifestDatabase.h"
#include "SandTablePlayer.h"
#include "types.h"

#include "api.h"
#include "app_console.h"
#include "usb_pd.h"
#include "drm/drm_license.h"
#include "sockets.h"
#include "storage/jobs/job_queue.h"
#include "storage/jobs/job_processor.h"
#include "storage/jobs/conversion_executor.h"
#include "storage/jobs/thumbnail_executor.h"
#include "storage/jobs/download_executor.h"
#include "PatternDownloader.h"
#include "pattern_handler.h"

#include <koios/ota.h>

static const char* TAG = "main";

static sand_table::MotionController* g_motion_controller = nullptr;

extern "C" void app_main(void)
{
    esp_event_loop_create_default();

    stusb_init();

    kd_common_init();
    kd_common_set_device_info("tranquil", FIRMWARE_VARIANT);
    app_console_register_commands();

    // Cloud OTA (koios-sdk). Self-schedules once the cloudlink session is up
    // and the device JWT is available; NULL config uses the default OTA host.
    koios_ota_init(nullptr);

    // Initialize ConfigManager early (before components that need config)
    auto cfg_err = sand_table::ConfigManager::instance().init();
    if (cfg_err != ESP_OK) {
        ESP_LOGW(TAG, "ConfigManager init failed: %s, using defaults", esp_err_to_name(cfg_err));
    }

    ManifestDatabase::instance().initialize();
    drm_license_init();

    // Initialize job processing system
    jobs::JobQueue::instance().initialize();
    std::unordered_map<jobs::JobType, std::unique_ptr<jobs::IJobExecutor>> executors;
    executors[jobs::JobType::Conversion] = std::make_unique<jobs::ConversionExecutor>();
    executors[jobs::JobType::Thumbnail] = std::make_unique<jobs::ThumbnailExecutor>();
    executors[jobs::JobType::Download] = std::make_unique<jobs::DownloadExecutor>();
    jobs::JobProcessorConfig job_config;
    job_config.on_job_complete = [](const jobs::Job& job, const jobs::JobResult& result) {
        if (job.type == jobs::JobType::Download) {
            PatternDownloader::instance().notifyDownloadComplete(
                job.pattern_external_uuid, result.success, result.error);
            PatternHandler::instance().notifyPatternDownloadComplete(
                job.pattern_external_uuid, result.success);
        }
        };
    esp_err_t job_err = jobs::JobProcessor::instance().init(std::move(executors), job_config);
    if (job_err != ESP_OK) {
        ESP_LOGE(TAG, "JobProcessor init FAILED: %s - background jobs will not run",
            esp_err_to_name(job_err));
    }
    else {
        ESP_LOGI(TAG, "Job processing system initialized");
    }

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

        // Route registration through the kd_common wrapper so the CORS
        // pre-handler applies to the LED routes too.
        kd_common_api_register_handlers([](httpd_handle_t server) {
            PixelDriver::attach_api(server, kd_common_api_register_uri_handler);
            });
    }

    tranquil_api_init();
    cloud_sockets_init();

    auto ret = g_motion_controller->home();
    if (ret.is_err()) {
        ESP_LOGE(TAG, "Failed to home motion controller");
    }
}
