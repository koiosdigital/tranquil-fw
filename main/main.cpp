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
#include "cJSON.h"

#include "sand_table.h"
#include "config_manager.h"
#include "console_commands.h"
#include "ManifestDatabase.h"
#include "SandTablePlayer.h"
#include "types.h"

#include "api.h"
#include "app_console.h"
#include "drm/drm_license.h"
#include "sockets.h"
#include "storage/jobs/job_queue.h"
#include "storage/jobs/job_processor.h"
#include "storage/jobs/conversion_executor.h"
#include "storage/jobs/thumbnail_executor.h"
#include "storage/jobs/download_executor.h"
#include "PatternDownloader.h"
#include "pattern_handler.h"
#include "download_progress.h"
#include "pipeline_progress.h"

#include <koios/ota.h>

static const char* TAG = "main";

static sand_table::MotionController* g_motion_controller = nullptr;

// Route every cJSON allocation to PSRAM. Parsed JSON trees (manifest records,
// REST bodies, job payloads) are never DMA'd, so keeping them off the scarce
// internal heap removes a recurring fragmentation source. heap_caps_free works
// for both PSRAM and the internal fallback, so mixed lifetimes are safe.
static void* cjson_psram_malloc(size_t sz)
{
    void* p = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM);
    return p ? p : malloc(sz);  // fall back to internal only if PSRAM is full
}

static void cjson_psram_free(void* p)
{
    heap_caps_free(p);
}

extern "C" void app_main(void)
{
    // Install the cJSON PSRAM allocator before any cJSON use anywhere.
    cJSON_Hooks cjson_hooks = { cjson_psram_malloc, cjson_psram_free };
    cJSON_InitHooks(&cjson_hooks);

    esp_event_loop_create_default();

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
        // A final failure anywhere in the download→conversion→thumbnail chain
        // ends the pipeline. Emit a terminal signal so clients clear their UI
        // instead of leaving a spinner stuck at the last percent.
        if (!result.success) {
            std::string uuid = job.pattern_external_uuid;
            if (uuid.empty() && job.pattern_id != 0) {
                auto pattern = ManifestDatabase::instance().getPattern(job.pattern_id);
                if (pattern) uuid = pattern->external_uuid;
            }
            if (!uuid.empty()) {
                switch (job.type) {
                case jobs::JobType::Download:
                    DownloadProgressBroadcaster::instance().fail(uuid, result.error);
                    break;
                case jobs::JobType::Conversion:
                    // No .dat produced — the pattern is unusable.
                    DownloadProgressBroadcaster::instance().fail(uuid, result.error);
                    pipeline_progress::conversion(uuid, "failed", 0, result.error);
                    break;
                case jobs::JobType::Thumbnail:
                    // The .dat exists and plays fine; only the (optional)
                    // preview failed, and it regenerates on demand. Finish the
                    // download bar rather than failing the whole download.
                    DownloadProgressBroadcaster::instance().complete(uuid);
                    pipeline_progress::thumb(uuid, "failed", 0, result.error);
                    break;
                }
            }
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

        // Map the NVS-persisted strip config (small int codes) to the driver
        // enums. Values are validated on entry, but clamp defensively.
        const uint8_t fmt_code = (led_config.format >= 3 && led_config.format <= 5)
            ? led_config.format : 3;
        const PixelFormat format = static_cast<PixelFormat>(fmt_code);
        const LedICType ic_type = (led_config.ic_type <= 2)
            ? static_cast<LedICType>(led_config.ic_type) : LedICType::WS2812;
        const ColorOrder color_order = (led_config.color_order <= 5)
            ? static_cast<ColorOrder>(led_config.color_order) : ColorOrder::GRB;

        // LED pin stays in sdkconfig (hardware config); strip type/layout is
        // runtime config from NVS (console: `led_config`, REST: /api/config).
        PixelDriver::addChannel(ChannelConfig(
            (gpio_num_t)CONFIG_LED_PIN, led_config.led_count, format, "",
            ic_type, color_order, led_config.white_swap));
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
