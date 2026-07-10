// Thin app layer over koios-sdk's cloudlink: tranquil-specific protobuf
// dispatch (DRM/license/purchase/pattern) and the device-initiated boot
// sequence. Connection lifecycle (backoff, welcome, device JWT, failure
// cascade, mTLS) lives in the SDK.

#include "sockets.h"
#include "handlers.h"
#include "messages.h"
#include "PatternDownloader.h"

#include <esp_log.h>
#include <esp_heap_caps.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <koios/cloudlink.h>
#include <kd/v1/tranquil.pb-c.h>

static const char* TAG = "cloud_sockets";

namespace {

    // Inbound cap. Coredumps and full purchase/pattern-list bundles can be
    // large; the old hand-rolled layer capped at 8 KB, which truncated them.
    // tranquil has PSRAM, so 32 KB is comfortable.
    constexpr size_t MAX_MSG_SIZE = 32 * 1024;

    // Protobuf unpacks land in PSRAM (tranquil has it) rather than internal RAM.
    void* spiram_alloc(void*, size_t size) {
        return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    }
    void spiram_free(void*, void* ptr) {
        heap_caps_free(ptr);
    }
    ProtobufCAllocator spiram_allocator = {
        .alloc = spiram_alloc,
        .free = spiram_free,
        .allocator_data = nullptr,
    };

    void on_message(const uint8_t* data, size_t len) {
        auto* msg = kd__v1__tranquil_message__unpack(&spiram_allocator, len, data);
        if (msg) {
            cloud_handle_message(msg);
            kd__v1__tranquil_message__free_unpacked(msg, &spiram_allocator);
        }
        else {
            ESP_LOGW(TAG, "Failed to unpack message (%zu bytes)", len);
        }
    }

    void on_session_ready() {
        // Device-initiated boot handshake. On the vn path the backend only
        // answers requests (it does not push an unsolicited JoinResponse), so
        // the device must drive: report identity, then request its license /
        // purchases / claim.
        handlers_on_connected();
    }

    void on_disconnect() {
        ESP_LOGI(TAG, "Cloud session lost");
    }

    koios_cloudlink_config_t cloudlink_cfg() {
        koios_cloudlink_config_t cfg = {};
        cfg.url = CLOUD_SOCKETS_URL;
        cfg.auth_mode = KOIOS_CLOUD_AUTH_MTLS;
        // Class (chip) and project (IDF project name) are reported by the SDK;
        // we only supply the build variant.
        cfg.variant = FIRMWARE_VARIANT;
        cfg.max_msg_size = MAX_MSG_SIZE;
        cfg.on_session_ready = on_session_ready;
        cfg.on_disconnect = on_disconnect;
        cfg.on_message = on_message;
        return cfg;
    }

    // Cert-cache invalidation: cloudlink fetches the device cert from
    // kp_identity once and caches it for the process lifetime. The minimal
    // correct way to present a renewed cert is a full deinit + re-init. That
    // can't run inline — sockets_invalidate_cert_cache() is called from the
    // cert-renew handler, which runs in cloudlink's on_message context (the WS
    // client's own event task); deinit there would destroy the client from
    // inside its own event handler. So spawn a short task, let the dispatch
    // unwind, then bounce the link.
    bool cert_reload_pending = false;

    void cert_reload_task(void*) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Reconnecting with renewed certificate");
        koios_cloudlink_deinit();
        koios_cloudlink_config_t cfg = cloudlink_cfg();
        koios_cloudlink_init(&cfg);

        cert_reload_pending = false;
        vTaskDelete(nullptr);
    }

}  // namespace

void cloud_sockets_init() {
    // Pattern downloads run through the job queue; the downloader is transport
    // agnostic (plain HTTPS to the store), it just needs to exist.
    PatternDownloader::instance().init();

    koios_cloudlink_config_t cfg = cloudlink_cfg();
    koios_cloudlink_init(&cfg);

    ESP_LOGI(TAG, "Cloud sockets initialized (cloudlink)");
}

void cloud_sockets_deinit() {
    PatternDownloader::instance().shutdown();
    koios_cloudlink_deinit();
}

bool cloud_sockets_is_connected() {
    return koios_cloudlink_is_connected();
}

void sockets_invalidate_cert_cache() {
    if (cert_reload_pending) return;
    cert_reload_pending = true;
    if (xTaskCreate(cert_reload_task, "cert_reload", 6144, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Failed to spawn cert reload task");
        cert_reload_pending = false;
    }
}
