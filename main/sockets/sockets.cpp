// Thin app layer over koios-sdk's cloudlink: tranquil-specific protobuf
// dispatch (DRM/license/purchase/pattern) and the device-initiated boot
// sequence. Connection lifecycle (backoff, welcome, device JWT, failure
// cascade, mTLS) lives in the SDK.

#include "sockets.h"
#include "handlers.h"
#include "messages.h"
#include "PatternDownloader.h"
#include "websocket_server.h"

#include <esp_log.h>
#include <esp_heap_caps.h>

#include <kd_common.h>
#include <kd_crypto.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <cstring>

#include <koios/cloudlink.h>
#include <kd/v1/tranquil.pb-c.h>

static const char* TAG = "cloud_sockets";

namespace {

    // Message size cap handed to cloudlink (cfg.max_msg_size): applies to
    // BOTH directions — the largest inbound frame it will unpack and the
    // largest outbound message koios_cloudlink_send() accepts. 32 KB:
    // coredumps and full purchase/pattern-list bundles exceed the 8 KB the
    // old hand-rolled layer allowed, and tranquil has PSRAM to spare.
    constexpr size_t MAX_MSG_SIZE = 32 * 1024;

    // Factory reset sets this; inbound messages are dropped from then on
    // (see cloud_sockets_quiesce).
    std::atomic<bool> quiesced{ false };

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
        if (quiesced.load(std::memory_order_acquire)) {
            ESP_LOGW(TAG, "Quiesced: dropping %zu-byte cloud message", len);
            return;
        }
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
        // Advertise the platform-assigned device UUID (from the welcome
        // frame) alongside the existing _koiosdigital service TXT records
        // (model/type/version).
        char device_id[48];
        if (koios_cloudlink_get_device_id(device_id, sizeof(device_id))) {
            kd_common_mdns_add_svc_record("_koiosdigital", "device_id", device_id);
        }

        // Device-initiated boot handshake. On the vn path the backend only
        // answers requests (it does not push an unsolicited JoinResponse), so
        // the device must drive: report identity, then request its license /
        // purchases / claim.
        handlers_on_connected();
    }

    void on_disconnect() {
        ESP_LOGI(TAG, "Cloud session lost");
    }

    // Seamless certificate renewal (mTLS). When the server sends a
    // cert.renew_required control frame, cloudlink calls on_cert_renew_csr:
    // regenerate the CSR from the DS peripheral key (non-destructive — the
    // private key never leaves the secure element) and hand back a malloc'd
    // PEM copy that cloudlink takes ownership of and frees. Return nullptr to
    // decline.
    char* on_cert_renew_csr() {
        esp_err_t err = kd_common_crypto_regenerate_csr();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "CSR regeneration failed: %s", esp_err_to_name(err));
            return nullptr;
        }

        size_t csr_len = 0;
        if (kd_common_get_csr(nullptr, &csr_len) != ESP_OK || csr_len == 0) {
            ESP_LOGE(TAG, "No CSR available after regeneration");
            return nullptr;
        }

        auto* csr = static_cast<char*>(malloc(csr_len + 1));
        if (!csr) return nullptr;

        if (kd_common_get_csr(csr, &csr_len) != ESP_OK) {
            free(csr);
            return nullptr;
        }
        csr[csr_len] = '\0';
        return csr;
    }

    // cloudlink hands back the signed cert via cert.renew_response. Store the
    // full chain (leaf + issuing intermediates), not just the leaf: edge mTLS
    // validates the device against the platform root, so the device must
    // present the intermediates in its TLS handshake to build a path to that
    // root. chain_pem is leaf-first and complete; fall back to the bare leaf
    // only if the server omitted it. After this returns, cloudlink clears its
    // cached TLS material and reconnects on its own — no need to call
    // sockets_invalidate_cert_cache() (that drives the unused protobuf path).
    void on_cert_renewed(const char* cert_pem, const char* chain_pem) {
        if (!cert_pem) return;

        const char* to_store = (chain_pem && chain_pem[0]) ? chain_pem : cert_pem;
        size_t len = strlen(to_store);

        esp_err_t err = kd_common_set_device_cert(to_store, len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to store renewed cert: %s", esp_err_to_name(err));
            return;
        }
        ESP_LOGI(TAG, "Renewed certificate stored (%zu bytes, %s)", len,
            to_store == chain_pem ? "fullchain" : "leaf only");
    }

    // Restart policy. cloudlink escalates persistent cloud failures all the
    // way to a device restart (socket -> network reset -> reboot). A table
    // that someone is driving over the LAN must not reboot under them just
    // because the cloud is unreachable; connected LAN WebSocket clients are
    // the cheap "in use" signal (the app holds one open while it is in the
    // foreground). Registered after every koios_cloudlink_init().
    bool may_restart(void*) {
        const size_t lan_clients = ws_connected_client_count();
        if (lan_clients > 0) {
            ESP_LOGW(TAG, "Cloud-failure restart deferred: %zu LAN WebSocket client(s) connected",
                lan_clients);
            return false;
        }
        return true;
    }

    void install_restart_policy() {
        koios_cloudlink_set_restart_policy(may_restart, nullptr);
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
        cfg.on_cert_renew_csr = on_cert_renew_csr;
        cfg.on_cert_renewed = on_cert_renewed;
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
    std::atomic<bool> cert_reload_pending{ false };

    void cert_reload_task(void*) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Reconnecting with renewed certificate");
        koios_cloudlink_deinit();
        koios_cloudlink_config_t cfg = cloudlink_cfg();
        koios_cloudlink_init(&cfg);
        install_restart_policy();

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
    install_restart_policy();

    ESP_LOGI(TAG, "Cloud sockets initialized (cloudlink)");
}

void cloud_sockets_deinit() {
    PatternDownloader::instance().shutdown();
    koios_cloudlink_deinit();
}

void cloud_sockets_quiesce() {
    quiesced.store(true, std::memory_order_release);
    // Refuses new download jobs (queueDownload -> "not initialized"); the
    // job processor itself is stopped by the caller.
    PatternDownloader::instance().shutdown();
    ESP_LOGW(TAG, "Cloud sockets quiesced: inbound messages dropped until reboot");
}

bool cloud_sockets_is_connected() {
    return koios_cloudlink_is_connected();
}

bool cloud_sockets_is_ready() {
    return koios_cloudlink_is_ready();
}

size_t cloud_sockets_max_msg_size() {
    return MAX_MSG_SIZE;
}

void sockets_invalidate_cert_cache() {
    // Atomic test-and-set: two renew handlers racing here must not spawn
    // two reload tasks (each would deinit/reinit cloudlink concurrently).
    if (cert_reload_pending.exchange(true)) return;
    if (xTaskCreate(cert_reload_task, "cert_reload", 6144, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Failed to spawn cert reload task");
        cert_reload_pending = false;
    }
}
