#include "websocket_server.h"
#include "message_dispatcher.h"
#include "raii_utils.hpp"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstring>
#include <algorithm>

static const char* TAG = "ws_server";

// Maximum concurrent WebSocket clients (memory constrained)
static constexpr size_t MAX_CLIENTS = 4;

// Maximum incoming frame size. Inbound frames are small control/command
// protobufs; 8KB matches the documented WS frame contract (CLAUDE.md).
static constexpr size_t MAX_FRAME_SIZE = 1024 * 8;

// Connected client socket file descriptors
static int connected_clients[MAX_CLIENTS] = { -1, -1, -1, -1 };
static SemaphoreHandle_t clients_mutex = nullptr;
static httpd_handle_t ws_server = nullptr;

// Add client to tracking list
static bool add_client(int fd) {
    raii::MutexGuard guard(clients_mutex, pdMS_TO_TICKS(100));
    if (!guard) {
        return false;
    }

    // A reconnecting client can reuse an fd number whose old session never
    // sent a close frame — dedupe so one fd never occupies two slots.
    for (size_t i = 0; i < MAX_CLIENTS; i++) {
        if (connected_clients[i] == fd) {
            return true;
        }
    }

    for (size_t i = 0; i < MAX_CLIENTS; i++) {
        if (connected_clients[i] == -1) {
            connected_clients[i] = fd;
            ESP_LOGI(TAG, "Client connected: fd=%d (slot %zu)", fd, i);
            return true;
        }
    }

    ESP_LOGW(TAG, "Max clients reached, rejecting fd=%d", fd);
    return false;
}

// Remove client from tracking list
static void remove_client(int fd) {
    // Long timeout: this also runs from httpd's session-close path (free_ctx),
    // and failing to take the mutex there would permanently leak the slot.
    raii::MutexGuard guard(clients_mutex, pdMS_TO_TICKS(1000));
    if (!guard) {
        return;
    }

    for (size_t i = 0; i < MAX_CLIENTS; i++) {
        if (connected_clients[i] == fd) {
            connected_clients[i] = -1;
            ESP_LOGI(TAG, "Client disconnected: fd=%d", fd);
            break;
        }
    }
}

// Session-close hook. httpd invokes free_ctx on EVERY close path — graceful
// close frame, abrupt disconnect, LRU purge, and httpd_stop() on WiFi loss —
// so this is the single reliable place to reclaim the client slot. The fd is
// smuggled in the ctx pointer (offset by +1 because httpd skips a NULL ctx).
static void ws_session_closed(void* ctx) {
    remove_client(static_cast<int>(reinterpret_cast<intptr_t>(ctx)) - 1);
}

// WebSocket handler
static esp_err_t ws_handler(httpd_req_t* req) {
    if (req->method == HTTP_GET) {
        // WebSocket handshake request
        int fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "WebSocket handshake request from fd=%d", fd);

        if (!add_client(fd)) {
            // Reject connection - too many clients
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Too many connections");
            return ESP_FAIL;
        }

        // Reclaim the slot on any session close (abrupt disconnect, LRU
        // purge, server stop) — a close frame is not guaranteed.
        req->sess_ctx = reinterpret_cast<void*>(static_cast<intptr_t>(fd + 1));
        req->free_ctx = ws_session_closed;

        return ESP_OK;
    }

    // This is a WebSocket frame
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;

    // Get frame info (payload length)
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get frame info: %s", esp_err_to_name(ret));
        return ret;
    }

    // Handle control frames
    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        ESP_LOGI(TAG, "Client sent close frame");
        remove_client(httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_PING) {
        // The initial recv only read frame info — ws_pkt.payload is NULL.
        // The ping body must be read before it can be echoed (and must be
        // drained regardless, or it desyncs subsequent frame parsing).
        // Control frames are capped at 125 bytes by the WS spec.
        uint8_t ping_body[126];
        if (ws_pkt.len > sizeof(ping_body) - 1) {
            ESP_LOGE(TAG, "Ping payload too large: %zu", ws_pkt.len);
            return ESP_FAIL;
        }
        if (ws_pkt.len > 0) {
            ws_pkt.payload = ping_body;
            ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to receive ping payload: %s", esp_err_to_name(ret));
                return ret;
            }
        }

        // Respond with pong echoing the ping payload
        httpd_ws_frame_t pong = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_PONG,
            .payload = ws_pkt.len > 0 ? ping_body : nullptr,
            .len = ws_pkt.len
        };
        return httpd_ws_send_frame(req, &pong);
    }

    if (ws_pkt.len == 0) {
        return ESP_OK;
    }

    if (ws_pkt.len > MAX_FRAME_SIZE) {
        ESP_LOGE(TAG, "Frame too large: %zu", ws_pkt.len);
        return ESP_FAIL;
    }

    // Allocate the inbound frame buffer in PSRAM — it is only memcpy'd into by
    // the socket recv and read by the protobuf decoder, never DMA'd, so it does
    // not need scarce internal RAM. (free() below is valid on SPIRAM pointers.)
    uint8_t* buf = static_cast<uint8_t*>(heap_caps_malloc(ws_pkt.len, MALLOC_CAP_SPIRAM));
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes", ws_pkt.len);
        return ESP_ERR_NO_MEM;
    }

    ws_pkt.payload = buf;
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive frame: %s", esp_err_to_name(ret));
        free(buf);
        return ret;
    }

    // Dispatch message through unified dispatcher
    // The dispatcher handles response routing via ResponseRouter
    int fd = httpd_req_to_sockfd(req);

    // Self-heal client tracking. add_client() runs once on the GET handshake,
    // but httpd's LRU/session teardown can fire free_ctx (-> remove_client) on
    // a socket that stays open — dropping it from connected_clients while it
    // keeps sending frames, so every broadcast reaches 0 targets. An inbound
    // frame proves this fd is a live WS client, so re-register it (idempotent).
    // Combined with the client keepalive ping this keeps broadcasts flowing.
    add_client(fd);

    MessageContext ctx = MessageContext::localWS(fd);

    ret = MessageDispatcher::instance().dispatch(buf, ws_pkt.len, ctx);
    free(buf);

    if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Message dispatch failed: %s", esp_err_to_name(ret));
    }

    return ESP_OK;
}

// Async send structure for broadcasting
struct async_send_arg {
    httpd_handle_t hd;
    int fd;
    uint8_t* data;
    size_t len;
};

// Pool for async send args (avoids heap fragmentation in hot path).
// Allocated from the httpd task, the esp_timer task (player state
// broadcasts) and the cloudlink task, and freed from the httpd task —
// the bitmap read-modify-write must be atomic across tasks/cores.
static constexpr size_t ASYNC_ARG_POOL_SIZE = 8;
static async_send_arg async_arg_pool[ASYNC_ARG_POOL_SIZE];
static uint8_t async_arg_used = 0;  // Bitmap, guarded by async_pool_lock
static portMUX_TYPE async_pool_lock = portMUX_INITIALIZER_UNLOCKED;

static async_send_arg* pool_alloc_arg() {
    portENTER_CRITICAL(&async_pool_lock);
    for (size_t i = 0; i < ASYNC_ARG_POOL_SIZE; i++) {
        if (!(async_arg_used & (1 << i))) {
            async_arg_used |= (1 << i);
            portEXIT_CRITICAL(&async_pool_lock);
            return &async_arg_pool[i];
        }
    }
    portEXIT_CRITICAL(&async_pool_lock);
    // Pool exhausted, fall back to malloc
    return static_cast<async_send_arg*>(malloc(sizeof(async_send_arg)));
}

static void pool_free_arg(async_send_arg* arg) {
    if (arg >= &async_arg_pool[0] && arg < &async_arg_pool[ASYNC_ARG_POOL_SIZE]) {
        portENTER_CRITICAL(&async_pool_lock);
        async_arg_used &= ~(1 << (arg - &async_arg_pool[0]));
        portEXIT_CRITICAL(&async_pool_lock);
        return;
    }
    free(arg);  // Was from fallback malloc
}

static void async_send_worker(void* arg) {
    async_send_arg* a = static_cast<async_send_arg*>(arg);

    httpd_ws_frame_t pkt = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_BINARY,
        .payload = a->data,
        .len = a->len
    };

    esp_err_t ret = httpd_ws_send_frame_async(a->hd, a->fd, &pkt);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Async send failed to fd=%d: %s", a->fd, esp_err_to_name(ret));
    }

    free(a->data);
    pool_free_arg(a);
}

esp_err_t websocket_server_init(httpd_handle_t server) {
    // Called on every WiFi reconnect (kd_common restarts the httpd server) —
    // create the mutex once and keep it across server generations.
    if (!clients_mutex) {
        clients_mutex = xSemaphoreCreateMutex();
        if (!clients_mutex) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    {
        raii::MutexGuard guard(clients_mutex, portMAX_DELAY);
        // Old sessions were closed with the previous server instance
        // (their free_ctx cleared the slots); reset defensively anyway.
        for (size_t i = 0; i < MAX_CLIENTS; i++) {
            connected_clients[i] = -1;
        }
        ws_server = server;
    }

    static httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = nullptr,
        .is_websocket = true,
        .handle_ws_control_frames = true
    };

    esp_err_t ret = httpd_register_uri_handler(server, &ws_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register /ws handler: %s", esp_err_to_name(ret));
        // Keep clients_mutex: it is shared across server restarts and may
        // be in use by a concurrent broadcast.
        return ret;
    }

    ESP_LOGI(TAG, "WebSocket server initialized at /ws");
    return ESP_OK;
}

esp_err_t websocket_send(int fd, const uint8_t* data, size_t len) {
    if (fd < 0) return ESP_ERR_INVALID_ARG;
    // Server not started yet (e.g. WiFi not connected, still provisioning)
    if (!clients_mutex) return ESP_ERR_INVALID_STATE;

    // Hold clients_mutex across the queue_work call. httpd invokes each WS
    // session's free_ctx (-> remove_client, which blocks on this mutex)
    // inside httpd_stop() BEFORE the server handle is freed — so as long as
    // fd is still registered here, ws_server is guaranteed alive. This is
    // what makes broadcasts safe against kd_common stopping the server on
    // WiFi loss while the state-broadcast timer keeps firing.
    raii::MutexGuard guard(clients_mutex, pdMS_TO_TICKS(100));
    if (!guard) {
        return ESP_ERR_TIMEOUT;
    }

    bool known = false;
    for (size_t i = 0; i < MAX_CLIENTS; i++) {
        if (connected_clients[i] == fd) {
            known = true;
            break;
        }
    }
    if (!known || !ws_server) {
        // Client already disconnected (or server stopped) — not an error
        // worth propagating; the data simply has no live recipient.
        return ESP_ERR_INVALID_STATE;
    }

    // Copy data for async send. One broadcast fans this out to up to
    // MAX_CLIENTS copies simultaneously; keep them in PSRAM (the payload is
    // only read by the socket write, never DMA'd) to spare internal RAM.
    uint8_t* data_copy = static_cast<uint8_t*>(heap_caps_malloc(len, MALLOC_CAP_SPIRAM));
    if (!data_copy) return ESP_ERR_NO_MEM;
    memcpy(data_copy, data, len);

    async_send_arg* arg = pool_alloc_arg();
    if (!arg) {
        free(data_copy);
        return ESP_ERR_NO_MEM;
    }

    arg->hd = ws_server;
    arg->fd = fd;
    arg->data = data_copy;
    arg->len = len;

    esp_err_t ret = httpd_queue_work(ws_server, async_send_worker, arg);
    if (ret != ESP_OK) {
        free(data_copy);
        pool_free_arg(arg);
    }

    return ret;
}

esp_err_t websocket_broadcast(const uint8_t* data, size_t len) {
    // Server not started yet (e.g. WiFi not connected, still provisioning)
    if (!clients_mutex) return ESP_ERR_INVALID_STATE;

    // Snapshot the fd list, then send without holding the mutex —
    // websocket_send takes it itself (non-recursive mutex) and re-validates
    // each fd, so a client removed between snapshot and send is skipped.
    int fds[MAX_CLIENTS];
    {
        raii::MutexGuard guard(clients_mutex, pdMS_TO_TICKS(100));
        if (!guard) {
            return ESP_ERR_TIMEOUT;
        }
        for (size_t i = 0; i < MAX_CLIENTS; i++) {
            fds[i] = connected_clients[i];
        }
    }

    esp_err_t last_err = ESP_OK;
    for (size_t i = 0; i < MAX_CLIENTS; i++) {
        if (fds[i] >= 0) {
            esp_err_t ret = websocket_send(fds[i], data, len);
            if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                last_err = ret;
            }
        }
    }

    return last_err;
}

size_t websocket_client_count() {
    if (!clients_mutex) return 0;

    raii::MutexGuard guard(clients_mutex, pdMS_TO_TICKS(100));
    if (!guard) {
        return 0;
    }

    size_t count = 0;
    for (size_t i = 0; i < MAX_CLIENTS; i++) {
        if (connected_clients[i] >= 0) {
            count++;
        }
    }

    return count;
}
