#include "websocket_server.h"
#include "message_dispatcher.h"
#include "raii_utils.hpp"
#include "kd_api.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <lwip/sockets.h>
#include <atomic>
#include <cstring>
#include <algorithm>

static const char* TAG = "ws_server";

// Maximum concurrent WebSocket clients (memory constrained)
static constexpr size_t MAX_CLIENTS = 4;

// Maximum incoming frame size. Inbound frames are small control/command
// protobufs; 8KB matches the documented WS frame contract (CLAUDE.md).
static constexpr size_t MAX_FRAME_SIZE = 1024 * 8;

// Outstanding async sends. httpd's work mailbox is 32 deep and, with
// CONFIG_HTTPD_QUEUE_WORK_BLOCKING, httpd_queue_work blocks its caller when
// the mailbox is full - so every sender (the esp_timer task included) is
// gated here first, below the mailbox depth, and drops the frame at the
// source when the gate cannot be taken within a short wait. The arg pool has
// exactly one slot per permit; nothing falls back to malloc.
static constexpr UBaseType_t MAX_INFLIGHT_SENDS = 24;
static constexpr TickType_t INFLIGHT_WAIT_TICKS = pdMS_TO_TICKS(50);

// Connected client socket file descriptors
static int connected_clients[MAX_CLIENTS] = { -1, -1, -1, -1 };
static SemaphoreHandle_t clients_mutex = nullptr;
static SemaphoreHandle_t inflight_sem = nullptr;
static httpd_handle_t ws_server = nullptr;
static std::atomic<uint32_t> s_dropped_frames{ 0 };

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
    // Block: this runs on the httpd task (session close / free_ctx) which
    // never holds clients_mutex itself, and every other holder is short.
    // A timeout here permanently leaked the slot; four leaks and no client
    // could connect until the next server restart.
    raii::MutexGuard guard(clients_mutex, portMAX_DELAY);
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

// True when fd is in the client table. On lock timeout the answer is "yes":
// the caller has already confirmed the fd is a live WS session with httpd.
static bool client_known(int fd) {
    raii::MutexGuard guard(clients_mutex, pdMS_TO_TICKS(100));
    if (!guard) {
        return true;
    }
    for (size_t i = 0; i < MAX_CLIENTS; i++) {
        if (connected_clients[i] == fd) {
            return true;
        }
    }
    return false;
}

// Session-close hook. httpd invokes free_ctx on EVERY close path — graceful
// close frame, abrupt disconnect, LRU purge, and httpd_stop() on WiFi loss —
// so this is the single reliable place to reclaim the client slot. The fd is
// smuggled in the ctx pointer (offset by +1 because httpd skips a NULL ctx).
static void ws_session_closed(void* ctx) {
    remove_client(static_cast<int>(reinterpret_cast<intptr_t>(ctx)) - 1);
}

// Read and discard the body of a frame we will not process. The header has
// already been consumed, so leaving the payload unread would desync every
// frame after it.
static esp_err_t drain_frame(httpd_req_t* req, httpd_ws_frame_t* pkt) {
    if (pkt->len == 0) return ESP_OK;
    if (pkt->len > MAX_FRAME_SIZE) {
        ESP_LOGE(TAG, "Frame too large to drain: %zu", pkt->len);
        return ESP_FAIL;
    }
    uint8_t* buf = static_cast<uint8_t*>(heap_caps_malloc(pkt->len, MALLOC_CAP_SPIRAM));
    if (!buf) return ESP_ERR_NO_MEM;
    pkt->payload = buf;
    esp_err_t ret = httpd_ws_recv_frame(req, pkt, pkt->len);
    free(buf);
    return ret;
}

// WebSocket handler
static esp_err_t ws_handler(httpd_req_t* req) {
    if (req->method == HTTP_GET) {
        // WebSocket handshake request
        int fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "WebSocket handshake request from fd=%d", fd);

        if (!add_client(fd)) {
            // Reject: httpd already sent the 101 upgrade before calling us,
            // so writing an HTTP error here would land inside the WS stream.
            // Failing the handler closes the socket cleanly instead.
            return ESP_FAIL;
        }

        // Per-socket send behaviour. httpd's default send timeout is 5 s and
        // every WS write runs synchronously on the httpd task, so one phone
        // that went to sleep with a full TCP window stalled every other
        // client for 5 s per frame. Fail fast instead, and let TCP keepalive
        // reap half-open sockets iOS leaves behind.
        struct timeval snd_to = { .tv_sec = 1, .tv_usec = 500000 };
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &snd_to, sizeof(snd_to));
        int ka = 1, ka_idle = 20, ka_intvl = 5, ka_cnt = 3;
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &ka_idle, sizeof(ka_idle));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &ka_intvl, sizeof(ka_intvl));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &ka_cnt, sizeof(ka_cnt));

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
        int fd = httpd_req_to_sockfd(req);
        remove_client(fd);
        // Complete the closing handshake: echo a close frame and tear the
        // session down. Browsers wait for the server's close before dropping
        // TCP; without it the socket lingered in one of the few slots.
        httpd_ws_frame_t close_frame = {
            .final = true, .fragmented = false, .type = HTTPD_WS_TYPE_CLOSE,
            .payload = nullptr, .len = 0
        };
        httpd_ws_send_frame(req, &close_frame);
        httpd_sess_trigger_close(req->handle, fd);
        return ESP_OK;
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_PONG) {
        // Unsolicited pongs (some clients use them as keepalive) carry no
        // protobuf; drain the payload and ignore. Control frames are capped
        // at 125 bytes by the WS spec.
        if (ws_pkt.len > 0) {
            uint8_t pong_body[126];
            if (ws_pkt.len > sizeof(pong_body)) {
                ESP_LOGE(TAG, "Pong payload too large: %zu", ws_pkt.len);
                return ESP_FAIL;
            }
            ws_pkt.payload = pong_body;
            ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to drain pong payload: %s", esp_err_to_name(ret));
                return ret;
            }
        }
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

    // Only a complete binary frame carries a TranquilMessage. Text frames
    // and fragmented messages are not part of the protocol: drain and
    // ignore them rather than feeding them to the protobuf decoder.
    if (ws_pkt.type != HTTPD_WS_TYPE_BINARY || !ws_pkt.final || ws_pkt.fragmented) {
        ESP_LOGW(TAG, "Ignoring unsupported frame from fd=%d (type=%d final=%d fragmented=%d len=%zu)",
            httpd_req_to_sockfd(req), static_cast<int>(ws_pkt.type),
            ws_pkt.final, ws_pkt.fragmented, ws_pkt.len);
        return drain_frame(req, &ws_pkt);
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

// Static pool for async send args, one slot per in-flight permit. Allocated
// from the httpd task, the esp_timer task (player state broadcasts), the
// api_events task and the cloudlink task, and freed from the httpd task —
// the bitmap read-modify-write must be atomic across tasks/cores.
static constexpr size_t ASYNC_ARG_POOL_SIZE = MAX_INFLIGHT_SENDS;
static_assert(ASYNC_ARG_POOL_SIZE <= 32, "pool bitmap is 32 bits");
static async_send_arg async_arg_pool[ASYNC_ARG_POOL_SIZE];
static uint32_t async_arg_used = 0;  // Bitmap, guarded by async_pool_lock
static portMUX_TYPE async_pool_lock = portMUX_INITIALIZER_UNLOCKED;

static async_send_arg* pool_alloc_arg() {
    portENTER_CRITICAL(&async_pool_lock);
    for (size_t i = 0; i < ASYNC_ARG_POOL_SIZE; i++) {
        if (!(async_arg_used & (1u << i))) {
            async_arg_used |= (1u << i);
            portEXIT_CRITICAL(&async_pool_lock);
            return &async_arg_pool[i];
        }
    }
    portEXIT_CRITICAL(&async_pool_lock);
    return nullptr;  // Never malloc: the caller drops the frame.
}

static void pool_free_arg(async_send_arg* arg) {
    if (arg >= &async_arg_pool[0] && arg < &async_arg_pool[ASYNC_ARG_POOL_SIZE]) {
        portENTER_CRITICAL(&async_pool_lock);
        async_arg_used &= ~(1u << (arg - &async_arg_pool[0]));
        portEXIT_CRITICAL(&async_pool_lock);
    }
}

// Work items queued to an httpd instance that was stopped (WiFi loss) never
// run: their slots and permits would leak. Called from websocket_server_init
// with clients_mutex held, after the old instance is fully gone.
static void reclaim_orphaned_sends() {
    portENTER_CRITICAL(&async_pool_lock);
    const uint32_t used = async_arg_used;
    async_arg_used = 0;
    portEXIT_CRITICAL(&async_pool_lock);

    unsigned reclaimed = 0;
    for (size_t i = 0; i < ASYNC_ARG_POOL_SIZE; i++) {
        if (used & (1u << i)) {
            free(async_arg_pool[i].data);
            async_arg_pool[i].data = nullptr;
            reclaimed++;
        }
    }
    for (unsigned i = 0; i < reclaimed; i++) {
        xSemaphoreGive(inflight_sem);
    }
    if (reclaimed) {
        ESP_LOGW(TAG, "Reclaimed %u sends orphaned by the previous httpd instance", reclaimed);
    }
}

static void async_send_worker(void* arg) {
    async_send_arg* a = static_cast<async_send_arg*>(arg);

    // The session may have closed - and the fd number been reused by a plain
    // HTTP request - between queueing and now. Only write to an fd httpd
    // still reports as a WS session and that is still in our client table.
    const bool live = httpd_ws_get_fd_info(a->hd, a->fd) == HTTPD_WS_CLIENT_WEBSOCKET &&
                      client_known(a->fd);
    if (live) {
        httpd_ws_frame_t pkt = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_BINARY,
            .payload = a->data,
            .len = a->len
        };

        esp_err_t ret = httpd_ws_send_frame_async(a->hd, a->fd, &pkt);
        if (ret != ESP_OK) {
            // A failed write means the peer is gone or wedged (send timeout).
            // Close the session now rather than paying the timeout again on
            // every subsequent broadcast; free_ctx reclaims the slot.
            ESP_LOGW(TAG, "Async send failed to fd=%d: %s - closing", a->fd, esp_err_to_name(ret));
            httpd_sess_trigger_close(a->hd, a->fd);
        }
        else {
            // httpd only credits LRU activity on RECEIVE, so a client that just
            // listens to broadcasts looks idle and is the first purged when HTTP
            // traffic (thumbnail grids) saturates the socket budget. Mark it live.
            httpd_sess_update_lru_counter(a->hd, a->fd);
        }
    }
    else {
        ESP_LOGD(TAG, "Skipping send to fd=%d: session gone", a->fd);
    }

    free(a->data);
    a->data = nullptr;
    pool_free_arg(a);
    xSemaphoreGive(inflight_sem);
}

// Runs just before kd_common stops the httpd server (WiFi loss), while the
// handle is still valid: forget it so no new work is queued to a dying
// instance. free_ctx clears the client table during httpd_stop() itself.
static void ws_server_stopping() {
    raii::MutexGuard guard(clients_mutex, portMAX_DELAY);
    ws_server = nullptr;
}

esp_err_t websocket_server_init(httpd_handle_t server) {
    // Called on every WiFi reconnect (kd_common restarts the httpd server) —
    // create the mutex and the in-flight gate once and keep them across
    // server generations.
    if (!clients_mutex) {
        clients_mutex = xSemaphoreCreateMutex();
        if (!clients_mutex) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    if (!inflight_sem) {
        inflight_sem = xSemaphoreCreateCounting(MAX_INFLIGHT_SENDS, MAX_INFLIGHT_SENDS);
        if (!inflight_sem) {
            ESP_LOGE(TAG, "Failed to create send gate");
            return ESP_ERR_NO_MEM;
        }
        kd_common_api_register_stop_hook(ws_server_stopping);
    }

    {
        raii::MutexGuard guard(clients_mutex, portMAX_DELAY);
        // Old sessions were closed with the previous server instance
        // (their free_ctx cleared the slots); reset defensively anyway.
        for (size_t i = 0; i < MAX_CLIENTS; i++) {
            connected_clients[i] = -1;
        }
        reclaim_orphaned_sends();
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
    if (fd < 0 || !data || len == 0) return ESP_ERR_INVALID_ARG;
    // Server not started yet (e.g. WiFi not connected, still provisioning)
    if (!clients_mutex || !inflight_sem) return ESP_ERR_INVALID_STATE;

    // Gate first, before clients_mutex, so a saturated send queue never
    // parks other senders on the mutex. A permit that cannot be had within
    // the short wait means the frame is dropped here, at the source.
    if (xSemaphoreTake(inflight_sem, INFLIGHT_WAIT_TICKS) != pdTRUE) {
        const uint32_t dropped = s_dropped_frames.fetch_add(1) + 1;
        ESP_LOGD(TAG, "Send queue saturated, dropping %zu-byte frame for fd=%d (%lu dropped)",
            len, fd, static_cast<unsigned long>(dropped));
        return ESP_ERR_TIMEOUT;
    }
    // Returned on every early exit below; cancelled once the worker owns it.
    raii::SemaphoreGiver permit(inflight_sem);

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
        // Cannot happen while permits and slots stay 1:1; treat as a drop.
        free(data_copy);
        s_dropped_frames.fetch_add(1);
        ESP_LOGW(TAG, "Async arg pool exhausted, dropping frame for fd=%d", fd);
        return ESP_ERR_NO_MEM;
    }

    arg->hd = ws_server;
    arg->fd = fd;
    arg->data = data_copy;
    arg->len = len;

    esp_err_t ret = httpd_queue_work(ws_server, async_send_worker, arg);
    if (ret != ESP_OK) {
        free(data_copy);
        arg->data = nullptr;
        pool_free_arg(arg);
        return ret;
    }

    permit.cancel();  // released by async_send_worker
    return ESP_OK;
}

esp_err_t websocket_broadcast_except(const uint8_t* data, size_t len, int exclude_fd) {
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
        if (fds[i] >= 0 && fds[i] != exclude_fd) {
            esp_err_t ret = websocket_send(fds[i], data, len);
            if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                last_err = ret;
            }
        }
    }

    return last_err;
}

esp_err_t websocket_broadcast(const uint8_t* data, size_t len) {
    return websocket_broadcast_except(data, len, -1);
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

size_t ws_connected_client_count() {
    return websocket_client_count();
}
