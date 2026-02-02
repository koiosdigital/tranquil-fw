#include "websocket_server.h"
#include "message_dispatcher.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstring>
#include <algorithm>

static const char* TAG = "ws_server";

// Maximum concurrent WebSocket clients (memory constrained)
static constexpr size_t MAX_CLIENTS = 4;

// Maximum incoming frame size (8KB sufficient for local API)
static constexpr size_t MAX_FRAME_SIZE = 8192;

// Connected client socket file descriptors
static int connected_clients[MAX_CLIENTS] = { -1, -1, -1, -1 };
static SemaphoreHandle_t clients_mutex = nullptr;
static httpd_handle_t ws_server = nullptr;

// Add client to tracking list
static bool add_client(int fd) {
    if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    for (size_t i = 0; i < MAX_CLIENTS; i++) {
        if (connected_clients[i] == -1) {
            connected_clients[i] = fd;
            ESP_LOGI(TAG, "Client connected: fd=%d (slot %zu)", fd, i);
            xSemaphoreGive(clients_mutex);
            return true;
        }
    }

    xSemaphoreGive(clients_mutex);
    ESP_LOGW(TAG, "Max clients reached, rejecting fd=%d", fd);
    return false;
}

// Remove client from tracking list
static void remove_client(int fd) {
    if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    for (size_t i = 0; i < MAX_CLIENTS; i++) {
        if (connected_clients[i] == fd) {
            connected_clients[i] = -1;
            ESP_LOGI(TAG, "Client disconnected: fd=%d", fd);
            break;
        }
    }

    xSemaphoreGive(clients_mutex);
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
        // Respond with pong
        httpd_ws_frame_t pong = {
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_PONG,
            .payload = ws_pkt.payload,
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

    // Allocate buffer for payload
    uint8_t* buf = static_cast<uint8_t*>(malloc(ws_pkt.len));
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

// Pool for async send args (avoids heap fragmentation in hot path)
static constexpr size_t ASYNC_ARG_POOL_SIZE = 8;
static async_send_arg async_arg_pool[ASYNC_ARG_POOL_SIZE];
static uint8_t async_arg_used = 0;  // Bitmap

static async_send_arg* pool_alloc_arg() {
    for (size_t i = 0; i < ASYNC_ARG_POOL_SIZE; i++) {
        if (!(async_arg_used & (1 << i))) {
            async_arg_used |= (1 << i);
            return &async_arg_pool[i];
        }
    }
    // Pool exhausted, fall back to malloc
    return static_cast<async_send_arg*>(malloc(sizeof(async_send_arg)));
}

static void pool_free_arg(async_send_arg* arg) {
    for (size_t i = 0; i < ASYNC_ARG_POOL_SIZE; i++) {
        if (&async_arg_pool[i] == arg) {
            async_arg_used &= ~(1 << i);
            return;
        }
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
    clients_mutex = xSemaphoreCreateMutex();
    if (!clients_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    ws_server = server;

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
        vSemaphoreDelete(clients_mutex);
        clients_mutex = nullptr;
        return ret;
    }

    ESP_LOGI(TAG, "WebSocket server initialized at /ws");
    return ESP_OK;
}

esp_err_t websocket_send(int fd, const uint8_t* data, size_t len) {
    if (!ws_server || fd < 0) return ESP_ERR_INVALID_ARG;

    // Copy data for async send
    uint8_t* data_copy = static_cast<uint8_t*>(malloc(len));
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
    if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t last_err = ESP_OK;
    for (size_t i = 0; i < MAX_CLIENTS; i++) {
        if (connected_clients[i] >= 0) {
            esp_err_t ret = websocket_send(connected_clients[i], data, len);
            if (ret != ESP_OK) {
                last_err = ret;
            }
        }
    }

    xSemaphoreGive(clients_mutex);
    return last_err;
}

size_t websocket_client_count() {
    if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return 0;
    }

    size_t count = 0;
    for (size_t i = 0; i < MAX_CLIENTS; i++) {
        if (connected_clients[i] >= 0) {
            count++;
        }
    }

    xSemaphoreGive(clients_mutex);
    return count;
}
