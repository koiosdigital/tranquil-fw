// Response router implementation
#include "response_router.h"
#include "websocket_server.h"
#include "messages.h"

#include <esp_log.h>
#include <esp_heap_caps.h>

static const char* TAG = "response_router";

ResponseRouter& ResponseRouter::instance() {
    static ResponseRouter instance;
    return instance;
}

esp_err_t ResponseRouter::route(
    const ResponseMessage& response,
    const MessageContext& ctx,
    const HandleResult& result
) {
    if (!response.valid()) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    // Handle REST responses (special case - converted by REST adapter)
    if (ctx.source == MessageSource::REST_API) {
        // REST responses are handled by the REST adapter directly
        // This router doesn't handle REST serialization
        return ESP_OK;
    }

    // Broadcast to all local clients if requested
    if (result.broadcast) {
        esp_err_t bcast_ret = broadcastToLocal(response);
        if (bcast_ret != ESP_OK) {
            ESP_LOGW(TAG, "Broadcast failed: %s", esp_err_to_name(bcast_ret));
            ret = bcast_ret;
        }
    }
    // Otherwise send unicast to original requester
    else if (ctx.source == MessageSource::LOCAL_WEBSOCKET && ctx.socket_fd >= 0) {
        esp_err_t send_ret = sendToLocalClient(ctx.socket_fd, response);
        if (send_ret != ESP_OK) {
            ESP_LOGW(TAG, "Unicast to fd=%d failed: %s", ctx.socket_fd, esp_err_to_name(send_ret));
            ret = send_ret;
        }
    }
    // If from cloud, respond back to cloud
    else if (ctx.source == MessageSource::CLOUD_WEBSOCKET) {
        esp_err_t cloud_ret = sendToCloud(response);
        if (cloud_ret != ESP_OK) {
            ESP_LOGW(TAG, "Cloud response failed: %s", esp_err_to_name(cloud_ret));
            ret = cloud_ret;
        }
    }

    // Additionally send to cloud if requested (for state changes)
    if (result.send_to_cloud && ctx.source != MessageSource::CLOUD_WEBSOCKET) {
        esp_err_t cloud_ret = sendToCloud(response);
        if (cloud_ret != ESP_OK) {
            ESP_LOGW(TAG, "Cloud send failed: %s", esp_err_to_name(cloud_ret));
        }
    }

    return ret;
}

esp_err_t ResponseRouter::broadcastToLocal(const ResponseMessage& response) {
    if (!response.valid()) return ESP_ERR_INVALID_ARG;
    return websocket_broadcast(response.data(), response.len());
}

esp_err_t ResponseRouter::sendToCloud(const ResponseMessage& response) {
    if (!response.valid()) return ESP_ERR_INVALID_ARG;

    bool queued = cloud_msg_queue_raw(response.data(), response.len());

    return queued ? ESP_OK : ESP_FAIL;
}

esp_err_t ResponseRouter::forwardToCloud(const uint8_t* data, size_t len) {
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;

    bool queued = cloud_msg_queue_raw(data, len);

    return queued ? ESP_OK : ESP_FAIL;
}

esp_err_t ResponseRouter::sendToLocalClient(int fd, const ResponseMessage& response) {
    if (!response.valid() || fd < 0) return ESP_ERR_INVALID_ARG;
    return websocket_send(fd, response.data(), response.len());
}

esp_err_t ResponseRouter::sendToRest(httpd_req_t* req, const ResponseMessage& response) {
    (void)req;
    (void)response;
    // REST responses need JSON conversion, handled by REST adapter
    // This function is a placeholder for future JSON serialization
    return ESP_ERR_NOT_SUPPORTED;
}
