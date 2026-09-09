// Response router implementation
#include "response_router.h"
#include "websocket_server.h"
#include "messages.h"
#include "sockets.h"

#include <esp_log.h>
#include <esp_heap_caps.h>

static const char* TAG = "response_router";

ResponseRouter& ResponseRouter::instance() {
    static ResponseRouter instance;
    return instance;
}

// Routing rules:
//   * broadcast        -> every local WS client (the requester included when
//                         it is a local client)
//   * otherwise        -> unicast to the requesting local client
//   * cloud requester  -> ALWAYS answered on the cloud link, in addition to
//                         any local broadcast (a cloud command must get its
//                         CommandResult; a cloud SetSchedule must both notify
//                         LAN clients and confirm to the cloud)
//   * send_to_cloud    -> also pushed to the cloud (deduped against the above)
esp_err_t ResponseRouter::route(
    const ResponseMessage& response,
    const MessageContext& ctx,
    const HandleResult& result
) {
    if (!response.valid()) {
        return ESP_ERR_INVALID_ARG;
    }

    // REST responses are serialized by the REST adapter; nothing to route.
    if (ctx.source == MessageSource::REST_API) {
        return ESP_OK;
    }

    esp_err_t ret = ESP_OK;
    bool sent_to_cloud = false;

    if (result.broadcast) {
        esp_err_t bcast_ret = broadcastToLocal(response);
        if (bcast_ret != ESP_OK && bcast_ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Broadcast failed: %s", esp_err_to_name(bcast_ret));
            ret = bcast_ret;
        }
    }
    else if (ctx.source == MessageSource::LOCAL_WEBSOCKET && ctx.socket_fd >= 0) {
        esp_err_t send_ret = sendToLocalClient(ctx.socket_fd, response);
        if (send_ret != ESP_OK && send_ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Unicast to fd=%d failed: %s", ctx.socket_fd, esp_err_to_name(send_ret));
            ret = send_ret;
        }
    }

    if (ctx.source == MessageSource::CLOUD_WEBSOCKET) {
        esp_err_t cloud_ret = sendToCloud(response);
        sent_to_cloud = true;
        if (cloud_ret != ESP_OK) {
            ESP_LOGW(TAG, "Cloud response failed: %s", esp_err_to_name(cloud_ret));
            ret = cloud_ret;
        }
    }

    if (result.send_to_cloud && !sent_to_cloud) {
        esp_err_t cloud_ret = sendToCloud(response);
        if (cloud_ret != ESP_OK) {
            ESP_LOGD(TAG, "Cloud send skipped/failed: %s", esp_err_to_name(cloud_ret));
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
    return forwardToCloud(response.data(), response.len());
}

esp_err_t ResponseRouter::forwardToCloud(const uint8_t* data, size_t len) {
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;

    // Only hand messages to cloudlink while the session is actually up.
    // Queuing while offline just fills the outbox with telemetry that is
    // stale by the time the link comes back (the reconnect handshake then
    // re-syncs everything that matters).
    if (!cloud_sockets_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

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
    return ESP_ERR_NOT_SUPPORTED;
}
