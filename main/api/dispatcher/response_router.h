// Response router for unified API
// Routes responses to appropriate destinations based on context
#pragma once

#include "message_context.h"
#include "handler_base.h"
#include <esp_err.h>

class ResponseRouter {
public:
    static ResponseRouter& instance();

    // Route a response based on the original request context and handle result
    esp_err_t route(
        const ResponseMessage& response,
        const MessageContext& ctx,
        const HandleResult& result
    );

    // Broadcast a message to all local WebSocket clients
    esp_err_t broadcastToLocal(const ResponseMessage& response);

    // Send a message to the cloud connection
    esp_err_t sendToCloud(const ResponseMessage& response);

    // Forward raw message data to cloud (for cloud forwarding)
    esp_err_t forwardToCloud(const uint8_t* data, size_t len);

private:
    ResponseRouter() = default;

    // Send to specific local WebSocket client
    esp_err_t sendToLocalClient(int fd, const ResponseMessage& response);

    // Send JSON response to REST client
    esp_err_t sendToRest(httpd_req_t* req, const ResponseMessage& response);
};
