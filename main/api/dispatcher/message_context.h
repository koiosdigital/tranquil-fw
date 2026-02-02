// Message context for unified API dispatcher
// Tracks message origin for proper response routing
#pragma once

#include <cstdint>
#include <esp_http_server.h>

enum class MessageSource {
    LOCAL_WEBSOCKET,   // Local client via /ws
    CLOUD_WEBSOCKET,   // Cloud server connection
    REST_API,          // HTTP REST endpoint
    INTERNAL           // Internal/system generated
};

struct MessageContext {
    MessageSource source;

    // For LOCAL_WEBSOCKET: socket fd for unicast response
    int socket_fd;

    // For REST_API: httpd_req_t pointer
    httpd_req_t* http_req;

    // Request ID for response correlation (e.g., pattern downloads)
    uint32_t request_id;

    // True if response should only go to requester (not broadcast)
    bool unicast_only;

    // Factory methods for common contexts
    static MessageContext localWS(int fd) {
        MessageContext ctx{};
        ctx.source = MessageSource::LOCAL_WEBSOCKET;
        ctx.socket_fd = fd;
        ctx.http_req = nullptr;
        ctx.request_id = 0;
        ctx.unicast_only = false;
        return ctx;
    }

    static MessageContext cloud() {
        MessageContext ctx{};
        ctx.source = MessageSource::CLOUD_WEBSOCKET;
        ctx.socket_fd = -1;
        ctx.http_req = nullptr;
        ctx.request_id = 0;
        ctx.unicast_only = false;
        return ctx;
    }

    static MessageContext rest(httpd_req_t* req) {
        MessageContext ctx{};
        ctx.source = MessageSource::REST_API;
        ctx.socket_fd = -1;
        ctx.http_req = req;
        ctx.request_id = 0;
        ctx.unicast_only = true;  // REST always unicast
        return ctx;
    }

    static MessageContext internal() {
        MessageContext ctx{};
        ctx.source = MessageSource::INTERNAL;
        ctx.socket_fd = -1;
        ctx.http_req = nullptr;
        ctx.request_id = 0;
        ctx.unicast_only = false;
        return ctx;
    }

    bool isLocal() const {
        return source == MessageSource::LOCAL_WEBSOCKET || source == MessageSource::REST_API;
    }

    bool isCloud() const {
        return source == MessageSource::CLOUD_WEBSOCKET;
    }
};
