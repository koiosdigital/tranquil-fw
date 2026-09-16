// Message context for the protobuf dispatcher: where a message came from
// and where its reply goes. REST never enters the dispatcher (REST handlers
// call the ops core directly), so there is no REST source.
#pragma once

#include <cstdint>

enum class MessageSource {
    LOCAL_WEBSOCKET,   // Local client via /ws
    CLOUD_WEBSOCKET,   // Cloud server connection
    INTERNAL           // Internal/system generated
};

struct MessageContext {
    MessageSource source = MessageSource::INTERNAL;

    // For LOCAL_WEBSOCKET: socket fd for unicast replies
    int socket_fd = -1;

    // The sender's correlation id (TranquilMessage.request_id), or nullptr.
    // Filled centrally by MessageDispatcher from the request envelope and
    // valid only for the duration of that dispatch. Every reply built through
    // HandlerBase::makeCommandResult() / serialize() echoes it.
    const char* request_id = nullptr;

    static MessageContext localWS(int fd) {
        MessageContext ctx;
        ctx.source = MessageSource::LOCAL_WEBSOCKET;
        ctx.socket_fd = fd;
        return ctx;
    }

    static MessageContext cloud() {
        MessageContext ctx;
        ctx.source = MessageSource::CLOUD_WEBSOCKET;
        return ctx;
    }

    static MessageContext internal() {
        return MessageContext{};
    }

    bool isLocal() const { return source == MessageSource::LOCAL_WEBSOCKET; }
    bool isCloud() const { return source == MessageSource::CLOUD_WEBSOCKET; }
};
