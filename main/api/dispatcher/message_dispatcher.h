// Message dispatcher for unified API
// Routes incoming messages to appropriate handlers
#pragma once

#include "handler_base.h"
#include "response_router.h"
#include <vector>
#include <memory>
#include <esp_err.h>

class MessageDispatcher {
public:
    static MessageDispatcher& instance();

    // Register a handler (takes ownership)
    void registerHandler(std::unique_ptr<HandlerBase> handler);

    // Dispatch incoming message from any source
    // Automatically deserializes, routes to handler, and sends response
    esp_err_t dispatch(
        const uint8_t* data,
        size_t len,
        const MessageContext& ctx
    );

    // Dispatch already-parsed message
    // response is populated if handler produces one
    esp_err_t dispatch(
        Kd__V1__TranquilMessage* msg,
        const MessageContext& ctx,
        ResponseMessage* response = nullptr,
        HandleResult* result = nullptr
    );

    // Check if a message type can be handled
    bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const;

    // Check if message type is allowed from the given source
    bool isAllowedFrom(Kd__V1__TranquilMessage__MessageCase msg_case, MessageSource source) const;

private:
    MessageDispatcher() = default;

    std::vector<std::unique_ptr<HandlerBase>> handlers_;

    HandlerBase* findHandler(Kd__V1__TranquilMessage__MessageCase msg_case);
};

// Helper to initialize all standard handlers
void dispatcher_register_standard_handlers();
