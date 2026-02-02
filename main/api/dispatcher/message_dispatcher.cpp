// Message dispatcher implementation
#include "message_dispatcher.h"
#include <esp_log.h>

static const char* TAG = "msg_dispatcher";

MessageDispatcher& MessageDispatcher::instance() {
    static MessageDispatcher instance;
    return instance;
}

void MessageDispatcher::registerHandler(std::unique_ptr<HandlerBase> handler) {
    if (handler) {
        ESP_LOGI(TAG, "Registered handler for %zu message types",
            handler->supportedMessages().size());
        handlers_.push_back(std::move(handler));
    }
}

esp_err_t MessageDispatcher::dispatch(
    const uint8_t* data,
    size_t len,
    const MessageContext& ctx
) {
    if (!data || len == 0) {
        ESP_LOGE(TAG, "Invalid message data");
        return ESP_ERR_INVALID_ARG;
    }

    // Deserialize message
    Kd__V1__TranquilMessage* msg = kd__v1__tranquil_message__unpack(nullptr, len, data);
    if (!msg) {
        ESP_LOGE(TAG, "Failed to unpack message");
        return ESP_FAIL;
    }

    ResponseMessage response;
    HandleResult result;
    esp_err_t ret = dispatch(msg, ctx, &response, &result);

    // If handler wants to forward to cloud, do it with original data
    if (result.forward_to_cloud) {
        ESP_LOGI(TAG, "Forwarding message type %d to cloud", msg->message_case);
        ResponseRouter::instance().forwardToCloud(data, len);
    }

    // Route response if there is one
    if (result.has_response && response.valid()) {
        ResponseRouter::instance().route(response, ctx, result);
    }

    kd__v1__tranquil_message__free_unpacked(msg, nullptr);
    return ret;
}

esp_err_t MessageDispatcher::dispatch(
    Kd__V1__TranquilMessage* msg,
    const MessageContext& ctx,
    ResponseMessage* response_out,
    HandleResult* result_out
) {
    if (!msg) {
        ESP_LOGE(TAG, "Null message");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGD(TAG, "Dispatching message type %d from %s",
        msg->message_case,
        ctx.source == MessageSource::LOCAL_WEBSOCKET ? "local" :
        ctx.source == MessageSource::CLOUD_WEBSOCKET ? "cloud" :
        ctx.source == MessageSource::REST_API ? "rest" : "internal");

    // Check if message type is allowed from this source
    if (!isAllowedFrom(msg->message_case, ctx.source)) {
        ESP_LOGW(TAG, "Message type %d not allowed from source %d",
            msg->message_case, static_cast<int>(ctx.source));

        // Return error response
        if (response_out) {
            *response_out = HandlerBase::makeCommandResult(false, "Not allowed from this source");
        }
        if (result_out) {
            *result_out = HandleResult::error();
        }
        return ESP_ERR_NOT_ALLOWED;
    }

    // Find handler
    HandlerBase* handler = findHandler(msg->message_case);
    if (!handler) {
        ESP_LOGW(TAG, "No handler for message type %d", msg->message_case);
        if (result_out) {
            *result_out = HandleResult::notHandled();
        }
        return ESP_ERR_NOT_FOUND;
    }

    // Execute handler
    ResponseMessage response;
    HandleResult result = handler->handle(msg, ctx, response);

    ESP_LOGD(TAG, "Handler result: handled=%d, has_response=%d, broadcast=%d, forward=%d",
        result.handled, result.has_response, result.broadcast, result.forward_to_cloud);

    // Return results
    if (response_out && response.valid()) {
        *response_out = std::move(response);
    }
    if (result_out) {
        *result_out = result;
    }

    return result.handled ? ESP_OK : ESP_ERR_NOT_FOUND;
}

bool MessageDispatcher::canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const {
    for (const auto& handler : handlers_) {
        if (handler->canHandle(msg_case)) {
            return true;
        }
    }
    return false;
}

bool MessageDispatcher::isAllowedFrom(
    Kd__V1__TranquilMessage__MessageCase msg_case,
    MessageSource source
) const {
    for (const auto& handler : handlers_) {
        if (handler->canHandle(msg_case)) {
            // Check local-only restriction
            if (handler->isLocalOnly(msg_case)) {
                if (source == MessageSource::CLOUD_WEBSOCKET) {
                    return false;
                }
            }
            // Check cloud-only restriction
            if (handler->isCloudOnly(msg_case)) {
                if (source != MessageSource::CLOUD_WEBSOCKET) {
                    return false;
                }
            }
            return true;
        }
    }
    // Unknown message types are allowed by default
    return true;
}

HandlerBase* MessageDispatcher::findHandler(Kd__V1__TranquilMessage__MessageCase msg_case) {
    for (auto& handler : handlers_) {
        if (handler->canHandle(msg_case)) {
            return handler.get();
        }
    }
    return nullptr;
}

// Include handlers for registration
#include "player_handler.h"
#include "pattern_handler.h"
#include "system_handler.h"
#include "playlist_handler.h"
#include "led_handler.h"
#include "config_handler.h"
#include "preset_handler.h"
#include "schedule_handler.h"
#include "license_handler.h"

void dispatcher_register_standard_handlers() {
    auto& dispatcher = MessageDispatcher::instance();

    // Note: Using existing singleton instances via unique_ptr wrapper
    // This is a bit awkward but allows handlers to be singletons while
    // still participating in the dispatcher's ownership model

    // Player control handler
    class PlayerHandlerWrapper : public HandlerBase {
    public:
        HandleResult handle(const Kd__V1__TranquilMessage* msg, const MessageContext& ctx, ResponseMessage& response) override {
            return PlayerHandler::instance().handle(msg, ctx, response);
        }
        bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override {
            return PlayerHandler::instance().canHandle(msg_case);
        }
        std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override {
            return PlayerHandler::instance().supportedMessages();
        }
    };
    dispatcher.registerHandler(std::make_unique<PlayerHandlerWrapper>());

    // Pattern management handler
    class PatternHandlerWrapper : public HandlerBase {
    public:
        HandleResult handle(const Kd__V1__TranquilMessage* msg, const MessageContext& ctx, ResponseMessage& response) override {
            return PatternHandler::instance().handle(msg, ctx, response);
        }
        bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override {
            return PatternHandler::instance().canHandle(msg_case);
        }
        std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override {
            return PatternHandler::instance().supportedMessages();
        }
    };
    dispatcher.registerHandler(std::make_unique<PatternHandlerWrapper>());

    // System handler
    class SystemHandlerWrapper : public HandlerBase {
    public:
        HandleResult handle(const Kd__V1__TranquilMessage* msg, const MessageContext& ctx, ResponseMessage& response) override {
            return SystemHandler::instance().handle(msg, ctx, response);
        }
        bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override {
            return SystemHandler::instance().canHandle(msg_case);
        }
        std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override {
            return SystemHandler::instance().supportedMessages();
        }
    };
    dispatcher.registerHandler(std::make_unique<SystemHandlerWrapper>());

    // Playlist handler
    class PlaylistHandlerWrapper : public HandlerBase {
    public:
        HandleResult handle(const Kd__V1__TranquilMessage* msg, const MessageContext& ctx, ResponseMessage& response) override {
            return PlaylistHandler::instance().handle(msg, ctx, response);
        }
        bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override {
            return PlaylistHandler::instance().canHandle(msg_case);
        }
        std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override {
            return PlaylistHandler::instance().supportedMessages();
        }
    };
    dispatcher.registerHandler(std::make_unique<PlaylistHandlerWrapper>());

    // LED handler
    class LEDHandlerWrapper : public HandlerBase {
    public:
        HandleResult handle(const Kd__V1__TranquilMessage* msg, const MessageContext& ctx, ResponseMessage& response) override {
            return LEDHandler::instance().handle(msg, ctx, response);
        }
        bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override {
            return LEDHandler::instance().canHandle(msg_case);
        }
        std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override {
            return LEDHandler::instance().supportedMessages();
        }
    };
    dispatcher.registerHandler(std::make_unique<LEDHandlerWrapper>());

    // Config handler (local-only)
    class ConfigHandlerWrapper : public HandlerBase {
    public:
        HandleResult handle(const Kd__V1__TranquilMessage* msg, const MessageContext& ctx, ResponseMessage& response) override {
            return ConfigHandler::instance().handle(msg, ctx, response);
        }
        bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override {
            return ConfigHandler::instance().canHandle(msg_case);
        }
        std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override {
            return ConfigHandler::instance().supportedMessages();
        }
        bool isLocalOnly(Kd__V1__TranquilMessage__MessageCase) const override { return true; }
    };
    dispatcher.registerHandler(std::make_unique<ConfigHandlerWrapper>());

    // Preset handler (local-only)
    class PresetHandlerWrapper : public HandlerBase {
    public:
        HandleResult handle(const Kd__V1__TranquilMessage* msg, const MessageContext& ctx, ResponseMessage& response) override {
            return PresetHandler::instance().handle(msg, ctx, response);
        }
        bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override {
            return PresetHandler::instance().canHandle(msg_case);
        }
        std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override {
            return PresetHandler::instance().supportedMessages();
        }
        bool isLocalOnly(Kd__V1__TranquilMessage__MessageCase) const override { return true; }
    };
    dispatcher.registerHandler(std::make_unique<PresetHandlerWrapper>());

    // Schedule handler
    class ScheduleHandlerWrapper : public HandlerBase {
    public:
        HandleResult handle(const Kd__V1__TranquilMessage* msg, const MessageContext& ctx, ResponseMessage& response) override {
            return ScheduleHandler::instance().handle(msg, ctx, response);
        }
        bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override {
            return ScheduleHandler::instance().canHandle(msg_case);
        }
        std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override {
            return ScheduleHandler::instance().supportedMessages();
        }
    };
    dispatcher.registerHandler(std::make_unique<ScheduleHandlerWrapper>());

    // License handler (local-only for store token requests)
    class LicenseHandlerWrapper : public HandlerBase {
    public:
        HandleResult handle(const Kd__V1__TranquilMessage* msg, const MessageContext& ctx, ResponseMessage& response) override {
            return LicenseHandler::instance().handle(msg, ctx, response);
        }
        bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override {
            return LicenseHandler::instance().canHandle(msg_case);
        }
        std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override {
            return LicenseHandler::instance().supportedMessages();
        }
        bool isLocalOnly(Kd__V1__TranquilMessage__MessageCase msg_case) const override {
            return LicenseHandler::instance().isLocalOnly(msg_case);
        }
    };
    dispatcher.registerHandler(std::make_unique<LicenseHandlerWrapper>());

    ESP_LOGI(TAG, "Registered standard message handlers");
}
