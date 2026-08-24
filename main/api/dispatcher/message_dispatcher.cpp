// Message dispatcher implementation
#include "message_dispatcher.h"
#include <esp_log.h>
#include <esp_heap_caps.h>

static const char* TAG = "msg_dispatcher";

// Decoded protobuf trees for local WS frames land in PSRAM rather than scarce
// internal RAM (mirrors the cloud path in sockets.cpp). Not DMA'd.
namespace {
void* pb_spiram_alloc(void*, size_t size) {
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
}
void pb_spiram_free(void*, void* ptr) {
    heap_caps_free(ptr);
}
ProtobufCAllocator g_spiram_allocator = {
    .alloc = pb_spiram_alloc,
    .free = pb_spiram_free,
    .allocator_data = nullptr,
};
}  // namespace

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

    // Deserialize message (decoded tree allocated in PSRAM)
    Kd__V1__TranquilMessage* msg = kd__v1__tranquil_message__unpack(&g_spiram_allocator, len, data);
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

    // Route response if there is one. A handler that wanted to respond but
    // produced no payload hit the serialize() size cap — send an explicit
    // error instead of leaving the client waiting forever.
    if (result.has_response) {
        if (!response.valid()) {
            ESP_LOGE(TAG, "Handler for type %d produced no payload (response too large?)",
                msg->message_case);
            response = HandlerBase::makeCommandResult(false, "Response too large");
            result.broadcast = false;
            result.send_to_cloud = false;
        }
        ResponseRouter::instance().route(response, ctx, result);
    }

    kd__v1__tranquil_message__free_unpacked(msg, &g_spiram_allocator);
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
#include "schedule_handler.h"

// Non-owning adapter that lets a singleton handler participate in the
// dispatcher's unique_ptr ownership model without duplicating boilerplate.
template <typename H>
class SingletonHandlerWrapper : public HandlerBase {
public:
    HandleResult handle(const Kd__V1__TranquilMessage* msg, const MessageContext& ctx, ResponseMessage& response) override {
        return H::instance().handle(msg, ctx, response);
    }
    bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override {
        return H::instance().canHandle(msg_case);
    }
    std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override {
        return H::instance().supportedMessages();
    }
    bool isLocalOnly(Kd__V1__TranquilMessage__MessageCase msg_case) const override {
        return H::instance().isLocalOnly(msg_case);
    }
};

// The local request/response surface lives on the REST API; the dispatcher
// keeps only what the sockets still need: cloud sync (patterns, playlists,
// schedule, player control, system) and real-time push to local clients.
// Config, preset, and license store-token messages were local-only and are
// now served by /api/config, /api/presets, and /api/license/store-token.
void dispatcher_register_standard_handlers() {
    auto& dispatcher = MessageDispatcher::instance();

    dispatcher.registerHandler(std::make_unique<SingletonHandlerWrapper<PlayerHandler>>());
    dispatcher.registerHandler(std::make_unique<SingletonHandlerWrapper<PatternHandler>>());
    dispatcher.registerHandler(std::make_unique<SingletonHandlerWrapper<SystemHandler>>());
    dispatcher.registerHandler(std::make_unique<SingletonHandlerWrapper<PlaylistHandler>>());
    dispatcher.registerHandler(std::make_unique<SingletonHandlerWrapper<LEDHandler>>());
    dispatcher.registerHandler(std::make_unique<SingletonHandlerWrapper<ScheduleHandler>>());

    ESP_LOGI(TAG, "Registered standard message handlers");
}
