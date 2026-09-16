// Base class for unified message handlers
#pragma once

#include "message_context.h"
#include "ops/op_result.h"
#include <kd/v1/tranquil.pb-c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <memory>
#include <vector>

// Result of handling a message
struct HandleResult {
    bool handled = false;          // Message was processed
    bool has_response = false;     // Response should be sent
    bool broadcast = false;        // Response should broadcast to all local clients
    bool forward_to_cloud = false; // Forward original message to cloud
    bool send_to_cloud = false;    // Send response to cloud (in addition to local)

    static HandleResult ok(bool broadcast_to_locals = false, bool also_send_to_cloud = false) {
        return { true, true, broadcast_to_locals, false, also_send_to_cloud };
    }

    static HandleResult noResponse() {
        return { true, false, false, false, false };
    }

    static HandleResult forwardToCloud() {
        return { true, false, false, true, false };
    }

    static HandleResult notHandled() {
        return { false, false, false, false, false };
    }

    static HandleResult error() {
        return { true, true, false, false, false };
    }
};

// Owning wrapper for protobuf response messages
// Automatically frees allocated memory on destruction
class ResponseMessage {
public:
    ResponseMessage() : data_(nullptr), len_(0), owns_data_(false) {}

    // Take ownership of serialized data
    ResponseMessage(uint8_t* data, size_t len) : data_(data), len_(len), owns_data_(true) {}

    ~ResponseMessage() {
        if (owns_data_ && data_) {
            free(data_);
        }
    }

    // Move only
    ResponseMessage(ResponseMessage&& other) noexcept
        : data_(other.data_), len_(other.len_), owns_data_(other.owns_data_) {
        other.data_ = nullptr;
        other.len_ = 0;
        other.owns_data_ = false;
    }

    ResponseMessage& operator=(ResponseMessage&& other) noexcept {
        if (this != &other) {
            if (owns_data_ && data_) free(data_);
            data_ = other.data_;
            len_ = other.len_;
            owns_data_ = other.owns_data_;
            other.data_ = nullptr;
            other.len_ = 0;
            other.owns_data_ = false;
        }
        return *this;
    }

    // No copy
    ResponseMessage(const ResponseMessage&) = delete;
    ResponseMessage& operator=(const ResponseMessage&) = delete;

    uint8_t* data() const { return data_; }
    size_t len() const { return len_; }
    bool valid() const { return data_ != nullptr && len_ > 0; }

    // Release ownership (caller takes responsibility for freeing)
    uint8_t* release() {
        owns_data_ = false;
        return data_;
    }

private:
    uint8_t* data_;
    size_t len_;
    bool owns_data_;
};

// Base class for all message handlers
class HandlerBase {
public:
    virtual ~HandlerBase() = default;

    // Handle a message and return serialized response
    // Returns: HandleResult indicating what to do with the response
    // response: Serialized protobuf response (if has_response is true)
    virtual HandleResult handle(
        const Kd__V1__TranquilMessage* msg,
        const MessageContext& ctx,
        ResponseMessage& response
    ) = 0;

    // Return true if this handler can process the given message type
    virtual bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const = 0;

    // Return list of message types this handler supports
    virtual std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const = 0;

    // Return true if this message type should only be handled from local sources
    virtual bool isLocalOnly(Kd__V1__TranquilMessage__MessageCase msg_case) const {
        (void)msg_case;
        return false;
    }

    // Return true if this message type should only be handled from cloud
    virtual bool isCloudOnly(Kd__V1__TranquilMessage__MessageCase msg_case) const {
        (void)msg_case;
        return false;
    }

    // The one esp_err -> CommandResult mapping for every WS handler.
    // error_code carries the esp_err_t value (0 on success); request_id is
    // echoed on both the CommandResult and the envelope when the request
    // carried one (MessageContext::request_id).
    static ResponseMessage makeCommandResult(bool success, const char* detail = nullptr,
                                             const char* request_id = nullptr,
                                             int32_t error_code = ESP_FAIL) {
        Kd__V1__CommandResult result = KD__V1__COMMAND_RESULT__INIT;
        result.success = success;
        result.error_code = success ? 0 : error_code;
        result.detail = const_cast<char*>(detail ? detail : "");
        result.request_id = const_cast<char*>(request_id ? request_id : "");

        Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
        resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_COMMAND_RESULT;
        resp.command_result = &result;

        return serialize(&resp, request_id);
    }

    static ResponseMessage makeCommandResult(const OpResult& r, const char* request_id = nullptr) {
        return makeCommandResult(r.ok(), r.detail, request_id, static_cast<int32_t>(r.err));
    }

    // Pack a TranquilMessage. When `request_id` is set it is echoed on the
    // envelope (the payload is left untouched: the caller's struct is copied
    // shallowly, so it may stay const).
    static ResponseMessage serialize(const Kd__V1__TranquilMessage* msg, const char* request_id = nullptr) {
        Kd__V1__TranquilMessage env = *msg;
        if (request_id && request_id[0]) {
            env.request_id = const_cast<char*>(request_id);
        }

        size_t len = kd__v1__tranquil_message__get_packed_size(&env);

        // Response ceiling. Matches the cloud link's 32 KB cap; local WS
        // frames have no smaller limit outbound. Buffers live in PSRAM.
        static constexpr size_t MAX_RESPONSE = 32 * 1024;
        if (len > MAX_RESPONSE) {
            ESP_LOGE("HandlerBase", "Response exceeds %zu bytes: %zu", MAX_RESPONSE, len);
            return ResponseMessage();
        }

        // PSRAM, not internal RAM: every handler response (up to 32 KB, and a
        // pattern-list broadcast fans out several copies) is only read by the
        // socket write / per-client copy, never DMA'd. ResponseMessage frees
        // with free(), which is valid on SPIRAM pointers.
        uint8_t* buf = static_cast<uint8_t*>(heap_caps_malloc(len, MALLOC_CAP_SPIRAM));
        if (!buf) return ResponseMessage();

        size_t packed = kd__v1__tranquil_message__pack(&env, buf);
        if (packed != len) {
            free(buf);
            return ResponseMessage();
        }

        return ResponseMessage(buf, len);
    }
};

// Pack a TranquilMessage outside a handler (state snapshot builders in
// main/api/state and main/api/ops).
inline ResponseMessage pack_tranquil_message(const Kd__V1__TranquilMessage* msg, const char* request_id = nullptr) {
    return HandlerBase::serialize(msg, request_id);
}
