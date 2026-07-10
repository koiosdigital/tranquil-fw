// Base class for unified message handlers
#pragma once

#include "message_context.h"
#include <kd/v1/tranquil.pb-c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <memory>
#include <vector>

// Result of handling a message
struct HandleResult {
    bool handled;           // Message was processed
    bool has_response;      // Response should be sent
    bool broadcast;         // Response should broadcast to all local clients
    bool forward_to_cloud;  // Forward original message to cloud
    bool send_to_cloud;     // Send response to cloud (in addition to local)

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

    // Helper to create a CommandResult response (public for use by dispatcher)
    static ResponseMessage makeCommandResult(bool success, const char* detail = nullptr) {
        Kd__V1__CommandResult result = KD__V1__COMMAND_RESULT__INIT;
        result.success = success;
        result.error_code = success ? 0 : -1;
        result.detail = detail ? const_cast<char*>(detail) : nullptr;

        Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
        resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_COMMAND_RESULT;
        resp.command_result = &result;

        return serialize(&resp);
    }

protected:
    // Helper to serialize a response message
    static ResponseMessage serialize(const Kd__V1__TranquilMessage* msg) {
        size_t len = kd__v1__tranquil_message__get_packed_size(msg);

        // Enforce max response size for local API
        static constexpr size_t MAX_RESPONSE = 16 * 1024;
        if (len > MAX_RESPONSE) {
            ESP_LOGE("HandlerBase", "Response exceeds %zu bytes: %zu", MAX_RESPONSE, len);
            return ResponseMessage();
        }

        uint8_t* buf = static_cast<uint8_t*>(malloc(len));
        if (!buf) return ResponseMessage();

        size_t packed = kd__v1__tranquil_message__pack(msg, buf);
        if (packed != len) {
            free(buf);
            return ResponseMessage();
        }

        return ResponseMessage(buf, len);
    }
};
