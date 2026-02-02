// Unified config handler for motion/LED configuration (local-only)
#pragma once

#include "handler_base.h"

class ConfigHandler : public HandlerBase {
public:
    static ConfigHandler& instance();

    HandleResult handle(
        const Kd__V1__TranquilMessage* msg,
        const MessageContext& ctx,
        ResponseMessage& response
    ) override;

    bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override;
    std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override;

private:
    ConfigHandler() = default;

    HandleResult handleGetConfig(ResponseMessage& response);
    HandleResult handleSetConfig(const Kd__V1__SetConfigRequest* msg, ResponseMessage& response);
    HandleResult handleClearCalibration(ResponseMessage& response);
};
