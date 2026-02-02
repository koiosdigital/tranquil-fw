// Unified LED control handler
#pragma once

#include "handler_base.h"

class LEDHandler : public HandlerBase {
public:
    static LEDHandler& instance();

    HandleResult handle(
        const Kd__V1__TranquilMessage* msg,
        const MessageContext& ctx,
        ResponseMessage& response
    ) override;

    bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override;
    std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override;

private:
    LEDHandler() = default;

    HandleResult handleLEDConfigRequest(ResponseMessage& response);
    HandleResult handleSetLEDChannel(const Kd__V1__SetLEDChannel* msg, ResponseMessage& response);
    HandleResult handleGetLEDEffects(ResponseMessage& response);
};
