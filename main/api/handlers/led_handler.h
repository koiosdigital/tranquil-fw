// Unified LED control handler: a thin protobuf codec over led_ops / led_state.
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
};
