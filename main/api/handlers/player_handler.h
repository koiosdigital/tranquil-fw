// Unified player control handler: a thin protobuf codec over player_ops.
#pragma once

#include "handler_base.h"

class PlayerHandler : public HandlerBase {
public:
    static PlayerHandler& instance();

    HandleResult handle(
        const Kd__V1__TranquilMessage* msg,
        const MessageContext& ctx,
        ResponseMessage& response
    ) override;

    bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override;

    std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override;

private:
    PlayerHandler() = default;
};
