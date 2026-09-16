// Unified playlist handler: a thin protobuf codec over library_ops.
#pragma once

#include "handler_base.h"

class PlaylistHandler : public HandlerBase {
public:
    static PlaylistHandler& instance();

    HandleResult handle(
        const Kd__V1__TranquilMessage* msg,
        const MessageContext& ctx,
        ResponseMessage& response
    ) override;

    bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override;
    std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override;

private:
    PlaylistHandler() = default;
};
