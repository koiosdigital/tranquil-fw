// Unified schedule handler: a thin protobuf codec over schedule_ops.
#pragma once

#include "handler_base.h"

class ScheduleHandler : public HandlerBase {
public:
    static ScheduleHandler& instance();

    HandleResult handle(
        const Kd__V1__TranquilMessage* msg,
        const MessageContext& ctx,
        ResponseMessage& response
    ) override;

    bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override;
    std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override;

private:
    ScheduleHandler() = default;
};
