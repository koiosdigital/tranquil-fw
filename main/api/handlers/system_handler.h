// Unified system handler: a thin protobuf codec over system_ops
// (Ping, SystemInfo, Home, Reboot, FactoryReset).
#pragma once

#include "handler_base.h"

class SystemHandler : public HandlerBase {
public:
    static SystemHandler& instance();

    HandleResult handle(
        const Kd__V1__TranquilMessage* msg,
        const MessageContext& ctx,
        ResponseMessage& response
    ) override;

    bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override;

    std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override;

    // Factory reset wipes the table; it is a LAN-only operation. Reboot and
    // homing stay cloud-capable (support / remote recovery).
    bool isLocalOnly(Kd__V1__TranquilMessage__MessageCase msg_case) const override;

private:
    SystemHandler() = default;
};
