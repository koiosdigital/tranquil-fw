// Unified system handler
// Handles system-related protobuf messages (Ping, SystemInfo, Home, FactoryReset)
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

private:
    SystemHandler() = default;

    // Individual message handlers
    HandleResult handlePing(const Kd__V1__Ping* msg, ResponseMessage& response);
    HandleResult handleGetSystemInfo(ResponseMessage& response);
    HandleResult handleFactoryReset(const Kd__V1__FactoryResetRequest* msg, ResponseMessage& response);
    HandleResult handleHomeRequest(const Kd__V1__HomeRequest* msg, ResponseMessage& response);
};
