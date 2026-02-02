// License handler
// Handles license-related protobuf messages (GetStoreToken)
#pragma once

#include "handler_base.h"

class LicenseHandler : public HandlerBase {
public:
    static LicenseHandler& instance();

    HandleResult handle(
        const Kd__V1__TranquilMessage* msg,
        const MessageContext& ctx,
        ResponseMessage& response
    ) override;

    bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override;

    std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override;

    // Store token requests are local-only (not from cloud)
    bool isLocalOnly(Kd__V1__TranquilMessage__MessageCase msg_case) const override;

private:
    LicenseHandler() = default;

    HandleResult handleGetStoreToken(ResponseMessage& response);
};
