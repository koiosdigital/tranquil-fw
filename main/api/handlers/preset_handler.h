// Unified preset handler (local-only)
#pragma once

#include "handler_base.h"

class PresetHandler : public HandlerBase {
public:
    static PresetHandler& instance();

    HandleResult handle(
        const Kd__V1__TranquilMessage* msg,
        const MessageContext& ctx,
        ResponseMessage& response
    ) override;

    bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override;
    std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override;

private:
    PresetHandler() = default;

    HandleResult handleListPresets(ResponseMessage& response);
    HandleResult handleLoadPreset(const Kd__V1__LoadPresetRequest* msg, ResponseMessage& response);
};
