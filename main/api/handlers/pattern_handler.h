// Unified pattern handler: a thin protobuf codec over library_ops, plus the
// cloud download forwarding.
#pragma once

#include "handler_base.h"

#include <string>

class PatternHandler : public HandlerBase {
public:
    static PatternHandler& instance();

    HandleResult handle(
        const Kd__V1__TranquilMessage* msg,
        const MessageContext& ctx,
        ResponseMessage& response
    ) override;

    bool canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const override;

    std::vector<Kd__V1__TranquilMessage__MessageCase> supportedMessages() const override;

    // Called from the job-completion hook when a cloud download job finishes.
    // Clears the download tracking and, on success, schedules a push of the
    // updated pattern list to LAN clients and the cloud.
    void notifyPatternDownloadComplete(const std::string& pattern_uuid, bool success);

private:
    PatternHandler() = default;
};
