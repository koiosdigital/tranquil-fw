// Unified pattern handler
// Handles pattern-related protobuf messages
// Supports local pattern management and cloud download forwarding
#pragma once

#include "handler_base.h"

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

private:
    PatternHandler() = default;

    // Individual message handlers
    HandleResult handleDownloadedPatternsRequest(ResponseMessage& response);
    HandleResult handleDeletePattern(const Kd__V1__DeletePattern* msg, ResponseMessage& response);
    HandleResult handleModifyPattern(const Kd__V1__ModifyPattern* msg, ResponseMessage& response);
    HandleResult handleGetPatternRequest(const Kd__V1__GetPatternRequest* msg, ResponseMessage& response);
    HandleResult handleRequestPatternDownload(
        const Kd__V1__RequestPatternDownload* msg,
        const MessageContext& ctx,
        ResponseMessage& response
    );
};
