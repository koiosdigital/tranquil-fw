// Unified player control handler
// Handles all player-related protobuf messages from local and cloud
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

    // Build PlayerState response (public for broadcaster access)
    ResponseMessage buildPlayerStateResponse();

private:
    PlayerHandler() = default;

    // Individual message handlers
    HandleResult handlePlayPattern(const Kd__V1__PlayPattern* msg, ResponseMessage& response);
    HandleResult handleSetPaused(const Kd__V1__SetPaused* msg, ResponseMessage& response);
    HandleResult handleStopPlayback(ResponseMessage& response);
    HandleResult handlePlaylistPlay(const Kd__V1__PlaylistPlay* msg, ResponseMessage& response);
    HandleResult handleSetShuffle(const Kd__V1__PlaylistSetShuffle* msg, ResponseMessage& response);
    HandleResult handleSetLoop(const Kd__V1__SetLoop* msg, ResponseMessage& response);
    HandleResult handleSetFeedRate(const Kd__V1__SetFeedRate* msg, ResponseMessage& response);
    HandleResult handlePlaylistNavigate(const Kd__V1__PlaylistNavigateCommand* msg, ResponseMessage& response);
    HandleResult handleGetPlayerState(ResponseMessage& response);
};
