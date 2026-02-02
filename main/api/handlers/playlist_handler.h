// Unified playlist handler
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

    HandleResult handlePlaylistsRequest(ResponseMessage& response);
    HandleResult handleCreatePlaylist(const Kd__V1__CreatePlaylist* msg, ResponseMessage& response);
    HandleResult handleUpdatePlaylist(const Kd__V1__UpdatePlaylist* msg, ResponseMessage& response);
    HandleResult handleDeletePlaylist(const Kd__V1__DeletePlaylist* msg, ResponseMessage& response);
};
