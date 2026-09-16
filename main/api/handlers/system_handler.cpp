// Unified system handler implementation
#include "system_handler.h"
#include "system_info.h"
#include "ops/system_ops.h"

SystemHandler& SystemHandler::instance() {
    static SystemHandler instance;
    return instance;
}

HandleResult SystemHandler::handle(
    const Kd__V1__TranquilMessage* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    const char* rid = ctx.request_id;

    switch (msg->message_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PING: {
            Kd__V1__Pong pong = KD__V1__PONG__INIT;
            pong.timestamp = msg->ping ? msg->ping->timestamp : 0;

            Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
            resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_PONG;
            resp.pong = &pong;

            response = serialize(&resp, rid);
            return HandleResult::ok(false, false);
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_SYSTEM_INFO:
            response = system_info::build_message(rid);
            return HandleResult::ok(false, false);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_FACTORY_RESET_REQUEST: {
            const auto* m = msg->factory_reset_request;
            // Send the result before the restart timer fires.
            response = makeCommandResult(system_ops::factory_reset(m ? m->reason : nullptr), rid);
            return HandleResult::ok(false, false);
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_REBOOT_REQUEST: {
            const auto* m = msg->reboot_request;
            response = makeCommandResult(system_ops::reboot(m ? m->reason : nullptr), rid);
            return HandleResult::ok(false, false);
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_HOME_REQUEST: {
            const auto* m = msg->home_request;
            const bool force_full = m ? (m->force_full_calibration != 0) : false;

            // Homing runs asynchronously (tens of seconds); the reply confirms
            // the run STARTED. Completion is observable through
            // SystemInfo.is_homing / is_homed, pushed to LAN + cloud when the
            // run ends.
            OpResult r = system_ops::home(force_full);

            Kd__V1__HomeResponse home_resp = KD__V1__HOME_RESPONSE__INIT;
            home_resp.success = r.ok();
            home_resp.error = const_cast<char*>(r.ok() ? "" : r.detail);

            Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
            resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_HOME_RESPONSE;
            resp.home_response = &home_resp;

            response = serialize(&resp, rid);
            return HandleResult::ok(false, false);
        }

        default:
            return HandleResult::notHandled();
    }
}

bool SystemHandler::canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const {
    switch (msg_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PING:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_SYSTEM_INFO:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_FACTORY_RESET_REQUEST:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_REBOOT_REQUEST:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_HOME_REQUEST:
            return true;
        default:
            return false;
    }
}

std::vector<Kd__V1__TranquilMessage__MessageCase> SystemHandler::supportedMessages() const {
    return {
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_PING,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_SYSTEM_INFO,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_FACTORY_RESET_REQUEST,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_REBOOT_REQUEST,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_HOME_REQUEST
    };
}

bool SystemHandler::isLocalOnly(Kd__V1__TranquilMessage__MessageCase msg_case) const {
    return msg_case == KD__V1__TRANQUIL_MESSAGE__MESSAGE_FACTORY_RESET_REQUEST;
}
