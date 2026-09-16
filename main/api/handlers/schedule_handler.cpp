// Unified schedule handler implementation
#include "schedule_handler.h"
#include "schedule_manager.h"
#include "ops/schedule_ops.h"

#include <esp_log.h>

static const char* TAG = "schedule_handler";

ScheduleHandler& ScheduleHandler::instance() {
    static ScheduleHandler instance;
    return instance;
}

HandleResult ScheduleHandler::handle(
    const Kd__V1__TranquilMessage* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    const char* rid = ctx.request_id;

    switch (msg->message_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE_REQUEST:
            ESP_LOGI(TAG, "ScheduleRequest from %s", ctx.isLocal() ? "local" : "cloud");
            response = ScheduleManager::instance().buildScheduleMessage(rid);
            return response.valid() ? HandleResult::ok() : HandleResult::error();

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE: {
            const auto* m = msg->schedule;
            ESP_LOGI(TAG, "SetSchedule from %s with %zu items%s",
                ctx.isLocal() ? "local" : "cloud",
                m ? m->n_schedule_items : 0,
                (m && m->quiet_hours) ? " + quiet hours" : "");

            if (!m) {
                response = makeCommandResult(false, "Invalid schedule message", rid, ESP_ERR_INVALID_ARG);
                return HandleResult::ok();
            }

            auto items = ScheduleManager::itemsFromProto(m);
            auto quiet = ScheduleManager::quietFromProto(m);

            // The requester is answered with the stored schedule below, so the
            // notifier's push leaves it out; everyone else (LAN and cloud)
            // gets the debounced Schedule push.
            OpResult r = schedule_ops::set_schedule(std::move(items), quiet,
                ctx.isLocal() ? ctx.socket_fd : -1, ctx.isCloud());
            if (!r.ok()) {
                response = makeCommandResult(r, rid);
                return HandleResult::ok();
            }

            response = ScheduleManager::instance().buildScheduleMessage(rid);
            return HandleResult::ok();
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_RUN_SCHEDULE_ACTION: {
            const auto* m = msg->run_schedule_action;
            if (!m || !m->action) {
                response = makeCommandResult(false, "Missing action", rid, ESP_ERR_INVALID_ARG);
                return HandleResult::ok();
            }
            auto action = ScheduleManager::actionFromProto(m->action);
            ESP_LOGI(TAG, "RunScheduleAction %s (force=%d) from %s",
                ScheduleManager::actionName(action.type), m->force, ctx.isLocal() ? "local" : "cloud");
            response = makeCommandResult(schedule_ops::run_action(action, m->force != 0), rid);
            return HandleResult::ok();
        }

        default:
            return HandleResult::notHandled();
    }
}

bool ScheduleHandler::canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const {
    switch (msg_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE_REQUEST:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_RUN_SCHEDULE_ACTION:
            return true;
        default:
            return false;
    }
}

std::vector<Kd__V1__TranquilMessage__MessageCase> ScheduleHandler::supportedMessages() const {
    return {
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE_REQUEST,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_RUN_SCHEDULE_ACTION,
    };
}
