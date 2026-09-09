// Unified schedule handler implementation
#include "schedule_handler.h"
#include "schedule_manager.h"

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
    switch (msg->message_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE_REQUEST:
            return handleScheduleRequest(ctx, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE:
            return handleSetSchedule(msg->schedule, ctx, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_RUN_SCHEDULE_ACTION:
            return handleRunAction(msg->run_schedule_action, ctx, response);

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

HandleResult ScheduleHandler::handleScheduleRequest(
    const MessageContext& ctx,
    ResponseMessage& response
) {
    ESP_LOGI(TAG, "ScheduleRequest from %s", ctx.isLocal() ? "local" : "cloud");
    response = ScheduleManager::instance().buildScheduleMessage();
    return response.valid() ? HandleResult::ok() : HandleResult::error();
}

HandleResult ScheduleHandler::handleSetSchedule(
    const Kd__V1__TranquilSchedule* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    ESP_LOGI(TAG, "SetSchedule from %s with %zu items%s",
        ctx.isLocal() ? "local" : "cloud",
        msg ? msg->n_schedule_items : 0,
        (msg && msg->quiet_hours) ? " + quiet hours" : "");

    if (!msg) {
        response = makeCommandResult(false, "Invalid schedule message");
        return HandleResult::ok();
    }

    auto items = ScheduleManager::itemsFromProto(msg);
    auto quiet = ScheduleManager::quietFromProto(msg);

    esp_err_t ret = ScheduleManager::instance().setSchedule(std::move(items), quiet);
    if (ret == ESP_ERR_INVALID_SIZE) {
        response = makeCommandResult(false, "Too many schedule items or quiet windows");
        return HandleResult::ok();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set schedule: %s", esp_err_to_name(ret));
        response = makeCommandResult(false, "Failed to save schedule");
        return HandleResult::ok();
    }

    // Answer the requester with the stored schedule (ids assigned). Everyone
    // else, LAN and cloud, gets the debounced Schedule push from the
    // library notifier that setSchedule() posted.
    response = ScheduleManager::instance().buildScheduleMessage();
    return HandleResult::ok();
}

HandleResult ScheduleHandler::handleRunAction(
    const Kd__V1__RunScheduleAction* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    if (!msg || !msg->action) {
        response = makeCommandResult(false, "Missing action");
        return HandleResult::ok();
    }
    auto action = ScheduleManager::actionFromProto(msg->action);
    ESP_LOGI(TAG, "RunScheduleAction %s (force=%d) from %s",
        ScheduleManager::actionName(action.type), msg->force, ctx.isLocal() ? "local" : "cloud");
    auto res = ScheduleManager::instance().runAction(action, msg->force != 0);
    response = makeCommandResult(res.ok, res.detail);
    return HandleResult::ok();
}
