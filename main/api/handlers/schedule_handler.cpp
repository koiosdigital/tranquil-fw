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

        default:
            return HandleResult::notHandled();
    }
}

bool ScheduleHandler::canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const {
    switch (msg_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE_REQUEST:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE:
            return true;
        default:
            return false;
    }
}

std::vector<Kd__V1__TranquilMessage__MessageCase> ScheduleHandler::supportedMessages() const {
    return {
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE_REQUEST,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE
    };
}

HandleResult ScheduleHandler::handleScheduleRequest(
    const MessageContext& ctx,
    ResponseMessage& response
) {
    ESP_LOGI(TAG, "ScheduleRequest from %s", ctx.isLocal() ? "local" : "cloud");

    // If from cloud, this is requesting our current schedule
    // If from local, same - return current schedule

    auto& mgr = ScheduleManager::instance();

    // Build protobuf response. Locals, not statics: concurrent handler tasks
    // mutating shared static vectors double-free their backing stores.
    Kd__V1__TranquilSchedule schedule_proto;
    std::vector<Kd__V1__TranquilScheduleItem> items_storage;
    std::vector<Kd__V1__TranquilScheduleItem*> item_ptrs;
    std::vector<Kd__V1__ScheduleAction> actions_storage;

    mgr.toProto(&schedule_proto, items_storage, item_ptrs, actions_storage);

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE;
    resp.schedule = &schedule_proto;

    response = serialize(&resp);
    return HandleResult::ok();
}

HandleResult ScheduleHandler::handleSetSchedule(
    const Kd__V1__TranquilSchedule* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    ESP_LOGI(TAG, "SetSchedule from %s with %zu items",
        ctx.isLocal() ? "local" : "cloud",
        msg ? msg->n_schedule_items : 0);

    if (!msg) {
        response = makeCommandResult(false, "Invalid schedule message");
        return HandleResult::ok();
    }

    // Convert protobuf to internal format
    auto items = ScheduleManager::fromProto(msg);

    // Set the schedule
    esp_err_t ret = ScheduleManager::instance().setSchedule(items);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set schedule: %s", esp_err_to_name(ret));
        response = makeCommandResult(false, "Failed to save schedule");
        return HandleResult::ok();
    }

    // Log schedule items for debugging
    for (const auto& item : items) {
        ESP_LOGI(TAG, "  Schedule: days=0x%02x time=%02u:%02u action=%d uuid=%s",
            item.days_of_week,
            item.time_of_day / 3600,
            (item.time_of_day % 3600) / 60,
            item.action_type,
            item.uuid.c_str());
    }

    // Return the updated schedule as confirmation
    auto& mgr = ScheduleManager::instance();

    Kd__V1__TranquilSchedule schedule_proto;
    std::vector<Kd__V1__TranquilScheduleItem> items_storage;
    std::vector<Kd__V1__TranquilScheduleItem*> item_ptrs;
    std::vector<Kd__V1__ScheduleAction> actions_storage;

    mgr.toProto(&schedule_proto, items_storage, item_ptrs, actions_storage);

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_SCHEDULE;
    resp.schedule = &schedule_proto;

    response = serialize(&resp);

    // Broadcast to locals if from cloud, also send to cloud if from local
    if (ctx.source == MessageSource::CLOUD_WEBSOCKET) {
        return HandleResult::ok(true, false);  // Broadcast to locals
    } else {
        return HandleResult::ok(true, true);   // Broadcast to locals and send to cloud
    }
}
