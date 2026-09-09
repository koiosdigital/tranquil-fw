// Unified system handler implementation
#include "system_handler.h"
#include "system_info.h"
#include "homing_service.h"
#include "SandTablePlayer.h"
#include "factory_reset.h"

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>

static const char* TAG = "system_handler";

SystemHandler& SystemHandler::instance() {
    static SystemHandler instance;
    return instance;
}

HandleResult SystemHandler::handle(
    const Kd__V1__TranquilMessage* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    (void)ctx;

    switch (msg->message_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_PING:
            return handlePing(msg->ping, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_SYSTEM_INFO:
            return handleGetSystemInfo(response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_FACTORY_RESET_REQUEST:
            return handleFactoryReset(msg->factory_reset_request, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_REBOOT_REQUEST:
            return handleReboot(msg->reboot_request, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_HOME_REQUEST:
            return handleHomeRequest(msg->home_request, response);

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

HandleResult SystemHandler::handlePing(
    const Kd__V1__Ping* msg,
    ResponseMessage& response
) {
    Kd__V1__Pong pong = KD__V1__PONG__INIT;
    pong.timestamp = msg ? msg->timestamp : 0;

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_PONG;
    resp.pong = &pong;

    response = serialize(&resp);
    return HandleResult::ok(false, false);
}

HandleResult SystemHandler::handleGetSystemInfo(ResponseMessage& response) {
    response = system_info::build_message();
    return HandleResult::ok(false, false);
}

// Timer callback to restart the device once the response is on the wire
static void restart_timer_callback(void* arg) {
    (void)arg;
    ESP_LOGW(TAG, "Restarting device...");
    esp_restart();
}

static void schedule_restart(uint32_t delay_ms) {
    esp_timer_handle_t restart_timer;
    esp_timer_create_args_t timer_args = {
        .callback = restart_timer_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "restart",
    };
    if (esp_timer_create(&timer_args, &restart_timer) == ESP_OK) {
        esp_timer_start_once(restart_timer, static_cast<uint64_t>(delay_ms) * 1000);
        ESP_LOGI(TAG, "Device will restart in %lums", static_cast<unsigned long>(delay_ms));
    }
    else {
        ESP_LOGW(TAG, "Failed to create restart timer, restarting immediately");
        esp_restart();
    }
}

HandleResult SystemHandler::handleFactoryReset(
    const Kd__V1__FactoryResetRequest* msg,
    ResponseMessage& response
) {
    const char* reason = (msg && msg->reason) ? msg->reason : "no reason";
    ESP_LOGW(TAG, "Factory reset requested: %s", reason);

    // Perform the factory reset (clears NVS, SD card, etc.)
    esp_err_t err = tranquil_factory_reset();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Factory reset failed: %s", esp_err_to_name(err));
        response = makeCommandResult(false, "Factory reset failed");
        return HandleResult::ok(false, false);
    }

    // Send success response before scheduling restart
    response = makeCommandResult(true);
    schedule_restart(300);
    return HandleResult::ok(false, false);
}

HandleResult SystemHandler::handleReboot(
    const Kd__V1__TranquilRebootRequest* msg,
    ResponseMessage& response
) {
    const char* reason = (msg && msg->reason && msg->reason[0]) ? msg->reason : "requested";
    ESP_LOGW(TAG, "Reboot requested: %s", reason);

    // Stop motion first so the arm is not mid-move when power drops.
    SandTablePlayer::stop();

    response = makeCommandResult(true);
    schedule_restart(500);
    return HandleResult::ok(false, false);
}

HandleResult SystemHandler::handleHomeRequest(
    const Kd__V1__HomeRequest* msg,
    ResponseMessage& response
) {
    bool force_full = msg ? msg->force_full_calibration : false;

    ESP_LOGI(TAG, "Home request (force_full=%d)", force_full);

    // Homing runs asynchronously (tens of seconds); the reply confirms the
    // run STARTED. Completion is observable through SystemInfo.is_homing /
    // is_homed, which is pushed to LAN + cloud when the run ends.
    Kd__V1__HomeResponse home_resp = KD__V1__HOME_RESPONSE__INIT;
    esp_err_t err = homing_service::start(force_full);
    if (err == ESP_OK) {
        home_resp.success = true;
        home_resp.error = nullptr;
    }
    else {
        home_resp.success = false;
        home_resp.error = const_cast<char*>(
            err == ESP_ERR_INVALID_STATE ? "Homing already in progress" : "Failed to start homing");
    }

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_HOME_RESPONSE;
    resp.home_response = &home_resp;

    response = serialize(&resp);
    return HandleResult::ok(false, false);
}
