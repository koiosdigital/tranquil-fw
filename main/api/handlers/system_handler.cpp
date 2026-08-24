// Unified system handler implementation
#include "system_handler.h"
#include "SandTablePlayer.h"
#include "factory_reset.h"

#include <esp_log.h>
#include <esp_app_desc.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <kd_common.h>

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
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_HOME_REQUEST
    };
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
    // Locals, not statics: httpd, the cloudlink task, and the state broadcaster
    // can run this concurrently, and two callers racing between
    // get_packed_size() and pack() inside serialize() corrupt the message.
    // These are tiny and only need to outlive the serialize() call below.
    Kd__V1__SystemInfo info = KD__V1__SYSTEM_INFO__INIT;
    char model[] = "tranquil";

    const esp_app_desc_t* app_desc = esp_app_get_description();

    info.firmware_version = const_cast<char*>(app_desc->version);
    info.hardware_model = model;
    info.device_id = kd_common_get_device_name();
    info.is_homed = SandTablePlayer::isHomed();
    info.free_heap = esp_get_free_heap_size();
    info.hostname = kd_common_get_wifi_hostname();

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_SYSTEM_INFO;
    resp.system_info = &info;

    response = serialize(&resp);
    return HandleResult::ok(false, false);
}

// Timer callback to restart device after factory reset
static void restart_timer_callback(void* arg) {
    (void)arg;
    ESP_LOGW(TAG, "Restarting device after factory reset...");
    esp_restart();
}

// Convert MotionError to string for error messages
static const char* motion_error_to_string(sand_table::MotionError err) {
    switch (err) {
        case sand_table::MotionError::None: return "None";
        case sand_table::MotionError::NotHomed: return "Not homed";
        case sand_table::MotionError::OutOfBounds: return "Out of bounds";
        case sand_table::MotionError::QueueFull: return "Queue full";
        case sand_table::MotionError::QueueEmpty: return "Queue empty";
        case sand_table::MotionError::InvalidState: return "Invalid state";
        case sand_table::MotionError::HardwareFault: return "Hardware fault";
        case sand_table::MotionError::StallDetected: return "Stall detected";
        case sand_table::MotionError::Timeout: return "Timeout";
        case sand_table::MotionError::HomingFailed: return "Homing failed";
        case sand_table::MotionError::EmergencyStop: return "Emergency stop";
        default: return "Unknown error";
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

    // Schedule restart after a short delay to allow response to be sent
    esp_timer_handle_t restart_timer;
    esp_timer_create_args_t timer_args = {
        .callback = restart_timer_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "factory_reset_restart",
    };

    err = esp_timer_create(&timer_args, &restart_timer);
    if (err == ESP_OK) {
        esp_timer_start_once(restart_timer, 300 * 1000);  // 300ms delay
        ESP_LOGI(TAG, "Device will restart in 300ms");
    } else {
        ESP_LOGW(TAG, "Failed to create restart timer, restarting immediately");
        esp_restart();
    }

    return HandleResult::ok(false, false);
}

HandleResult SystemHandler::handleHomeRequest(
    const Kd__V1__HomeRequest* msg,
    ResponseMessage& response
) {
    bool force_full = msg ? msg->force_full_calibration : false;

    ESP_LOGI(TAG, "Home request (force_full=%d)", force_full);

    // Locals, not statics: this handler can be dispatched concurrently from
    // httpd and the cloudlink task; shared statics race in serialize().
    Kd__V1__HomeResponse home_resp = KD__V1__HOME_RESPONSE__INIT;
    char error_buf[128];

    auto* motion_controller = SandTablePlayer::getMotionController();
    if (!motion_controller) {
        home_resp.success = false;
        home_resp.error = const_cast<char*>("Motion controller not initialized");
    } else {
        auto result = motion_controller->home(force_full);
        if (result.is_ok()) {
            home_resp.success = true;
            home_resp.error = nullptr;
        } else {
            home_resp.success = false;
            snprintf(error_buf, sizeof(error_buf), "Homing failed: %s", motion_error_to_string(result.error()));
            home_resp.error = error_buf;
        }
    }

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_HOME_RESPONSE;
    resp.home_response = &home_resp;

    response = serialize(&resp);
    return HandleResult::ok(false, false);
}
