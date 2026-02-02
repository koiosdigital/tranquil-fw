// Unified config handler implementation (local-only)
#include "config_handler.h"
#include "config_manager.h"

#include <esp_log.h>
#include <cstring>

static const char* TAG = "config_handler";

ConfigHandler& ConfigHandler::instance() {
    static ConfigHandler instance;
    return instance;
}

HandleResult ConfigHandler::handle(
    const Kd__V1__TranquilMessage* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    // Config messages are local-only
    if (!ctx.isLocal()) {
        ESP_LOGW(TAG, "Config message from non-local source, ignoring");
        return HandleResult::notHandled();
    }

    switch (msg->message_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_CONFIG_REQUEST:
            return handleGetConfig(response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_CONFIG_REQUEST:
            return handleSetConfig(msg->set_config_request, response);

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_CLEAR_CALIBRATION_REQUEST:
            return handleClearCalibration(response);

        default:
            return HandleResult::notHandled();
    }
}

bool ConfigHandler::canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const {
    switch (msg_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_CONFIG_REQUEST:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_CONFIG_REQUEST:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_CLEAR_CALIBRATION_REQUEST:
            return true;
        default:
            return false;
    }
}

std::vector<Kd__V1__TranquilMessage__MessageCase> ConfigHandler::supportedMessages() const {
    return {
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_CONFIG_REQUEST,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_CONFIG_REQUEST,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_CLEAR_CALIBRATION_REQUEST
    };
}

HandleResult ConfigHandler::handleGetConfig(ResponseMessage& response) {
    auto& cfg = sand_table::ConfigManager::instance();
    const auto& motion = cfg.motion_config();
    const auto& led = cfg.led_config();
    const auto& cal = cfg.calibration();

    ESP_LOGI(TAG, "GetConfig request");

    // Build response
    static Kd__V1__GetConfigResponse config_resp = KD__V1__GET_CONFIG_RESPONSE__INIT;
    static Kd__V1__MotionConfigProto motion_proto = KD__V1__MOTION_CONFIG_PROTO__INIT;
    static Kd__V1__LEDConfigProto led_proto = KD__V1__LEDCONFIG_PROTO__INIT;
    static Kd__V1__CalibrationDataProto cal_proto = KD__V1__CALIBRATION_DATA_PROTO__INIT;
    static char preset_id[32] = {0};

    // Fill motion config
    motion_proto.steps_per_rev = motion.steps_per_rev;
    motion_proto.microsteps = motion.microsteps;
    motion_proto.theta_gear_ratio_x100 = motion.theta_gear_ratio_x100;
    motion_proto.pinion_diameter_mm = motion.pinion_diameter_mm;
    motion_proto.theta_max_rpm = motion.theta_max_rpm;
    motion_proto.rho_max_rpm = motion.rho_max_rpm;
    motion_proto.theta_current_ma = motion.theta_current_ma;
    motion_proto.rho_current_ma = motion.rho_current_ma;
    motion_proto.stallguard_threshold = motion.stallguard_threshold;
    motion_proto.default_accel_mm_s2 = motion.default_accel;
    motion_proto.max_accel_mm_s2 = motion.max_accel;

    // Fill LED config
    led_proto.has_leds = led.has_leds;
    led_proto.led_count = led.led_count;
    led_proto.is_rgbw = led.is_rgbw;

    // Fill calibration data
    cal_proto.theta_steps_per_rotation = cal.theta_steps_per_rotation;
    cal_proto.rho_max_steps = cal.rho_max_steps;
    cal_proto.is_valid = cal.is_valid;
    cal_proto.timestamp = cal.timestamp;

    // Active preset
    strncpy(preset_id, cfg.active_preset_id(), sizeof(preset_id) - 1);

    config_resp.motion = &motion_proto;
    config_resp.led = &led_proto;
    config_resp.calibration = &cal_proto;
    config_resp.active_preset_id = preset_id;

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_CONFIG_RESPONSE;
    resp.get_config_response = &config_resp;

    response = serialize(&resp);
    return HandleResult::ok();
}

HandleResult ConfigHandler::handleSetConfig(
    const Kd__V1__SetConfigRequest* msg,
    ResponseMessage& response
) {
    if (!msg) {
        response = makeCommandResult(false, "Invalid message");
        return HandleResult::ok();
    }

    auto& cfg = sand_table::ConfigManager::instance();
    bool changed = false;

    // Update motion config if provided
    if (msg->motion) {
        sand_table::RuntimeMotionConfig motion = cfg.motion_config();

        if (msg->motion->steps_per_rev > 0) motion.steps_per_rev = msg->motion->steps_per_rev;
        if (msg->motion->microsteps > 0) motion.microsteps = msg->motion->microsteps;
        if (msg->motion->theta_gear_ratio_x100 != 0) motion.theta_gear_ratio_x100 = msg->motion->theta_gear_ratio_x100;
        if (msg->motion->pinion_diameter_mm > 0) motion.pinion_diameter_mm = msg->motion->pinion_diameter_mm;
        if (msg->motion->theta_max_rpm > 0) motion.theta_max_rpm = msg->motion->theta_max_rpm;
        if (msg->motion->rho_max_rpm > 0) motion.rho_max_rpm = msg->motion->rho_max_rpm;
        if (msg->motion->theta_current_ma > 0) motion.theta_current_ma = msg->motion->theta_current_ma;
        if (msg->motion->rho_current_ma > 0) motion.rho_current_ma = msg->motion->rho_current_ma;
        if (msg->motion->stallguard_threshold > 0) motion.stallguard_threshold = msg->motion->stallguard_threshold;
        if (msg->motion->default_accel_mm_s2 > 0) motion.default_accel = msg->motion->default_accel_mm_s2;
        if (msg->motion->max_accel_mm_s2 > 0) motion.max_accel = msg->motion->max_accel_mm_s2;

        esp_err_t ret = cfg.set_motion_config(motion);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set motion config: %s", esp_err_to_name(ret));
        } else {
            changed = true;
        }
    }

    // Update LED config if provided
    if (msg->led) {
        sand_table::RuntimeLEDConfig led = cfg.led_config();
        led.has_leds = msg->led->has_leds;
        if (msg->led->led_count > 0) led.led_count = msg->led->led_count;
        led.is_rgbw = msg->led->is_rgbw;

        esp_err_t ret = cfg.set_led_config(led);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set LED config: %s", esp_err_to_name(ret));
        } else {
            changed = true;
        }
    }

    ESP_LOGI(TAG, "SetConfig: changed=%d", changed);

    // Build response
    static Kd__V1__SetConfigResponse set_resp = KD__V1__SET_CONFIG_RESPONSE__INIT;
    set_resp.success = changed;
    set_resp.error = nullptr;

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_CONFIG_RESPONSE;
    resp.set_config_response = &set_resp;

    response = serialize(&resp);
    return HandleResult::ok();
}

HandleResult ConfigHandler::handleClearCalibration(ResponseMessage& response) {
    auto& cfg = sand_table::ConfigManager::instance();

    esp_err_t ret = cfg.clear_calibration();

    ESP_LOGI(TAG, "ClearCalibration: %s", esp_err_to_name(ret));

    // Build response
    static Kd__V1__ClearCalibrationResponse clear_resp = KD__V1__CLEAR_CALIBRATION_RESPONSE__INIT;
    clear_resp.success = (ret == ESP_OK);

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_CLEAR_CALIBRATION_RESPONSE;
    resp.clear_calibration_response = &clear_resp;

    response = serialize(&resp);
    return HandleResult::ok();
}
