// Unified LED control handler implementation
#include "led_handler.h"
#include "led_state.h"
#include "ops/led_ops.h"

#include <esp_log.h>
#include <algorithm>
#include <cstring>
#include <string>

static const char* TAG = "led_handler";

LEDHandler& LEDHandler::instance() {
    static LEDHandler instance;
    return instance;
}

HandleResult LEDHandler::handle(
    const Kd__V1__TranquilMessage* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    const char* rid = ctx.request_id;

    switch (msg->message_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_LED_CONFIG_REQUEST:
            response = led_state::build_config_message(rid);
            return HandleResult::ok();

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_LED_CHANNEL: {
            const auto* m = msg->set_led_channel;
            if (!m) {
                response = makeCommandResult(false, "Invalid message", rid, ESP_ERR_INVALID_ARG);
                return HandleResult::ok();
            }

            // Partial update: proto3 `optional` fields carry presence (protobuf-c
            // exposes it as the synthetic *_case), effect_id "" = unchanged,
            // absent color = unchanged. So a cloud "power on" never zeroes
            // brightness. Range clamping happens in led_state.
            led_state::ChannelPatch patch;
            if (m->effect_id && m->effect_id[0]) patch.effect_id = std::string(m->effect_id);
            if (m->_brightness_case == KD__V1__SET_LEDCHANNEL___BRIGHTNESS_BRIGHTNESS) {
                patch.brightness = static_cast<int>(std::min<uint32_t>(m->brightness, 255));
            }
            if (m->_speed_case == KD__V1__SET_LEDCHANNEL___SPEED_SPEED) {
                patch.speed = static_cast<int>(std::min<uint32_t>(m->speed, 10));
            }
            if (m->_enabled_case == KD__V1__SET_LEDCHANNEL___ENABLED_ENABLED) {
                patch.enabled = m->enabled != 0;
            }
            if (m->color) {
                patch.color = PixelColor(
                    static_cast<uint8_t>(m->color->r), static_cast<uint8_t>(m->color->g),
                    static_cast<uint8_t>(m->color->b), static_cast<uint8_t>(m->color->w),
                    static_cast<uint8_t>(m->color->cw));
            }

            ESP_LOGI(TAG, "SetLEDChannel: ch=%u", m->channel);
            // Requester gets the CommandResult; the resulting LEDConfig snapshot
            // reaches LAN clients and the cloud through led_state::notify_changed().
            response = makeCommandResult(led_ops::set_channel(m->channel, patch), rid);
            return HandleResult::ok();
        }

        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_LED_EFFECTS:
            response = led_ops::build_effects_message(rid);
            return HandleResult::ok();

        default:
            return HandleResult::notHandled();
    }
}

bool LEDHandler::canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const {
    switch (msg_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_LED_CONFIG_REQUEST:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_LED_CHANNEL:
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_LED_EFFECTS:
            return true;
        default:
            return false;
    }
}

std::vector<Kd__V1__TranquilMessage__MessageCase> LEDHandler::supportedMessages() const {
    return {
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_LED_CONFIG_REQUEST,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_SET_LED_CHANNEL,
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_LED_EFFECTS
    };
}
