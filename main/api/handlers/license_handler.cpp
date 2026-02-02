// License handler implementation
#include "license_handler.h"
#include "drm/drm_license.h"

#include <esp_log.h>

static const char* TAG = "license_handler";

LicenseHandler& LicenseHandler::instance() {
    static LicenseHandler instance;
    return instance;
}

HandleResult LicenseHandler::handle(
    const Kd__V1__TranquilMessage* msg,
    const MessageContext& ctx,
    ResponseMessage& response
) {
    (void)ctx;

    switch (msg->message_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_STORE_TOKEN_REQUEST:
            return handleGetStoreToken(response);

        default:
            return HandleResult::notHandled();
    }
}

bool LicenseHandler::canHandle(Kd__V1__TranquilMessage__MessageCase msg_case) const {
    switch (msg_case) {
        case KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_STORE_TOKEN_REQUEST:
            return true;
        default:
            return false;
    }
}

std::vector<Kd__V1__TranquilMessage__MessageCase> LicenseHandler::supportedMessages() const {
    return {
        KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_STORE_TOKEN_REQUEST
    };
}

bool LicenseHandler::isLocalOnly(Kd__V1__TranquilMessage__MessageCase msg_case) const {
    // Store token requests should only come from local clients
    return msg_case == KD__V1__TRANQUIL_MESSAGE__MESSAGE_GET_STORE_TOKEN_REQUEST;
}

HandleResult LicenseHandler::handleGetStoreToken(ResponseMessage& response) {
    static Kd__V1__StoreTokenResponse token_resp = KD__V1__STORE_TOKEN_RESPONSE__INIT;
    static char token_buffer[DRM_STORE_TOKEN_MAX_SIZE + 1];

    if (!drm_license_is_valid()) {
        token_resp.success = false;
        token_resp.store_token = nullptr;
        token_resp.error = const_cast<char*>("No valid license");
        ESP_LOGW(TAG, "Store token request failed: no valid license");
    } else {
        size_t token_len = 0;
        esp_err_t ret = drm_license_get_store_token(token_buffer, &token_len);
        if (ret == ESP_OK && token_len > 0) {
            token_resp.success = true;
            token_resp.store_token = token_buffer;
            token_resp.error = nullptr;
            ESP_LOGI(TAG, "Store token retrieved (%zu bytes)", token_len);
        } else {
            token_resp.success = false;
            token_resp.store_token = nullptr;
            token_resp.error = const_cast<char*>("No store token in license");
            ESP_LOGW(TAG, "Store token request failed: no token in license");
        }
    }

    Kd__V1__TranquilMessage resp = KD__V1__TRANQUIL_MESSAGE__INIT;
    resp.message_case = KD__V1__TRANQUIL_MESSAGE__MESSAGE_STORE_TOKEN_RESPONSE;
    resp.store_token_response = &token_resp;

    response = serialize(&resp);
    return HandleResult::ok(false, false);  // Unicast response only
}
