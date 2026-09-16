// Result of one operation in the ops core (main/api/ops).
//
// REST handlers turn this into an HTTP status through
// api_common::send_op_error(); WebSocket handlers turn it into a
// CommandResult through HandlerBase::makeCommandResult(). `detail` is always
// a static string (never null, empty on success) so both can hand it on
// without an ownership check.
#pragma once

#include <esp_err.h>

struct OpResult {
    esp_err_t err = ESP_OK;
    const char* detail = "";

    bool ok() const { return err == ESP_OK; }

    static OpResult success() { return OpResult{ ESP_OK, "" }; }

    // A failure always carries a non-OK code, even if the caller passed ESP_OK.
    static OpResult fail(esp_err_t e, const char* d) {
        return OpResult{ e == ESP_OK ? ESP_FAIL : e, d ? d : "" };
    }
};
