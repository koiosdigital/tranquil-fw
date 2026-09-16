// Asynchronous homing.
//
// MotionController::home() runs the whole seek in the CALLER's task and can
// take tens of seconds. Running it inside an httpd handler froze every REST
// request and WebSocket frame for the duration (and the app's own request
// timed out). This service runs it on a short-lived task instead; callers
// get an immediate "started" and watch `is_homing` / `is_homed` via
// SystemInfo, which is pushed to LAN + cloud when the run finishes.
//
// The homing task stops the player first and waits for it to report
// STOPPED (the motion controller refuses to home during playback).
#pragma once

#include <esp_err.h>

namespace homing_service {

// Start a homing run. ESP_ERR_INVALID_STATE if one is already running,
// ESP_FAIL if there is no motion controller, ESP_ERR_NO_MEM if the task
// could not be created.
esp_err_t start(bool force_full_calibration);

bool is_homing();

}  // namespace homing_service
