#pragma once

#include "esp_err.h"

/**
 * Perform a factory reset of the device.
 *
 * Clears:
 * - WiFi credentials
 * - NTP configuration
 * - Claim token
 * - Calibration data
 * - Robot/motion configuration
 * - LED/PixelDriver configuration
 * - Schedule configuration
 * - License file (SD)
 * - All patterns and playlists (SD)
 * - Manifest database (SD)
 *
 * Preserves:
 * - Device certificate
 * - DS parameters (private key material)
 *
 * @return ESP_OK on success
 */
esp_err_t tranquil_factory_reset();
