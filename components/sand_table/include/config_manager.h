#pragma once

#include <cstdint>
#include <esp_err.h>
#include <vector>

namespace sand_table {

// Forward declarations
struct Preset;

// =============================================================================
// Runtime Motion Configuration (stored in NVS)
// =============================================================================

struct RuntimeMotionConfig {
    // Mechanical
    uint32_t steps_per_rev = 200;
    uint16_t microsteps = 16;
    int32_t pinion_diameter_mm = 12;

    // Velocity limits (RPM)
    int32_t theta_max_rpm = 15;
    int32_t rho_max_rpm = 15;

    // Motor current (mA)
    uint16_t theta_current_ma = 400;
    uint16_t rho_current_ma = 400;

    // StallGuard
    uint8_t stallguard_threshold = 30;

    // Acceleration (mm/s^2)
    float default_accel = 100.0f;
    float max_accel = 200.0f;

    // Derived calculations
    uint32_t effective_steps_per_rev() const {
        return steps_per_rev * microsteps;
    }
};

// =============================================================================
// Runtime LED Configuration (stored in NVS)
// =============================================================================

struct RuntimeLEDConfig {
    bool has_leds = true;
    uint16_t led_count = 143;
    bool is_rgbw = true;
};

// =============================================================================
// Calibration Data (stored in NVS)
// =============================================================================

struct CalibrationData {
    int32_t theta_steps_per_rotation = 0;
    int32_t rho_max_steps = 0;
    bool is_valid = false;
    uint32_t timestamp = 0;  // epoch seconds
};

// =============================================================================
// Preset Info (for listing presets)
// =============================================================================

struct PresetInfo {
    const char* id;
    const char* name;
    const char* description;
};

// =============================================================================
// ConfigManager - Singleton for managing runtime configuration
// =============================================================================

class ConfigManager {
public:
    static ConfigManager& instance();

    // Initialization - must be called before using any other methods
    esp_err_t init();

    // Motion config access
    const RuntimeMotionConfig& motion_config() const { return motion_; }
    esp_err_t set_motion_config(const RuntimeMotionConfig& config);

    // LED config access
    const RuntimeLEDConfig& led_config() const { return led_; }
    esp_err_t set_led_config(const RuntimeLEDConfig& config);

    // Calibration data
    const CalibrationData& calibration() const { return calibration_; }
    esp_err_t save_calibration(const CalibrationData& data);
    esp_err_t clear_calibration();
    bool has_valid_calibration() const { return calibration_.is_valid; }

    // Preset management
    std::vector<PresetInfo> list_presets() const;
    esp_err_t load_preset(const char* preset_id);
    const char* active_preset_id() const { return active_preset_id_; }

    // Reset to defaults
    esp_err_t reset_to_defaults();

private:
    ConfigManager() = default;
    ~ConfigManager() = default;

    // Non-copyable
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    // NVS namespace
    static constexpr const char* kNamespace = "tranquil_cfg";

    // NVS keys - Motion
    static constexpr const char* kKeyStepsPerRev = "steps_per_rev";
    static constexpr const char* kKeyMicrosteps = "microsteps";
    // "theta_gear" was the legacy gear-ratio key; the coupling now derives
    // from homing-observed calibration, so the key is no longer read (stale
    // values may linger in NVS on upgraded devices — harmless).
    static constexpr const char* kKeyPinionDia = "pinion_dia";
    static constexpr const char* kKeyThetaMaxRpm = "theta_rpm";
    static constexpr const char* kKeyRhoMaxRpm = "rho_rpm";
    static constexpr const char* kKeyThetaCurrMa = "theta_curr";
    static constexpr const char* kKeyRhoCurrMa = "rho_curr";
    static constexpr const char* kKeySgThreshold = "sg_thresh";
    static constexpr const char* kKeyAccelDefault = "accel_def";
    static constexpr const char* kKeyAccelMax = "accel_max";

    // NVS keys - LED
    static constexpr const char* kKeyHasLeds = "has_leds";
    static constexpr const char* kKeyLedCount = "led_count";
    static constexpr const char* kKeyLedIsRgbw = "led_rgbw";

    // NVS keys - Calibration
    static constexpr const char* kKeyCalThetaSteps = "cal_theta";
    static constexpr const char* kKeyCalRhoMax = "cal_rho";
    static constexpr const char* kKeyCalValid = "cal_valid";
    static constexpr const char* kKeyCalTimestamp = "cal_time";

    // NVS keys - Preset
    static constexpr const char* kKeyActivePreset = "preset_id";

    // Load helpers
    esp_err_t load_motion_config();
    esp_err_t load_led_config();
    esp_err_t load_calibration();
    esp_err_t load_active_preset_id();

    // Save helpers
    esp_err_t save_motion_config();
    esp_err_t save_led_config();

    // Apply defaults from sdkconfig
    void apply_defaults_from_sdkconfig();

    // In-memory cache
    RuntimeMotionConfig motion_;
    RuntimeLEDConfig led_;
    CalibrationData calibration_;
    char active_preset_id_[32] = {0};

    bool initialized_ = false;
};

} // namespace sand_table
