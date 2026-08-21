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
    // Mechanical. steps_per_rev * microsteps is the only mechanical quantity
    // the kinematics consume (rho coupling compensation); the rho radial scale
    // comes from homing-observed calibration, so no pinion/lead geometry is
    // stored. Acceleration is not configurable here either — the ramp shape is
    // set by MotionConfig::STEP_ACCEL_STEPS_S2 (steps/s^2).
    uint32_t steps_per_rev = 200;
    uint16_t microsteps = 16;

    // Velocity limit (RPM) — default path feedrate. Theta spin rate is bounded
    // separately by the compile-time MotionConfig::THETA_MAX_ROT_PER_MIN clamp.
    int32_t rho_max_rpm = 15;

    // Motor current (mA)
    uint16_t theta_current_ma = 400;
    uint16_t rho_current_ma = 400;

    // StallGuard
    uint8_t stallguard_threshold = 30;

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
    // Removed keys ("theta_gear" gear ratio, "pinion_dia" pinion diameter,
    // "accel_def"/"accel_max" mm/s^2 acceleration, "theta_rpm" theta max RPM)
    // are no longer read: the coupling derives from homing-observed
    // calibration, the ramp shape is a compile-time step acceleration, and the
    // theta spin rate is bounded by the compile-time THETA_MAX_ROT_PER_MIN
    // clamp. Stale values may linger in NVS on upgraded devices — harmless,
    // they are simply ignored.
    static constexpr const char* kKeyRhoMaxRpm = "rho_rpm";
    static constexpr const char* kKeyThetaCurrMa = "theta_curr";
    static constexpr const char* kKeyRhoCurrMa = "rho_curr";
    static constexpr const char* kKeySgThreshold = "sg_thresh";

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
