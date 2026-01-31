#include "config_manager.h"
#include "config.h"
#include "presets.h"
#include "nvs_handle.h"

#include <esp_log.h>
#include <cstring>
#include <cmath>

static const char* TAG = "ConfigManager";

namespace sand_table {

    ConfigManager& ConfigManager::instance() {
        static ConfigManager instance;
        return instance;
    }

    esp_err_t ConfigManager::init() {
        if (initialized_) {
            return ESP_OK;
        }

        ESP_LOGI(TAG, "Initializing ConfigManager");

        // Try to load from NVS, fall back to hardcoded defaults
        esp_err_t err = load_motion_config();
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "No motion config in NVS, applying defaults");
            apply_defaults_from_sdkconfig();
            save_motion_config();
        }
        else if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "Error loading motion config: %s, using defaults", esp_err_to_name(err));
            apply_defaults_from_sdkconfig();
        }

        err = load_led_config();
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "No LED config in NVS, applying defaults");
            // LED defaults are set in apply_defaults_from_sdkconfig
            save_led_config();
        }
        else if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "Error loading LED config: %s, using defaults", esp_err_to_name(err));
        }

        err = load_calibration();
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "Error loading calibration: %s", esp_err_to_name(err));
            calibration_ = CalibrationData{};  // Reset to defaults (invalid)
        }

        err = load_active_preset_id();
        if (err != ESP_OK) {
            active_preset_id_[0] = '\0';  // No active preset
        }

        initialized_ = true;

        ESP_LOGI(TAG, "ConfigManager initialized - motion: gear_ratio=%.2f, theta_curr=%dmA",
            motion_.theta_gear_ratio(), motion_.theta_current_ma);
        ESP_LOGI(TAG, "LED config: has_leds=%d, count=%d, rgbw=%d",
            led_.has_leds, led_.led_count, led_.is_rgbw);
        ESP_LOGI(TAG, "Calibration: valid=%d, theta_steps=%d, rho_max=%d",
            calibration_.is_valid, calibration_.theta_steps_per_rotation, calibration_.rho_max_steps);

        return ESP_OK;
    }

    void ConfigManager::apply_defaults_from_sdkconfig() {
        // Motion defaults - use values from tranquil4_tabletop preset as baseline
        // These are only used on first boot before a preset is loaded
        motion_.steps_per_rev = 200;
        motion_.microsteps = 16;  // TMC2209 default
        motion_.theta_gear_ratio_x100 = 805;  // 8.05:1
        motion_.pinion_diameter_mm = 12;
        motion_.theta_max_rpm = 15;
        motion_.rho_max_rpm = 15;
        motion_.theta_current_ma = 400;
        motion_.rho_current_ma = 400;
        motion_.stallguard_threshold = 30;
        motion_.default_accel = 100.0f;
        motion_.max_accel = 200.0f;

        // LED defaults - assume LEDs present with typical tabletop config
        led_.has_leds = true;
        led_.led_count = 143;
        led_.is_rgbw = true;

        // Calibration starts invalid
        calibration_ = CalibrationData{};
    }

    // =============================================================================
    // Load/Save Motion Config
    // =============================================================================

    esp_err_t ConfigManager::load_motion_config() {
        kd::NvsHandle nvs(kNamespace, NVS_READONLY);
        if (!nvs) {
            return nvs.open_error();
        }

        // Load each field individually
        // If any key is missing, we'll use the default value already set
        nvs.get_u32(kKeyStepsPerRev, &motion_.steps_per_rev);
        nvs.get_u16(kKeyMicrosteps, &motion_.microsteps);
        nvs.get_i32(kKeyThetaGearX100, &motion_.theta_gear_ratio_x100);
        nvs.get_i32(kKeyPinionDia, &motion_.pinion_diameter_mm);
        nvs.get_i32(kKeyThetaMaxRpm, &motion_.theta_max_rpm);
        nvs.get_i32(kKeyRhoMaxRpm, &motion_.rho_max_rpm);
        nvs.get_u16(kKeyThetaCurrMa, &motion_.theta_current_ma);
        nvs.get_u16(kKeyRhoCurrMa, &motion_.rho_current_ma);
        nvs.get_u8(kKeySgThreshold, &motion_.stallguard_threshold);

        // Load floats as u32 bit representations
        uint32_t accel_bits;
        if (nvs.get_u32(kKeyAccelDefault, &accel_bits) == ESP_OK) {
            memcpy(&motion_.default_accel, &accel_bits, sizeof(float));
        }
        if (nvs.get_u32(kKeyAccelMax, &accel_bits) == ESP_OK) {
            memcpy(&motion_.max_accel, &accel_bits, sizeof(float));
        }

        // Check if we found at least one key (to distinguish empty from missing)
        return nvs.find_key(kKeyStepsPerRev);
    }

    esp_err_t ConfigManager::save_motion_config() {
        kd::NvsHandle nvs(kNamespace, NVS_READWRITE);
        if (!nvs) {
            ESP_LOGE(TAG, "Failed to open NVS for writing: %s", esp_err_to_name(nvs.open_error()));
            return nvs.open_error();
        }

        esp_err_t err;

        err = nvs.set_u32(kKeyStepsPerRev, motion_.steps_per_rev);
        if (err != ESP_OK) return err;

        err = nvs.set_u16(kKeyMicrosteps, motion_.microsteps);
        if (err != ESP_OK) return err;

        err = nvs.set_i32(kKeyThetaGearX100, motion_.theta_gear_ratio_x100);
        if (err != ESP_OK) return err;

        err = nvs.set_i32(kKeyPinionDia, motion_.pinion_diameter_mm);
        if (err != ESP_OK) return err;

        err = nvs.set_i32(kKeyThetaMaxRpm, motion_.theta_max_rpm);
        if (err != ESP_OK) return err;

        err = nvs.set_i32(kKeyRhoMaxRpm, motion_.rho_max_rpm);
        if (err != ESP_OK) return err;

        err = nvs.set_u16(kKeyThetaCurrMa, motion_.theta_current_ma);
        if (err != ESP_OK) return err;

        err = nvs.set_u16(kKeyRhoCurrMa, motion_.rho_current_ma);
        if (err != ESP_OK) return err;

        err = nvs.set_u8(kKeySgThreshold, motion_.stallguard_threshold);
        if (err != ESP_OK) return err;

        // Store floats as u32 bit representations
        uint32_t accel_bits;
        memcpy(&accel_bits, &motion_.default_accel, sizeof(float));
        err = nvs.set_u32(kKeyAccelDefault, accel_bits);
        if (err != ESP_OK) return err;

        memcpy(&accel_bits, &motion_.max_accel, sizeof(float));
        err = nvs.set_u32(kKeyAccelMax, accel_bits);
        if (err != ESP_OK) return err;

        return nvs.commit();
    }

    esp_err_t ConfigManager::set_motion_config(const RuntimeMotionConfig& config) {
        motion_ = config;
        return save_motion_config();
    }

    // =============================================================================
    // Load/Save LED Config
    // =============================================================================

    esp_err_t ConfigManager::load_led_config() {
        kd::NvsHandle nvs(kNamespace, NVS_READONLY);
        if (!nvs) {
            return nvs.open_error();
        }

        uint8_t val;
        if (nvs.get_u8(kKeyHasLeds, &val) == ESP_OK) {
            led_.has_leds = (val != 0);
        }

        nvs.get_u16(kKeyLedCount, &led_.led_count);

        if (nvs.get_u8(kKeyLedIsRgbw, &val) == ESP_OK) {
            led_.is_rgbw = (val != 0);
        }

        return nvs.find_key(kKeyHasLeds);
    }

    esp_err_t ConfigManager::save_led_config() {
        kd::NvsHandle nvs(kNamespace, NVS_READWRITE);
        if (!nvs) {
            return nvs.open_error();
        }

        esp_err_t err;

        err = nvs.set_u8(kKeyHasLeds, led_.has_leds ? 1 : 0);
        if (err != ESP_OK) return err;

        err = nvs.set_u16(kKeyLedCount, led_.led_count);
        if (err != ESP_OK) return err;

        err = nvs.set_u8(kKeyLedIsRgbw, led_.is_rgbw ? 1 : 0);
        if (err != ESP_OK) return err;

        return nvs.commit();
    }

    esp_err_t ConfigManager::set_led_config(const RuntimeLEDConfig& config) {
        led_ = config;
        return save_led_config();
    }

    // =============================================================================
    // Calibration Data
    // =============================================================================

    esp_err_t ConfigManager::load_calibration() {
        kd::NvsHandle nvs(kNamespace, NVS_READONLY);
        if (!nvs) {
            return nvs.open_error();
        }

        uint8_t valid;
        esp_err_t err = nvs.get_u8(kKeyCalValid, &valid);
        if (err != ESP_OK) {
            calibration_.is_valid = false;
            return err;
        }

        calibration_.is_valid = (valid != 0);
        if (!calibration_.is_valid) {
            return ESP_OK;  // No calibration data, but that's fine
        }

        nvs.get_i32(kKeyCalThetaSteps, &calibration_.theta_steps_per_rotation);
        nvs.get_i32(kKeyCalRhoMax, &calibration_.rho_max_steps);
        nvs.get_u32(kKeyCalTimestamp, &calibration_.timestamp);

        return ESP_OK;
    }

    esp_err_t ConfigManager::save_calibration(const CalibrationData& data) {
        kd::NvsHandle nvs(kNamespace, NVS_READWRITE);
        if (!nvs) {
            return nvs.open_error();
        }

        esp_err_t err;

        err = nvs.set_u8(kKeyCalValid, data.is_valid ? 1 : 0);
        if (err != ESP_OK) return err;

        err = nvs.set_i32(kKeyCalThetaSteps, data.theta_steps_per_rotation);
        if (err != ESP_OK) return err;

        err = nvs.set_i32(kKeyCalRhoMax, data.rho_max_steps);
        if (err != ESP_OK) return err;

        err = nvs.set_u32(kKeyCalTimestamp, data.timestamp);
        if (err != ESP_OK) return err;

        err = nvs.commit();
        if (err == ESP_OK) {
            calibration_ = data;
            ESP_LOGI(TAG, "Saved calibration: theta=%d, rho_max=%d",
                data.theta_steps_per_rotation, data.rho_max_steps);
        }

        return err;
    }

    esp_err_t ConfigManager::clear_calibration() {
        calibration_ = CalibrationData{};
        calibration_.is_valid = false;

        kd::NvsHandle nvs(kNamespace, NVS_READWRITE);
        if (!nvs) {
            return nvs.open_error();
        }

        nvs.set_u8(kKeyCalValid, 0);
        return nvs.commit();
    }

    // =============================================================================
    // Preset Management
    // =============================================================================

    esp_err_t ConfigManager::load_active_preset_id() {
        kd::NvsHandle nvs(kNamespace, NVS_READONLY);
        if (!nvs) {
            return nvs.open_error();
        }

        size_t len = sizeof(active_preset_id_);
        return nvs.get_str(kKeyActivePreset, active_preset_id_, &len);
    }

    std::vector<PresetInfo> ConfigManager::list_presets() const {
        std::vector<PresetInfo> result;
        result.reserve(kPresetCount);

        for (size_t i = 0; i < kPresetCount; ++i) {
            result.push_back({
                .id = kPresets[i]->id,
                .name = kPresets[i]->name,
                .description = kPresets[i]->description,
                });
        }

        return result;
    }

    esp_err_t ConfigManager::load_preset(const char* preset_id) {
        const Preset* preset = find_preset(preset_id);
        if (!preset) {
            ESP_LOGE(TAG, "Preset not found: %s", preset_id);
            return ESP_ERR_NOT_FOUND;
        }

        ESP_LOGI(TAG, "Loading preset: %s", preset->name);

        // Apply preset values
        motion_ = preset->motion;
        led_ = preset->led;

        // Save to NVS
        esp_err_t err = save_motion_config();
        if (err != ESP_OK) return err;

        err = save_led_config();
        if (err != ESP_OK) return err;

        // Save active preset ID
        kd::NvsHandle nvs(kNamespace, NVS_READWRITE);
        if (!nvs) {
            return nvs.open_error();
        }

        err = nvs.set_str(kKeyActivePreset, preset_id);
        if (err != ESP_OK) return err;

        err = nvs.commit();
        if (err == ESP_OK) {
            strncpy(active_preset_id_, preset_id, sizeof(active_preset_id_) - 1);
            active_preset_id_[sizeof(active_preset_id_) - 1] = '\0';
        }

        return err;
    }

    // =============================================================================
    // Reset to Defaults
    // =============================================================================

    esp_err_t ConfigManager::reset_to_defaults() {
        ESP_LOGI(TAG, "Resetting to defaults");

        apply_defaults_from_sdkconfig();
        calibration_ = CalibrationData{};
        active_preset_id_[0] = '\0';

        esp_err_t err = save_motion_config();
        if (err != ESP_OK) return err;

        err = save_led_config();
        if (err != ESP_OK) return err;

        err = clear_calibration();
        if (err != ESP_OK) return err;

        // Clear active preset
        kd::NvsHandle nvs(kNamespace, NVS_READWRITE);
        if (nvs) {
            nvs.erase_key(kKeyActivePreset);
            nvs.commit();
        }

        return ESP_OK;
    }

    // =============================================================================
    // MechanicalConfig Runtime Accessors
    // =============================================================================

    uint32_t MechanicalConfig::steps_per_rev() {
        return ConfigManager::instance().motion_config().steps_per_rev;
    }

    uint32_t MechanicalConfig::microsteps() {
        return ConfigManager::instance().motion_config().microsteps;
    }

    uint32_t MechanicalConfig::effective_steps_per_rev() {
        return ConfigManager::instance().motion_config().effective_steps_per_rev();
    }

    int32_t MechanicalConfig::theta_gear_ratio_x100() {
        return ConfigManager::instance().motion_config().theta_gear_ratio_x100;
    }

    double MechanicalConfig::theta_gear_ratio() {
        return ConfigManager::instance().motion_config().theta_gear_ratio();
    }

    int32_t MechanicalConfig::steps_per_theta_rotation() {
        return ConfigManager::instance().motion_config().steps_per_theta_rotation();
    }

    int32_t MechanicalConfig::pinion_diameter_mm() {
        return ConfigManager::instance().motion_config().pinion_diameter_mm;
    }

    double MechanicalConfig::pinion_circumference_mm() {
        return ConfigManager::instance().motion_config().pinion_diameter_mm * M_PI;
    }

    double MechanicalConfig::rho_steps_per_mm() {
        const auto& cfg = ConfigManager::instance().motion_config();
        return static_cast<double>(cfg.effective_steps_per_rev()) /
            (cfg.pinion_diameter_mm * M_PI);
    }

    double MechanicalConfig::rho_steps_per_theta_step() {
        return ConfigManager::instance().motion_config().rho_steps_per_theta_step();
    }

    // =============================================================================
    // MotionConfig Runtime Accessors
    // =============================================================================

    int32_t MotionConfig::theta_max_rpm() {
        return ConfigManager::instance().motion_config().theta_max_rpm;
    }

    int32_t MotionConfig::rho_max_rpm() {
        return ConfigManager::instance().motion_config().rho_max_rpm;
    }

    uint16_t MotionConfig::theta_irun_ma() {
        return ConfigManager::instance().motion_config().theta_current_ma;
    }

    uint16_t MotionConfig::theta_ihold_ma() {
        return ConfigManager::instance().motion_config().theta_current_ma / 2;
    }

    uint16_t MotionConfig::rho_irun_ma() {
        return ConfigManager::instance().motion_config().rho_current_ma;
    }

    uint16_t MotionConfig::rho_ihold_ma() {
        return ConfigManager::instance().motion_config().rho_current_ma / 2;
    }

    uint8_t MotionConfig::stallguard_threshold() {
        return ConfigManager::instance().motion_config().stallguard_threshold;
    }

    float MotionConfig::default_accel() {
        return ConfigManager::instance().motion_config().default_accel;
    }

    float MotionConfig::max_accel() {
        return ConfigManager::instance().motion_config().max_accel;
    }

} // namespace sand_table
