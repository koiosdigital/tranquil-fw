#include "tmc2209.h"
#include "esp_log.h"

#include <cmath>
#include <algorithm>

namespace {
    constexpr const char* TAG = "TMC2209";

    // Current calculation constants (from datasheet)
    constexpr float kVfs = 0.325f;       // Full-scale voltage
    constexpr float kRsense = 0.11f;     // Sense resistor
    constexpr float kRint = 0.02f;       // Internal resistance
    constexpr float kSqrt2 = 1.41421356f;

    // Default register values
    constexpr uint32_t kDefaultChopconf = 0x10000053;
    constexpr uint32_t kDefaultPwmconf = 0xC40C001E;
    constexpr uint32_t kDefaultIholdIrun = 0x00081010;
} // namespace

namespace tmc {

    TMC2209Stepper::TMC2209Stepper(UartBus& bus, uint8_t addr) noexcept
        : bus_(bus)
        , addr_(addr)
    {
        if (addr > kMaxAddress) {
            ESP_LOGE(TAG, "Invalid address %u (max %u)", addr, kMaxAddress);
        }
    }

    esp_err_t TMC2209Stepper::initialize() {
        if (!bus_.is_initialized()) {
            return ESP_ERR_INVALID_STATE;
        }
        return reset_to_defaults();
    }

    esp_err_t TMC2209Stepper::reset_to_defaults() {
        // Configure GCONF: enable PDN_DISABLE and MSTEP_REG_SELECT
        constexpr uint32_t gconf = (1U << 6) | (1U << 7);
        if (auto err = bus_.write_register(addr_, static_cast<uint8_t>(Register::GCONF), gconf); err != ESP_OK) {
            return err;
        }

        // Reset shadow registers and write defaults
        shadow_ = ShadowRegisters{};

        if (auto err = bus_.write_register(addr_, static_cast<uint8_t>(Register::IHOLD_IRUN), shadow_.ihold_irun); err != ESP_OK) {
            return err;
        }

        if (auto err = bus_.write_register(addr_, static_cast<uint8_t>(Register::COOLCONF), shadow_.coolconf); err != ESP_OK) {
            return err;
        }

        if (auto err = bus_.write_register(addr_, static_cast<uint8_t>(Register::TPWMTHRS), shadow_.tpwmthrs); err != ESP_OK) {
            return err;
        }

        if (auto err = bus_.write_register(addr_, static_cast<uint8_t>(Register::TCOOLTHRS), shadow_.tcoolthrs); err != ESP_OK) {
            return err;
        }

        if (auto err = bus_.write_register(addr_, static_cast<uint8_t>(Register::THIGH), 0); err != ESP_OK) {
            return err;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        initialized_ = true;
        ESP_LOGI(TAG, "Defaults loaded for addr %u", addr_);
        return ESP_OK;
    }

    uint8_t TMC2209Stepper::calculate_current_scale(uint16_t milliamps) noexcept {
        if (milliamps == 0) {
            return 0;
        }

        // Datasheet formula: CS = (I_rms * 32 * (Rsense + Rint) * sqrt(2)) / Vfs - 1
        const float irms = static_cast<float>(milliamps) / 1000.0f;
        const float cs_float = (irms * 32.0f * (kRsense + kRint) * kSqrt2) / kVfs - 1.0f;
        const int cs = static_cast<int>(std::round(cs_float));
        return static_cast<uint8_t>(std::clamp(cs, 0, static_cast<int>(kMaxCurrentScale)));
    }

    esp_err_t TMC2209Stepper::set_motor_current(uint16_t milliamps) {
        const uint16_t clamped_ma = std::min(milliamps, kMaxCurrentmA);
        if (milliamps > kMaxCurrentmA) {
            ESP_LOGW(TAG, "Clamped %u mA to %u mA", milliamps, kMaxCurrentmA);
        }

        const uint8_t cs = calculate_current_scale(clamped_ma);

        // Update IHOLD and IRUN fields (bits 0-4 for IHOLD, bits 8-12 for IRUN)
        shadow_.ihold_irun &= ~((0x1FU << 8) | 0x1FU);
        shadow_.ihold_irun |= (static_cast<uint32_t>(cs) << 8) | cs;

        ESP_LOGI(TAG, "Current %u mA -> CS=%u", clamped_ma, cs);
        return bus_.write_register(addr_, static_cast<uint8_t>(Register::IHOLD_IRUN), shadow_.ihold_irun);
    }

    esp_err_t TMC2209Stepper::set_direction(bool clockwise) {
        auto gconf_opt = bus_.read_register(addr_, static_cast<uint8_t>(Register::GCONF));
        uint32_t gconf = gconf_opt.value_or(0);

        if (clockwise) {
            gconf &= ~(1U << 4);
        }
        else {
            gconf |= (1U << 4);
        }

        return bus_.write_register(addr_, static_cast<uint8_t>(Register::GCONF), gconf);
    }

    esp_err_t TMC2209Stepper::set_hold_current_percentage(uint8_t percent) {
        if (percent > 100) {
            return ESP_ERR_INVALID_ARG;
        }

        const uint8_t irun = (shadow_.ihold_irun >> 8) & 0x1F;
        const uint8_t ihold = (irun * percent) / 100;

        shadow_.ihold_irun = (shadow_.ihold_irun & ~0x1FU) | ihold;
        ESP_LOGI(TAG, "Hold %u%% -> IHOLD=%u", percent, ihold);
        return bus_.write_register(addr_, static_cast<uint8_t>(Register::IHOLD_IRUN), shadow_.ihold_irun);
    }

    esp_err_t TMC2209Stepper::set_microstep_resolution(MicrostepResolution resolution) {
        auto chopconf_opt = bus_.read_register(addr_, static_cast<uint8_t>(Register::CHOPCONF));
        uint32_t chopconf = chopconf_opt.value_or(kDefaultChopconf);

        chopconf &= ~(0xFU << 24);
        chopconf |= (static_cast<uint32_t>(resolution) & 0xF) << 24;

        return bus_.write_register(addr_, static_cast<uint8_t>(Register::CHOPCONF), chopconf);
    }

    esp_err_t TMC2209Stepper::set_interpolation_enable(bool enable) {
        auto chopconf_opt = bus_.read_register(addr_, static_cast<uint8_t>(Register::CHOPCONF));
        uint32_t chopconf = chopconf_opt.value_or(kDefaultChopconf);

        if (enable) {
            chopconf |= (1U << 28);
        }
        else {
            chopconf &= ~(1U << 28);
        }

        return bus_.write_register(addr_, static_cast<uint8_t>(Register::CHOPCONF), chopconf);
    }

    esp_err_t TMC2209Stepper::set_stealthchop_enable(bool enable) {
        auto gconf_opt = bus_.read_register(addr_, static_cast<uint8_t>(Register::GCONF));
        uint32_t gconf = gconf_opt.value_or(0);

        // Bit 2: en_spreadcycle. Clear to enable StealthChop, set to force SpreadCycle
        if (enable) {
            gconf &= ~(1U << 2);
        }
        else {
            gconf |= (1U << 2);
        }

        return bus_.write_register(addr_, static_cast<uint8_t>(Register::GCONF), gconf);
    }

    esp_err_t TMC2209Stepper::set_stealthchop_threshold(uint32_t threshold) {
        shadow_.tpwmthrs = threshold;
        return bus_.write_register(addr_, static_cast<uint8_t>(Register::TPWMTHRS), shadow_.tpwmthrs);
    }

    esp_err_t TMC2209Stepper::set_stealthchop_pwm_gradient(uint8_t gradient) {
        auto pwmconf_opt = bus_.read_register(addr_, static_cast<uint8_t>(Register::PWMCONF));
        uint32_t pwmconf = pwmconf_opt.value_or(kDefaultPwmconf);

        pwmconf &= ~(0xFFU << 16);
        pwmconf |= static_cast<uint32_t>(gradient) << 16;

        return bus_.write_register(addr_, static_cast<uint8_t>(Register::PWMCONF), pwmconf);
    }

    esp_err_t TMC2209Stepper::set_stealthchop_pwm_amplitude(uint8_t amplitude) {
        auto pwmconf_opt = bus_.read_register(addr_, static_cast<uint8_t>(Register::PWMCONF));
        uint32_t pwmconf = pwmconf_opt.value_or(kDefaultPwmconf);

        pwmconf &= ~(0xFFU << 8);
        pwmconf |= static_cast<uint32_t>(amplitude) << 8;

        return bus_.write_register(addr_, static_cast<uint8_t>(Register::PWMCONF), pwmconf);
    }

    esp_err_t TMC2209Stepper::set_chopper_mode(bool constant_off_time) {
        auto chopconf_opt = bus_.read_register(addr_, static_cast<uint8_t>(Register::CHOPCONF));
        uint32_t chopconf = chopconf_opt.value_or(kDefaultChopconf);

        if (constant_off_time) {
            chopconf |= (1U << 14);
        }
        else {
            chopconf &= ~(1U << 14);
        }

        return bus_.write_register(addr_, static_cast<uint8_t>(Register::CHOPCONF), chopconf);
    }

    esp_err_t TMC2209Stepper::set_chopper_off_time(uint8_t toff) {
        if (toff < 1 || toff > 15) {
            return ESP_ERR_INVALID_ARG;
        }

        auto chopconf_opt = bus_.read_register(addr_, static_cast<uint8_t>(Register::CHOPCONF));
        uint32_t chopconf = chopconf_opt.value_or(kDefaultChopconf);

        chopconf = (chopconf & ~0xFU) | toff;
        return bus_.write_register(addr_, static_cast<uint8_t>(Register::CHOPCONF), chopconf);
    }

    esp_err_t TMC2209Stepper::set_chopper_hysteresis_start(uint8_t hstrt) {
        if (hstrt > 7) {
            return ESP_ERR_INVALID_ARG;
        }

        auto chopconf_opt = bus_.read_register(addr_, static_cast<uint8_t>(Register::CHOPCONF));
        uint32_t chopconf = chopconf_opt.value_or(kDefaultChopconf);

        chopconf &= ~(0x7U << 4);
        chopconf |= static_cast<uint32_t>(hstrt) << 4;

        return bus_.write_register(addr_, static_cast<uint8_t>(Register::CHOPCONF), chopconf);
    }

    esp_err_t TMC2209Stepper::set_chopper_hysteresis_end(int8_t hend) {
        if (hend < -3 || hend > 12) {
            return ESP_ERR_INVALID_ARG;
        }

        const uint8_t hend_reg = static_cast<uint8_t>(hend + 3) & 0xF;

        auto chopconf_opt = bus_.read_register(addr_, static_cast<uint8_t>(Register::CHOPCONF));
        uint32_t chopconf = chopconf_opt.value_or(kDefaultChopconf);

        chopconf &= ~(0xFU << 7);
        chopconf |= static_cast<uint32_t>(hend_reg) << 7;

        return bus_.write_register(addr_, static_cast<uint8_t>(Register::CHOPCONF), chopconf);
    }

    esp_err_t TMC2209Stepper::set_chopper_blank_time(uint8_t tbl) {
        if (tbl > 3) {
            return ESP_ERR_INVALID_ARG;
        }

        auto chopconf_opt = bus_.read_register(addr_, static_cast<uint8_t>(Register::CHOPCONF));
        uint32_t chopconf = chopconf_opt.value_or(kDefaultChopconf);

        chopconf &= ~(0x3U << 15);
        chopconf |= static_cast<uint32_t>(tbl) << 15;

        return bus_.write_register(addr_, static_cast<uint8_t>(Register::CHOPCONF), chopconf);
    }

    esp_err_t TMC2209Stepper::set_stallguard_callback(gpio_num_t diag_pin, StallGuardCallback callback) {
        diag_pin_ = diag_pin;
        callback_ = std::move(callback);

        if (diag_pin_ == GPIO_NUM_NC || !callback_) {
            return ESP_OK;
        }

        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << diag_pin_,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_POSEDGE
        };

        if (auto err = gpio_config(&cfg); err != ESP_OK) {
            return err;
        }

        if (auto err = gpio_install_isr_service(0); err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            return err;
        }

        return gpio_isr_handler_add(diag_pin_, stallguard_isr_handler, this);
    }

    esp_err_t TMC2209Stepper::set_stallguard_min_speed(uint32_t min_speed) {
        shadow_.tcoolthrs = min_speed;
        return bus_.write_register(addr_, static_cast<uint8_t>(Register::TCOOLTHRS), shadow_.tcoolthrs);
    }

    esp_err_t TMC2209Stepper::set_coolstep_enable(bool enable) {
        if (enable) {
            shadow_.coolconf |= (1U << 0);
        }
        else {
            shadow_.coolconf &= ~(1U << 0);
        }
        return bus_.write_register(addr_, static_cast<uint8_t>(Register::COOLCONF), shadow_.coolconf);
    }

    esp_err_t TMC2209Stepper::set_coolstep_min_current(uint8_t min_current) {
        if (min_current > 1) {
            return ESP_ERR_INVALID_ARG;
        }

        shadow_.coolconf &= ~(1U << 15);
        shadow_.coolconf |= static_cast<uint32_t>(min_current) << 15;

        return bus_.write_register(addr_, static_cast<uint8_t>(Register::COOLCONF), shadow_.coolconf);
    }

    esp_err_t TMC2209Stepper::set_coolstep_current_increment(uint8_t increment) {
        if (increment > 3) {
            return ESP_ERR_INVALID_ARG;
        }

        shadow_.coolconf &= ~(0x3U << 13);
        shadow_.coolconf |= static_cast<uint32_t>(increment) << 13;

        return bus_.write_register(addr_, static_cast<uint8_t>(Register::COOLCONF), shadow_.coolconf);
    }

    esp_err_t TMC2209Stepper::set_coolstep_upper_threshold(uint8_t threshold) {
        if (threshold > 15) {
            return ESP_ERR_INVALID_ARG;
        }

        shadow_.coolconf &= ~(0xFU << 8);
        shadow_.coolconf |= static_cast<uint32_t>(threshold) << 8;

        return bus_.write_register(addr_, static_cast<uint8_t>(Register::COOLCONF), shadow_.coolconf);
    }

    esp_err_t TMC2209Stepper::set_coolstep_lower_threshold(uint8_t threshold) {
        if (threshold > 15) {
            return ESP_ERR_INVALID_ARG;
        }

        shadow_.coolconf &= ~0xFU;
        shadow_.coolconf |= threshold;

        return bus_.write_register(addr_, static_cast<uint8_t>(Register::COOLCONF), shadow_.coolconf);
    }

    esp_err_t TMC2209Stepper::set_stallguard_threshold(uint8_t threshold) {
        // Note: Lower values = MORE sensitive (easier stall detection)
        //       Higher values = LESS sensitive (requires more load)
        //       Typical range: 50-100 for normal operation
        if (threshold < 30) {
            ESP_LOGW(TAG, "SGTHRS %u may be too sensitive, consider 30-100 range", threshold);
        }

        shadow_.sgthrs = threshold;
        return bus_.write_register(addr_, static_cast<uint8_t>(Register::SGTHRS), shadow_.sgthrs);
    }

    std::optional<DriverStatus> TMC2209Stepper::get_driver_status() {
        auto status_opt = bus_.read_register(addr_, static_cast<uint8_t>(Register::DRV_STATUS));
        if (!status_opt) {
            return std::nullopt;
        }
        return DriverStatus{ .raw = *status_opt };
    }

    uint16_t TMC2209Stepper::get_stallguard_result() {
        auto result_opt = bus_.read_register(addr_, static_cast<uint8_t>(Register::SG_RESULT));
        if (!result_opt) {
            return 0;
        }
        return static_cast<uint16_t>(*result_opt & 0x3FF);
    }

    bool TMC2209Stepper::is_stalled() {
        // The TMC2209 signals a stall when SG_RESULT drops to or below
        // 2*SGTHRS (datasheet section 5.4 "StallGuard4"). The old test
        // (SG_RESULT == 0) almost never fires — SG_RESULT rarely reaches
        // exactly zero even in a hard stall — so it reported "not stalled"
        // through real stalls. Homing does NOT depend on this path (it uses
        // the DIAG pin); this only feeds the reported status flag.
        const uint16_t sgthrs = static_cast<uint16_t>(shadow_.sgthrs & 0xFF);
        if (sgthrs == 0) {
            return false;  // Threshold unset: no meaningful stall comparison
        }
        return get_stallguard_result() <= static_cast<uint16_t>(sgthrs * 2);
    }

    uint16_t TMC2209Stepper::get_actual_current() {
        // CS_ACTUAL (DRV_STATUS bits 16..20) is the current scale the driver is
        // actually applying after CoolStep/StallGuard regulation. The old code
        // read MSCURACT, which holds the instantaneous coil sine values
        // (CUR_A/CUR_B) — not a current scale — so the returned mA was garbage.
        auto status = get_driver_status();
        if (!status) {
            return 0;
        }

        const uint16_t cs = status->current_scaling();
        // Inverse of calculate_current_scale():
        //   I_rms = (CS+1) * Vfs / (32 * (Rsense + Rint) * sqrt(2))
        const float irms_ma = static_cast<float>(cs + 1) * kVfs /
            (32.0f * (kRsense + kRint) * kSqrt2) * 1000.0f;
        return static_cast<uint16_t>(std::round(irms_ma));
    }

    bool TMC2209Stepper::is_stealthchop_active() {
        auto status = get_driver_status();
        return status && status->stealthchop_active();
    }

    std::optional<CurrentConfig> TMC2209Stepper::get_current_config() const noexcept {
        CurrentConfig config{
            .ihold = static_cast<uint8_t>(shadow_.ihold_irun & 0x1F),
            .irun = static_cast<uint8_t>((shadow_.ihold_irun >> 8) & 0x1F),
            .iholddelay = static_cast<uint8_t>((shadow_.ihold_irun >> 16) & 0x0F)
        };

        ESP_LOGD(TAG, "Config: IHOLD=%u IRUN=%u IHOLDDELAY=%u", config.ihold, config.irun, config.iholddelay);
        return config;
    }

    void IRAM_ATTR TMC2209Stepper::stallguard_isr_handler(void* arg) {
        auto* self = static_cast<TMC2209Stepper*>(arg);
        self->handle_stallguard_interrupt();
    }

    void TMC2209Stepper::handle_stallguard_interrupt() {
        if (callback_) {
            const bool stalled = gpio_get_level(diag_pin_) == 1;
            callback_(addr_, stalled);
        }
    }
} // namespace tmc
