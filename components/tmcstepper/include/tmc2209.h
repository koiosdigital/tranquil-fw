#pragma once

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "uartbus.h"

#include <cstdint>
#include <functional>
#include <optional>

namespace tmc {

enum class MicrostepResolution : uint8_t {
    Full = 8,            // 1/1 step
    Half = 7,            // 1/2 step
    Quarter = 6,         // 1/4 step
    Eighth = 5,          // 1/8 step
    Sixteenth = 4,       // 1/16 step
    ThirtySecond = 3,    // 1/32 step
    SixtyFourth = 2,     // 1/64 step
    OneTwentyEighth = 1, // 1/128 step
    TwoFiftySixth = 0,   // 1/256 step
};

struct CurrentConfig {
    uint8_t ihold;
    uint8_t irun;
    uint8_t iholddelay;
};

// Decodes the TMC2209 DRV_STATUS register (0x6F). NOTE: this layout is
// specific to the 2209 and differs from the TMC2130/5160 — the 2209 has no
// StallGuard result or dedicated stall bit in this register (StallGuard is
// read from SG_RESULT 0x41, and a stall is reported on the DIAG pin), and the
// short/open-load/over-temperature flags live in the low byte. Fields per the
// TMC2209 datasheet (Rev 1.09), section 5.1 "DRV_STATUS".
struct DriverStatus {
    uint32_t raw;

    [[nodiscard]] bool overtemperature_warning() const noexcept { return (raw & (1U << 0)) != 0; }   // otpw
    [[nodiscard]] bool overtemperature_shutdown() const noexcept { return (raw & (1U << 1)) != 0; }   // ot
    [[nodiscard]] bool short_to_ground_a() const noexcept { return (raw & (1U << 2)) != 0; }          // s2ga
    [[nodiscard]] bool short_to_ground_b() const noexcept { return (raw & (1U << 3)) != 0; }          // s2gb
    [[nodiscard]] bool low_side_short_a() const noexcept { return (raw & (1U << 4)) != 0; }           // s2vsa
    [[nodiscard]] bool low_side_short_b() const noexcept { return (raw & (1U << 5)) != 0; }           // s2vsb
    [[nodiscard]] bool open_load_a() const noexcept { return (raw & (1U << 6)) != 0; }                // ola
    [[nodiscard]] bool open_load_b() const noexcept { return (raw & (1U << 7)) != 0; }                // olb
    [[nodiscard]] uint8_t current_scaling() const noexcept { return static_cast<uint8_t>((raw >> 16) & 0x1F); }  // CS_ACTUAL
    [[nodiscard]] bool stealthchop_active() const noexcept { return (raw & (1U << 30)) != 0; }        // stealth
    [[nodiscard]] bool standstill() const noexcept { return (raw & (1U << 31)) != 0; }                // stst
};

class TMC2209Stepper {
public:
    using StallGuardCallback = std::function<void(uint8_t addr, bool stalled)>;

    static constexpr uint8_t kMaxAddress = 3;
    static constexpr uint16_t kMaxCurrentmA = 2000;
    static constexpr uint8_t kMaxCurrentScale = 31;

    TMC2209Stepper(UartBus& bus, uint8_t addr) noexcept;
    ~TMC2209Stepper() = default;

    // Non-copyable, moveable
    TMC2209Stepper(const TMC2209Stepper&) = delete;
    TMC2209Stepper& operator=(const TMC2209Stepper&) = delete;
    TMC2209Stepper(TMC2209Stepper&&) = default;
    TMC2209Stepper& operator=(TMC2209Stepper&&) = default;

    [[nodiscard]] esp_err_t initialize();
    [[nodiscard]] esp_err_t reset_to_defaults();

    // Motor configuration
    [[nodiscard]] esp_err_t set_direction(bool clockwise);
    [[nodiscard]] esp_err_t set_motor_current(uint16_t milliamps);
    [[nodiscard]] esp_err_t set_hold_current_percentage(uint8_t percent);
    [[nodiscard]] esp_err_t set_microstep_resolution(MicrostepResolution resolution);
    [[nodiscard]] esp_err_t set_interpolation_enable(bool enable);

    // StealthChop configuration
    [[nodiscard]] esp_err_t set_stealthchop_enable(bool enable);
    [[nodiscard]] esp_err_t set_stealthchop_threshold(uint32_t threshold);
    [[nodiscard]] esp_err_t set_stealthchop_pwm_gradient(uint8_t gradient);
    [[nodiscard]] esp_err_t set_stealthchop_pwm_amplitude(uint8_t amplitude);

    // Chopper configuration
    [[nodiscard]] esp_err_t set_chopper_mode(bool constant_off_time);
    [[nodiscard]] esp_err_t set_chopper_off_time(uint8_t toff);
    [[nodiscard]] esp_err_t set_chopper_hysteresis_start(uint8_t hstrt);
    [[nodiscard]] esp_err_t set_chopper_hysteresis_end(int8_t hend);
    [[nodiscard]] esp_err_t set_chopper_blank_time(uint8_t tbl);

    // StallGuard configuration
    [[nodiscard]] esp_err_t set_stallguard_callback(gpio_num_t diag_pin, StallGuardCallback callback);
    [[nodiscard]] esp_err_t set_stallguard_threshold(uint8_t threshold);
    [[nodiscard]] esp_err_t set_stallguard_min_speed(uint32_t min_speed);

    // CoolStep configuration
    [[nodiscard]] esp_err_t set_coolstep_enable(bool enable);
    [[nodiscard]] esp_err_t set_coolstep_min_current(uint8_t min_current);
    [[nodiscard]] esp_err_t set_coolstep_current_increment(uint8_t increment);
    [[nodiscard]] esp_err_t set_coolstep_upper_threshold(uint8_t threshold);
    [[nodiscard]] esp_err_t set_coolstep_lower_threshold(uint8_t threshold);

    // Status queries
    [[nodiscard]] std::optional<DriverStatus> get_driver_status();
    [[nodiscard]] bool is_stalled();
    [[nodiscard]] uint16_t get_stallguard_result();
    [[nodiscard]] uint16_t get_actual_current();
    [[nodiscard]] bool is_stealthchop_active();
    [[nodiscard]] std::optional<CurrentConfig> get_current_config() const noexcept;

    // Accessors
    [[nodiscard]] uint8_t address() const noexcept { return addr_; }
    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }

private:
    static void IRAM_ATTR stallguard_isr_handler(void* arg);
    void handle_stallguard_interrupt();
    [[nodiscard]] static uint8_t calculate_current_scale(uint16_t milliamps) noexcept;

    // Register addresses
    enum class Register : uint8_t {
        GCONF = 0x00,
        GSTAT = 0x01,
        IFCNT = 0x02,
        NODECONF = 0x03,
        OTPPROG = 0x04,
        OTPREAD = 0x05,
        IOIN = 0x06,
        FACTORYCONF = 0x07,
        IHOLD_IRUN = 0x10,
        TPOWERDOWN = 0x11,
        TSTEP = 0x12,
        TPWMTHRS = 0x13,
        TCOOLTHRS = 0x14,
        THIGH = 0x15,
        VACTUAL = 0x22,
        SGTHRS = 0x40,
        SG_RESULT = 0x41,
        COOLCONF = 0x42,
        MSCNT = 0x6A,
        MSCURACT = 0x6B,
        CHOPCONF = 0x6C,
        DCCTRL = 0x6E,
        DRV_STATUS = 0x6F,
        PWMCONF = 0x70,
        PWM_SCALE = 0x71,
        PWM_AUTO = 0x72
    };

    // Shadow registers for write-only registers
    struct ShadowRegisters {
        uint32_t ihold_irun = 0x00081010;
        uint32_t tpowerdown = 0;
        uint32_t tpwmthrs = 0;
        uint32_t vactual = 0;
        uint32_t tcoolthrs = 0;
        uint32_t sgthrs = 0;
        uint32_t coolconf = 0;
    };

    UartBus& bus_;
    uint8_t addr_;
    gpio_num_t diag_pin_ = GPIO_NUM_NC;
    StallGuardCallback callback_;
    bool initialized_ = false;
    ShadowRegisters shadow_;
};

} // namespace tmc

// Backwards compatibility aliases
using MicrostepResolution = tmc::MicrostepResolution;
using TMC2209Stepper = tmc::TMC2209Stepper;

// Legacy enum value mappings
constexpr auto FULL = tmc::MicrostepResolution::Full;
constexpr auto HALF = tmc::MicrostepResolution::Half;
constexpr auto QUARTER = tmc::MicrostepResolution::Quarter;
constexpr auto EIGHTH = tmc::MicrostepResolution::Eighth;
constexpr auto SIXTEENTH = tmc::MicrostepResolution::Sixteenth;
constexpr auto THIRTYSECOND = tmc::MicrostepResolution::ThirtySecond;
constexpr auto SIXTYFOURTH = tmc::MicrostepResolution::SixtyFourth;
constexpr auto ONETWENTYEIGHTH = tmc::MicrostepResolution::OneTwentyEighth;
constexpr auto TWOFIFTYSIXTH = tmc::MicrostepResolution::TwoFiftySixth;
