#pragma once

#include "types.h"
#include "config.h"
#include "stepper_driver.h"

#include "tmc2209.h"
#include "driver/gpio.h"

#include <atomic>

namespace sand_table {

/// Controls the homing sequence for both axes.
/// - Theta uses Hall effect sensor
/// - Rho uses TMC2209 StallGuard for sensorless homing
class HomingController {
public:
    struct HomingResult {
        bool success = false;
        MotionError error = MotionError::None;
        int32_t position_steps = 0;
    };

    enum class HomingState {
        Idle,
        // Rho phases
        RhoSeekingMax,
        RhoSeekingMin,
        // Theta phases
        ThetaBackingOff,
        ThetaSeekingFirstEdge,
        ThetaSeekingSecondEdge,
        // Terminal states
        Complete,
        Error
    };

    HomingController(
        StepperDriver& theta,
        StepperDriver& rho,
        tmc::TMC2209Stepper& theta_tmc,
        tmc::TMC2209Stepper& rho_tmc
    );

    ~HomingController();

    // Non-copyable
    HomingController(const HomingController&) = delete;
    HomingController& operator=(const HomingController&) = delete;

    /// Initialize homing hardware (Hall sensor GPIO, StallGuard)
    [[nodiscard]] Result<void> init();

    /// Home theta axis using Hall effect sensor
    [[nodiscard]] HomingResult home_theta();

    /// Home rho axis using StallGuard
    [[nodiscard]] HomingResult home_rho();

    /// Full homing sequence (rho first for safety, then theta)
    [[nodiscard]] Result<void> home_all();

    /// Check if Hall sensor is currently triggered
    [[nodiscard]] bool is_hall_triggered() const;

    /// Check if rho StallGuard detected a stall
    [[nodiscard]] bool is_rho_stalled() const;

    /// Get current homing state
    [[nodiscard]] HomingState state() const noexcept {
        return state_.load(std::memory_order_acquire);
    }

    /// Check if homing is in progress
    [[nodiscard]] bool is_homing() const noexcept {
        return homing_active_.load(std::memory_order_acquire);
    }

    /// Abort current homing operation
    void abort();

    /// Get calibration results (valid after successful homing)
    [[nodiscard]] int32_t rho_max_steps() const noexcept { return rho_max_steps_; }
    [[nodiscard]] int32_t theta_steps_per_rotation() const noexcept { return theta_steps_per_rotation_; }

private:
    StepperDriver& theta_;
    StepperDriver& rho_;
    tmc::TMC2209Stepper& theta_tmc_;
    tmc::TMC2209Stepper& rho_tmc_;

    std::atomic<HomingState> state_{HomingState::Idle};
    std::atomic<bool> homing_active_{false};
    std::atomic<bool> abort_requested_{false};

    // Sensor state (updated by ISR)
    volatile bool hall_triggered_ = false;
    volatile bool rho_stall_triggered_ = false;

    // Calibration results
    int32_t rho_max_steps_ = 0;
    int32_t theta_steps_per_rotation_ = 0;

    // Homing parameters
    static constexpr uint32_t kMaxHomingSteps = 100000;
    static constexpr uint32_t kHomingStepIntervalUs = 500;  // 2000 steps/sec
    static constexpr uint32_t kSlowHomingStepIntervalUs = 2000;  // 500 steps/sec

    // ISR handlers
    static void IRAM_ATTR hall_isr_handler(void* arg);
    static void rho_stall_callback(uint8_t addr, bool stalled);

    // Internal homing functions
    HomingResult seek_rho_max();
    HomingResult seek_rho_min();
    HomingResult calibrate_theta();

    // Helper to check abort and stall conditions
    bool should_stop_rho() const;
    bool should_stop_theta() const;
};

} // namespace sand_table
