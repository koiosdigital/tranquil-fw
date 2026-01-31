#pragma once

#include "types.h"
#include "config.h"
#include "stepper_driver.h"

#include "tmc2209.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <atomic>

namespace sand_table {

    // Forward declarations
    class CoordinatedStepperController;
    class ConfigManager;

    /// Controls the homing sequence for both axes using RMT-based stepping.
    /// - Theta uses Hall effect sensor (NEGEDGE interrupt)
    /// - Rho uses TMC2209 StallGuard for sensorless homing (POSEDGE interrupt)
    ///
    /// Step generation uses RMT peripheral via CoordinatedStepperController.
    /// Homing creates large step counters, and sensor ISRs immediately stop
    /// RMT transmission when triggered (no chunked polling).
    class HomingController {
    public:
        struct HomingResult {
            bool success = false;
            MotionError error = MotionError::None;
            int32_t position_steps = 0;
        };

        enum class HomingState {
            Idle,
            RhoSeekingMax,
            RhoSeekingMin,
            ThetaCalibrating,
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

        /// Set the stepper controller for RMT-based motion
        void set_stepper_controller(CoordinatedStepperController* controller) {
            stepper_controller_ = controller;
        }

        /// Home theta axis using Hall effect sensor
        [[nodiscard]] HomingResult home_theta();

        /// Home rho axis using StallGuard
        [[nodiscard]] HomingResult home_rho();

        /// Full homing sequence - uses cached calibration if available
        /// If force_full is true, always performs full calibration
        [[nodiscard]] Result<void> home_all(bool force_full = false);

        /// Quick homing using cached calibration data
        /// Requires valid calibration in ConfigManager
        [[nodiscard]] Result<void> home_quick();

        /// Force full recalibration (clears calibration and performs full home)
        [[nodiscard]] Result<void> force_recalibrate();

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

        /// Set homed state without running homing sequence (for development)
        void _set_homed(int32_t theta_steps_per_rot, int32_t rho_max) {
            theta_steps_per_rotation_ = theta_steps_per_rot;
            rho_max_steps_ = rho_max;
            state_.store(HomingState::Complete, std::memory_order_release);
            homing_active_.store(false, std::memory_order_release);
        }

        /// Get calibration results (valid after successful homing)
        [[nodiscard]] int32_t rho_max_steps() const noexcept { return rho_max_steps_; }
        [[nodiscard]] int32_t theta_steps_per_rotation() const noexcept { return theta_steps_per_rotation_; }

    private:
        StepperDriver& theta_;
        StepperDriver& rho_;
        tmc::TMC2209Stepper& theta_tmc_;
        tmc::TMC2209Stepper& rho_tmc_;
        CoordinatedStepperController* stepper_controller_ = nullptr;

        std::atomic<HomingState> state_{ HomingState::Idle };
        std::atomic<bool> homing_active_{ false };
        std::atomic<bool> abort_requested_{ false };

        // Sensor flags set by ISRs
        volatile bool hall_triggered_ = false;
        volatile bool rho_stall_triggered_ = false;

        // Calibration results
        int32_t rho_max_steps_ = 0;
        int32_t theta_steps_per_rotation_ = 0;

        // Homing parameters
        static constexpr uint32_t kMaxHomingSteps = 100000;

        // Homing step intervals (constant speed, no acceleration)
        // These are CRITICAL for StallGuard to work reliably
        static constexpr uint32_t kRhoHomingIntervalUs = 750;   // 750µs/step = 1333 steps/sec
        static constexpr uint32_t kThetaHomingIntervalUs = 400; // 400µs/step = 2500 steps/sec

        static constexpr uint32_t kGearRatio = static_cast<uint32_t>(MechanicalConfig::THETA_GEAR_RATIO);

        // ISR handlers - these directly stop RMT transmission when sensors trigger
        static void IRAM_ATTR hall_isr_handler(void* arg);
        static void IRAM_ATTR diag_isr_handler(void* arg);

        // Enable/disable individual ISRs (only enable the relevant one during each phase)
        void enable_hall_isr();
        void disable_hall_isr();
        void enable_diag_isr();
        void disable_diag_isr();

        // Internal homing methods (full calibration)
        HomingResult seek_rho_max();
        HomingResult seek_rho_min();
        HomingResult calibrate_theta();

        // Internal homing methods (quick mode - uses cached values)
        HomingResult home_rho_to_center(int32_t rho_max_steps);
        HomingResult home_theta_single();

        // Save calibration results to NVS
        void save_calibration_to_nvs();
    };

} // namespace sand_table
