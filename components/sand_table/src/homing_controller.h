#pragma once

#include "types.h"
#include "config.h"
#include "stepper_driver.h"

#include "tmc2209.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <atomic>

namespace sand_table {

    /// Controls the homing sequence for both axes using timer-based stepping.
    /// - Theta uses Hall effect sensor
    /// - Rho uses TMC2209 StallGuard for sensorless homing
    ///
    /// Step generation happens in a hardware timer ISR for precise timing
    /// and non-blocking operation.
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

        /// Initialize homing hardware (Hall sensor GPIO, StallGuard, timer)
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

        /// Set homed state without running homing sequence (for development)
        /// @param theta_steps_per_rot Steps for one full theta rotation
        /// @param rho_max Steps for full rho travel
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

        // Timer for step generation
        gptimer_handle_t step_timer_ = nullptr;
        SemaphoreHandle_t completion_sem_ = nullptr;

        std::atomic<HomingState> state_{ HomingState::Idle };
        std::atomic<bool> homing_active_{ false };
        std::atomic<bool> abort_requested_{ false };

        // Sensor state (updated by ISR)
        volatile bool hall_triggered_ = false;
        volatile bool rho_stall_triggered_ = false;

        // Homing control state (used by timer ISR)
        struct HomingControl {
            uint32_t step_count = 0;
            uint32_t max_steps = 0;
            uint32_t theta_step_counter = 0;  // For coupled rho motion
            int32_t rho_max_steps = 0;
            int32_t theta_steps_per_rot = 0;
            bool first_hall_edge_found = false;
            MotionError error = MotionError::None;
        };
        volatile HomingControl homing_ctrl_;

        // Calibration results
        int32_t rho_max_steps_ = 0;
        int32_t theta_steps_per_rotation_ = 0;

        // Homing parameters (matched to main branch)
        static constexpr uint32_t kMaxHomingSteps = 100000;
        static constexpr uint64_t kRhoStepIntervalUs = 750;   // Rho homing step interval
        static constexpr uint64_t kThetaStepIntervalUs = 400; // Theta homing step interval
        static constexpr uint32_t kGearRatio = static_cast<uint32_t>(MechanicalConfig::THETA_GEAR_RATIO);

        // ISR handlers
        static void IRAM_ATTR hall_isr_handler(void* arg);
        static void IRAM_ATTR diag_isr_handler(void* arg);
        static bool IRAM_ATTR step_timer_callback(gptimer_handle_t timer,
                                                   const gptimer_alarm_event_data_t* edata,
                                                   void* user_ctx);

        // Timer ISR step generation
        void IRAM_ATTR generate_homing_steps();

        // Internal homing phase starters (called from ISR context)
        void IRAM_ATTR start_rho_max_isr();
        void IRAM_ATTR start_rho_min_isr();
        void IRAM_ATTR start_theta_calibration_isr();

        // Start timer with specified interval
        void start_timer(uint64_t interval_us);
        void stop_timer();

        // Wait for homing phase to complete
        HomingResult wait_for_completion();
    };

} // namespace sand_table
