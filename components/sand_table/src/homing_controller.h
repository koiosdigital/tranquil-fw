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
    struct CalibrationData;

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

        /// (Re)write the rho StallGuard registers (SGTHRS from live config +
        /// TCOOLTHRS) to the driver and verify the UART link. Idempotent.
        /// Called from init(), at the start of every home (so a threshold
        /// change takes effect without a reboot and TCOOLTHRS is freshly
        /// armed), and directly after set_rho_sgthrs. Returns false if the
        /// rho driver did not acknowledge over UART.
        bool configure_stallguard();

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

        // StallGuard is invalid at standstill and during initial
        // acceleration, and DIAG can still be asserted from a previous
        // stall. Each rho seek first moves this far with the DIAG ISR
        // DISABLED before arming stall detection. A real stall inside the
        // blanking window grinds for at most this many steps (~150ms).
        static constexpr int32_t kStallBlankingSteps = 200;

        // A stall reported with less total travel than this is a false
        // trigger (residual DIAG / StallGuard noise), not the opposite hard
        // stop. Refuse to treat it as a valid calibration.
        static constexpr int32_t kMinRhoTravelSteps = 1000;

        // Theta analogue of kMinRhoTravelSteps: an edge-to-edge measurement
        // below this is a spurious hall re-trigger (field ripple/EMI), not a
        // real drum revolution. Half the tabletop nominal keeps every
        // supported drive in-band (12:1 coffee measures ~1.5x nominal;
        // a missed edge can't inflate the value past kMaxHomingSteps, so
        // only the low side needs guarding). The measurement is the
        // denominator of ALL rho coupling compensation — a saved glitch
        // value would over-drive rho by orders of magnitude.
        static constexpr int32_t kMinThetaStepsPerRotation =
            MechanicalConfig::NOMINAL_STEPS_PER_THETA_ROTATION / 2;

        // Rho homing is two-pass, 3D-printer style: a FAST seek locates each
        // hard stop, then up to kRhoSlowHomeAttempts SLOW homes refine it
        // (back out one motor revolution, slowly re-approach into the stop in
        // the same direction). The slow pass makes the reference position
        // repeatable: the stop point overshoots the true stop by the DIAG
        // detection latency, which scales with speed.
        //
        // Speeds are CRITICAL for StallGuard. TMC2209 datasheet Rev 1.03
        // section 11.5 "Limits of StallGuard4 Operation": SG_RESULT is
        // unstable below ~1 motor revolution/second (low back-EMF). At
        // 200 fullstep/rev x 16 microstep = 3200 microstep/rev, 1 rev/s =
        // 312us/step:
        //  - Fast pass 250us -> 4000 steps/s = 1.25 rev/s, comfortably above
        //    the floor. Bounded below by the motor's pull-in rate (no ramp).
        //  - Slow pass 400us -> 2500 steps/s = 0.78 rev/s, slightly under the
        //    nominal floor but only used for a short, known re-approach with
        //    built-in retries; slower = smaller DIAG-latency overshoot =
        //    tighter reference. Pull toward 312us if slow-pass stalls prove
        //    unreliable on this motor.
        static constexpr uint32_t kRhoFastHomingIntervalUs = 250;  // 4000 steps/s ~= 1.25 rev/s
        static constexpr uint32_t kRhoSlowHomingIntervalUs = 400;  // 2500 steps/s ~= 0.78 rev/s
        static constexpr int kRhoSlowHomeAttempts = 3;
        // Theta homes on the Hall sensor, NOT StallGuard, so the 1 rev/s floor
        // does not apply; this only trades edge-detection latency vs mechanical
        // safety. 400us -> 2500 steps/s ~= 0.78 motor rev/s.
        static constexpr uint32_t kThetaHomingIntervalUs = 400; // 2500 steps/s ~= 0.78 rev/s (theta motor)

        // Rho motor steps mechanically dragged per theta motor step (one
        // rho motor revolution per theta drum revolution). SIGNED with
        // MechanicalConfig::COUPLING_SIGN, same convention as the
        // CoordinateTransformer. Prefers the observed steps-per-drum-rev —
        // this run's, then cached calibration — and only falls back to the
        // nominal constant for the first-ever calibration, where it merely
        // bounds the seek moves' companion rho compensation.
        [[nodiscard]] double rho_steps_per_theta_step() const;

        // ISR handlers - these directly stop RMT transmission when sensors trigger
        static void IRAM_ATTR hall_isr_handler(void* arg);
        static void IRAM_ATTR diag_isr_handler(void* arg);

        // Enable/disable individual ISRs (only enable the relevant one during each phase)
        void enable_hall_isr();
        void disable_hall_isr();
        void enable_diag_isr();
        void disable_diag_isr();

        // Cached calibration is usable only when marked valid AND in the
        // plausibility band (guards torn NVS writes / legacy garbage that
        // would poison the coupling denominator).
        [[nodiscard]] static bool calibration_plausible(const CalibrationData& calib);

        // --- Rho two-pass homing primitives ---

        // One armed seek toward a rho hard stop: blanking move (DIAG ISR off,
        // StallGuard invalid at spin-up), then arm the DIAG ISR and drive
        // until stall or max_steps. outward=true seeks the edge (+rho),
        // false seeks the center (-rho). Leaves the DIAG ISR disabled.
        struct RhoSeek {
            bool stalled = false;
            int32_t steps_moved = 0;   // including the blanking move
        };
        RhoSeek rho_seek_stop(bool outward, uint32_t max_steps, uint32_t interval_us);

        // Slow-home refinement at a stop just found by a fast seek: back out
        // one motor revolution, then slowly re-approach in the SAME direction
        // until the stop re-triggers (3D-printer style). Retries up to
        // kRhoSlowHomeAttempts times if the slow approach doesn't produce a
        // plausible stall. On success the carriage rests pressed against the
        // stop at slow-approach precision.
        bool rho_slow_home(bool outward);

        // Full rho calibration: fast+slow home the edge, then fast+slow home
        // the center; position_steps = refined edge-to-center travel.
        HomingResult calibrate_rho();

        // Internal homing methods (full calibration)
        HomingResult calibrate_theta();

        // Internal homing methods (quick mode - uses cached values)
        HomingResult home_rho_to_center(int32_t rho_max_steps);
        HomingResult home_theta_single();

        // Save calibration results to NVS
        void save_calibration_to_nvs();
    };

} // namespace sand_table
