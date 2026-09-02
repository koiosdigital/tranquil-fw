#pragma once

#include "types.h"
#include "config.h"

#include <cmath>

namespace sand_table {

    /// Handles coordinate transformations between polar and step domains.
    /// Uses calibrated values from homing for accurate conversions.
    class CoordinateTransformer {
    public:
        CoordinateTransformer() = default;

        /// Set calibrated values from homing
        void set_calibration(int32_t steps_per_theta_rot, int32_t rho_max_steps) noexcept {
            steps_per_theta_rot_ = steps_per_theta_rot;
            rho_max_steps_ = rho_max_steps;
            // Coupling is a mechanical invariant: one theta drum revolution
            // drags the rho drive by exactly one rho motor revolution. The
            // homing-observed steps-per-drum-rev therefore fully determines
            // the rho steps induced per theta motor step — no configured
            // gear ratio involved. SIGNED: COUPLING_SIGN flips the logical
            // pairing when exactly one axis has ROBOT_*_INVERT_DIRECTION set
            // (see config.h); every consumer of this ratio (compensation,
            // position readout, wrap unwind) inherits the sign from here.
            rho_steps_per_theta_step_ = (steps_per_theta_rot_ > 0)
                ? MechanicalConfig::COUPLING_SIGN *
                    static_cast<double>(MechanicalConfig::effective_steps_per_rev()) /
                    static_cast<double>(steps_per_theta_rot_)
                : 0.0;
        }

        /// Check if calibration values are set
        [[nodiscard]] bool is_calibrated() const noexcept {
            return steps_per_theta_rot_ > 0 && rho_max_steps_ > 0;
        }

        /// Reset fractional accumulators (call after homing or position reset)
        void reset_accumulators() noexcept {
            theta_fractional_accumulator_ = 0.0;
            rho_fractional_accumulator_ = 0.0;
        }

        /// Convert polar deltas to motor step deltas with coupling compensation.
        /// Uses fractional accumulation to eliminate rounding drift.
        void delta_polar_to_motor_steps(
            double delta_theta_rad,
            double delta_rho_norm,
            int32_t& theta_motor_steps,
            int32_t& rho_motor_steps
        ) {
            // Convert theta delta to exact motor steps
            double theta_rotations = delta_theta_rad / (2.0 * M_PI);
            double theta_steps_exact = theta_rotations * static_cast<double>(steps_per_theta_rot_);

            // Add to theta accumulator and extract integer steps
            theta_fractional_accumulator_ += theta_steps_exact;
            theta_motor_steps = static_cast<int32_t>(std::round(theta_fractional_accumulator_));
            theta_fractional_accumulator_ -= static_cast<double>(theta_motor_steps);

            // Convert rho delta to base steps
            double rho_base_exact = delta_rho_norm * static_cast<double>(rho_max_steps_);

            // Coupling compensation: counteract mechanical rho movement from theta rotation
            double rho_counteract = static_cast<double>(theta_motor_steps) * rho_steps_per_theta_step_;

            // Add to rho accumulator (base movement + coupling counteraction)
            rho_fractional_accumulator_ += rho_base_exact + rho_counteract;

            // Extract integer steps and keep fractional remainder
            rho_motor_steps = static_cast<int32_t>(std::round(rho_fractional_accumulator_));
            rho_fractional_accumulator_ -= static_cast<double>(rho_motor_steps);
        }

        /// Convert motor step positions to polar coordinates (for display only)
        [[nodiscard]] PolarPosition steps_to_polar(int32_t theta_steps, int32_t rho_steps) const {
            if (steps_per_theta_rot_ <= 0 || rho_max_steps_ <= 0) {
                // rho_max_steps_ == 0 would divide to Inf/NaN below (NaN
                // escapes the clamps and reaches JSON serializers)
                return { 0.0, 0.0 };
            }

            // Convert theta steps to radians (normalized to [0, 2pi))
            double motor_rotations = static_cast<double>(theta_steps) / steps_per_theta_rot_;
            double theta = motor_rotations * 2.0 * M_PI;
            theta = normalize_angle(theta);

            double rho = rho_norm_unclamped(theta_steps, rho_steps);

            // Clamp rho to valid range
            if (rho < 0.0) rho = 0.0;
            if (rho > 1.0) rho = 1.0;

            return { theta, rho };
        }

        /// Coupling-corrected rho WITHOUT the [0,1] clamp. Diagnostic: a
        /// value outside [0,1] means the counters have been commanded past
        /// the physical rails (the clamped display value hides that).
        [[nodiscard]] double rho_norm_unclamped(int32_t theta_steps, int32_t rho_steps) const {
            if (rho_max_steps_ <= 0) {
                return 0.0;
            }
            double rho_counteract_steps = static_cast<double>(theta_steps) * rho_steps_per_theta_step_;
            double effective_rho_steps = static_cast<double>(rho_steps) - rho_counteract_steps;
            return effective_rho_steps / static_cast<double>(rho_max_steps_);
        }

        /// Get calibrated steps per full theta rotation
        [[nodiscard]] int32_t steps_per_theta_rotation() const noexcept {
            return steps_per_theta_rot_;
        }

        /// Get calibrated rho max steps
        [[nodiscard]] int32_t rho_max_steps() const noexcept {
            return rho_max_steps_;
        }

        /// Coupling constant this transformer is actually applying
        /// (rho motor steps per theta motor step, snapshot from calibration)
        [[nodiscard]] double rho_steps_per_theta_step() const noexcept {
            return rho_steps_per_theta_step_;
        }

        /// Rho counteraction this transformer accumulates over exactly one
        /// theta rotation (= what a position wrap must unwind). Derived from
        /// the SAME snapshot the per-step compensation uses, so wrap unwind
        /// and accumulation cannot desync even if runtime config changes
        /// after calibration.
        [[nodiscard]] int32_t rho_counteract_per_rotation() const noexcept {
            return static_cast<int32_t>(std::lround(
                rho_steps_per_theta_step_ * static_cast<double>(steps_per_theta_rot_)));
        }

        /// Get theta fractional accumulator (for debugging)
        [[nodiscard]] double theta_accumulator() const noexcept {
            return theta_fractional_accumulator_;
        }

        /// Get rho fractional accumulator (for debugging)
        [[nodiscard]] double rho_accumulator() const noexcept {
            return rho_fractional_accumulator_;
        }

    private:
        int32_t steps_per_theta_rot_ = 0;
        int32_t rho_max_steps_ = 0;

        // Fractional accumulators (prevent rounding drift over many segments)
        double theta_fractional_accumulator_ = 0.0;
        double rho_fractional_accumulator_ = 0.0;

        // Rho motor steps mechanically dragged per theta motor step.
        // Derived from the observed calibration in set_calibration();
        // zero (no compensation) until calibrated. NOTE: nothing on the
        // motion path checks is_calibrated() — the only motion gate is
        // MotionController::is_homed_, so callers of set_calibration are
        // responsible for passing validated values.
        double rho_steps_per_theta_step_ = 0.0;
    };

} // namespace sand_table
