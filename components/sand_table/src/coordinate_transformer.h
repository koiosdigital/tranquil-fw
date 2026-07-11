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
            // Snapshot the runtime-configurable gear ratio here rather than
            // baking in the compile-time default — a user-configured ratio
            // otherwise silently diverges from the coupling math.
            gear_ratio_ = MechanicalConfig::theta_gear_ratio();
            if (gear_ratio_ <= 0.0) {
                gear_ratio_ = MechanicalConfig::THETA_GEAR_RATIO;
            }
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
            double rho_counteract = static_cast<double>(theta_motor_steps) / gear_ratio_;

            // Add to rho accumulator (base movement + coupling counteraction)
            rho_fractional_accumulator_ += rho_base_exact + rho_counteract;

            // Extract integer steps and keep fractional remainder
            rho_motor_steps = static_cast<int32_t>(std::round(rho_fractional_accumulator_));
            rho_fractional_accumulator_ -= static_cast<double>(rho_motor_steps);
        }

        /// Convert motor step positions to polar coordinates (for display only)
        [[nodiscard]] PolarPosition steps_to_polar(int32_t theta_steps, int32_t rho_steps) const {
            if (steps_per_theta_rot_ <= 0) {
                return { 0.0, 0.0 };
            }

            // Convert theta steps to radians (normalized to [0, 2pi))
            double motor_rotations = static_cast<double>(theta_steps) / steps_per_theta_rot_;
            double theta = motor_rotations * 2.0 * M_PI;
            theta = normalize_angle(theta);

            // Account for coupling when calculating displayed rho
            double rho_counteract_steps = static_cast<double>(theta_steps) / gear_ratio_;
            double effective_rho_steps = static_cast<double>(rho_steps) - rho_counteract_steps;
            double rho = effective_rho_steps / static_cast<double>(rho_max_steps_);

            // Clamp rho to valid range
            if (rho < 0.0) rho = 0.0;
            if (rho > 1.0) rho = 1.0;

            return { theta, rho };
        }

        /// Get calibrated steps per full theta rotation
        [[nodiscard]] int32_t steps_per_theta_rotation() const noexcept {
            return steps_per_theta_rot_;
        }

        /// Get calibrated rho max steps
        [[nodiscard]] int32_t rho_max_steps() const noexcept {
            return rho_max_steps_;
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

        // Refreshed from runtime config in set_calibration()
        double gear_ratio_ = MechanicalConfig::THETA_GEAR_RATIO;
    };

} // namespace sand_table
