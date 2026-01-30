#pragma once

#include "types.h"
#include "config.h"
#include "esp_log.h"

#include <cmath>

namespace sand_table {

/// Handles coordinate transformations between polar and step domains.
/// Uses calibrated values from homing for accurate conversions.
/// Simplified delta-based approach (like main branch).
class CoordinateTransformer {
public:
    CoordinateTransformer() = default;

    /// Set calibrated values from homing
    void set_calibration(int32_t steps_per_theta_rot, int32_t rho_max_steps) noexcept {
        steps_per_theta_rot_ = steps_per_theta_rot;
        rho_max_steps_ = rho_max_steps;
    }

    /// Check if calibration values are set
    [[nodiscard]] bool is_calibrated() const noexcept {
        return steps_per_theta_rot_ > 0 && rho_max_steps_ > 0;
    }

    /// Reset fractional accumulators (call after homing or position reset)
    void reset_accumulators() noexcept {
        theta_fractional_accumulator_ = 0.0;
        rho_fractional_accumulator_ = 0.0;
        // Also reset debug tracking
        reset_debug_tracking();
    }

    /// Reset debug cumulative tracking
    static void reset_debug_tracking() noexcept {
        debug_cumulative_steps_ = 0;
        debug_cumulative_theta_ = 0.0;
    }

    // Debug tracking (static so it persists across segments)
    static inline int64_t debug_cumulative_steps_ = 0;
    static inline double debug_cumulative_theta_ = 0.0;

    /// Convert polar deltas to motor step deltas with coupling compensation.
    /// Uses fractional accumulation to eliminate rounding drift.
    /// @param delta_theta_rad Theta change in radians (from min-rotation calculation)
    /// @param delta_rho_norm Rho change in normalized units
    /// @param theta_motor_steps Output: theta motor steps to execute
    /// @param rho_motor_steps Output: rho motor steps to execute (includes coupling)
    void delta_polar_to_motor_steps(
        double delta_theta_rad,
        double delta_rho_norm,
        int32_t& theta_motor_steps,
        int32_t& rho_motor_steps
    ) {
        // Convert theta delta to EXACT motor steps
        double theta_rotations = delta_theta_rad / (2.0 * M_PI);
        double theta_steps_exact = theta_rotations * static_cast<double>(steps_per_theta_rot_);

        // Add to theta accumulator and extract integer steps
        theta_fractional_accumulator_ += theta_steps_exact;
        theta_motor_steps = static_cast<int32_t>(std::round(theta_fractional_accumulator_));
        theta_fractional_accumulator_ -= static_cast<double>(theta_motor_steps);

        // Track cumulative for drift debugging
        debug_cumulative_theta_ += delta_theta_rad;
        debug_cumulative_steps_ += theta_motor_steps;

        // Log drift between mathematical expectation and actual commanded steps
        double expected_cumulative = (debug_cumulative_theta_ / (2.0 * M_PI)) * steps_per_theta_rot_;
        double drift = static_cast<double>(debug_cumulative_steps_) - expected_cumulative;
        if (std::fabs(drift) > 0.1) {
            ESP_LOGW("CoordTrans", "DRIFT: cum_theta=%.6f rad, expect=%.2f, cmd=%lld, drift=%.4f, accum=%.6f",
                     debug_cumulative_theta_, expected_cumulative, debug_cumulative_steps_, drift, theta_fractional_accumulator_);
        }

        // Convert rho delta to base steps
        double rho_base_exact = delta_rho_norm * static_cast<double>(rho_max_steps_);

        // Coupling compensation: use ACTUAL theta motor steps (what physically happens)
        // The motor steps by theta_motor_steps, which mechanically moves rho
        // We must counteract this exact amount
        double rho_counteract = static_cast<double>(theta_motor_steps) / kGearRatio;

        // Add to rho accumulator (base movement + coupling counteraction)
        rho_fractional_accumulator_ += rho_base_exact + rho_counteract;

        // Extract integer steps and keep fractional remainder
        rho_motor_steps = static_cast<int32_t>(std::round(rho_fractional_accumulator_));
        rho_fractional_accumulator_ -= static_cast<double>(rho_motor_steps);
    }

    /// Convert motor step positions to polar coordinates (for display only)
    [[nodiscard]] PolarPosition steps_to_polar(int32_t theta_steps, int32_t rho_steps) const {
        if (steps_per_theta_rot_ <= 0) {
            return {0.0, 0.0};
        }

        // Convert theta steps to radians (normalized to [0, 2π))
        double motor_rotations = static_cast<double>(theta_steps) / steps_per_theta_rot_;
        double theta = motor_rotations * 2.0 * M_PI;
        theta = normalize_angle(theta);

        // Account for coupling when calculating displayed rho
        double rho_counteract_steps = static_cast<double>(theta_steps) / kGearRatio;
        double effective_rho_steps = static_cast<double>(rho_steps) - rho_counteract_steps;
        double rho = effective_rho_steps / static_cast<double>(rho_max_steps_);

        // Clamp rho to valid range
        if (rho < 0.0) rho = 0.0;
        if (rho > 1.0) rho = 1.0;

        return {theta, rho};
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
    // Calibration values (set during homing)
    int32_t steps_per_theta_rot_ = 0;
    int32_t rho_max_steps_ = 0;

    // Fractional accumulators (prevent rounding drift over many segments)
    double theta_fractional_accumulator_ = 0.0;
    double rho_fractional_accumulator_ = 0.0;

    // Config constants - use double precision for accurate coupling compensation
    // THETA_GEAR_RATIO is now double in config.h (no float intermediate)
    static constexpr double kGearRatio = MechanicalConfig::THETA_GEAR_RATIO;
};

} // namespace sand_table
