#pragma once

#include "types.h"
#include "config.h"

#include <cmath>

namespace sand_table {

/// Handles coordinate transformations between polar and step domains.
/// Uses calibrated values from homing for accurate conversions.
/// Coordinate system:
///   - theta: radians (0 to 2π)
///   - rho: normalized (0 = center, 1 = maximum radius)
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

    /// Convert motor step positions to polar coordinates
    /// This accounts for coupling: the rack-and-pinion mechanism means
    /// rho position is affected by theta position
    [[nodiscard]] PolarPosition steps_to_polar(int32_t theta_steps, int32_t rho_steps) const;

    /// Convert polar position to absolute step position
    /// @param polar Polar position (theta in radians, rho normalized 0-1)
    /// @param theta_steps Output: absolute theta steps
    /// @param rho_steps Output: absolute rho steps (raw, without coupling)
    void polar_to_absolute_steps(
        const PolarPosition& polar,
        int32_t& theta_steps,
        int32_t& rho_steps
    ) const;

    /// Calculate motor step deltas with coupling compensation
    /// This uses integer step positions to avoid floating-point accumulation errors
    /// @param current_theta_steps Current theta position in steps
    /// @param current_rho_steps Current rho position in steps
    /// @param target_theta_steps Target theta position in steps
    /// @param target_rho_steps Target rho position in steps (raw, without coupling)
    /// @param delta_theta_steps Output: theta motor steps to execute
    /// @param delta_rho_steps Output: rho motor steps to execute (includes compensation)
    void calculate_coupled_delta_steps(
        int32_t current_theta_steps,
        int32_t current_rho_steps,
        int32_t target_theta_steps,
        int32_t target_rho_steps,
        int32_t& delta_theta_steps,
        int32_t& delta_rho_steps
    ) const;

    /// Get calibrated steps per full theta rotation
    [[nodiscard]] int32_t steps_per_theta_rotation() const noexcept {
        return steps_per_theta_rot_;
    }

    /// Get calibrated rho max steps
    [[nodiscard]] int32_t rho_max_steps() const noexcept {
        return rho_max_steps_;
    }

private:
    // Calibration values (set during homing)
    int32_t steps_per_theta_rot_ = 0;
    int32_t rho_max_steps_ = 0;

    // Config constants
    static constexpr float kGearRatio = MechanicalConfig::THETA_GEAR_RATIO;
    static constexpr float kRhoStepsPerThetaStep = MechanicalConfig::RHO_STEPS_PER_THETA_STEP;

    /// Convert polar position to steps (without coupling compensation)
    void polar_to_steps(const PolarPosition& polar, int32_t& theta_steps, int32_t& rho_steps) const;

    /// Convert normalized rho (0-1) to steps using calibrated maximum
    [[nodiscard]] int32_t normalized_rho_to_steps(double normalized_rho) const;

    /// Convert steps to normalized rho (0-1) using calibrated maximum
    [[nodiscard]] double steps_to_normalized_rho(int32_t steps) const;
};

// =============================================================================
// Inline Implementations
// =============================================================================

inline PolarPosition CoordinateTransformer::steps_to_polar(
    int32_t theta_steps,
    int32_t rho_steps) const
{
    if (steps_per_theta_rot_ <= 0) {
        return {0.0, 0.0};
    }

    // Convert theta steps to radians
    // motor rotations = theta_steps / steps_per_theta_rot
    // theta_radians = motor_rotations * 2π
    double motor_rotations = static_cast<double>(theta_steps) / steps_per_theta_rot_;
    double theta = motor_rotations * 2.0 * M_PI;
    theta = normalize_angle(theta);

    // Account for coupling: theta rotation affects rho position
    // Counteract theta influence on rho
    // Use std::round() to avoid truncation errors that accumulate over many segments
    double rho_counteract_steps = static_cast<double>(theta_steps) / kGearRatio;
    double rho = steps_to_normalized_rho(static_cast<int32_t>(std::round(
        static_cast<double>(rho_steps) - rho_counteract_steps
    )));

    return {theta, rho};
}

inline void CoordinateTransformer::polar_to_steps(
    const PolarPosition& polar,
    int32_t& theta_steps,
    int32_t& rho_steps) const
{
    // DON'T normalize theta - let it accumulate naturally
    // This ensures closed loops return to exact starting position
    // (normalization is only done for display in steps_to_polar)
    double drive_gear_rotations = polar.theta / (2.0 * M_PI);
    theta_steps = static_cast<int32_t>(std::round(drive_gear_rotations * steps_per_theta_rot_));

    // Convert normalized rho to steps
    rho_steps = normalized_rho_to_steps(polar.rho);
}

inline void CoordinateTransformer::polar_to_absolute_steps(
    const PolarPosition& polar,
    int32_t& theta_steps,
    int32_t& rho_steps) const
{
    // Public wrapper for polar_to_steps - converts polar position to absolute step counts
    polar_to_steps(polar, theta_steps, rho_steps);
}

inline void CoordinateTransformer::calculate_coupled_delta_steps(
    int32_t current_theta_steps,
    int32_t current_rho_steps,
    int32_t target_theta_steps,
    int32_t target_rho_steps,
    int32_t& delta_theta_steps,
    int32_t& delta_rho_steps) const
{
    // Pure integer arithmetic for step deltas - no floating point accumulation errors
    // NO wraparound: for continuous patterns, theta accumulates naturally.
    // The path planner provides continuous (non-normalized) theta values.

    delta_theta_steps = target_theta_steps - current_theta_steps;

    // Rho delta
    int32_t raw_rho_delta = target_rho_steps - current_rho_steps;

    // Add coupling compensation:
    // When theta motor rotates, the rack-and-pinion causes unintended rho movement.
    // We must ADD counteracting steps to rho to compensate.
    // Use integer division with rounding: (|delta| + gear_ratio/2) / gear_ratio
    const int32_t gear_ratio_int = static_cast<int32_t>(kGearRatio);
    int32_t rho_counteract = (std::abs(delta_theta_steps) + gear_ratio_int / 2) / gear_ratio_int;
    if (delta_theta_steps < 0) {
        rho_counteract = -rho_counteract;
    }

    delta_rho_steps = raw_rho_delta + rho_counteract;
}

inline int32_t CoordinateTransformer::normalized_rho_to_steps(double normalized_rho) const {
    if (rho_max_steps_ <= 0) {
        return 0;
    }
    // Use std::round() to avoid truncation errors that accumulate over many segments
    return static_cast<int32_t>(std::round(normalized_rho * static_cast<double>(rho_max_steps_)));
}

inline double CoordinateTransformer::steps_to_normalized_rho(int32_t steps) const {
    if (rho_max_steps_ <= 0) {
        return 0.0;
    }
    return static_cast<double>(steps) / static_cast<double>(rho_max_steps_);
}

} // namespace sand_table
