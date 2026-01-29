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

    /// Convert polar delta to motor steps with coupling compensation
    /// @param start_polar Starting polar position
    /// @param target_polar Target polar position
    /// @param theta_motor_steps Output: theta motor steps to execute
    /// @param rho_motor_steps Output: rho motor steps to execute (includes compensation)
    void calculate_coupled_motor_steps(
        const PolarPosition& start_polar,
        const PolarPosition& target_polar,
        int32_t& theta_motor_steps,
        int32_t& rho_motor_steps
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
    double rho_counteract_steps = static_cast<double>(theta_steps) / kGearRatio;
    double rho = steps_to_normalized_rho(static_cast<int32_t>(
        static_cast<double>(rho_steps) - rho_counteract_steps
    ));

    return {theta, rho};
}

inline void CoordinateTransformer::polar_to_steps(
    const PolarPosition& polar,
    int32_t& theta_steps,
    int32_t& rho_steps) const
{
    // Convert theta from radians to steps
    // drive_gear_rotations = theta / 2π
    // theta_steps = drive_gear_rotations * steps_per_theta_rot
    double drive_gear_rotations = polar.theta / (2.0 * M_PI);
    theta_steps = static_cast<int32_t>(drive_gear_rotations * steps_per_theta_rot_);

    // Convert normalized rho to steps
    rho_steps = normalized_rho_to_steps(polar.rho);
}

inline void CoordinateTransformer::calculate_coupled_motor_steps(
    const PolarPosition& start_polar,
    const PolarPosition& target_polar,
    int32_t& theta_motor_steps,
    int32_t& rho_motor_steps) const
{
    // Calculate delta in polar coordinates
    PolarPosition delta_polar;
    delta_polar.theta = calculate_min_rotation(target_polar.theta, start_polar.theta);
    delta_polar.rho = target_polar.rho - start_polar.rho;

    // Convert delta to base steps
    int32_t rho_base;
    polar_to_steps(delta_polar, theta_motor_steps, rho_base);

    // Add coupling compensation:
    // When theta rotates, the rack-and-pinion coupling causes unintended rho movement.
    // We must ADD counteracting steps to rho to compensate.
    // counteract = theta_motor_steps / gear_ratio
    double rho_counteract_steps = static_cast<double>(theta_motor_steps) / kGearRatio;
    rho_motor_steps = rho_base + static_cast<int32_t>(rho_counteract_steps);
}

inline int32_t CoordinateTransformer::normalized_rho_to_steps(double normalized_rho) const {
    if (rho_max_steps_ <= 0) {
        return 0;
    }
    return static_cast<int32_t>(normalized_rho * static_cast<double>(rho_max_steps_));
}

inline double CoordinateTransformer::steps_to_normalized_rho(int32_t steps) const {
    if (rho_max_steps_ <= 0) {
        return 0.0;
    }
    return static_cast<double>(steps) / static_cast<double>(rho_max_steps_);
}

} // namespace sand_table
