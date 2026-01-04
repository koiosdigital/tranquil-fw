#pragma once

#include "types.h"
#include "config.h"

#include <cmath>

namespace sand_table {

/// Handles coordinate transformations between polar and step domains,
/// including the critical coupling compensation for the rack-and-pinion mechanism.
class CoordinateTransformer {
public:
    CoordinateTransformer() = default;

    /// Convert polar position delta to step counts with coupling compensation.
    /// This is the primary method for motion planning.
    ///
    /// @param delta The polar movement delta (theta in degrees, rho in mm)
    /// @param rho_accumulator Maintains fractional steps between calls to prevent drift
    /// @return Step counts for both axes
    [[nodiscard]] StepPosition delta_polar_to_steps(
        const PolarPosition& delta,
        float& rho_accumulator
    ) const;

    /// Calculate coupling compensation steps for a given theta movement.
    /// When theta rotates, the rack-and-pinion mechanism causes unintended
    /// radial movement that must be compensated.
    ///
    /// @param theta_motor_steps Number of theta motor steps
    /// @return Number of rho steps needed to compensate
    [[nodiscard]] float calculate_coupling_compensation(int32_t theta_motor_steps) const;

    /// Convert polar coordinates to Cartesian
    [[nodiscard]] static CartesianPosition polar_to_cartesian(const PolarPosition& p);

    /// Convert Cartesian coordinates to polar
    [[nodiscard]] static PolarPosition cartesian_to_polar(const CartesianPosition& c);

    /// Calculate the approximate path length between two polar positions
    /// Uses Cartesian approximation for short segments
    [[nodiscard]] static float calculate_path_length(
        const PolarPosition& start,
        const PolarPosition& end
    );

    /// Check if a position is within mechanical bounds
    [[nodiscard]] static bool is_within_bounds(const PolarPosition& pos);

    /// Convert step position to polar coordinates
    [[nodiscard]] StepPosition polar_to_steps(const PolarPosition& pos) const;

    /// Convert step position to polar coordinates
    [[nodiscard]] PolarPosition steps_to_polar(const StepPosition& steps) const;

    /// Get the minimum angular distance between two angles (in degrees)
    [[nodiscard]] static float min_angular_distance(float from, float to);

private:
    // Constants derived from MechanicalConfig
    static constexpr float kThetaStepsPerDeg = MechanicalConfig::THETA_STEPS_PER_DEG;
    static constexpr float kRhoStepsPerMm = MechanicalConfig::RHO_STEPS_PER_MM;
    static constexpr float kRhoStepsPerThetaStep = MechanicalConfig::RHO_STEPS_PER_THETA_STEP;
    static constexpr int8_t kCouplingDirection = MechanicalConfig::COUPLING_DIRECTION;
};

// =============================================================================
// Inline Implementations
// =============================================================================

inline StepPosition CoordinateTransformer::delta_polar_to_steps(
    const PolarPosition& delta,
    float& rho_accumulator) const
{
    // Convert theta degrees to motor steps
    const float theta_steps_float = delta.theta * kThetaStepsPerDeg;
    const int32_t theta_steps = static_cast<int32_t>(std::round(theta_steps_float));

    // Convert rho mm to motor steps (raw, without compensation)
    const float raw_rho_steps = delta.rho * kRhoStepsPerMm;

    // Calculate coupling compensation
    // When theta rotates, the pinion gear pulls/pushes the rack
    const float compensation = calculate_coupling_compensation(theta_steps);

    // Add compensation and accumulator for sub-step precision
    const float total_rho = raw_rho_steps + compensation + rho_accumulator;

    // Round to integer steps and store fractional remainder
    const int32_t rho_steps = static_cast<int32_t>(std::round(total_rho));
    rho_accumulator = total_rho - static_cast<float>(rho_steps);

    return {theta_steps, rho_steps};
}

inline float CoordinateTransformer::calculate_coupling_compensation(
    int32_t theta_motor_steps) const
{
    // For each theta motor step, the stage rotates by a small amount.
    // This rotation causes the rack to move relative to the stationary pinion.
    //
    // compensation = theta_steps * (1 / gear_ratio) * direction
    //
    // The direction is determined empirically based on the mechanical setup.

    return static_cast<float>(theta_motor_steps) *
           kRhoStepsPerThetaStep *
           static_cast<float>(kCouplingDirection);
}

inline CartesianPosition CoordinateTransformer::polar_to_cartesian(
    const PolarPosition& p)
{
    const float theta_rad = degrees_to_radians(p.theta);
    return {
        p.rho * std::cos(theta_rad),
        p.rho * std::sin(theta_rad)
    };
}

inline PolarPosition CoordinateTransformer::cartesian_to_polar(
    const CartesianPosition& c)
{
    const float rho = std::sqrt(c.x * c.x + c.y * c.y);
    float theta = radians_to_degrees(std::atan2(c.y, c.x));

    // Normalize to [0, 360)
    while (theta < 0.0f) theta += 360.0f;
    while (theta >= 360.0f) theta -= 360.0f;

    return {theta, rho};
}

inline float CoordinateTransformer::calculate_path_length(
    const PolarPosition& start,
    const PolarPosition& end)
{
    // For short segments (which is our use case), Cartesian approximation is accurate enough
    const auto start_cart = polar_to_cartesian(start);
    const auto end_cart = polar_to_cartesian(end);

    const float dx = end_cart.x - start_cart.x;
    const float dy = end_cart.y - start_cart.y;

    return std::sqrt(dx * dx + dy * dy);
}

inline bool CoordinateTransformer::is_within_bounds(const PolarPosition& pos) {
    return pos.rho >= static_cast<float>(MechanicalConfig::RHO_MIN_MM) &&
           pos.rho <= static_cast<float>(MechanicalConfig::RHO_MAX_MM);
}

inline StepPosition CoordinateTransformer::polar_to_steps(const PolarPosition& pos) const {
    return {
        static_cast<int32_t>(std::round(pos.theta * kThetaStepsPerDeg)),
        static_cast<int32_t>(std::round(pos.rho * kRhoStepsPerMm))
    };
}

inline PolarPosition CoordinateTransformer::steps_to_polar(const StepPosition& steps) const {
    // Note: This doesn't account for coupling, so it's only accurate after homing
    // or for small movements where coupling has been compensated
    return {
        static_cast<float>(steps.theta) / kThetaStepsPerDeg,
        static_cast<float>(steps.rho) / kRhoStepsPerMm
    };
}

inline float CoordinateTransformer::min_angular_distance(float from, float to) {
    float diff = to - from;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

} // namespace sand_table
