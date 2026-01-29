#pragma once

#include "types.h"
#include "config.h"

#include <functional>
#include <cmath>

namespace sand_table {

// =============================================================================
// Interpolation Mode Selection (for A/B testing)
// =============================================================================

/// Path interpolation mode for A/B testing.
/// TO SWITCH BACK TO CARTESIAN: Use InterpolationMode::Cartesian (the default)
enum class InterpolationMode {
    Cartesian,   // Linear interpolation in XY space, breaks into ~100 segments
    DirectPolar  // Direct polar move like main branch, single segment per move
};

/// Path planner that breaks moves into small segments.
/// Supports both Cartesian interpolation and direct polar modes.
/// Outputs delta-based segments (min-rotation for theta).
class PathPlanner {
public:
    /// Callback for emitting segments
    /// Return false to stop segment generation (e.g., if buffer is full)
    using SegmentCallback = std::function<bool(MotionSegment&)>;

    PathPlanner() = default;

    /// Set the callback for emitting segments
    void set_segment_callback(SegmentCallback cb) {
        callback_ = std::move(cb);
    }

    /// Set interpolation mode for A/B testing
    /// TO SWITCH BACK TO CARTESIAN: Use InterpolationMode::Cartesian
    void set_interpolation_mode(InterpolationMode mode) noexcept {
        mode_ = mode;
    }

    /// Get current interpolation mode
    [[nodiscard]] InterpolationMode get_interpolation_mode() const noexcept {
        return mode_;
    }

    /// Set the current position (e.g., after homing)
    void set_current_position(const PolarPosition& pos) noexcept {
        current_position_ = pos;
    }

    /// Get the current position maintained by the path planner
    [[nodiscard]] const PolarPosition& current_position() const noexcept {
        return current_position_;
    }

    /// Plan a move from current to target position
    /// Uses the configured interpolation mode (Cartesian or DirectPolar)
    ///
    /// @param current Current polar position (theta: radians, rho: 0-1)
    /// @param target Target polar position (theta: radians, rho: 0-1)
    /// @param feedrate_rpm Speed in RPM
    /// @return Error if move could not be planned
    [[nodiscard]] Result<void> plan_linear_move(
        const PolarPosition& current,
        const PolarPosition& target,
        float feedrate_rpm
    );

    /// Plan a direct polar move (single segment, like main branch)
    /// This bypasses Cartesian interpolation for better precision.
    ///
    /// @param current Current polar position (theta: radians, rho: 0-1)
    /// @param target Target polar position (theta: radians, rho: 0-1)
    /// @param feedrate_rpm Speed in RPM
    /// @return Error if move could not be planned
    [[nodiscard]] Result<void> plan_direct_polar_move(
        const PolarPosition& current,
        const PolarPosition& target,
        float feedrate_rpm
    );

    /// Get cumulative delta tracking (for debugging)
    [[nodiscard]] double debug_cumulative_theta() const noexcept { return debug_cumulative_theta_; }
    [[nodiscard]] double debug_cumulative_rho() const noexcept { return debug_cumulative_rho_; }
    void debug_reset_cumulative() noexcept { debug_cumulative_theta_ = 0; debug_cumulative_rho_ = 0; }

private:
    SegmentCallback callback_;
    PolarPosition current_position_{0.0, 0.0};

    // Interpolation mode (Cartesian vs DirectPolar for A/B testing)
    // DirectPolar: Single segment per move, no polar→cartesian→polar conversion errors
    // Cartesian: ~100 segments with linear XY interpolation (can accumulate float errors)
    InterpolationMode mode_ = InterpolationMode::DirectPolar;  // Using DirectPolar for testing

    // Debug: track cumulative deltas to verify they sum correctly
    double debug_cumulative_theta_ = 0.0;
    double debug_cumulative_rho_ = 0.0;

    // Segment length in normalized units (~1% of table)
    static constexpr double kSegmentLength = 0.01;

    /// Internal: Cartesian interpolation (original behavior)
    [[nodiscard]] Result<void> plan_cartesian_interpolation(
        const PolarPosition& current,
        const PolarPosition& target,
        float feedrate_rpm
    );
};

// =============================================================================
// Inline Implementation
// =============================================================================

inline Result<void> PathPlanner::plan_linear_move(
    const PolarPosition& current,
    const PolarPosition& target,
    float feedrate_rpm)
{
    // Branch based on interpolation mode
    // TO SWITCH BACK TO CARTESIAN: Use InterpolationMode::Cartesian
    if (mode_ == InterpolationMode::DirectPolar) {
        return plan_direct_polar_move(current, target, feedrate_rpm);
    }
    return plan_cartesian_interpolation(current, target, feedrate_rpm);
}

inline Result<void> PathPlanner::plan_direct_polar_move(
    const PolarPosition& current,
    const PolarPosition& target,
    float feedrate_rpm)
{
    if (!callback_) {
        return Result<void>::err(MotionError::InvalidState);
    }

    // Direct polar move - single segment like main branch
    // This avoids Cartesian round-trip conversion precision loss
    MotionSegment segment;

    // Use min-rotation for theta delta (shortest path)
    segment.delta_theta_rad = calculate_min_rotation(target.theta, current.theta);
    segment.delta_rho_norm = target.rho - current.rho;

    // Calculate distance for velocity profile (like main branch)
    // Main branch uses: max(thetaDelta / (2*PI), rhoDelta)
    // This normalizes theta to [0,1] range for comparison with rho
    const double theta_normalized = std::fabs(segment.delta_theta_rad) / (2.0 * M_PI);
    const double rho_delta = std::fabs(segment.delta_rho_norm);
    segment.distance = static_cast<float>(std::max(theta_normalized, rho_delta));

    // Ensure minimum distance for moves that are essentially stationary
    if (segment.distance < 0.001f) {
        // Negligible move - update position and return
        current_position_ = target;
        return Result<void>::ok();
    }

    segment.nominal_velocity = feedrate_rpm;
    segment.is_last_segment = true;

    // Motor steps will be calculated at execution time, not here
    segment.delta_theta_steps = 0;
    segment.delta_rho_steps = 0;

    if (!callback_(segment)) {
        return Result<void>::err(MotionError::QueueFull);
    }

    // Store the final position for the next move
    current_position_ = target;
    return Result<void>::ok();
}

inline Result<void> PathPlanner::plan_cartesian_interpolation(
    const PolarPosition& current,
    const PolarPosition& target,
    float feedrate_rpm)
{
    if (!callback_) {
        return Result<void>::err(MotionError::InvalidState);
    }

    // SPECIAL CASE: Move to/from center (rho=0)
    // At center, theta is undefined - skip all trig and just move rho.
    // This avoids atan2 singularity and spurious theta commands.
    constexpr double kCenterThreshold = 0.001;  // Effectively zero

    if (target.rho < kCenterThreshold) {
        // Moving TO center: just retract rho, no theta
        MotionSegment segment;
        segment.delta_theta_rad = 0.0;
        segment.delta_rho_norm = -current.rho;
        segment.distance = static_cast<float>(current.rho);
        segment.nominal_velocity = feedrate_rpm;
        segment.is_last_segment = true;
        segment.delta_theta_steps = 0;
        segment.delta_rho_steps = 0;

        if (std::fabs(segment.delta_rho_norm) > 0.001) {
            if (!callback_(segment)) {
                return Result<void>::err(MotionError::QueueFull);
            }
        }

        current_position_ = target;
        return Result<void>::ok();
    }

    if (current.rho < kCenterThreshold) {
        // Moving FROM center: direct polar move to target (theta + rho)
        // Current theta is meaningless at center, so we move to target theta directly.
        // No Cartesian math needed - just a simple polar move.
        MotionSegment segment;
        segment.delta_theta_rad = calculate_min_rotation(target.theta, current.theta);
        segment.delta_rho_norm = target.rho;  // From ~0 to target
        segment.distance = static_cast<float>(target.rho);  // Distance dominated by rho
        segment.nominal_velocity = feedrate_rpm;
        segment.is_last_segment = true;
        segment.delta_theta_steps = 0;
        segment.delta_rho_steps = 0;

        if (!callback_(segment)) {
            return Result<void>::err(MotionError::QueueFull);
        }

        current_position_ = target;
        return Result<void>::ok();
    }

    // Convert polar to Cartesian for linear interpolation
    // x = rho * cos(theta), y = rho * sin(theta)
    const double start_x = current.rho * std::cos(current.theta);
    const double start_y = current.rho * std::sin(current.theta);
    const double end_x = target.rho * std::cos(target.theta);
    const double end_y = target.rho * std::sin(target.theta);

    // Calculate Cartesian distance (in normalized units since rho is 0-1)
    const double delta_x = end_x - start_x;
    const double delta_y = end_y - start_y;
    const double total_distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);

    if (total_distance < 0.001) {
        // Negligible move - update position and return
        current_position_ = target;
        return Result<void>::ok();
    }

    // Calculate number of segments based on Cartesian distance
    const uint32_t num_segments = std::max<uint32_t>(
        1u, static_cast<uint32_t>(std::ceil(total_distance / kSegmentLength))
    );

    const float dist_per_seg = static_cast<float>(total_distance / num_segments);

    // Generate segments by interpolating linearly in Cartesian space
    PolarPosition prev_pos = current;
    for (uint32_t i = 0; i < num_segments; ++i) {
        // Linear interpolation parameter (0 to 1)
        const double t = static_cast<double>(i + 1) / static_cast<double>(num_segments);

        // Interpolate in Cartesian space
        const double seg_x = start_x + t * delta_x;
        const double seg_y = start_y + t * delta_y;

        // Convert back to polar
        const double seg_rho = std::sqrt(seg_x * seg_x + seg_y * seg_y);
        const double seg_theta = std::atan2(seg_y, seg_x);  // Returns [-π, π]

        // Normalize to [0, 2π] for consistency
        const double seg_theta_norm = (seg_theta < 0) ? seg_theta + 2.0 * M_PI : seg_theta;

        MotionSegment segment;

        // Use min-rotation for theta delta (shortest path, like main branch)
        // This avoids multi-rotation accumulation and keeps things simple
        segment.delta_theta_rad = calculate_min_rotation(seg_theta_norm, prev_pos.theta);
        segment.delta_rho_norm = seg_rho - prev_pos.rho;
        segment.distance = dist_per_seg;
        segment.nominal_velocity = feedrate_rpm;
        segment.is_last_segment = (i == num_segments - 1);

        // Motor steps will be calculated at execution time, not here
        segment.delta_theta_steps = 0;
        segment.delta_rho_steps = 0;

        if (!callback_(segment)) {
            return Result<void>::err(MotionError::QueueFull);
        }

        // Update prev_pos for next segment
        prev_pos = {seg_theta_norm, seg_rho};
    }

    // Store the final position for the next move
    current_position_ = target;
    return Result<void>::ok();
}

} // namespace sand_table
