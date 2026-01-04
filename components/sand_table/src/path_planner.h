#pragma once

#include "types.h"
#include "config.h"
#include "coordinate_transformer.h"

#include <functional>
#include <cmath>

namespace sand_table {

/// Path planner that breaks moves into ~1mm segments with coupling compensation.
/// Works with the VelocityPlanner to feed segments for execution.
class PathPlanner {
public:
    /// Callback for emitting segments (typically to VelocityPlanner)
    /// Return false to stop segment generation (e.g., if buffer is full)
    using SegmentCallback = std::function<bool(MotionSegment&)>;

    PathPlanner() = default;

    /// Set the callback for emitting segments
    void set_segment_callback(SegmentCallback cb) {
        callback_ = std::move(cb);
    }

    /// Plan a linear move from current to target position
    /// Breaks the move into ~1mm segments and emits them via callback
    ///
    /// @param current Current polar position
    /// @param target Target polar position
    /// @param feedrate Desired velocity in mm/s
    /// @return Error if move could not be planned
    [[nodiscard]] Result<void> plan_linear_move(
        const PolarPosition& current,
        const PolarPosition& target,
        float feedrate
    );

    /// Plan a spiral move (constant rho change over N rotations)
    ///
    /// @param current Current polar position
    /// @param start_rho Starting radius in mm
    /// @param end_rho Ending radius in mm
    /// @param rotations Number of rotations (can be fractional)
    /// @param feedrate Desired velocity in mm/s
    /// @return Error if move could not be planned
    [[nodiscard]] Result<void> plan_spiral(
        const PolarPosition& current,
        float start_rho,
        float end_rho,
        float rotations,
        float feedrate
    );

    /// Get the current position maintained by the path planner
    [[nodiscard]] const PolarPosition& current_position() const noexcept {
        return current_position_;
    }

    /// Set the current position (e.g., after homing)
    void set_current_position(const PolarPosition& pos) noexcept {
        current_position_ = pos;
    }

    /// Reset the rho accumulator (call after homing)
    void reset_accumulator() noexcept {
        rho_accumulator_ = 0.0f;
    }

private:
    SegmentCallback callback_;
    CoordinateTransformer transformer_;
    float rho_accumulator_ = 0.0f;  // Fractional step accumulator
    PolarPosition current_position_{0.0f, 0.0f};

    /// Create a single segment between two positions
    [[nodiscard]] MotionSegment create_segment(
        const PolarPosition& start,
        const PolarPosition& end,
        float feedrate,
        float length
    );
};

// =============================================================================
// Inline Implementations
// =============================================================================

inline MotionSegment PathPlanner::create_segment(
    const PolarPosition& start,
    const PolarPosition& end,
    float feedrate,
    float length)
{
    MotionSegment segment;

    // Calculate polar deltas
    segment.delta_theta_deg = CoordinateTransformer::min_angular_distance(start.theta, end.theta);
    segment.delta_rho_mm = end.rho - start.rho;
    segment.length_mm = length;

    // Convert to steps with coupling compensation
    PolarPosition delta{segment.delta_theta_deg, segment.delta_rho_mm};
    StepPosition steps = transformer_.delta_polar_to_steps(delta, rho_accumulator_);

    segment.delta_theta_steps = steps.theta;
    segment.delta_rho_steps = steps.rho;

    // Set velocity parameters (will be refined by VelocityPlanner)
    segment.nominal_velocity = feedrate;
    segment.acceleration = MotionConfig::DEFAULT_ACCEL_MM_S2;

    return segment;
}

inline Result<void> PathPlanner::plan_linear_move(
    const PolarPosition& current,
    const PolarPosition& target,
    float feedrate)
{
    if (!callback_) {
        return Result<void>::err(MotionError::InvalidState);
    }

    // Check bounds
    if (!CoordinateTransformer::is_within_bounds(target)) {
        return Result<void>::err(MotionError::OutOfBounds);
    }

    // Clamp feedrate
    feedrate = std::clamp(feedrate, MotionConfig::MIN_VELOCITY_MM_S,
                          MotionConfig::MAX_VELOCITY_MM_S);

    // Calculate total path length
    const float total_length = CoordinateTransformer::calculate_path_length(current, target);

    if (total_length < 0.01f) {
        // Negligible move, skip
        return Result<void>::ok();
    }

    // Calculate number of segments
    const uint32_t num_segments = std::max<uint32_t>(
        1u,
        static_cast<uint32_t>(std::ceil(total_length / MotionConfig::SEGMENT_LENGTH_MM))
    );

    // Calculate delta per segment
    const float delta_theta = CoordinateTransformer::min_angular_distance(current.theta, target.theta);
    const float delta_rho = target.rho - current.rho;

    const float theta_per_seg = delta_theta / static_cast<float>(num_segments);
    const float rho_per_seg = delta_rho / static_cast<float>(num_segments);
    const float length_per_seg = total_length / static_cast<float>(num_segments);

    // Generate segments
    PolarPosition segment_start = current;

    for (uint32_t i = 0; i < num_segments; ++i) {
        PolarPosition segment_end{
            segment_start.theta + theta_per_seg,
            segment_start.rho + rho_per_seg
        };

        // Create segment
        MotionSegment segment = create_segment(
            segment_start, segment_end, feedrate, length_per_seg
        );

        // Mark last segment
        segment.is_last_segment = (i == num_segments - 1);

        // Emit segment
        if (!callback_(segment)) {
            // Callback returned false, buffer might be full
            // Return error so caller can retry later
            return Result<void>::err(MotionError::QueueFull);
        }

        segment_start = segment_end;
    }

    // Update current position
    current_position_ = target;

    return Result<void>::ok();
}

inline Result<void> PathPlanner::plan_spiral(
    const PolarPosition& current,
    float start_rho,
    float end_rho,
    float rotations,
    float feedrate)
{
    if (!callback_) {
        return Result<void>::err(MotionError::InvalidState);
    }

    // Validate bounds
    if (!MechanicalConfig::is_rho_in_bounds(start_rho) ||
        !MechanicalConfig::is_rho_in_bounds(end_rho)) {
        return Result<void>::err(MotionError::OutOfBounds);
    }

    // Clamp feedrate
    feedrate = std::clamp(feedrate, MotionConfig::MIN_VELOCITY_MM_S,
                          MotionConfig::MAX_VELOCITY_MM_S);

    // Calculate total angular distance
    const float total_theta = rotations * 360.0f;

    // Estimate total path length (approximate as spiral arc length)
    const float avg_rho = (start_rho + end_rho) / 2.0f;
    const float arc_length = degrees_to_radians(std::abs(total_theta)) * avg_rho;
    const float radial_length = std::abs(end_rho - start_rho);
    const float total_length = std::sqrt(arc_length * arc_length + radial_length * radial_length);

    if (total_length < 0.01f) {
        return Result<void>::ok();
    }

    // Calculate number of segments
    const uint32_t num_segments = std::max<uint32_t>(
        1u,
        static_cast<uint32_t>(std::ceil(total_length / MotionConfig::SEGMENT_LENGTH_MM))
    );

    // Calculate delta per segment
    const float theta_per_seg = total_theta / static_cast<float>(num_segments);
    const float rho_per_seg = (end_rho - start_rho) / static_cast<float>(num_segments);

    // Generate segments
    PolarPosition segment_start{current.theta, start_rho};

    for (uint32_t i = 0; i < num_segments; ++i) {
        PolarPosition segment_end{
            segment_start.theta + theta_per_seg,
            segment_start.rho + rho_per_seg
        };

        // Calculate segment length
        float segment_length = CoordinateTransformer::calculate_path_length(
            segment_start, segment_end
        );

        // Create segment
        MotionSegment segment = create_segment(
            segment_start, segment_end, feedrate, segment_length
        );

        // Mark last segment
        segment.is_last_segment = (i == num_segments - 1);

        // Emit segment
        if (!callback_(segment)) {
            return Result<void>::err(MotionError::QueueFull);
        }

        segment_start = segment_end;
    }

    // Update current position
    current_position_ = {segment_start.theta, end_rho};

    return Result<void>::ok();
}

} // namespace sand_table
