#pragma once

#include "types.h"
#include "config.h"

#include <functional>
#include <cmath>

namespace sand_table {

    /// Path planner that breaks moves into small segments using Cartesian interpolation.
    /// Linear interpolation in XY space produces smooth curves in polar coordinates.
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

        /// Set the current position (e.g., after homing)
        void set_current_position(const PolarPosition& pos) noexcept {
            current_position_ = pos;
        }

        /// Get the current position maintained by the path planner
        [[nodiscard]] const PolarPosition& current_position() const noexcept {
            return current_position_;
        }

        /// Plan a move from current to target position using Cartesian interpolation
        /// This creates a straight line in XY space, which produces smooth curves.
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

        /// Plan a direct polar move (no Cartesian interpolation)
        /// This moves theta and rho directly, creating arcs in XY space.
        ///
        /// @param current Current polar position (theta: radians, rho: 0-1)
        /// @param target Target polar position (theta: radians, rho: 0-1)
        /// @param feedrate_rpm Speed in RPM
        /// @return Error if move could not be planned
        [[nodiscard]] Result<void> plan_polar_move(
            const PolarPosition& current,
            const PolarPosition& target,
            float feedrate_rpm
        );

    private:
        SegmentCallback callback_;
        PolarPosition current_position_{ 0.0, 0.0 };

        // Segment length in normalized units (~1% of table)
        static constexpr double kSegmentLength = 0.01;
    };

    // =============================================================================
    // Inline Implementation
    // =============================================================================

    inline Result<void> PathPlanner::plan_linear_move(
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
            MotionSegment segment;
            segment.delta_theta_rad = calculate_min_rotation(target.theta, current.theta);
            segment.delta_rho_norm = target.rho;  // From ~0 to target
            segment.distance = static_cast<float>(target.rho);  // Distance dominated by rho
            segment.nominal_velocity = feedrate_rpm;
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

            // Use min-rotation for theta delta (shortest path)
            segment.delta_theta_rad = calculate_min_rotation(seg_theta_norm, prev_pos.theta);
            segment.delta_rho_norm = seg_rho - prev_pos.rho;
            segment.distance = dist_per_seg;
            segment.nominal_velocity = feedrate_rpm;

            // Motor steps will be calculated at execution time
            segment.delta_theta_steps = 0;
            segment.delta_rho_steps = 0;

            if (!callback_(segment)) {
                return Result<void>::err(MotionError::QueueFull);
            }

            // Update prev_pos for next segment
            prev_pos = { seg_theta_norm, seg_rho };
        }

        // Store the final position for the next move
        current_position_ = target;
        return Result<void>::ok();
    }

    inline Result<void> PathPlanner::plan_polar_move(
        const PolarPosition& current,
        const PolarPosition& target,
        float feedrate_rpm)
    {
        if (!callback_) {
            return Result<void>::err(MotionError::InvalidState);
        }

        // Calculate delta theta with direction preference
        // - For multi-rotation moves (>= 2π), preserve full rotation count
        // - For sub-rotation moves (< 2π), pick the shorter direction
        const double raw_delta = target.theta - current.theta;
        double delta_theta;
        if (std::fabs(raw_delta) >= 2.0 * M_PI) {
            // Multi-rotation: preserve full intent (e.g., 0 to 20π = 10 rotations)
            delta_theta = raw_delta;
        } else {
            // Sub-rotation: pick shorter direction (e.g., 1.9π to 0.1π = +0.2π not -1.8π)
            delta_theta = std::fmod(raw_delta, 2.0 * M_PI);
            if (delta_theta > M_PI) delta_theta -= 2.0 * M_PI;
            else if (delta_theta < -M_PI) delta_theta += 2.0 * M_PI;
        }
        const double delta_rho = target.rho - current.rho;

        // Skip negligible moves
        if (std::fabs(delta_theta) < 0.001 && std::fabs(delta_rho) < 0.001) {
            current_position_ = target;
            return Result<void>::ok();
        }

        // Estimate total motor steps to determine if segmentation is needed
        // RMT driver has kMaxIntervalsPerSegment = 32768 limit
        // Use 75% of limit to leave headroom for rho coupling and accumulator variance
        constexpr double kStepsPerThetaRotation = 25760.0;  // 200 * 16 * 8.05 (default config)
        constexpr uint32_t kMaxStepsPerSegment = 24576;     // 75% of 32768

        const double theta_rotations = std::fabs(delta_theta) / (2.0 * M_PI);
        const uint32_t estimated_theta_steps = static_cast<uint32_t>(
            theta_rotations * kStepsPerThetaRotation);

        // Calculate number of segments needed
        const uint32_t num_segments = std::max<uint32_t>(
            1u,
            static_cast<uint32_t>(std::ceil(
                static_cast<double>(estimated_theta_steps) / kMaxStepsPerSegment))
        );

        // Calculate per-segment deltas
        const double theta_per_segment = delta_theta / num_segments;
        const double rho_per_segment = delta_rho / num_segments;

        // Calculate distance for velocity planning (arc length approximation)
        const double avg_rho = (current.rho + target.rho) / 2.0;
        const double arc_component = avg_rho * std::fabs(delta_theta);
        const double total_distance = std::sqrt(arc_component * arc_component + delta_rho * delta_rho);
        const float dist_per_segment = static_cast<float>(
            std::max(total_distance / num_segments, 0.001));

        // Emit segments
        for (uint32_t i = 0; i < num_segments; ++i) {
            MotionSegment segment;
            segment.delta_theta_rad = theta_per_segment;
            segment.delta_rho_norm = rho_per_segment;
            segment.distance = dist_per_segment;
            segment.nominal_velocity = feedrate_rpm;
            segment.delta_theta_steps = 0;  // Calculated later by transformer
            segment.delta_rho_steps = 0;

            if (!callback_(segment)) {
                return Result<void>::err(MotionError::QueueFull);
            }
        }

        current_position_ = target;
        return Result<void>::ok();
    }

} // namespace sand_table
