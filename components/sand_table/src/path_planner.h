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

        /// Set calibrated step scales (from homing) so segment splitting is
        /// computed against the real machine, not compile-time defaults.
        void set_calibration(int32_t steps_per_theta_rot, int32_t rho_max_steps) noexcept {
            if (steps_per_theta_rot > 0) steps_per_theta_rot_ = steps_per_theta_rot;
            if (rho_max_steps > 0) rho_max_steps_ = rho_max_steps;
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
        /// @param feedrate_rpm Speed in normalized units/min (see MotionController::move_to)
        /// @param base_feedrate_rpm User feedrate without transient boosts,
        ///        used for live feedrate scaling (0 = same as feedrate_rpm)
        /// @return Error if move could not be planned
        [[nodiscard]] Result<void> plan_linear_move(
            const PolarPosition& current,
            const PolarPosition& target,
            float feedrate_rpm,
            float base_feedrate_rpm = 0.0f
        );

        /// Plan a direct polar move (no Cartesian interpolation)
        /// This moves theta and rho directly, creating arcs in XY space.
        ///
        /// @param current Current polar position (theta: radians, rho: 0-1)
        /// @param target Target polar position (theta: radians, rho: 0-1)
        /// @param feedrate_rpm Speed in normalized units/min (see MotionController::move_to)
        /// @param base_feedrate_rpm User feedrate without transient boosts,
        ///        used for live feedrate scaling (0 = same as feedrate_rpm)
        /// @return Error if move could not be planned
        [[nodiscard]] Result<void> plan_polar_move(
            const PolarPosition& current,
            const PolarPosition& target,
            float feedrate_rpm,
            float base_feedrate_rpm = 0.0f
        );

    private:
        SegmentCallback callback_;
        PolarPosition current_position_{ 0.0, 0.0 };

        // Calibrated step scales (defaults match nominal mechanics; homing overrides)
        int32_t steps_per_theta_rot_ = MechanicalConfig::NOMINAL_STEPS_PER_THETA_ROTATION;
        int32_t rho_max_steps_ = 20000;

        // Segment length in normalized units (~1% of table)
        static constexpr double kSegmentLength = 0.01;

        // Hard per-segment step budget. Must stay comfortably below
        // CoordinatedStepperController::kMaxIntervalsPerSegment (which rejects,
        // not truncates, oversized segments). 75% leaves headroom for rho
        // coupling compensation and accumulator variance.
        static constexpr uint32_t kMaxStepsPerSegment =
            (MotionConfig::MAX_SEGMENT_STEPS * 3) / 4;

        /// Estimate the major-axis motor steps a (delta_theta, delta_rho) move needs,
        /// including the rho coupling compensation contribution.
        [[nodiscard]] uint32_t estimate_steps(double delta_theta_rad, double delta_rho_norm) const {
            const double theta_steps = std::fabs(delta_theta_rad) / (2.0 * M_PI) *
                static_cast<double>(steps_per_theta_rot_);
            // Rho motor moves for the commanded rho delta plus coupling counteraction
            // (one rho motor rev dragged per theta drum rev, so per theta motor
            // step: effective_steps_per_rev / steps_per_theta_rot).
            const double rho_steps = std::fabs(delta_rho_norm) * static_cast<double>(rho_max_steps_) +
                theta_steps * static_cast<double>(MechanicalConfig::effective_steps_per_rev()) /
                    static_cast<double>(steps_per_theta_rot_);
            const double major = std::fmax(theta_steps, rho_steps);
            return static_cast<uint32_t>(std::ceil(major));
        }

        /// Emit a delta move, splitting it into as many segments as needed to
        /// respect kMaxStepsPerSegment. Updates current_position_ incrementally
        /// per emitted segment so a mid-move failure leaves the planner position
        /// consistent with what was actually queued.
        /// @param end_position Absolute position after the full delta (assigned
        ///        exactly on full success to avoid accumulation error).
        [[nodiscard]] Result<void> emit_split(
            double delta_theta_rad,
            double delta_rho_norm,
            double distance_norm,
            float feedrate_rpm,
            const PolarPosition& end_position,
            float base_feedrate_rpm = 0.0f)
        {
            const uint32_t est = estimate_steps(delta_theta_rad, delta_rho_norm);
            const uint32_t num_segments = (est > 0)
                ? std::max<uint32_t>(1u, (est + kMaxStepsPerSegment - 1) / kMaxStepsPerSegment)
                : 1u;

            const double theta_per_seg = delta_theta_rad / num_segments;
            const double rho_per_seg = delta_rho_norm / num_segments;
            const float dist_per_seg = static_cast<float>(
                std::fmax(distance_norm / num_segments, 0.001));

            for (uint32_t i = 0; i < num_segments; ++i) {
                MotionSegment segment;
                segment.delta_theta_rad = theta_per_seg;
                segment.delta_rho_norm = rho_per_seg;
                segment.distance = dist_per_seg;
                segment.nominal_velocity = feedrate_rpm;
                segment.base_feedrate = (base_feedrate_rpm > 0.0f)
                    ? base_feedrate_rpm : feedrate_rpm;
                segment.delta_theta_steps = 0;  // Calculated later by transformer
                segment.delta_rho_steps = 0;

                if (!callback_(segment)) {
                    return Result<void>::err(MotionError::QueueFull);
                }

                // Advance planner position by what was actually queued
                current_position_.theta += theta_per_seg;
                current_position_.rho += rho_per_seg;
            }

            // Snap to the exact target to avoid floating-point drift
            current_position_ = end_position;
            return Result<void>::ok();
        }
    };

    // =============================================================================
    // Inline Implementation
    // =============================================================================

    inline Result<void> PathPlanner::plan_linear_move(
        const PolarPosition& current,
        const PolarPosition& target,
        float feedrate_rpm,
        float base_feedrate_rpm)
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
            if (std::fabs(current.rho) > 0.001) {
                return emit_split(0.0, -current.rho, current.rho, feedrate_rpm, target,
                    base_feedrate_rpm);
            }
            current_position_ = target;
            return Result<void>::ok();
        }

        if (current.rho < kCenterThreshold) {
            // Moving FROM center: direct polar move to target (theta + rho)
            // Current theta is meaningless at center, so we move to target theta directly.
            return emit_split(
                calculate_min_rotation(target.theta, current.theta),
                target.rho,  // From ~0 to target
                target.rho,  // Distance dominated by rho
                feedrate_rpm,
                target,
                base_feedrate_rpm);
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

            // Use min-rotation for theta delta (shortest path). A piece passing
            // near the center can flip theta by up to π in one interpolation
            // step (a real half-turn of the arm) — emit_split subdivides it so
            // no single segment exceeds the step budget.
            const double d_theta = calculate_min_rotation(seg_theta_norm, prev_pos.theta);
            const double d_rho = seg_rho - prev_pos.rho;

            auto result = emit_split(d_theta, d_rho, dist_per_seg, feedrate_rpm,
                { current_position_.theta + d_theta, seg_rho }, base_feedrate_rpm);
            if (result.is_err()) {
                return result;
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
        float feedrate_rpm,
        float base_feedrate_rpm)
    {
        if (!callback_) {
            return Result<void>::err(MotionError::InvalidState);
        }

        // Theta is CONTINUOUS in pattern space (theta-rho files wind theta
        // monotonically past 2π), so the delta between consecutive points is
        // meant literally — direction and magnitude included. Folding deltas
        // in (π, 2π) to the "shorter direction" reverses any sweep longer
        // than half a turn: the table draws the complementary arc backwards
        // while the thumbnail (which interpolates through the authored
        // points) shows the intended path. Take the raw delta unchanged.
        const double delta_theta = target.theta - current.theta;
        const double delta_rho = target.rho - current.rho;

        // Skip negligible moves
        if (std::fabs(delta_theta) < 0.001 && std::fabs(delta_rho) < 0.001) {
            current_position_ = target;
            return Result<void>::ok();
        }

        // Calculate distance for velocity planning (arc length approximation)
        const double avg_rho = (current.rho + target.rho) / 2.0;
        const double arc_component = avg_rho * std::fabs(delta_theta);
        const double total_distance = std::sqrt(arc_component * arc_component + delta_rho * delta_rho);

        return emit_split(delta_theta, delta_rho, total_distance, feedrate_rpm, target,
            base_feedrate_rpm);
    }

} // namespace sand_table
