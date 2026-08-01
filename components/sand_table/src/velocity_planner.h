#pragma once

#include "types.h"
#include "config.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace sand_table {

/// Velocity planner with lookahead buffer for smooth motion.
/// Implements a two-pass algorithm to calculate optimal entry/exit velocities
/// at segment junctions, enabling smooth cornering and constant acceleration.
class VelocityPlanner {
public:
    static constexpr size_t kLookaheadDepth = MotionConfig::LOOKAHEAD_DEPTH;

    VelocityPlanner() = default;

    /// Add a segment to the lookahead buffer, triggers recalculation
    /// @return true if segment was added, false if buffer is full
    [[nodiscard]] bool add_segment(MotionSegment& segment);

    /// Get next segment for execution (removes from buffer)
    /// @return The next segment, or nullopt if buffer is empty
    [[nodiscard]] std::optional<MotionSegment> pop_segment();

    /// Peek at the next segment without removing it
    [[nodiscard]] std::optional<MotionSegment> peek_segment() const;

    /// Get current number of segments in buffer
    [[nodiscard]] size_t segment_count() const noexcept { return count_; }

    /// Check if buffer is empty
    [[nodiscard]] bool empty() const noexcept { return count_ == 0; }

    /// Check if buffer is full
    [[nodiscard]] bool full() const noexcept { return count_ >= kLookaheadDepth; }

    /// Clear all segments from buffer
    void clear() noexcept;

private:
    std::array<MotionSegment, kLookaheadDepth> segments_{};
    size_t head_ = 0;  // Next position to write
    size_t tail_ = 0;  // Next position to read
    size_t count_ = 0;

    // Track last transferred segment's exit velocities for continuity
    float last_theta_exit_velocity_ = 0.0f;
    float last_rho_exit_velocity_ = 0.0f;

    /// Recalculate per-motor junction velocities across the buffer.
    /// Acceleration-limited ramping between the junction velocities is done
    /// per-segment by CoordinatedStepperController::calculate_velocity_profile;
    /// this pass only decides where each motor must stop (direction reversals).
    void recalculate();

    /// Get segment at buffer index (with wraparound)
    [[nodiscard]] MotionSegment& at(size_t index);
    [[nodiscard]] const MotionSegment& at(size_t index) const;
};

// =============================================================================
// Inline Implementations
// =============================================================================

inline void VelocityPlanner::clear() noexcept {
    head_ = 0;
    tail_ = 0;
    count_ = 0;
    last_theta_exit_velocity_ = 0.0f;
    last_rho_exit_velocity_ = 0.0f;
}

inline MotionSegment& VelocityPlanner::at(size_t index) {
    return segments_[(tail_ + index) % kLookaheadDepth];
}

inline const MotionSegment& VelocityPlanner::at(size_t index) const {
    return segments_[(tail_ + index) % kLookaheadDepth];
}

inline bool VelocityPlanner::add_segment(MotionSegment& segment) {
    if (full()) {
        return false;
    }

    // Store segment
    segments_[head_] = segment;
    head_ = (head_ + 1) % kLookaheadDepth;
    count_++;

    // Recalculate velocities with new segment
    recalculate();

    // Update the input segment with calculated velocities
    if (count_ == 1) {
        segment = at(0);
    }

    return true;
}

inline std::optional<MotionSegment> VelocityPlanner::pop_segment() {
    if (empty()) {
        return std::nullopt;
    }

    MotionSegment segment = segments_[tail_];

    // Save exit velocity for next segment's entry
    last_theta_exit_velocity_ = segment.theta_exit_velocity;
    last_rho_exit_velocity_ = segment.rho_exit_velocity;

    tail_ = (tail_ + 1) % kLookaheadDepth;
    count_--;

    return segment;
}

inline std::optional<MotionSegment> VelocityPlanner::peek_segment() const {
    if (empty()) {
        return std::nullopt;
    }
    return segments_[tail_];
}

inline void VelocityPlanner::recalculate() {
    if (count_ == 0) {
        return;
    }

    auto sign = [](int32_t v) -> int {
        if (v > 0) return 1;
        if (v < 0) return -1;
        return 0;
    };

    // Per-motor velocity planning: only slow down when that motor reverses direction
    // The stepper driver handles acceleration profiles based on actual step counts

    // First segment: entry from previous segment's exit (for continuity)
    // But if this motor isn't moving, entry is 0
    MotionSegment& first = at(0);
    first.theta_entry_velocity = (sign(first.delta_theta_steps) != 0) ? last_theta_exit_velocity_ : 0.0f;
    first.rho_entry_velocity = (sign(first.delta_rho_steps) != 0) ? last_rho_exit_velocity_ : 0.0f;

    // Set exit velocities and propagate through junctions
    for (size_t i = 0; i < count_; ++i) {
        MotionSegment& seg = at(i);

        // Default: exit at nominal velocity (if motor is moving)
        seg.theta_exit_velocity = (sign(seg.delta_theta_steps) != 0) ? seg.nominal_velocity : 0.0f;
        seg.rho_exit_velocity = (sign(seg.delta_rho_steps) != 0) ? seg.nominal_velocity : 0.0f;

        // Check junction with next segment (if there is one)
        if (i < count_ - 1) {
            MotionSegment& next = at(i + 1);

            const int curr_theta_dir = sign(seg.delta_theta_steps);
            const int next_theta_dir = sign(next.delta_theta_steps);

            // Theta junction velocity
            if (curr_theta_dir == 0) {
                // Current segment doesn't move theta - next starts from 0
                next.theta_entry_velocity = 0.0f;
            } else if (next_theta_dir == 0) {
                // Next segment doesn't move theta - current can exit at full speed
                next.theta_entry_velocity = 0.0f;
            } else if (curr_theta_dir != next_theta_dir) {
                // Direction reversal - must stop
                seg.theta_exit_velocity = 0.0f;
                next.theta_entry_velocity = 0.0f;
            } else {
                // Same direction - maintain velocity
                next.theta_entry_velocity = seg.theta_exit_velocity;
            }

            const int curr_rho_dir = sign(seg.delta_rho_steps);
            const int next_rho_dir = sign(next.delta_rho_steps);

            // Rho junction velocity
            if (curr_rho_dir == 0) {
                // Current segment doesn't move rho - next starts from 0
                next.rho_entry_velocity = 0.0f;
            } else if (next_rho_dir == 0) {
                // Next segment doesn't move rho - current can exit at full speed
                next.rho_entry_velocity = 0.0f;
            } else if (curr_rho_dir != next_rho_dir) {
                // Direction reversal - must stop
                seg.rho_exit_velocity = 0.0f;
                next.rho_entry_velocity = 0.0f;
            } else {
                // Same direction - maintain velocity
                next.rho_entry_velocity = seg.rho_exit_velocity;
            }
        }
    }
}

} // namespace sand_table
