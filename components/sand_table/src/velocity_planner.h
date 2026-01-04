#pragma once

#include "types.h"
#include "config.h"

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

    /// Set default acceleration (mm/s^2)
    void set_acceleration(float accel) noexcept {
        acceleration_ = std::max(1.0f, accel);
    }

    /// Set junction deviation (mm) - affects cornering speed
    void set_junction_deviation(float deviation) noexcept {
        junction_deviation_ = std::max(0.001f, deviation);
    }

private:
    std::array<MotionSegment, kLookaheadDepth> segments_{};
    size_t head_ = 0;  // Next position to write
    size_t tail_ = 0;  // Next position to read
    size_t count_ = 0;

    float acceleration_ = static_cast<float>(MotionConfig::DEFAULT_ACCEL_MM_S2);
    float junction_deviation_ = MotionConfig::JUNCTION_DEVIATION_MM;

    /// Calculate junction velocity between two segments based on direction change
    [[nodiscard]] float calculate_junction_velocity(
        const MotionSegment& prev,
        const MotionSegment& next
    ) const;

    /// Run the two-pass lookahead algorithm
    void recalculate();

    /// Backward pass: propagate deceleration limits from end to start
    void reverse_pass();

    /// Forward pass: propagate acceleration limits from start to end
    void forward_pass();

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

    // Set default acceleration if not specified
    if (segment.acceleration <= 0.0f) {
        segment.acceleration = acceleration_;
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

inline float VelocityPlanner::calculate_junction_velocity(
    const MotionSegment& prev,
    const MotionSegment& next) const
{
    // Get direction vectors for both segments
    const auto prev_dir = prev.direction();
    const auto next_dir = next.direction();

    // Calculate dot product (cosine of angle between directions)
    const float cos_angle = prev_dir.dot(next_dir);

    // Handle edge cases
    if (cos_angle <= -0.99f) {
        // Near reversal (180 degrees) - must stop
        return 0.0f;
    }

    if (cos_angle >= 0.99f) {
        // Near straight line - can maintain velocity
        return std::min(prev.nominal_velocity, next.nominal_velocity);
    }

    // Calculate junction velocity based on deviation
    // Using the formula from Grbl/Marlin:
    // v_junction = sqrt(a * deviation / sin(theta/2))
    //
    // Where theta is the angle between segment directions
    // sin(theta/2) = sqrt((1 - cos(theta)) / 2)

    const float sin_half_theta = std::sqrt((1.0f - cos_angle) / 2.0f);
    const float v_junction = std::sqrt(
        acceleration_ * junction_deviation_ / sin_half_theta
    );

    // Clamp to the lower of the two segment velocities
    return std::min({
        v_junction,
        prev.nominal_velocity,
        next.nominal_velocity
    });
}

inline void VelocityPlanner::recalculate() {
    if (count_ == 0) {
        return;
    }

    // Single segment - simple case
    if (count_ == 1) {
        MotionSegment& seg = at(0);
        seg.entry_velocity = 0.0f;
        seg.exit_velocity = seg.is_last_segment ? 0.0f : seg.nominal_velocity;
        return;
    }

    // Calculate junction velocities between segments
    for (size_t i = 0; i < count_ - 1; ++i) {
        MotionSegment& curr = at(i);
        MotionSegment& next = at(i + 1);

        const float junction_vel = calculate_junction_velocity(curr, next);

        // This will be refined by the two-pass algorithm
        curr.exit_velocity = junction_vel;
        next.entry_velocity = junction_vel;
    }

    // First segment starts from zero (or current velocity if resuming)
    at(0).entry_velocity = 0.0f;

    // Last segment ends at zero if marked as last
    if (at(count_ - 1).is_last_segment) {
        at(count_ - 1).exit_velocity = 0.0f;
    }

    // Run two-pass algorithm
    reverse_pass();
    forward_pass();
}

inline void VelocityPlanner::reverse_pass() {
    // Work backward from end to start
    // Ensure each segment can decelerate to its exit velocity

    for (size_t i = count_; i > 0; --i) {
        MotionSegment& seg = at(i - 1);

        // Maximum entry velocity that allows deceleration to exit velocity
        // v_entry = sqrt(v_exit^2 + 2 * a * d)
        const float max_entry = std::sqrt(
            seg.exit_velocity * seg.exit_velocity +
            2.0f * seg.acceleration * seg.length_mm
        );

        // Take minimum of current entry and max achievable
        seg.entry_velocity = std::min(seg.entry_velocity, max_entry);

        // Clamp to nominal
        seg.entry_velocity = std::min(seg.entry_velocity, seg.nominal_velocity);

        // Propagate to previous segment's exit velocity
        if (i > 1) {
            MotionSegment& prev = at(i - 2);
            prev.exit_velocity = std::min(prev.exit_velocity, seg.entry_velocity);
        }
    }
}

inline void VelocityPlanner::forward_pass() {
    // Work forward from start to end
    // Ensure each segment can accelerate from its entry velocity

    for (size_t i = 0; i < count_; ++i) {
        MotionSegment& seg = at(i);

        // Maximum exit velocity achievable from entry velocity
        // v_exit = sqrt(v_entry^2 + 2 * a * d)
        const float max_exit = std::sqrt(
            seg.entry_velocity * seg.entry_velocity +
            2.0f * seg.acceleration * seg.length_mm
        );

        // Take minimum of current exit and max achievable
        seg.exit_velocity = std::min(seg.exit_velocity, max_exit);

        // Clamp to nominal
        seg.exit_velocity = std::min(seg.exit_velocity, seg.nominal_velocity);

        // Propagate to next segment's entry velocity
        if (i < count_ - 1) {
            MotionSegment& next = at(i + 1);
            next.entry_velocity = std::min(next.entry_velocity, seg.exit_velocity);
        }
    }
}

} // namespace sand_table
