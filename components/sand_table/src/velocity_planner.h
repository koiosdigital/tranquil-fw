#pragma once

#include "types.h"
#include "config.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace sand_table {

/// Velocity planner with lookahead for smooth motion across segment
/// junctions.
///
/// Junction model: the executor runs both motors off ONE time base (a shared
/// major-axis interval table; Bresenham interleaves the minor axis), so a
/// segment has a single scalar speed — the path feedrate. What must stay
/// continuous across a junction is each MOTOR's step rate. At path speed v
/// (units/min) an axis steps at v * s, where s = axis_steps / distance
/// (signed steps per normalized path unit) is fixed for the whole segment.
/// The junction is therefore executed as an instantaneous per-axis rate
/// change of v * (s_next - s_prev), and the junction speed limit is the
/// largest v for which no axis jumps by more than
/// MotionConfig::JUNCTION_MAX_AXIS_DV_STEPS_S:
///
///     v_j <= dv_max / |s_next - s_prev|      (per axis, take the min)
///
/// This subsumes the old binary rule (same direction = full speed, reversal
/// or moving<->stopped transition = dead stop) that stamped visible
/// stop/start marks into the sand: a rho reversal at the crest of a ripple
/// has tiny |s| on both sides and sails through at full path speed, while a
/// genuine sharp corner (large per-axis rate change, e.g. the theta flip
/// crossing center) is slowed exactly as much as needed and no further.
///
/// Around the junction limits, a backward then a forward pass bound every
/// junction by what STEP_ACCEL_STEPS_S2 can physically reach within each
/// segment. The buffer tail always plans down to ZERO, so everything handed
/// to the executor can stop cleanly no matter what arrives later;
/// MotionController gates pops on PLANNER_COMMIT_LOOKAHEAD so that
/// pessimistic tail is normally re-raised by newly arriving segments before
/// it is ever committed.
class VelocityPlanner {
public:
    static constexpr size_t kLookaheadDepth = MotionConfig::LOOKAHEAD_DEPTH;

    VelocityPlanner() = default;

    /// Add a segment to the lookahead buffer, triggers replanning.
    /// The segment must already carry its motor step deltas (the caller
    /// runs the coordinate transformer first) — junction limits are
    /// computed from actual motor steps, coupling compensation included.
    /// @return true if segment was added, false if buffer is full
    [[nodiscard]] bool add_segment(const MotionSegment& segment);

    /// Get next segment for execution (removes from buffer). Its
    /// entry/exit velocities are FROZEN from this point on — the caller
    /// must respect the commit-lookahead gate (see class comment).
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
    /// Per-segment kinematic quantities used by the planning passes.
    struct SegKinematics {
        float s_theta = 0.0f;      // signed theta steps per normalized unit
        float s_rho = 0.0f;        // signed rho steps per normalized unit
        float major_steps = 0.0f;  // Bresenham major-axis step count
        float spu = 0.0f;          // major steps per normalized unit
        float nominal = 0.0f;      // capped nominal path speed (units/min)
    };

    std::array<MotionSegment, kLookaheadDepth> segments_{};
    size_t head_ = 0;  // Next position to write
    size_t tail_ = 0;  // Next position to read
    size_t count_ = 0;

    // Scratch for recalculate() (member, not stack: replanning runs on
    // whatever task called move_to, and the planner is always used under
    // MotionController's planner mutex).
    std::array<SegKinematics, kLookaheadDepth> kin_{};

    // The last POPPED (committed) segment still constrains the buffer
    // head: its exit speed is frozen, and the head's entry must satisfy
    // the junction limit against its axis ratios.
    bool prev_valid_ = false;
    float prev_exit_upm_ = 0.0f;
    SegKinematics prev_kin_{};

    /// Recalculate junction velocities across the buffer (see class doc).
    void recalculate();

    [[nodiscard]] static SegKinematics kinematics_of(const MotionSegment& seg);

    /// Max path speed through the junction between segments a -> b such
    /// that no motor's step rate changes by more than the configured
    /// per-axis jump limit. Also bounded by both segments' nominals.
    [[nodiscard]] static float junction_limit(const SegKinematics& a,
                                              const SegKinematics& b);

    /// Speed reachable from v_upm across the full segment at
    /// STEP_ACCEL_STEPS_S2 (works for either direction of the pass:
    /// accelerating forward or decelerating viewed backward).
    [[nodiscard]] static float reachable(float v_upm, const SegKinematics& k);

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
    prev_valid_ = false;
    prev_exit_upm_ = 0.0f;
}

inline MotionSegment& VelocityPlanner::at(size_t index) {
    return segments_[(tail_ + index) % kLookaheadDepth];
}

inline const MotionSegment& VelocityPlanner::at(size_t index) const {
    return segments_[(tail_ + index) % kLookaheadDepth];
}

inline VelocityPlanner::SegKinematics
VelocityPlanner::kinematics_of(const MotionSegment& seg) {
    SegKinematics k;
    // emit_split floors distance at 0.001; guard anyway so a degenerate
    // segment can't divide by zero.
    const float dist = (seg.distance > 1e-4f) ? seg.distance : 1e-4f;
    k.s_theta = static_cast<float>(seg.delta_theta_steps) / dist;
    k.s_rho = static_cast<float>(seg.delta_rho_steps) / dist;
    k.major_steps = static_cast<float>(std::max(
        std::abs(seg.delta_theta_steps), std::abs(seg.delta_rho_steps)));
    k.spu = std::max(std::fabs(k.s_theta), std::fabs(k.s_rho));
    k.nominal = seg.nominal_velocity;
    return k;
}

inline float VelocityPlanner::junction_limit(const SegKinematics& a,
                                             const SegKinematics& b) {
    float vj = std::min(a.nominal, b.nominal);
    // dv limit is steps/s; speeds here are units/MIN, so convert once.
    const float dv_per_min = static_cast<float>(MotionConfig::junction_dv_steps_s()) * 60.0f;
    const float d_theta = std::fabs(b.s_theta - a.s_theta);
    if (d_theta > 1e-3f) {
        vj = std::min(vj, dv_per_min / d_theta);
    }
    const float d_rho = std::fabs(b.s_rho - a.s_rho);
    if (d_rho > 1e-3f) {
        vj = std::min(vj, dv_per_min / d_rho);
    }
    return vj;
}

inline float VelocityPlanner::reachable(float v_upm, const SegKinematics& k) {
    if (k.spu <= 1e-6f) {
        // Zero-step segment: no distance to accelerate over, pass through.
        return v_upm;
    }
    // v^2 = v0^2 + 2*a*d in major-axis step space, converted back to path
    // units via this segment's steps-per-unit.
    const float v_steps = v_upm * k.spu / 60.0f;
    const float v2 = v_steps * v_steps +
        2.0f * static_cast<float>(MotionConfig::accel_steps_s2()) * k.major_steps;
    return std::sqrt(v2) * 60.0f / k.spu;
}

inline bool VelocityPlanner::add_segment(const MotionSegment& segment) {
    if (full()) {
        return false;
    }

    segments_[head_] = segment;

    // Theta spin-rate safety cap, same formula as the executor's clamp in
    // calculate_velocity_profile. Capping nominal HERE keeps junction
    // planning consistent with what the executor will actually run —
    // otherwise a capped segment's neighbors get junction speeds the
    // executor then clamps into a discontinuity.
    MotionSegment& stored = segments_[head_];
    if (stored.delta_theta_steps != 0 && stored.distance > 0.0f) {
        const float rotations =
            std::fabs(static_cast<float>(stored.delta_theta_rad)) /
            (2.0f * static_cast<float>(M_PI));
        if (rotations > 1e-6f) {
            const float v_cap = MotionConfig::theta_max_rot_per_min() *
                stored.distance / rotations;
            stored.nominal_velocity = std::min(stored.nominal_velocity, v_cap);
        }
    }

    head_ = (head_ + 1) % kLookaheadDepth;
    count_++;

    recalculate();
    return true;
}

inline std::optional<MotionSegment> VelocityPlanner::pop_segment() {
    if (empty()) {
        return std::nullopt;
    }

    MotionSegment segment = segments_[tail_];

    // This segment is now committed: remember its exit speed and axis
    // ratios so the next head's entry stays continuous with it.
    prev_kin_ = kinematics_of(segment);
    prev_exit_upm_ = segment.exit_velocity;
    prev_valid_ = true;

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
    const size_t n = count_;
    if (n == 0) {
        return;
    }

    for (size_t i = 0; i < n; ++i) {
        kin_[i] = kinematics_of(at(i));
    }

    // Seed each exit with its junction limit; the tail always plans to a
    // full stop so committed segments can stop cleanly if nothing else
    // arrives (newly added segments re-raise it on the next replan).
    for (size_t i = 0; i + 1 < n; ++i) {
        at(i).exit_velocity = junction_limit(kin_[i], kin_[i + 1]);
    }
    at(n - 1).exit_velocity = 0.0f;

    // Backward pass: an exit must be low enough that the next segment can
    // still decelerate to its own exit within its length.
    for (size_t i = n - 1; i > 0; --i) {
        const float entry_cap = std::min(kin_[i].nominal,
            reachable(at(i).exit_velocity, kin_[i]));
        at(i - 1).exit_velocity = std::min(at(i - 1).exit_velocity, entry_cap);
    }

    // Head entry: continuity with the last committed (popped) segment,
    // whose exit is frozen. From rest (startup, post-clear) enter at 0.
    float v = 0.0f;
    if (prev_valid_) {
        v = std::min(prev_exit_upm_, junction_limit(prev_kin_, kin_[0]));
    }
    v = std::min(v, kin_[0].nominal);

    // Forward pass: acceleration feasibility, chaining entry[i+1] = exit[i].
    for (size_t i = 0; i < n; ++i) {
        at(i).entry_velocity = v;
        v = std::min(at(i).exit_velocity, reachable(v, kin_[i]));
        at(i).exit_velocity = v;
    }
}

} // namespace sand_table
