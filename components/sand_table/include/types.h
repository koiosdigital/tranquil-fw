#pragma once

#include <cstdint>
#include <cmath>
#include <optional>
#include <variant>

namespace sand_table {

    // =============================================================================
    // Error Handling
    // =============================================================================

    enum class MotionError {
        None,
        NotHomed,
        OutOfBounds,
        QueueFull,
        QueueEmpty,
        InvalidState,
        HardwareFault,
        StallDetected,
        Timeout,
        HomingFailed,
        EmergencyStop,
    };

    template<typename T>
    class Result {
    public:
        static Result ok(T value) { return Result(std::move(value)); }
        static Result err(MotionError error) { return Result(error); }

        [[nodiscard]] bool is_ok() const noexcept { return std::holds_alternative<T>(data_); }
        [[nodiscard]] bool is_err() const noexcept { return std::holds_alternative<MotionError>(data_); }

        [[nodiscard]] T& value()& { return std::get<T>(data_); }
        [[nodiscard]] const T& value() const& { return std::get<T>(data_); }
        [[nodiscard]] T&& value()&& { return std::get<T>(std::move(data_)); }

        [[nodiscard]] MotionError error() const { return std::get<MotionError>(data_); }

        // Monadic operations
        template<typename F>
        auto and_then(F&& f) -> Result<decltype(f(std::declval<T>()))> {
            if (is_ok()) {
                return Result<decltype(f(std::declval<T>()))>::ok(f(value()));
            }
            return Result<decltype(f(std::declval<T>()))>::err(error());
        }

    private:
        explicit Result(T value) : data_(std::move(value)) {}
        explicit Result(MotionError error) : data_(error) {}

        std::variant<T, MotionError> data_;
    };

    // Specialization for void
    template<>
    class Result<void> {
    public:
        static Result ok() { return Result(true); }
        static Result err(MotionError error) { return Result(error); }

        [[nodiscard]] bool is_ok() const noexcept { return success_; }
        [[nodiscard]] bool is_err() const noexcept { return !success_; }
        [[nodiscard]] MotionError error() const { return error_; }

    private:
        explicit Result(bool success) : success_(success), error_(MotionError::None) {}
        explicit Result(MotionError error) : success_(false), error_(error) {}

        bool success_;
        MotionError error_;
    };

    // =============================================================================
    // Position Types
    // =============================================================================

    struct PolarPosition {
        double theta;  // radians (0 to 2π, wraps)
        double rho;    // normalized radius (0.0 = center, 1.0 = maximum)

        [[nodiscard]] bool operator==(const PolarPosition& other) const noexcept {
            return std::abs(theta - other.theta) < 0.0001 &&
                std::abs(rho - other.rho) < 0.0001;
        }

        [[nodiscard]] bool operator!=(const PolarPosition& other) const noexcept {
            return !(*this == other);
        }
    };

    struct CartesianPosition {
        float x;  // mm
        float y;  // mm

        [[nodiscard]] float magnitude() const noexcept {
            return std::sqrt(x * x + y * y);
        }

        [[nodiscard]] CartesianPosition normalized() const noexcept {
            const float mag = magnitude();
            if (mag < 0.0001f) {
                return { 0.0f, 0.0f };
            }
            return { x / mag, y / mag };
        }

        [[nodiscard]] float dot(const CartesianPosition& other) const noexcept {
            return x * other.x + y * other.y;
        }
    };

    struct StepPosition {
        int32_t theta;  // motor steps
        int32_t rho;    // motor steps
    };

    // =============================================================================
    // Motion Segment
    // =============================================================================

    struct MotionSegment {
        // Delta movement in polar coordinates (set by PathPlanner)
        // Uses min-rotation for theta (shortest path, can be negative)
        double delta_theta_rad = 0.0;   // radians (min rotation, [-π, π])
        double delta_rho_norm = 0.0;    // normalized (0-1 scale)

        // Motor steps (WITH coupling compensation applied) - calculated at execution time
        int32_t delta_theta_steps = 0;
        int32_t delta_rho_steps = 0;

        // Normalized distance (for velocity planning)
        float distance = 0.0f;

        // Velocity profile - set by lookahead (per-motor, independent)
        // Each motor has its own entry/exit velocities - only slows when that motor reverses
        float theta_entry_velocity = 0.0f;   // RPM
        float theta_exit_velocity = 0.0f;    // RPM
        float rho_entry_velocity = 0.0f;     // RPM
        float rho_exit_velocity = 0.0f;      // RPM
        float nominal_velocity = 0.0f;       // RPM (shared nominal/cruise target)
        float acceleration = 0.0f;

        // The user feedrate this segment was planned at, WITHOUT transient
        // boosts (transit multipliers). The executor scales step intervals
        // live by current_feedrate / base_feedrate so feedrate changes apply
        // to already-queued and in-flight segments instantly, and boosted
        // segments keep their boost ratio. 0 = exempt from live scaling.
        float base_feedrate = 0.0f;

        [[nodiscard]] CartesianPosition direction() const noexcept {
            // Convert polar delta to approximate Cartesian direction
            // This is used for junction velocity calculations
            return CartesianPosition{
                static_cast<float>(delta_rho_norm),
                static_cast<float>(delta_theta_rad)
            }.normalized();
        }
    };

    // =============================================================================
    // Velocity Profile
    // =============================================================================

    struct VelocityProfile {
        uint32_t accel_steps = 0;
        uint32_t cruise_steps = 0;
        uint32_t decel_steps = 0;
        float entry_velocity = 0.0f;   // steps/s
        float cruise_velocity = 0.0f;  // steps/s
        float exit_velocity = 0.0f;    // steps/s
        float acceleration = 0.0f;     // steps/s^2

        [[nodiscard]] uint32_t total_steps() const noexcept {
            return accel_steps + cruise_steps + decel_steps;
        }
    };

    // =============================================================================
    // System State
    // =============================================================================

    enum class SystemState {
        Idle,
        Homing,
        Running,
        Paused,
        Error,
        EStop,
    };

    // =============================================================================
    // Motor Status
    // =============================================================================

    struct MotorStatus {
        int32_t position_steps = 0;
        bool is_enabled = false;
        bool is_homed = false;
        bool is_stalled = false;
    };

    struct RobotStatus {
        PolarPosition position{ 0.0, 0.0 };
        MotorStatus theta;
        MotorStatus rho;
        SystemState state = SystemState::Idle;
        bool is_homed = false;
        size_t queue_depth = 0;
        size_t queue_capacity = 0;
    };

    // =============================================================================
    // Motion Command
    // =============================================================================

    struct MotionCommand {
        PolarPosition target{ 0.0, 0.0 };
        float feedrate = 0.0f;     // RPM (matches main branch)
        bool is_relative = false;
        uint32_t command_id = 0;
    };

    // =============================================================================
    // Utility Functions
    // =============================================================================

    /// Normalize angle to [0, 2π) range
    /// Uses fmod for efficiency and to avoid iterative precision loss
    [[nodiscard]] inline double normalize_angle(double angle) noexcept {
        constexpr double TWO_PI = 2.0 * M_PI;
        angle = std::fmod(angle, TWO_PI);
        if (angle < 0.0) angle += TWO_PI;
        return angle;
    }

    /// Calculate minimum rotation between two angles (returns value in [-π, π])
    /// Uses fmod for efficiency and to avoid iterative precision loss
    [[nodiscard]] inline double calculate_min_rotation(double target, double current) noexcept {
        constexpr double TWO_PI = 2.0 * M_PI;
        double diff = std::fmod(target - current, TWO_PI);
        if (diff > M_PI) diff -= TWO_PI;
        else if (diff < -M_PI) diff += TWO_PI;
        return diff;
    }

    /// Check if position is within bounds (rho: [0, 1], theta unbounded for multi-rotation)
    [[nodiscard]] inline bool is_in_bounds(const PolarPosition& pos) noexcept {
        // Theta is unbounded to support cumulative rotation tracking
        // (needed for coupling compensation in multi-rotation patterns)
        return pos.rho >= 0.0 && pos.rho <= 1.0;
    }

} // namespace sand_table
