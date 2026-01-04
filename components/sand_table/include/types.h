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

    [[nodiscard]] T& value() & { return std::get<T>(data_); }
    [[nodiscard]] const T& value() const& { return std::get<T>(data_); }
    [[nodiscard]] T&& value() && { return std::get<T>(std::move(data_)); }

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
    float theta;  // degrees (no limits, can wrap)
    float rho;    // mm from center

    [[nodiscard]] bool operator==(const PolarPosition& other) const noexcept {
        return std::abs(theta - other.theta) < 0.001f &&
               std::abs(rho - other.rho) < 0.001f;
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
            return {0.0f, 0.0f};
        }
        return {x / mag, y / mag};
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
    // Motor steps (WITH coupling compensation applied)
    int32_t delta_theta_steps = 0;
    int32_t delta_rho_steps = 0;

    // Original polar movement (for direction calculations in lookahead)
    float delta_theta_deg = 0.0f;
    float delta_rho_mm = 0.0f;

    // Physical segment length in mm
    float length_mm = 0.0f;

    // Velocity profile (mm/s) - set by lookahead
    float entry_velocity = 0.0f;
    float nominal_velocity = 0.0f;
    float exit_velocity = 0.0f;
    float acceleration = 0.0f;

    // Flags
    bool is_last_segment = false;

    [[nodiscard]] CartesianPosition direction() const noexcept {
        // Convert polar delta to approximate Cartesian direction
        // This is used for junction velocity calculations
        // For small angles, we can approximate the direction
        // The theta component contributes perpendicular to the radius
        // The rho component contributes radially
        return CartesianPosition{
            delta_rho_mm,
            delta_theta_deg * 0.1f  // Scale factor for angle contribution
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
    PolarPosition position{0.0f, 0.0f};
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
    PolarPosition target{0.0f, 0.0f};
    float feedrate_mm_s = 0.0f;
    bool is_relative = false;
    uint32_t command_id = 0;
};

// =============================================================================
// Utility Functions
// =============================================================================

[[nodiscard]] inline float degrees_to_radians(float degrees) noexcept {
    return degrees * (M_PI / 180.0f);
}

[[nodiscard]] inline float radians_to_degrees(float radians) noexcept {
    return radians * (180.0f / M_PI);
}

[[nodiscard]] inline float normalize_angle_degrees(float angle) noexcept {
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

[[nodiscard]] inline float shortest_angular_distance(float from, float to) noexcept {
    float diff = to - from;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

} // namespace sand_table
