#pragma once

#include "types.h"
#include "config.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/gptimer.h"

#include <atomic>
#include <cstdint>
#include <memory>

namespace sand_table {

/// Low-level stepper motor driver using RMT for precise step generation
class StepperDriver {
public:
    struct Pins {
        gpio_num_t step;
        gpio_num_t dir;
        gpio_num_t enable;
    };

    StepperDriver(const Pins& pins, const char* name);
    ~StepperDriver();

    // Non-copyable, non-moveable
    StepperDriver(const StepperDriver&) = delete;
    StepperDriver& operator=(const StepperDriver&) = delete;

    /// Initialize GPIO and RMT
    [[nodiscard]] Result<void> init();

    /// Enable/disable motor driver
    void set_enabled(bool enabled);
    [[nodiscard]] bool is_enabled() const noexcept { return enabled_; }

    /// Set step direction (true = positive/CW, false = negative/CCW)
    void set_direction(bool positive);
    [[nodiscard]] bool get_direction() const noexcept { return direction_positive_; }

    /// Generate a single blocking step
    void step_once();

    /// Generate multiple steps at a fixed rate (blocking)
    /// @param steps Number of steps to generate
    /// @param interval_us Microseconds between steps
    void step_fixed_rate(uint32_t steps, uint32_t interval_us);

    /// Get current position in motor steps
    [[nodiscard]] int32_t position() const noexcept {
        return position_.load(std::memory_order_acquire);
    }

    /// Set current position (used during homing)
    void set_position(int32_t pos) noexcept {
        position_.store(pos, std::memory_order_release);
    }

    /// Reset position to zero
    void reset_position() noexcept { set_position(0); }

    /// Get driver name for logging
    [[nodiscard]] const char* name() const noexcept { return name_; }

    /// Get step pin (for direct access in ISR)
    [[nodiscard]] gpio_num_t step_pin() const noexcept { return pins_.step; }

private:
    friend class CoordinatedStepperController;

    Pins pins_;
    const char* name_;

    rmt_channel_handle_t rmt_channel_ = nullptr;
    rmt_encoder_handle_t step_encoder_ = nullptr;

    std::atomic<int32_t> position_{0};
    bool direction_positive_ = true;
    bool enabled_ = false;
    bool initialized_ = false;

    Result<void> init_gpio();
    Result<void> init_rmt();
};

/// Coordinates two stepper motors for synchronized motion using Bresenham
class CoordinatedStepperController {
public:
    CoordinatedStepperController(StepperDriver& theta, StepperDriver& rho);
    ~CoordinatedStepperController();

    // Non-copyable
    CoordinatedStepperController(const CoordinatedStepperController&) = delete;
    CoordinatedStepperController& operator=(const CoordinatedStepperController&) = delete;

    /// Initialize the controller (creates timer)
    [[nodiscard]] Result<void> init();

    /// Execute a motion segment with velocity profile
    /// This is a blocking call that returns when the segment is complete
    [[nodiscard]] Result<void> execute_segment(const MotionSegment& segment);

    /// Enable both motors
    void enable();

    /// Disable both motors
    void disable();

    /// Emergency stop - immediately halt all motion
    void emergency_stop();

    /// Check if motion is in progress
    [[nodiscard]] bool is_moving() const noexcept {
        return moving_.load(std::memory_order_acquire);
    }

    /// Get current positions
    [[nodiscard]] StepPosition get_position() const noexcept {
        return {theta_.position(), rho_.position()};
    }

private:
    StepperDriver& theta_;
    StepperDriver& rho_;

    gptimer_handle_t step_timer_ = nullptr;
    std::atomic<bool> moving_{false};
    std::atomic<bool> abort_requested_{false};

    // Bresenham state (used during motion execution)
    struct BresenhamState {
        int32_t theta_remaining = 0;
        int32_t rho_remaining = 0;
        int32_t error = 0;
        int8_t theta_dir = 0;
        int8_t rho_dir = 0;

        void clear() {
            theta_remaining = rho_remaining = error = 0;
            theta_dir = rho_dir = 0;
        }
    } bresenham_;

    // Velocity profile state
    struct VelocityState {
        uint32_t steps_taken = 0;
        uint32_t accel_steps = 0;
        uint32_t cruise_steps = 0;
        uint32_t decel_steps = 0;
        float current_velocity = 0.0f;
        float entry_velocity = 0.0f;
        float cruise_velocity = 0.0f;
        float exit_velocity = 0.0f;
        float acceleration = 0.0f;
    } velocity_;

    static bool step_timer_callback(
        gptimer_handle_t timer,
        const gptimer_alarm_event_data_t* edata,
        void* user_ctx
    );

    void generate_step();
    uint32_t calculate_next_interval();

    VelocityProfile calculate_velocity_profile(
        const MotionSegment& segment,
        float steps_per_mm
    ) const;
};

} // namespace sand_table
