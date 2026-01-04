#pragma once

#include "types.h"
#include "config.h"

#include "tmc2209.h"
#include "uartbus.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <atomic>
#include <memory>

// Forward declarations for private implementation classes
namespace sand_table {
class StepperDriver;
class CoordinatedStepperController;
class HomingController;
class CoordinateTransformer;
class VelocityPlanner;
class PathPlanner;
}

namespace sand_table {

/// Top-level motion controller that coordinates all subsystems.
/// Manages FreeRTOS tasks for planning and step execution.
class MotionController {
public:
    MotionController();
    ~MotionController();

    // Non-copyable
    MotionController(const MotionController&) = delete;
    MotionController& operator=(const MotionController&) = delete;

    /// Initialize all subsystems (GPIO, TMC, timers)
    [[nodiscard]] Result<void> init();

    /// Start FreeRTOS tasks for motion execution
    [[nodiscard]] Result<void> start();

    /// Stop tasks and disable motors
    void stop();

    // =========================================================================
    // Motor Control
    // =========================================================================

    /// Enable both motors
    void enable_motors();

    /// Disable both motors
    void disable_motors();

    // =========================================================================
    // Homing
    // =========================================================================

    /// Execute full homing sequence
    [[nodiscard]] Result<void> home();

    /// Check if system is homed
    [[nodiscard]] bool is_homed() const noexcept {
        return is_homed_.load(std::memory_order_acquire);
    }

    // =========================================================================
    // Motion Commands
    // =========================================================================

    /// Move to a polar position
    [[nodiscard]] Result<void> move_to(const PolarPosition& target, float feedrate);

    /// Draw a spiral pattern
    [[nodiscard]] Result<void> draw_spiral(
        float start_rho,
        float end_rho,
        float rotations,
        float feedrate
    );

    // =========================================================================
    // Control
    // =========================================================================

    /// Pause motion (can be resumed)
    void pause();

    /// Resume paused motion
    void resume();

    /// Emergency stop - immediately halt all motion
    void emergency_stop();

    /// Clear emergency stop state
    void clear_emergency_stop();

    // =========================================================================
    // Status
    // =========================================================================

    /// Get current polar position
    [[nodiscard]] PolarPosition get_position() const;

    /// Get current system state
    [[nodiscard]] SystemState get_state() const noexcept {
        return state_.load(std::memory_order_acquire);
    }

    /// Check if system is idle (ready for new commands)
    [[nodiscard]] bool is_idle() const noexcept {
        return get_state() == SystemState::Idle;
    }

    /// Check if motion is in progress
    [[nodiscard]] bool is_moving() const noexcept {
        return get_state() == SystemState::Running;
    }

    /// Get full robot status
    [[nodiscard]] RobotStatus get_status() const;

    /// Get queue depth (segments waiting for execution)
    [[nodiscard]] size_t queue_depth() const noexcept;

private:
    // TMC UART bus (shared by both drivers)
    std::unique_ptr<tmc::UartBus> tmc_bus_;

    // Stepper drivers
    std::unique_ptr<StepperDriver> theta_stepper_;
    std::unique_ptr<StepperDriver> rho_stepper_;

    // TMC2209 drivers
    std::unique_ptr<tmc::TMC2209Stepper> theta_tmc_;
    std::unique_ptr<tmc::TMC2209Stepper> rho_tmc_;

    // Coordinated motion controller
    std::unique_ptr<CoordinatedStepperController> stepper_controller_;

    // Homing controller
    std::unique_ptr<HomingController> homing_controller_;

    // Planning components (pimpl pattern for private headers)
    std::unique_ptr<CoordinateTransformer> transformer_;
    std::unique_ptr<VelocityPlanner> velocity_planner_;
    std::unique_ptr<PathPlanner> path_planner_;

    // State
    std::atomic<SystemState> state_{SystemState::Idle};
    std::atomic<bool> is_homed_{false};
    std::atomic<bool> emergency_stop_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> running_{false};

    // Current position tracking
    mutable SemaphoreHandle_t position_mutex_;
    PolarPosition current_position_{0.0f, static_cast<float>(MechanicalConfig::RHO_MIN_MM)};

    // FreeRTOS tasks
    TaskHandle_t stepper_task_ = nullptr;

    // Motor inactivity timer
    TimerHandle_t inactivity_timer_ = nullptr;
    static constexpr TickType_t kInactivityTimeout =
        pdMS_TO_TICKS(MotionConfig::MOTOR_INACTIVITY_TIMEOUT_MS);

    // Task functions
    static void stepper_task_entry(void* arg);
    void stepper_task_loop();

    // Timer callback
    static void inactivity_timer_callback(TimerHandle_t timer);

    // Internal helpers
    void update_position(const StepPosition& steps);
    void reset_inactivity_timer();
    Result<void> init_tmc();
};

} // namespace sand_table
