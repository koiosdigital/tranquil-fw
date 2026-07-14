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

#include "ring_buffer.h"

// Forward declarations for private implementation classes
namespace sand_table {
    class StepperDriver;
    class CoordinatedStepperController;
    class HomingController;
    class CoordinateTransformer;
    class PathPlanner;
    class VelocityPlanner;
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

        /// Execute homing sequence
        /// @param force_full If true, performs full calibration. Otherwise uses cached calibration if available.
        [[nodiscard]] Result<void> home(bool force_full = false);

        /// Check if system is homed
        [[nodiscard]] bool is_homed() const noexcept {
            return is_homed_.load(std::memory_order_acquire);
        }

        /// Skip homing and set calibration values directly (for development)
        /// @param theta_steps_per_rot Steps for one full theta rotation
        /// @param rho_max Steps for full rho travel
        void _set_homed(int32_t theta_steps_per_rot, int32_t rho_max);

        // =========================================================================
        // Motion Commands
        // =========================================================================

        /// Move to a polar position using direct polar motion (arcs in XY space)
        /// @param target Target position (theta in radians, unbounded for
        ///        continuous rotation; rho normalized 0-1)
        /// @param feedrate Constant path speed in normalized table units per
        ///        minute (1.0 = table radius). NOT motor RPM — angular speed
        ///        varies with radius so the ball's surface speed is uniform.
        /// @param base_feedrate The user feedrate WITHOUT transient boosts
        ///        (0 = same as feedrate). Live feedrate changes scale
        ///        execution by set_live_feedrate() / base_feedrate, so a
        ///        boosted move keeps its boost ratio.
        [[nodiscard]] Result<void> move_to(const PolarPosition& target, float feedrate = 0,
                                           float base_feedrate = 0);

        /// Move to a polar position using Cartesian interpolation (straight lines in XY space)
        /// @param target Target position (theta in radians 0-2π, rho normalized 0-1)
        /// @param feedrate Constant path speed, same units as move_to()
        /// @param base_feedrate See move_to()
        [[nodiscard]] Result<void> move_linear(const PolarPosition& target, float feedrate = 0,
                                               float base_feedrate = 0);

        /// Live feedrate override: applies INSTANTLY to queued and in-flight
        /// motion. Each executing chunk's step intervals are scaled by
        /// (live_feedrate / segment.base_feedrate), slew-limited per chunk.
        /// 0 disables the override (segments run at their planned speed).
        void set_live_feedrate(float feedrate_rpm) {
            live_feedrate_.store(feedrate_rpm, std::memory_order_relaxed);
        }

        // =========================================================================
        // Control
        // =========================================================================

        /// Pause motion (can be resumed)
        void pause();

        /// Resume paused motion
        void resume();

        /// Normal stop: abort the in-flight segment, discard everything
        /// queued (segment queue + velocity planner), and resync the
        /// planner position from the physical motor position. Blocks until
        /// the stepper task acknowledges the drain (bounded wait).
        /// Use this for user-initiated stops; emergency_stop() is for
        /// safety stops and leaves the controller in the EStop state.
        void halt_and_drain();

        /// Emergency stop - immediately halt all motion
        void emergency_stop();

        /// Clear emergency stop state
        void clear_emergency_stop();

        // =========================================================================
        // Status
        // =========================================================================

        /// Get current polar position (derived from motor counters; theta
        /// WRAPPED to [0, 2π) — display/API only)
        [[nodiscard]] PolarPosition get_position() const;

        /// Position in the path planner's CONTINUOUS frame (theta unwrapped;
        /// after a long sweep it can be many full turns). This is the frame
        /// move_to()/move_linear() plan their deltas from — use it, not
        /// get_position(), when rebasing absolute targets (e.g. a pattern's
        /// wound-up theta) onto the current position: an offset computed
        /// against the wrapped theta differs by whole rotations, and the
        /// first move physically unwinds all of them.
        [[nodiscard]] PolarPosition get_planning_position() const;

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
        [[nodiscard]] size_t queue_depth() const noexcept {
            return segment_queue_.size();
        }

        /// Progress information for motion execution
        struct MotionProgress {
            uint64_t steps_queued;       // Total steps queued since reset
            uint64_t steps_completed;    // Total steps completed since reset
            uint32_t current_segment_steps_total;  // Steps in current segment
            uint32_t current_segment_steps_done;   // Steps done in current segment
            size_t segments_queued;      // Segments in execution queue
            size_t planner_pending;      // Segments still in the velocity planner
            bool is_executing;           // Whether a segment is currently executing

            /// Get progress as a fraction (0.0 to 1.0)
            [[nodiscard]] float progress_fraction() const noexcept {
                if (steps_queued == 0) return 1.0f;
                return static_cast<float>(steps_completed) / static_cast<float>(steps_queued);
            }
        };

        /// Get current motion execution progress
        [[nodiscard]] MotionProgress get_motion_progress() const;

        /// Reset progress counters (call when starting a new pattern)
        void reset_progress_counters();

    private:
        // Segment queue (64 segments, power of 2)
        static constexpr size_t kSegmentQueueSize = 64;
        mutable RingBuffer<MotionSegment, kSegmentQueueSize> segment_queue_;

        // Path planner and velocity planner (lookahead)
        std::unique_ptr<PathPlanner> path_planner_;
        std::unique_ptr<VelocityPlanner> velocity_planner_;

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

        // Coordinate transformer (handles calibrated conversions)
        std::unique_ptr<CoordinateTransformer> transformer_;

        // State
        std::atomic<SystemState> state_{ SystemState::Idle };
        std::atomic<bool> is_homed_{ false };
        std::atomic<bool> emergency_stop_{ false };
        std::atomic<bool> paused_{ false };
        std::atomic<bool> running_{ false };

        // halt_and_drain() handshake: the stepper task owns the segment
        // queue's consumer side, so it performs the actual drain and clears
        // this flag as the acknowledgement.
        std::atomic<bool> drain_requested_{ false };
        // While set, enqueue_segment() refuses new segments (returns false)
        // so a task blocked mid-plan unwinds instead of refilling the
        // planner that is being drained.
        std::atomic<bool> abort_planning_{ false };

        // Guards velocity_planner_: it is mutated from the planning task
        // (enqueue_segment via move_to) AND the stepper task
        // (transfer_ready_segments / drain / e-stop clear) — its internals
        // are not atomic. mutable: locked in const progress getters.
        mutable SemaphoreHandle_t planner_mutex_ = nullptr;

        // Progress tracking
        std::atomic<uint64_t> total_steps_queued_{ 0 };
        std::atomic<uint64_t> total_steps_completed_{ 0 };

        // Live feedrate override (units/min, 0 = off). Read by the stepper
        // sequencer's chunk encoder via pointer (see init()).
        std::atomic<float> live_feedrate_{ 0.0f };

        // Position tracking: uses actual motor positions directly
        // No need for separate "target" tracking - motor positions are the source of truth
        // (Polar position is derived from motor steps only when needed for display/API)

        // FreeRTOS tasks
        TaskHandle_t stepper_task_ = nullptr;

        // Motor inactivity timer
        TimerHandle_t inactivity_timer_ = nullptr;
        static constexpr TickType_t kInactivityTimeout =
            pdMS_TO_TICKS(MotionConfig::MOTOR_INACTIVITY_TIMEOUT_MS);

        // Guards enable/disable ordering between motion-starting tasks and
        // the inactivity callback (timer service task): xTimerStop() does
        // not cancel an already-dispatched callback, so without this an
        // expiry racing enable_motors() powers the motors off UNDER a
        // starting move — which then runs (and desyncs position) silently.
        // The callback only disables if no enable happened since the timer
        // was armed (enable_epoch_ still equals armed_epoch_).
        SemaphoreHandle_t motor_power_mutex_ = nullptr;
        std::atomic<uint32_t> enable_epoch_{ 0 };
        std::atomic<uint32_t> armed_epoch_{ 0 };

        // Task functions
        static void stepper_task_entry(void* arg);
        void stepper_task_loop();

        // Timer callback
        static void inactivity_timer_callback(TimerHandle_t timer);

        // Internal helpers
        void reset_inactivity_timer();
        Result<void> init_tmc();

        // Position overflow handling (like main branch's handleStepOverflow)
        // Wraps theta position when it exceeds ±1 rotation and adjusts rho accordingly
        void handle_position_overflow();

        // Segment handling
        bool enqueue_segment(MotionSegment& segment);
        void transfer_ready_segments();
        Result<void> execute_segment(const MotionSegment& segment);
    };

} // namespace sand_table
