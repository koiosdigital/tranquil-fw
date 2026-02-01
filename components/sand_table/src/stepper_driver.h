#pragma once

#include "types.h"
#include "config.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <new>

namespace sand_table {

    // Forward declaration
    struct BresenhamState;

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

        /// Get direction pin
        [[nodiscard]] gpio_num_t dir_pin() const noexcept { return pins_.dir; }

    private:
        friend class CoordinatedStepperController;
        friend class RmtStepSequencer;

        Pins pins_;
        const char* name_;

        rmt_channel_handle_t rmt_channel_ = nullptr;
        rmt_encoder_handle_t step_encoder_ = nullptr;

        std::atomic<int32_t> position_{ 0 };
        bool direction_positive_ = true;
        bool enabled_ = false;
        bool initialized_ = false;

        Result<void> init_gpio();
        Result<void> init_rmt();
    };

    // =============================================================================
    // Bresenham State (shared between sequencer and controller)
    // =============================================================================

    struct BresenhamState {
        int32_t theta_remaining = 0;  // Decremented during execution
        int32_t rho_remaining = 0;    // Decremented during execution
        int32_t theta_total = 0;      // Original total (fixed during execution)
        int32_t rho_total = 0;        // Original total (fixed during execution)
        int32_t error = 0;
        int8_t theta_dir = 0;
        int8_t rho_dir = 0;

        void clear() {
            theta_remaining = rho_remaining = error = 0;
            theta_total = rho_total = 0;
            theta_dir = rho_dir = 0;
        }
    };

    // =============================================================================
    // RMT Step Sequencer
    // =============================================================================

    /// Generates step pulses using RMT peripheral with DMA
    /// Uses chunked Bresenham algorithm with TX-done callbacks for continuous streaming
    class RmtStepSequencer {
    public:
        static constexpr size_t kChunkSize = 64;  // Steps per chunk

        RmtStepSequencer() = default;
        ~RmtStepSequencer();

        // Non-copyable
        RmtStepSequencer(const RmtStepSequencer&) = delete;
        RmtStepSequencer& operator=(const RmtStepSequencer&) = delete;

        /// Initialize RMT channels for both axes
        [[nodiscard]] Result<void> init(gpio_num_t theta_step, gpio_num_t rho_step);

        /// Deinitialize and release resources
        void deinit();

        /// Execute a motion segment (blocking)
        /// @param bresenham Bresenham state (will be modified during execution)
        /// @param intervals Pre-computed interval table (in microseconds)
        /// @param total_steps Total steps on major axis
        /// @param theta_pos Atomic position counter for theta (updated during execution)
        /// @param rho_pos Atomic position counter for rho (updated during execution)
        [[nodiscard]] Result<void> execute(
            BresenhamState& bresenham,
            const uint16_t* intervals,
            uint32_t total_steps,
            std::atomic<int32_t>& theta_pos,
            std::atomic<int32_t>& rho_pos
        );

        /// Emergency stop - immediately halt transmission
        void stop();

        /// ISR-safe stop - sets flag and signals semaphore without blocking
        void IRAM_ATTR stop_from_isr();

        /// Check if currently executing
        [[nodiscard]] bool is_executing() const noexcept {
            return executing_.load(std::memory_order_acquire);
        }

        /// Get number of steps encoded/executed so far in current segment
        [[nodiscard]] uint32_t steps_executed() const noexcept {
            return steps_encoded_;
        }

        /// Get total steps in current segment
        [[nodiscard]] uint32_t total_steps() const noexcept {
            return total_steps_;
        }

    private:
        // RMT handles
        rmt_channel_handle_t theta_channel_ = nullptr;
        rmt_channel_handle_t rho_channel_ = nullptr;
        rmt_sync_manager_handle_t sync_manager_ = nullptr;
        rmt_encoder_handle_t copy_encoder_ = nullptr;

        // Double-buffered symbol chunks (ping-pong)
        rmt_symbol_word_t theta_symbols_[2][kChunkSize];
        rmt_symbol_word_t rho_symbols_[2][kChunkSize];
        size_t symbol_counts_[2] = { 0, 0 };
        uint8_t active_buffer_ = 0;

        // Execution state
        std::atomic<bool> executing_{ false };
        std::atomic<bool> stop_requested_{ false };
        std::atomic<int32_t> pending_tx_{ 0 };  // Count of pending transmissions
        uint32_t steps_encoded_ = 0;
        uint32_t total_steps_ = 0;
        BresenhamState* bresenham_ = nullptr;
        const uint16_t* intervals_ = nullptr;
        std::atomic<int32_t>* theta_pos_ = nullptr;
        std::atomic<int32_t>* rho_pos_ = nullptr;

        // Completion signaling
        SemaphoreHandle_t completion_sem_ = nullptr;

        // Channel completion tracking - counts how many channels have completed
        // When this reaches 2, both channels are done and we can proceed
        std::atomic<int> channels_done_{0};

        // Callback for TX done
        static bool IRAM_ATTR tx_done_callback(
            rmt_channel_handle_t channel,
            const rmt_tx_done_event_data_t* edata,
            void* user_ctx
        );

        // Encode next chunk of steps into RMT symbols (called from ISR)
        void IRAM_ATTR encode_chunk(uint8_t buffer_idx);

        // Helper to create step and idle symbols (called from ISR)
        static rmt_symbol_word_t IRAM_ATTR make_step_symbol(uint16_t interval_us);
        static rmt_symbol_word_t IRAM_ATTR make_idle_symbol(uint16_t interval_us);

        // Run one Bresenham iteration (called from ISR)
        void IRAM_ATTR bresenham_step(bool& step_theta, bool& step_rho);

        bool initialized_ = false;
    };

    /// Coordinates two stepper motors for synchronized motion using Bresenham
    class CoordinatedStepperController {
    public:
        CoordinatedStepperController(StepperDriver& theta, StepperDriver& rho);
        ~CoordinatedStepperController();

        // Non-copyable
        CoordinatedStepperController(const CoordinatedStepperController&) = delete;
        CoordinatedStepperController& operator=(const CoordinatedStepperController&) = delete;

        /// Initialize the controller (creates RMT sequencer)
        [[nodiscard]] Result<void> init();

        /// Execute a motion segment with velocity profile
        /// This is a blocking call that returns when the segment is complete
        [[nodiscard]] Result<void> execute_segment(const MotionSegment& segment);

        /// Execute constant-speed motion (for homing)
        /// Uses fixed step interval without acceleration/deceleration
        /// @param theta_steps Steps for theta axis (positive or negative)
        /// @param rho_steps Steps for rho axis (positive or negative)
        /// @param interval_us Microseconds between steps (constant throughout)
        [[nodiscard]] Result<void> execute_constant_speed(
            int32_t theta_steps,
            int32_t rho_steps,
            uint32_t interval_us
        );

        /// Enable both motors
        void enable();

        /// Disable both motors
        void disable();

        /// Emergency stop - immediately halt all motion
        void emergency_stop();

        /// ISR-safe emergency stop - sets flag without blocking
        void IRAM_ATTR emergency_stop_from_isr();

        /// Check if motion is in progress
        [[nodiscard]] bool is_moving() const noexcept {
            return rmt_sequencer_ && rmt_sequencer_->is_executing();
        }

        /// Get current positions
        [[nodiscard]] StepPosition get_position() const noexcept {
            return { theta_.position(), rho_.position() };
        }

        /// Progress information for current segment execution
        struct SegmentProgress {
            uint32_t steps_done;
            uint32_t steps_total;
            bool is_executing;
        };

        /// Get progress of current segment being executed
        [[nodiscard]] SegmentProgress get_segment_progress() const noexcept {
            if (!rmt_sequencer_) {
                return {0, 0, false};
            }
            return {
                rmt_sequencer_->steps_executed(),
                rmt_sequencer_->total_steps(),
                rmt_sequencer_->is_executing()
            };
        }

    private:
        StepperDriver& theta_;
        StepperDriver& rho_;

        // Custom deleter for internal RAM allocation
        struct InternalRamDeleter {
            void operator()(RmtStepSequencer* ptr) const {
                if (ptr) {
                    ptr->~RmtStepSequencer();
                    heap_caps_free(ptr);
                }
            }
        };

        // RMT sequencer must be in internal RAM for ISR callback safety
        std::unique_ptr<RmtStepSequencer, InternalRamDeleter> rmt_sequencer_;
        BresenhamState bresenham_;

        // Pre-computed step intervals for velocity profiles
        // Must be large enough for full rotation (25600 steps) plus margin
        static constexpr size_t kMaxIntervalsPerSegment = 32768;

        struct IntervalTable {
            uint16_t intervals[kMaxIntervalsPerSegment];  // Microseconds per step
            uint32_t total_steps = 0;

            void clear() { total_steps = 0; }
        } interval_table_;

        void prepare_interval_table(const VelocityProfile& profile, uint32_t total_steps);

        VelocityProfile calculate_velocity_profile(
            const MotionSegment& segment,
            float steps_per_unit
        ) const;
    };

} // namespace sand_table
