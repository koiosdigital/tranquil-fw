#pragma once

#include "types.h"
#include "config.h"
#include "gptimer_step_generator.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <atomic>
#include <cstdint>
#include <memory>

namespace sand_table {

// Forward declaration
struct BresenhamState;

// =============================================================================
// Step Generator Mode Selection (for A/B testing)
// =============================================================================

/// Step generator mode for A/B testing RMT vs GPTimer implementations.
/// TO SWITCH BACK TO RMT-ONLY: Use StepGeneratorMode::RMT (the default)
enum class StepGeneratorMode {
    RMT,      // Current RMT-based implementation (default)
    GPTimer   // GPTimer fallback matching main branch behavior
};

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

    std::atomic<int32_t> position_{0};
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

    /// Check if currently executing
    [[nodiscard]] bool is_executing() const noexcept {
        return executing_.load(std::memory_order_acquire);
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
    size_t symbol_counts_[2] = {0, 0};
    uint8_t active_buffer_ = 0;

    // Execution state
    std::atomic<bool> executing_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<int32_t> pending_tx_{0};  // Count of pending transmissions
    uint32_t steps_encoded_ = 0;
    uint32_t total_steps_ = 0;
    BresenhamState* bresenham_ = nullptr;
    const uint16_t* intervals_ = nullptr;
    std::atomic<int32_t>* theta_pos_ = nullptr;
    std::atomic<int32_t>* rho_pos_ = nullptr;

    // Completion signaling
    SemaphoreHandle_t completion_sem_ = nullptr;

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

    /// Initialize the controller (creates RMT sequencer and GPTimer generator)
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
        if (step_mode_ == StepGeneratorMode::GPTimer) {
            return gptimer_generator_.is_executing();
        }
        return rmt_sequencer_.is_executing();
    }

    /// Get current positions
    [[nodiscard]] StepPosition get_position() const noexcept {
        return {theta_.position(), rho_.position()};
    }

    /// Set step generator mode for A/B testing
    /// TO SWITCH BACK TO RMT-ONLY: Use StepGeneratorMode::RMT
    void set_step_generator_mode(StepGeneratorMode mode) noexcept {
        step_mode_ = mode;
    }

    /// Get current step generator mode
    [[nodiscard]] StepGeneratorMode get_step_generator_mode() const noexcept {
        return step_mode_;
    }

private:
    StepperDriver& theta_;
    StepperDriver& rho_;

    // Step generator mode (RMT vs GPTimer for A/B testing)
    // TO SWITCH BACK TO RMT-ONLY: Change default to StepGeneratorMode::RMT
    StepGeneratorMode step_mode_ = StepGeneratorMode::GPTimer;  // Default to GPTimer for testing

    RmtStepSequencer rmt_sequencer_;
    GpTimerStepGenerator gptimer_generator_;
    BresenhamState bresenham_;

    // Pre-computed step intervals for velocity profiles
    // RMT can handle arbitrary segment lengths via chunking
    static constexpr size_t kMaxIntervalsPerSegment = 4096;

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
