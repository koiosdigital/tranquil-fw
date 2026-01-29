#pragma once

// GPTimer-based step generator using fixed-interval + accumulator approach.
// This matches RBotFirmware's proven timer architecture:
// - Fixed 20us ISR interval (50kHz)
// - Accumulator-based step timing for sub-tick precision
// - Relative accumulators for coordinated multi-axis motion

#include "types.h"
#include "config.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_attr.h"

#include <atomic>
#include <cstdint>

namespace sand_table {

/// GPTimer-based step generator using RBotFirmware's accumulator approach.
/// Fixed 20us ISR with accumulator overflow for precise step timing.
class GpTimerStepGenerator {
public:
    // Timing constants matching RBotFirmware
    static constexpr uint32_t TICK_INTERVAL_US = 20;           // 20us = 50kHz ISR
    static constexpr uint32_t TICKS_PER_SEC = 1000000 / TICK_INTERVAL_US;  // 50000
    static constexpr uint64_t TTICKS_VALUE = 1000000000ULL;    // Accumulator overflow threshold
    static constexpr uint32_t MIN_STEP_RATE_PER_SEC = 10;      // Minimum step rate
    static constexpr uint64_t MIN_STEP_RATE_PER_TTICKS =
        (MIN_STEP_RATE_PER_SEC * TTICKS_VALUE) / TICKS_PER_SEC;

    GpTimerStepGenerator() = default;
    ~GpTimerStepGenerator();

    // Non-copyable
    GpTimerStepGenerator(const GpTimerStepGenerator&) = delete;
    GpTimerStepGenerator& operator=(const GpTimerStepGenerator&) = delete;

    /// Initialize GPTimer and GPIO
    [[nodiscard]] Result<void> init(
        gpio_num_t theta_step,
        gpio_num_t theta_dir,
        gpio_num_t rho_step,
        gpio_num_t rho_dir
    );

    /// Deinitialize and release resources
    void deinit();

    /// Execute a motion with coordinated stepping (blocking)
    [[nodiscard]] Result<void> execute(
        int32_t theta_steps,
        int32_t rho_steps,
        float feedrate_rpm,
        float distance,
        std::atomic<int32_t>& theta_pos,
        std::atomic<int32_t>& rho_pos
    );

    /// Emergency stop
    void stop();

    /// Check if currently executing
    [[nodiscard]] bool is_executing() const noexcept {
        return executing_.load(std::memory_order_acquire);
    }

private:
    gptimer_handle_t step_timer_ = nullptr;
    SemaphoreHandle_t completion_sem_ = nullptr;

    // GPIO pins
    gpio_num_t theta_step_pin_ = GPIO_NUM_NC;
    gpio_num_t theta_dir_pin_ = GPIO_NUM_NC;
    gpio_num_t rho_step_pin_ = GPIO_NUM_NC;
    gpio_num_t rho_dir_pin_ = GPIO_NUM_NC;

    // Execution state
    std::atomic<bool> executing_{false};
    std::atomic<bool> stop_requested_{false};

    // Step generation state (matching RBotFirmware architecture)
    struct StepState {
        // Total steps for this move (absolute values, fixed during move)
        uint32_t theta_total = 0;
        uint32_t rho_total = 0;

        // Current step counts (incremented as we step)
        uint32_t theta_count = 0;
        uint32_t rho_count = 0;

        // Direction (+1 or -1)
        int8_t theta_dir = 0;
        int8_t rho_dir = 0;

        // Index of axis with maximum steps (0=theta, 1=rho)
        int max_axis = 0;

        // Step rate in TTICKS units (added to accumulator each tick)
        uint64_t step_rate_per_tticks = 0;

        // Step accumulator - overflow triggers step generation
        uint64_t step_accumulator = 0;

        // Relative accumulator for minor axis (Bresenham-like coordination)
        // This is added each major step, step minor when it overflows max_steps
        uint32_t relative_accumulator = 0;

        // Pending step end flags (for non-blocking pulse generation)
        bool theta_step_pending = false;
        bool rho_step_pending = false;

        void clear() {
            theta_total = 0;
            rho_total = 0;
            theta_count = 0;
            rho_count = 0;
            theta_dir = 0;
            rho_dir = 0;
            max_axis = 0;
            step_rate_per_tticks = 0;
            step_accumulator = 0;
            relative_accumulator = 0;
            theta_step_pending = false;
            rho_step_pending = false;
        }
    } state_;

    // Position tracking (pointers to caller's atomics)
    std::atomic<int32_t>* theta_pos_ = nullptr;
    std::atomic<int32_t>* rho_pos_ = nullptr;

    bool initialized_ = false;

    /// Timer callback - called from ISR context at fixed 20us interval
    static bool IRAM_ATTR timer_callback(
        gptimer_handle_t timer,
        const gptimer_alarm_event_data_t* edata,
        void* user_ctx
    );

    /// ISR handler - fixed interval, accumulator-based stepping
    void IRAM_ATTR isr_step_handler();

    /// Handle step pulse end (called at start of each ISR)
    /// Returns true if any step was ended (early return to ensure min pulse width)
    bool IRAM_ATTR handle_step_end();

    /// Generate coordinated steps when accumulator overflows
    void IRAM_ATTR handle_step_motion();
};

} // namespace sand_table
