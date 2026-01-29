#pragma once

// GPTimer-based step generator for A/B testing against RMT implementation.
// This implementation closely matches the main branch's PolarRobot timer approach.
//
// TO SWITCH BACK TO RMT-ONLY:
// - In CoordinatedStepperController, set step_mode_ = StepGeneratorMode::RMT
// - Or simply don't call set_step_generator_mode()

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

/// GPTimer-based step generator matching main branch behavior.
/// Uses 1MHz timer with single Bresenham calculation per move.
/// Designed for A/B testing against RMT implementation.
class GpTimerStepGenerator {
public:
    GpTimerStepGenerator() = default;
    ~GpTimerStepGenerator();

    // Non-copyable
    GpTimerStepGenerator(const GpTimerStepGenerator&) = delete;
    GpTimerStepGenerator& operator=(const GpTimerStepGenerator&) = delete;

    /// Initialize GPTimer and GPIO
    /// @param theta_step Step pin for theta motor
    /// @param theta_dir Direction pin for theta motor
    /// @param rho_step Step pin for rho motor
    /// @param rho_dir Direction pin for rho motor
    [[nodiscard]] Result<void> init(
        gpio_num_t theta_step,
        gpio_num_t theta_dir,
        gpio_num_t rho_step,
        gpio_num_t rho_dir
    );

    /// Deinitialize and release resources
    void deinit();

    /// Execute a motion with coordinated stepping (blocking)
    /// Uses Bresenham line algorithm matching main branch exactly.
    ///
    /// @param theta_steps Number of theta steps (signed for direction)
    /// @param rho_steps Number of rho steps (signed for direction)
    /// @param feedrate_rpm Speed in RPM
    /// @param distance Normalized distance for timing calculation
    /// @param theta_pos Atomic position counter to update
    /// @param rho_pos Atomic position counter to update
    [[nodiscard]] Result<void> execute(
        int32_t theta_steps,
        int32_t rho_steps,
        float feedrate_rpm,
        float distance,
        std::atomic<int32_t>& theta_pos,
        std::atomic<int32_t>& rho_pos
    );

    /// Emergency stop - halts motion immediately
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

    // Bresenham state - matches main branch PolarRobot exactly
    // Uses the same algorithm for coordinated dual-axis motion
    struct BresenhamState {
        int32_t theta_total = 0;      // Original theta steps (for algorithm)
        int32_t rho_total = 0;        // Original rho steps (for algorithm)
        int32_t theta_remaining = 0;  // Remaining theta steps (for completion)
        int32_t rho_remaining = 0;    // Remaining rho steps (for completion)
        int32_t error = 0;            // Bresenham error term
        int8_t theta_dir = 0;         // +1 or -1
        int8_t rho_dir = 0;           // +1 or -1

        void clear() {
            theta_total = 0;
            rho_total = 0;
            theta_remaining = 0;
            rho_remaining = 0;
            error = 0;
            theta_dir = 0;
            rho_dir = 0;
        }
    } bresenham_;

    // Position tracking (pointers to caller's atomics)
    std::atomic<int32_t>* theta_pos_ = nullptr;
    std::atomic<int32_t>* rho_pos_ = nullptr;

    bool initialized_ = false;

    /// Timer callback - called from ISR context at step interval
    static bool IRAM_ATTR timer_callback(
        gptimer_handle_t timer,
        const gptimer_alarm_event_data_t* edata,
        void* user_ctx
    );

    /// Generate steps using Bresenham algorithm - called from ISR
    void IRAM_ATTR generate_step();
};

} // namespace sand_table
