// GPTimer-based step generator for A/B testing against RMT implementation.
// This implementation closely matches the main branch's PolarRobot timer approach.
//
// TO SWITCH BACK TO RMT-ONLY:
// - In CoordinatedStepperController, set step_mode_ = StepGeneratorMode::RMT
// - Or simply don't call set_step_generator_mode()

#include "gptimer_step_generator.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include <cmath>
#include <algorithm>

namespace sand_table {

static const char* TAG = "GPTimerStepGen";

GpTimerStepGenerator::~GpTimerStepGenerator() {
    deinit();
}

Result<void> GpTimerStepGenerator::init(
    gpio_num_t theta_step,
    gpio_num_t theta_dir,
    gpio_num_t rho_step,
    gpio_num_t rho_dir)
{
    if (initialized_) {
        return Result<void>::ok();
    }

    theta_step_pin_ = theta_step;
    theta_dir_pin_ = theta_dir;
    rho_step_pin_ = rho_step;
    rho_dir_pin_ = rho_dir;

    // Note: GPIO is already configured by StepperDriver, we just use the pins
    // Direction pins are also configured by StepperDriver

    // Create GPTimer at 1MHz (matching main branch exactly)
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1MHz = 1us resolution (same as main branch)
        .intr_priority = 0,
        .flags = {
            .intr_shared = false,
        },
    };

    esp_err_t err = gptimer_new_timer(&timer_config, &step_timer_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create GPTimer: %s", esp_err_to_name(err));
        return Result<void>::err(MotionError::HardwareFault);
    }

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback,
    };

    err = gptimer_register_event_callbacks(step_timer_, &cbs, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register timer callbacks: %s", esp_err_to_name(err));
        gptimer_del_timer(step_timer_);
        step_timer_ = nullptr;
        return Result<void>::err(MotionError::HardwareFault);
    }

    err = gptimer_enable(step_timer_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable GPTimer: %s", esp_err_to_name(err));
        gptimer_del_timer(step_timer_);
        step_timer_ = nullptr;
        return Result<void>::err(MotionError::HardwareFault);
    }

    completion_sem_ = xSemaphoreCreateBinary();
    if (!completion_sem_) {
        ESP_LOGE(TAG, "Failed to create completion semaphore");
        gptimer_disable(step_timer_);
        gptimer_del_timer(step_timer_);
        step_timer_ = nullptr;
        return Result<void>::err(MotionError::HardwareFault);
    }

    initialized_ = true;
    ESP_LOGI(TAG, "GPTimer step generator initialized (1MHz, pins: theta=%d/%d, rho=%d/%d)",
             theta_step, theta_dir, rho_step, rho_dir);
    return Result<void>::ok();
}

void GpTimerStepGenerator::deinit() {
    if (step_timer_) {
        gptimer_stop(step_timer_);
        gptimer_disable(step_timer_);
        gptimer_del_timer(step_timer_);
        step_timer_ = nullptr;
    }

    if (completion_sem_) {
        vSemaphoreDelete(completion_sem_);
        completion_sem_ = nullptr;
    }

    initialized_ = false;
}

Result<void> GpTimerStepGenerator::execute(
    int32_t theta_steps,
    int32_t rho_steps,
    float feedrate_rpm,
    float distance,
    std::atomic<int32_t>& theta_pos,
    std::atomic<int32_t>& rho_pos)
{
    if (!initialized_) {
        ESP_LOGE(TAG, "Not initialized");
        return Result<void>::err(MotionError::InvalidState);
    }

    if (executing_.load(std::memory_order_acquire)) {
        ESP_LOGE(TAG, "Already executing");
        return Result<void>::err(MotionError::InvalidState);
    }

    int32_t abs_theta = std::abs(theta_steps);
    int32_t abs_rho = std::abs(rho_steps);

    // No steps to execute
    if (abs_theta == 0 && abs_rho == 0) {
        return Result<void>::ok();
    }

    // Set directions via GPIO (matching main branch)
    gpio_set_level(theta_dir_pin_, theta_steps >= 0 ? 1 : 0);
    gpio_set_level(rho_dir_pin_, rho_steps >= 0 ? 1 : 0);

    // Initialize Bresenham state (matching main branch PolarRobot exactly)
    // Store original totals for algorithm AND remaining for completion tracking
    bresenham_.theta_total = abs_theta;
    bresenham_.rho_total = abs_rho;
    bresenham_.theta_remaining = abs_theta;
    bresenham_.rho_remaining = abs_rho;
    bresenham_.theta_dir = (theta_steps >= 0) ? 1 : -1;
    bresenham_.rho_dir = (rho_steps >= 0) ? 1 : -1;

    // Initialize Bresenham error term based on which axis has more steps
    // This matches main branch: positive error for theta-major, negative for rho-major
    if (bresenham_.theta_total >= bresenham_.rho_total) {
        bresenham_.error = bresenham_.theta_total / 2;  // Theta is primary axis
    } else {
        bresenham_.error = -bresenham_.rho_total / 2;   // Rho is primary axis
    }

    theta_pos_ = &theta_pos;
    rho_pos_ = &rho_pos;

    // Calculate step interval (matching main branch planMotion logic)
    uint32_t total_steps = abs_theta + abs_rho;
    uint64_t step_interval_us;

    if (feedrate_rpm <= 0) {
        feedrate_rpm = static_cast<float>(MotionConfig::RHO_MAX_SPEED_RPM);
    }

    if (distance > 0.0f && total_steps > 0) {
        // Calculate interval based on RPM and motion distance
        // This matches main branch: motionTimeSeconds = totalDistance / (feedRateRPM / 60.0f)
        float motion_time_sec = distance / (feedrate_rpm / 60.0f);
        step_interval_us = static_cast<uint64_t>((motion_time_sec * 1000000.0f) / total_steps);
    } else {
        // Fallback: conservative fixed rate (matching main branch fallback)
        step_interval_us = 500;  // 2000 steps/sec
    }

    // Clamp to reasonable bounds (matching main branch: 10kHz max, 1Hz min)
    step_interval_us = std::max<uint64_t>(100, std::min<uint64_t>(1000000, step_interval_us));

    executing_.store(true, std::memory_order_release);
    stop_requested_.store(false, std::memory_order_release);
    xSemaphoreTake(completion_sem_, 0);  // Clear semaphore

    // Start timer with calculated interval (auto-reload for continuous stepping)
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = step_interval_us,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };

    esp_err_t err = gptimer_set_alarm_action(step_timer_, &alarm_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set alarm: %s", esp_err_to_name(err));
        executing_.store(false, std::memory_order_release);
        return Result<void>::err(MotionError::HardwareFault);
    }

    gptimer_set_raw_count(step_timer_, 0);
    err = gptimer_start(step_timer_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(err));
        executing_.store(false, std::memory_order_release);
        return Result<void>::err(MotionError::HardwareFault);
    }

    ESP_LOGI(TAG, "GPTimer started: theta=%ld (dir=%d), rho=%ld (dir=%d), interval=%llu us",
             theta_steps, bresenham_.theta_dir, rho_steps, bresenham_.rho_dir,
             step_interval_us);

    // Wait for completion (with timeout)
    const TickType_t timeout_ticks = pdMS_TO_TICKS(30000);  // 30 second timeout
    if (xSemaphoreTake(completion_sem_, timeout_ticks) != pdTRUE) {
        ESP_LOGE(TAG, "GPTimer execution timeout");
        stop();
        return Result<void>::err(MotionError::Timeout);
    }

    if (stop_requested_.load(std::memory_order_acquire)) {
        return Result<void>::err(MotionError::EmergencyStop);
    }

    // Log final positions for debugging
    int32_t final_theta = theta_pos_->load(std::memory_order_acquire);
    int32_t final_rho = rho_pos_->load(std::memory_order_acquire);
    ESP_LOGI(TAG, "GPTimer motion complete: final pos theta=%ld, rho=%ld", final_theta, final_rho);
    return Result<void>::ok();
}

void GpTimerStepGenerator::stop() {
    stop_requested_.store(true, std::memory_order_release);

    if (step_timer_) {
        gptimer_stop(step_timer_);
    }

    bresenham_.clear();
    executing_.store(false, std::memory_order_release);

    // Signal completion in case execute() is waiting
    if (completion_sem_) {
        xSemaphoreGive(completion_sem_);
    }
}

bool IRAM_ATTR GpTimerStepGenerator::timer_callback(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t* edata,
    void* user_ctx)
{
    auto* self = static_cast<GpTimerStepGenerator*>(user_ctx);
    self->generate_step();
    return true;  // Return true to yield if higher priority task is woken
}

void IRAM_ATTR GpTimerStepGenerator::generate_step() {
    // Check for stop request
    if (stop_requested_.load(std::memory_order_acquire)) {
        gptimer_stop(step_timer_);
        executing_.store(false, std::memory_order_release);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(completion_sem_, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }

    bool step_theta = false;
    bool step_rho = false;

    // Use ORIGINAL totals for Bresenham algorithm (not remaining counts!)
    // This is critical - the algorithm needs constant values throughout the move
    int32_t thetaS = bresenham_.theta_total;
    int32_t rhoS = bresenham_.rho_total;

    // Bresenham algorithm - exact match to main branch generateSteps()
    // This is the critical section that must match for identical behavior
    if (thetaS >= rhoS) {
        // Theta is primary axis
        bresenham_.error -= rhoS;
        if (bresenham_.error < 0) {
            bresenham_.error += thetaS;
            step_rho = true;
        }
        step_theta = (bresenham_.theta_remaining > 0);
    } else {
        // Rho is primary axis
        bresenham_.error += thetaS;
        if (bresenham_.error > 0) {
            bresenham_.error -= rhoS;
            step_theta = true;
        }
        step_rho = (bresenham_.rho_remaining > 0);
    }

    // Generate step pulses (matching main branch timing)
    if (step_theta && bresenham_.theta_remaining > 0) {
        gpio_set_level(theta_step_pin_, 1);
        esp_rom_delay_us(HardwareConfig::MIN_STEP_PULSE_US);  // 2us pulse
        gpio_set_level(theta_step_pin_, 0);

        bresenham_.theta_remaining--;
        theta_pos_->fetch_add(bresenham_.theta_dir, std::memory_order_release);
    }

    if (step_rho && bresenham_.rho_remaining > 0) {
        gpio_set_level(rho_step_pin_, 1);
        esp_rom_delay_us(HardwareConfig::MIN_STEP_PULSE_US);  // 2us pulse
        gpio_set_level(rho_step_pin_, 0);

        bresenham_.rho_remaining--;
        rho_pos_->fetch_add(bresenham_.rho_dir, std::memory_order_release);
    }

    // Check for completion
    if (bresenham_.theta_remaining == 0 && bresenham_.rho_remaining == 0) {
        gptimer_stop(step_timer_);
        executing_.store(false, std::memory_order_release);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(completion_sem_, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

} // namespace sand_table
