// GPTimer-based step generator using fixed-interval + accumulator approach.
// This matches RBotFirmware's proven timer architecture for reliable stepping.

#include "gptimer_step_generator.h"
#include "esp_log.h"

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

    // Create GPTimer at 1MHz (1us resolution)
    // Timer fires every TICK_INTERVAL_US (20us) for fixed-interval stepping
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1MHz = 1us resolution
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
    ESP_LOGI(TAG, "GPTimer initialized (fixed %uus interval, pins: theta=%d/%d, rho=%d/%d)",
             TICK_INTERVAL_US, theta_step, theta_dir, rho_step, rho_dir);
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

    uint32_t abs_theta = static_cast<uint32_t>(std::abs(theta_steps));
    uint32_t abs_rho = static_cast<uint32_t>(std::abs(rho_steps));

    // No steps to execute
    if (abs_theta == 0 && abs_rho == 0) {
        return Result<void>::ok();
    }

    // Set directions via GPIO
    gpio_set_level(theta_dir_pin_, theta_steps >= 0 ? 1 : 0);
    gpio_set_level(rho_dir_pin_, rho_steps >= 0 ? 1 : 0);

    // Initialize step state
    state_.clear();
    state_.theta_total = abs_theta;
    state_.rho_total = abs_rho;
    state_.theta_dir = (theta_steps >= 0) ? 1 : -1;
    state_.rho_dir = (rho_steps >= 0) ? 1 : -1;

    // Determine which axis has the most steps (drives timing)
    if (abs_theta >= abs_rho) {
        state_.max_axis = 0;  // theta
    } else {
        state_.max_axis = 1;  // rho
    }

    // Calculate step rate in steps/second
    uint32_t max_steps = std::max(abs_theta, abs_rho);
    float step_rate_per_sec;

    if (feedrate_rpm <= 0) {
        feedrate_rpm = static_cast<float>(MotionConfig::RHO_MAX_SPEED_RPM);
    }

    if (distance > 0.0f && max_steps > 0) {
        // motion_time = distance / (feedrate_rpm / 60)
        float motion_time_sec = distance / (feedrate_rpm / 60.0f);
        // Ensure minimum motion time to prevent division issues
        if (motion_time_sec < 0.001f) {
            motion_time_sec = 0.001f;
        }
        step_rate_per_sec = static_cast<float>(max_steps) / motion_time_sec;
    } else {
        // Fallback: conservative fixed rate
        step_rate_per_sec = 2000.0f;
    }

    // Clamp step rate to hardware limits
    // Max: 25k steps/sec (each step needs 2 ISR ticks minimum at 50kHz)
    // Min: 10 steps/sec
    step_rate_per_sec = std::max(10.0f, std::min(25000.0f, step_rate_per_sec));

    // Convert to TTICKS units: step_rate_per_tticks = (steps_per_sec * TTICKS_VALUE) / TICKS_PER_SEC
    state_.step_rate_per_tticks = static_cast<uint64_t>(
        (static_cast<double>(step_rate_per_sec) * static_cast<double>(TTICKS_VALUE)) /
        static_cast<double>(TICKS_PER_SEC)
    );

    // Ensure minimum step rate
    if (state_.step_rate_per_tticks < MIN_STEP_RATE_PER_TTICKS) {
        state_.step_rate_per_tticks = MIN_STEP_RATE_PER_TTICKS;
    }

    theta_pos_ = &theta_pos;
    rho_pos_ = &rho_pos;

    executing_.store(true, std::memory_order_release);
    stop_requested_.store(false, std::memory_order_release);
    xSemaphoreTake(completion_sem_, 0);  // Clear semaphore

    // Configure timer for fixed interval (TICK_INTERVAL_US microseconds)
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = TICK_INTERVAL_US,
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

    // Calculate expected motion time for verification
    float expected_time_ms = (static_cast<float>(max_steps) / step_rate_per_sec) * 1000.0f;

    ESP_LOGI(TAG, "Motion: theta=%ld rho=%ld | rate=%.0f steps/s | time=%.0fms | max_axis=%s",
             theta_steps, rho_steps, step_rate_per_sec, expected_time_ms,
             (state_.max_axis == 0) ? "theta" : "rho");

    // Wait for completion
    const TickType_t timeout_ticks = pdMS_TO_TICKS(30000);
    if (xSemaphoreTake(completion_sem_, timeout_ticks) != pdTRUE) {
        ESP_LOGE(TAG, "Execution timeout");
        stop();
        return Result<void>::err(MotionError::Timeout);
    }

    if (stop_requested_.load(std::memory_order_acquire)) {
        return Result<void>::err(MotionError::EmergencyStop);
    }

    // Log completion with actual step counts (verify they match commanded)
    ESP_LOGD(TAG, "Done: theta_count=%lu/%lu rho_count=%lu/%lu",
             state_.theta_count, state_.theta_total,
             state_.rho_count, state_.rho_total);

    return Result<void>::ok();
}

void GpTimerStepGenerator::stop() {
    stop_requested_.store(true, std::memory_order_release);

    if (step_timer_) {
        gptimer_stop(step_timer_);
    }

    // Clear any pending step pulses
    gpio_set_level(theta_step_pin_, 0);
    gpio_set_level(rho_step_pin_, 0);

    state_.clear();
    executing_.store(false, std::memory_order_release);

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
    self->isr_step_handler();
    return false;  // No context switch needed unless we signal completion
}

void IRAM_ATTR GpTimerStepGenerator::isr_step_handler() {
    // Check for stop request
    if (stop_requested_.load(std::memory_order_acquire)) {
        gptimer_stop(step_timer_);
        // Clear any pending step pulses
        gpio_set_level(theta_step_pin_, 0);
        gpio_set_level(rho_step_pin_, 0);
        executing_.store(false, std::memory_order_release);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(completion_sem_, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }

    // Handle step pulse endings first - ensures minimum pulse width
    // This also updates position for each completed step
    bool steps_ended = handle_step_end();

    // Check if we've completed all steps AND ended all pulses
    // Both counts must match totals AND no pulses pending
    bool motion_complete =
        (state_.theta_count >= state_.theta_total) &&
        (state_.rho_count >= state_.rho_total) &&
        !state_.theta_step_pending &&
        !state_.rho_step_pending;

    if (motion_complete) {
        gptimer_stop(step_timer_);
        executing_.store(false, std::memory_order_release);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(completion_sem_, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }

    // NOTE: Don't early return after steps_ended - this was causing timing errors.
    // RBotFirmware processes accumulator every tick regardless of step endings.
    // Early return was skipping accumulator updates, causing ~33% slower motion.

    // Add step rate to accumulator
    state_.step_accumulator += state_.step_rate_per_tticks;

    // Check for accumulator overflow - time to generate steps
    if (state_.step_accumulator >= TTICKS_VALUE) {
        // Subtract overflow value (keep remainder for sub-tick precision)
        state_.step_accumulator -= TTICKS_VALUE;

        // Generate coordinated steps
        handle_step_motion();
    }
}

bool IRAM_ATTR GpTimerStepGenerator::handle_step_end() {
    bool any_ended = false;

    // End theta step pulse if pending
    if (state_.theta_step_pending) {
        gpio_set_level(theta_step_pin_, 0);
        state_.theta_step_pending = false;
        theta_pos_->fetch_add(state_.theta_dir, std::memory_order_release);
        any_ended = true;
    }

    // End rho step pulse if pending
    if (state_.rho_step_pending) {
        gpio_set_level(rho_step_pin_, 0);
        state_.rho_step_pending = false;
        rho_pos_->fetch_add(state_.rho_dir, std::memory_order_release);
        any_ended = true;
    }

    return any_ended;
}

void IRAM_ATTR GpTimerStepGenerator::handle_step_motion() {
    // Get max axis references
    uint32_t max_steps = (state_.max_axis == 0) ? state_.theta_total : state_.rho_total;
    uint32_t& max_count = (state_.max_axis == 0) ? state_.theta_count : state_.rho_count;
    uint32_t minor_steps = (state_.max_axis == 0) ? state_.rho_total : state_.theta_total;
    uint32_t& minor_count = (state_.max_axis == 0) ? state_.rho_count : state_.theta_count;
    gpio_num_t max_pin = (state_.max_axis == 0) ? theta_step_pin_ : rho_step_pin_;
    gpio_num_t minor_pin = (state_.max_axis == 0) ? rho_step_pin_ : theta_step_pin_;
    bool& max_pending = (state_.max_axis == 0) ? state_.theta_step_pending : state_.rho_step_pending;
    bool& minor_pending = (state_.max_axis == 0) ? state_.rho_step_pending : state_.theta_step_pending;

    // Step the major axis if not complete
    if (max_count < max_steps) {
        gpio_set_level(max_pin, 1);
        max_pending = true;
        max_count++;
    }

    // Check if minor axis needs to step (relative accumulator approach)
    // This is equivalent to Bresenham but matches RBotFirmware's implementation
    if (minor_count < minor_steps) {
        // Add minor axis total to relative accumulator
        state_.relative_accumulator += minor_steps;

        // If accumulator overflows max axis total, step the minor axis
        if (state_.relative_accumulator >= max_steps) {
            state_.relative_accumulator -= max_steps;

            gpio_set_level(minor_pin, 1);
            minor_pending = true;
            minor_count++;
        }
    }
}

} // namespace sand_table
