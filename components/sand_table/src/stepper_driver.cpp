#include "stepper_driver.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cmath>

namespace sand_table {

static const char* TAG = "StepperDriver";

// =============================================================================
// StepperDriver Implementation
// =============================================================================

StepperDriver::StepperDriver(const Pins& pins, const char* name)
    : pins_(pins)
    , name_(name)
{
}

StepperDriver::~StepperDriver() {
    if (rmt_channel_) {
        rmt_del_channel(rmt_channel_);
    }
    if (step_encoder_) {
        rmt_del_encoder(step_encoder_);
    }
}

Result<void> StepperDriver::init() {
    auto gpio_result = init_gpio();
    if (gpio_result.is_err()) {
        return gpio_result;
    }

    auto rmt_result = init_rmt();
    if (rmt_result.is_err()) {
        return rmt_result;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "%s driver initialized", name_);
    return Result<void>::ok();
}

Result<void> StepperDriver::init_gpio() {
    // Configure step pin
    gpio_config_t step_config = {
        .pin_bit_mask = (1ULL << pins_.step),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&step_config) != ESP_OK) {
        ESP_LOGE(TAG, "%s: Failed to configure step pin", name_);
        return Result<void>::err(MotionError::HardwareFault);
    }
    gpio_set_level(pins_.step, 0);

    // Configure direction pin
    gpio_config_t dir_config = {
        .pin_bit_mask = (1ULL << pins_.dir),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&dir_config) != ESP_OK) {
        ESP_LOGE(TAG, "%s: Failed to configure dir pin", name_);
        return Result<void>::err(MotionError::HardwareFault);
    }
    gpio_set_level(pins_.dir, 0);

    // Configure enable pin (active low typically)
    gpio_config_t enable_config = {
        .pin_bit_mask = (1ULL << pins_.enable),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&enable_config) != ESP_OK) {
        ESP_LOGE(TAG, "%s: Failed to configure enable pin", name_);
        return Result<void>::err(MotionError::HardwareFault);
    }
    gpio_set_level(pins_.enable, 1);  // Disabled by default (active low)

    return Result<void>::ok();
}

Result<void> StepperDriver::init_rmt() {
    // For now, we'll use GPIO + timer for step generation
    // RMT can be added later for more precise timing if needed
    return Result<void>::ok();
}

void StepperDriver::set_enabled(bool enabled) {
    enabled_ = enabled;
    // Active low enable pin
    gpio_set_level(pins_.enable, enabled ? 0 : 1);
    ESP_LOGD(TAG, "%s: %s", name_, enabled ? "enabled" : "disabled");
}

void StepperDriver::set_direction(bool positive) {
    direction_positive_ = positive;
    gpio_set_level(pins_.dir, positive ? 1 : 0);
}

void StepperDriver::step_once() {
    gpio_set_level(pins_.step, 1);
    esp_rom_delay_us(HardwareConfig::MIN_STEP_PULSE_US);
    gpio_set_level(pins_.step, 0);

    // Update position
    int32_t pos = position_.load(std::memory_order_relaxed);
    pos += direction_positive_ ? 1 : -1;
    position_.store(pos, std::memory_order_release);
}

void StepperDriver::step_fixed_rate(uint32_t steps, uint32_t interval_us) {
    for (uint32_t i = 0; i < steps; ++i) {
        step_once();
        if (interval_us > HardwareConfig::MIN_STEP_PULSE_US) {
            esp_rom_delay_us(interval_us - HardwareConfig::MIN_STEP_PULSE_US);
        }
    }
}

// =============================================================================
// CoordinatedStepperController Implementation
// =============================================================================

CoordinatedStepperController::CoordinatedStepperController(
    StepperDriver& theta,
    StepperDriver& rho)
    : theta_(theta)
    , rho_(rho)
{
}

CoordinatedStepperController::~CoordinatedStepperController() {
    if (step_timer_) {
        gptimer_stop(step_timer_);
        gptimer_disable(step_timer_);
        gptimer_del_timer(step_timer_);
    }
}

Result<void> CoordinatedStepperController::init() {
    // Create GPTimer for step generation
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = HardwareConfig::TIMER_RESOLUTION_HZ,
        .intr_priority = 0,
        .flags = {
            .intr_shared = false,
        },
    };

    if (gptimer_new_timer(&timer_config, &step_timer_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create step timer");
        return Result<void>::err(MotionError::HardwareFault);
    }

    gptimer_event_callbacks_t cbs = {
        .on_alarm = step_timer_callback,
    };

    if (gptimer_register_event_callbacks(step_timer_, &cbs, this) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register timer callback");
        return Result<void>::err(MotionError::HardwareFault);
    }

    if (gptimer_enable(step_timer_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable step timer");
        return Result<void>::err(MotionError::HardwareFault);
    }

    ESP_LOGI(TAG, "Coordinated stepper controller initialized");
    return Result<void>::ok();
}

void CoordinatedStepperController::enable() {
    theta_.set_enabled(true);
    rho_.set_enabled(true);
}

void CoordinatedStepperController::disable() {
    theta_.set_enabled(false);
    rho_.set_enabled(false);
}

void CoordinatedStepperController::emergency_stop() {
    abort_requested_.store(true, std::memory_order_release);
    gptimer_stop(step_timer_);
    moving_.store(false, std::memory_order_release);
    bresenham_.clear();
}

VelocityProfile CoordinatedStepperController::calculate_velocity_profile(
    const MotionSegment& segment,
    float steps_per_mm) const
{
    VelocityProfile profile;

    // Convert velocities from mm/s to steps/s
    profile.entry_velocity = segment.entry_velocity * steps_per_mm;
    profile.cruise_velocity = segment.nominal_velocity * steps_per_mm;
    profile.exit_velocity = segment.exit_velocity * steps_per_mm;
    profile.acceleration = segment.acceleration * steps_per_mm;

    // Calculate total steps
    const uint32_t total_steps = static_cast<uint32_t>(
        std::abs(segment.delta_theta_steps) + std::abs(segment.delta_rho_steps)
    ) / 2;  // Average since we're coordinating

    if (total_steps == 0 || profile.acceleration <= 0.0f) {
        return profile;
    }

    // Calculate acceleration and deceleration distances
    // d = (v_final^2 - v_initial^2) / (2 * a)
    const float accel_dist = (profile.cruise_velocity * profile.cruise_velocity -
                              profile.entry_velocity * profile.entry_velocity) /
                             (2.0f * profile.acceleration);

    const float decel_dist = (profile.cruise_velocity * profile.cruise_velocity -
                              profile.exit_velocity * profile.exit_velocity) /
                             (2.0f * profile.acceleration);

    profile.accel_steps = static_cast<uint32_t>(std::max(0.0f, accel_dist));
    profile.decel_steps = static_cast<uint32_t>(std::max(0.0f, decel_dist));

    // Check if we can reach cruise velocity (triangular profile if not)
    if (profile.accel_steps + profile.decel_steps > total_steps) {
        // Triangular profile - recalculate peak velocity
        // v_peak = sqrt((2*a*d + v_entry^2 + v_exit^2) / 2)
        const float v_peak_sq = (profile.acceleration * total_steps +
                                  0.5f * (profile.entry_velocity * profile.entry_velocity +
                                         profile.exit_velocity * profile.exit_velocity));
        profile.cruise_velocity = std::sqrt(std::max(0.0f, v_peak_sq));

        // Recalculate step counts
        profile.accel_steps = total_steps / 2;
        profile.decel_steps = total_steps - profile.accel_steps;
        profile.cruise_steps = 0;
    } else {
        profile.cruise_steps = total_steps - profile.accel_steps - profile.decel_steps;
    }

    return profile;
}

Result<void> CoordinatedStepperController::execute_segment(
    const MotionSegment& segment)
{
    if (moving_.load(std::memory_order_acquire)) {
        return Result<void>::err(MotionError::InvalidState);
    }

    // Setup Bresenham state
    bresenham_.theta_remaining = std::abs(segment.delta_theta_steps);
    bresenham_.rho_remaining = std::abs(segment.delta_rho_steps);
    bresenham_.theta_dir = (segment.delta_theta_steps >= 0) ? 1 : -1;
    bresenham_.rho_dir = (segment.delta_rho_steps >= 0) ? 1 : -1;

    // Initialize Bresenham error term
    if (bresenham_.theta_remaining >= bresenham_.rho_remaining) {
        bresenham_.error = bresenham_.theta_remaining / 2;
    } else {
        bresenham_.error = -bresenham_.rho_remaining / 2;
    }

    // Set directions
    theta_.set_direction(bresenham_.theta_dir > 0);
    rho_.set_direction(bresenham_.rho_dir > 0);

    // Calculate velocity profile
    // Use the average steps per mm for coordinated motion
    const float avg_steps_per_mm = (MechanicalConfig::THETA_STEPS_PER_DEG +
                                    MechanicalConfig::RHO_STEPS_PER_MM) / 2.0f;
    const VelocityProfile profile = calculate_velocity_profile(segment, avg_steps_per_mm);

    // Setup velocity state
    velocity_.steps_taken = 0;
    velocity_.accel_steps = profile.accel_steps;
    velocity_.cruise_steps = profile.cruise_steps;
    velocity_.decel_steps = profile.decel_steps;
    velocity_.entry_velocity = profile.entry_velocity;
    velocity_.cruise_velocity = profile.cruise_velocity;
    velocity_.exit_velocity = profile.exit_velocity;
    velocity_.acceleration = profile.acceleration;
    velocity_.current_velocity = profile.entry_velocity;

    // No steps to execute
    if (bresenham_.theta_remaining == 0 && bresenham_.rho_remaining == 0) {
        return Result<void>::ok();
    }

    // Clear abort flag
    abort_requested_.store(false, std::memory_order_release);
    moving_.store(true, std::memory_order_release);

    // Calculate initial step interval
    uint32_t interval = calculate_next_interval();

    // Configure and start timer
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = interval,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };
    gptimer_set_alarm_action(step_timer_, &alarm_config);
    gptimer_set_raw_count(step_timer_, 0);
    gptimer_start(step_timer_);

    // Wait for completion
    while (moving_.load(std::memory_order_acquire)) {
        vTaskDelay(1);

        if (abort_requested_.load(std::memory_order_acquire)) {
            gptimer_stop(step_timer_);
            moving_.store(false, std::memory_order_release);
            return Result<void>::err(MotionError::EmergencyStop);
        }
    }

    return Result<void>::ok();
}

bool IRAM_ATTR CoordinatedStepperController::step_timer_callback(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t* edata,
    void* user_ctx)
{
    auto* self = static_cast<CoordinatedStepperController*>(user_ctx);
    self->generate_step();
    return true;
}

void IRAM_ATTR CoordinatedStepperController::generate_step() {
    bool step_theta = false;
    bool step_rho = false;

    const int32_t theta_steps = bresenham_.theta_remaining;
    const int32_t rho_steps = bresenham_.rho_remaining;

    // Bresenham algorithm for coordinated motion
    if (theta_steps >= rho_steps) {
        bresenham_.error -= rho_steps;
        if (bresenham_.error < 0) {
            bresenham_.error += theta_steps;
            step_rho = true;
        }
        step_theta = (theta_steps > 0);
    } else {
        bresenham_.error += theta_steps;
        if (bresenham_.error > 0) {
            bresenham_.error -= rho_steps;
            step_theta = true;
        }
        step_rho = (rho_steps > 0);
    }

    // Generate step pulses
    if (step_theta && bresenham_.theta_remaining > 0) {
        gpio_set_level(theta_.pins_.step, 1);
    }
    if (step_rho && bresenham_.rho_remaining > 0) {
        gpio_set_level(rho_.pins_.step, 1);
    }

    // Minimum pulse width
    esp_rom_delay_us(HardwareConfig::MIN_STEP_PULSE_US);

    // Clear pulses
    if (step_theta && bresenham_.theta_remaining > 0) {
        gpio_set_level(theta_.pins_.step, 0);
        bresenham_.theta_remaining--;

        // Update position
        int32_t pos = theta_.position_.load(std::memory_order_relaxed);
        theta_.position_.store(pos + bresenham_.theta_dir, std::memory_order_release);
    }
    if (step_rho && bresenham_.rho_remaining > 0) {
        gpio_set_level(rho_.pins_.step, 0);
        bresenham_.rho_remaining--;

        // Update position
        int32_t pos = rho_.position_.load(std::memory_order_relaxed);
        rho_.position_.store(pos + bresenham_.rho_dir, std::memory_order_release);
    }

    // Update velocity
    velocity_.steps_taken++;

    // Check if motion complete
    if (bresenham_.theta_remaining == 0 && bresenham_.rho_remaining == 0) {
        gptimer_stop(step_timer_);
        moving_.store(false, std::memory_order_release);
        return;
    }

    // Calculate and set next interval
    uint32_t next_interval = calculate_next_interval();

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = next_interval,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };
    gptimer_set_alarm_action(step_timer_, &alarm_config);
}

uint32_t CoordinatedStepperController::calculate_next_interval() {
    const uint32_t steps = velocity_.steps_taken;

    // Determine which phase we're in and calculate velocity
    if (steps < velocity_.accel_steps) {
        // Acceleration phase: v = sqrt(v_entry^2 + 2*a*s)
        velocity_.current_velocity = std::sqrt(
            velocity_.entry_velocity * velocity_.entry_velocity +
            2.0f * velocity_.acceleration * static_cast<float>(steps)
        );
    } else if (steps < velocity_.accel_steps + velocity_.cruise_steps) {
        // Cruise phase
        velocity_.current_velocity = velocity_.cruise_velocity;
    } else {
        // Deceleration phase
        const uint32_t decel_steps_taken = steps - velocity_.accel_steps - velocity_.cruise_steps;
        const uint32_t decel_steps_remaining = velocity_.decel_steps - decel_steps_taken;

        velocity_.current_velocity = std::sqrt(
            velocity_.exit_velocity * velocity_.exit_velocity +
            2.0f * velocity_.acceleration * static_cast<float>(decel_steps_remaining)
        );
    }

    // Clamp velocity to reasonable bounds
    velocity_.current_velocity = std::max(
        100.0f,  // Minimum velocity to prevent divide by zero
        std::min(velocity_.current_velocity,
                 static_cast<float>(HardwareConfig::MAX_STEP_RATE_HZ))
    );

    // Convert velocity to interval: interval_us = 1,000,000 / velocity_steps_per_sec
    return static_cast<uint32_t>(
        static_cast<float>(HardwareConfig::TIMER_RESOLUTION_HZ) /
        velocity_.current_velocity
    );
}

} // namespace sand_table
