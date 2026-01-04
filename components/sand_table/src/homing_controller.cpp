#include "homing_controller.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace sand_table {

static const char* TAG = "HomingController";

// Global instance pointer for ISR callbacks
static HomingController* g_homing_instance = nullptr;

HomingController::HomingController(
    StepperDriver& theta,
    StepperDriver& rho,
    tmc::TMC2209Stepper& theta_tmc,
    tmc::TMC2209Stepper& rho_tmc)
    : theta_(theta)
    , rho_(rho)
    , theta_tmc_(theta_tmc)
    , rho_tmc_(rho_tmc)
{
    g_homing_instance = this;
}

HomingController::~HomingController() {
    abort();
    gpio_isr_handler_remove(PinConfig::THETA_HALL);
    g_homing_instance = nullptr;
}

Result<void> HomingController::init() {
    // Configure Hall sensor GPIO with interrupt
    gpio_config_t hall_config = {
        .pin_bit_mask = (1ULL << PinConfig::THETA_HALL),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // Active low
    };

    if (gpio_config(&hall_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Hall sensor GPIO");
        return Result<void>::err(MotionError::HardwareFault);
    }

    // Install GPIO ISR service (ignore if already installed)
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service");
        return Result<void>::err(MotionError::HardwareFault);
    }

    // Add Hall sensor ISR
    if (gpio_isr_handler_add(PinConfig::THETA_HALL, hall_isr_handler, this) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add Hall sensor ISR");
        return Result<void>::err(MotionError::HardwareFault);
    }

    // Configure StallGuard for rho axis
    (void)rho_tmc_.set_stallguard_threshold(MotionConfig::RHO_STALLGUARD_THRESHOLD);

    ESP_LOGI(TAG, "Homing controller initialized");
    return Result<void>::ok();
}

void IRAM_ATTR HomingController::hall_isr_handler(void* arg) {
    auto* self = static_cast<HomingController*>(arg);
    self->hall_triggered_ = true;
}

void HomingController::rho_stall_callback(uint8_t addr, bool stalled) {
    if (g_homing_instance && stalled && addr == PinConfig::RHO_TMC_ADDR) {
        g_homing_instance->rho_stall_triggered_ = true;
    }
}

bool HomingController::is_hall_triggered() const {
    // Check current GPIO level (active low)
    return gpio_get_level(PinConfig::THETA_HALL) == 0;
}

bool HomingController::is_rho_stalled() const {
    return rho_tmc_.is_stalled();
}

void HomingController::abort() {
    abort_requested_.store(true, std::memory_order_release);
    homing_active_.store(false, std::memory_order_release);
    state_.store(HomingState::Idle, std::memory_order_release);
}

bool HomingController::should_stop_rho() const {
    return abort_requested_.load(std::memory_order_acquire) ||
           rho_stall_triggered_;
}

bool HomingController::should_stop_theta() const {
    return abort_requested_.load(std::memory_order_acquire) ||
           hall_triggered_;
}

HomingController::HomingResult HomingController::seek_rho_max() {
    HomingResult result;
    state_.store(HomingState::RhoSeekingMax, std::memory_order_release);

    ESP_LOGI(TAG, "Seeking rho maximum...");

    // Clear stall flag
    rho_stall_triggered_ = false;

    // Move outward (positive direction)
    rho_.set_direction(true);
    rho_.set_enabled(true);

    uint32_t steps = 0;
    while (steps < kMaxHomingSteps && !should_stop_rho()) {
        rho_.step_once();
        steps++;
        esp_rom_delay_us(kHomingStepIntervalUs);
    }

    if (abort_requested_.load(std::memory_order_acquire)) {
        result.error = MotionError::EmergencyStop;
        return result;
    }

    if (steps >= kMaxHomingSteps) {
        ESP_LOGE(TAG, "Rho max homing failed - exceeded max steps");
        result.error = MotionError::HomingFailed;
        return result;
    }

    ESP_LOGI(TAG, "Rho max found at step %lu", steps);
    result.success = true;
    result.position_steps = steps;
    return result;
}

HomingController::HomingResult HomingController::seek_rho_min() {
    HomingResult result;
    state_.store(HomingState::RhoSeekingMin, std::memory_order_release);

    ESP_LOGI(TAG, "Seeking rho minimum...");

    // Clear stall flag
    rho_stall_triggered_ = false;

    // Move inward (negative direction)
    rho_.set_direction(false);
    rho_.set_enabled(true);

    uint32_t steps = 0;
    while (steps < kMaxHomingSteps && !should_stop_rho()) {
        rho_.step_once();
        steps++;
        esp_rom_delay_us(kHomingStepIntervalUs);
    }

    if (abort_requested_.load(std::memory_order_acquire)) {
        result.error = MotionError::EmergencyStop;
        return result;
    }

    if (steps >= kMaxHomingSteps) {
        ESP_LOGE(TAG, "Rho min homing failed - exceeded max steps");
        result.error = MotionError::HomingFailed;
        return result;
    }

    ESP_LOGI(TAG, "Rho min found after %lu steps", steps);
    result.success = true;
    result.position_steps = steps;
    return result;
}

HomingController::HomingResult HomingController::calibrate_theta() {
    HomingResult result;

    ESP_LOGI(TAG, "Calibrating theta axis...");

    theta_.set_enabled(true);
    rho_.set_enabled(true);  // Keep rho enabled for coupling compensation

    // If already on Hall sensor, back off first
    if (is_hall_triggered()) {
        state_.store(HomingState::ThetaBackingOff, std::memory_order_release);
        ESP_LOGI(TAG, "Backing off from Hall sensor...");

        hall_triggered_ = false;
        theta_.set_direction(false);  // Reverse direction
        rho_.set_direction(false);    // Coupled motion

        uint32_t backoff_steps = 0;
        const uint32_t gear_ratio = static_cast<uint32_t>(MechanicalConfig::THETA_GEAR_RATIO);
        uint32_t theta_step_counter = 0;

        while (backoff_steps < kMaxHomingSteps && is_hall_triggered()) {
            theta_.step_once();
            backoff_steps++;
            theta_step_counter++;

            // Coupled rho compensation
            if (theta_step_counter >= gear_ratio) {
                theta_step_counter = 0;
                rho_.step_once();
            }

            esp_rom_delay_us(kHomingStepIntervalUs);

            if (abort_requested_.load(std::memory_order_acquire)) {
                result.error = MotionError::EmergencyStop;
                return result;
            }
        }

        if (is_hall_triggered()) {
            ESP_LOGE(TAG, "Failed to back off from Hall sensor");
            result.error = MotionError::HomingFailed;
            return result;
        }
    }

    // Seek first edge (Hall sensor trigger)
    state_.store(HomingState::ThetaSeekingFirstEdge, std::memory_order_release);
    ESP_LOGI(TAG, "Seeking first Hall edge...");

    hall_triggered_ = false;
    theta_.set_direction(true);  // Forward direction
    rho_.set_direction(true);    // Coupled motion

    uint32_t seek_steps = 0;
    const uint32_t gear_ratio = static_cast<uint32_t>(MechanicalConfig::THETA_GEAR_RATIO);
    uint32_t theta_step_counter = 0;

    while (seek_steps < kMaxHomingSteps && !should_stop_theta()) {
        theta_.step_once();
        seek_steps++;
        theta_step_counter++;

        // Coupled rho compensation
        if (theta_step_counter >= gear_ratio) {
            theta_step_counter = 0;
            rho_.step_once();
        }

        esp_rom_delay_us(kHomingStepIntervalUs);
    }

    if (abort_requested_.load(std::memory_order_acquire)) {
        result.error = MotionError::EmergencyStop;
        return result;
    }

    if (!hall_triggered_) {
        ESP_LOGE(TAG, "Failed to find first Hall edge");
        result.error = MotionError::HomingFailed;
        return result;
    }

    ESP_LOGI(TAG, "First Hall edge found");

    // Seek second edge (complete rotation)
    state_.store(HomingState::ThetaSeekingSecondEdge, std::memory_order_release);
    ESP_LOGI(TAG, "Seeking second Hall edge for calibration...");

    hall_triggered_ = false;
    theta_step_counter = 0;
    uint32_t rotation_steps = 0;

    // Wait a bit for debounce
    vTaskDelay(pdMS_TO_TICKS(50));

    while (rotation_steps < kMaxHomingSteps && !should_stop_theta()) {
        theta_.step_once();
        rotation_steps++;
        theta_step_counter++;

        // Coupled rho compensation
        if (theta_step_counter >= gear_ratio) {
            theta_step_counter = 0;
            rho_.step_once();
        }

        esp_rom_delay_us(kHomingStepIntervalUs);
    }

    if (abort_requested_.load(std::memory_order_acquire)) {
        result.error = MotionError::EmergencyStop;
        return result;
    }

    if (!hall_triggered_) {
        ESP_LOGE(TAG, "Failed to find second Hall edge");
        result.error = MotionError::HomingFailed;
        return result;
    }

    ESP_LOGI(TAG, "Theta calibration complete: %lu steps per rotation", rotation_steps);

    result.success = true;
    result.position_steps = rotation_steps;
    return result;
}

HomingController::HomingResult HomingController::home_theta() {
    homing_active_.store(true, std::memory_order_release);
    abort_requested_.store(false, std::memory_order_release);

    auto result = calibrate_theta();

    if (result.success) {
        theta_steps_per_rotation_ = result.position_steps;
        theta_.reset_position();
        state_.store(HomingState::Complete, std::memory_order_release);
    } else {
        state_.store(HomingState::Error, std::memory_order_release);
    }

    homing_active_.store(false, std::memory_order_release);
    return result;
}

HomingController::HomingResult HomingController::home_rho() {
    homing_active_.store(true, std::memory_order_release);
    abort_requested_.store(false, std::memory_order_release);

    // First seek maximum
    auto max_result = seek_rho_max();
    if (!max_result.success) {
        state_.store(HomingState::Error, std::memory_order_release);
        homing_active_.store(false, std::memory_order_release);
        return max_result;
    }

    // Brief pause
    vTaskDelay(pdMS_TO_TICKS(100));

    // Then seek minimum
    auto min_result = seek_rho_min();
    if (!min_result.success) {
        state_.store(HomingState::Error, std::memory_order_release);
        homing_active_.store(false, std::memory_order_release);
        return min_result;
    }

    // Store calibration
    rho_max_steps_ = min_result.position_steps;
    rho_.reset_position();  // Home is at minimum (center)

    ESP_LOGI(TAG, "Rho homing complete: %ld steps travel", rho_max_steps_);

    state_.store(HomingState::Complete, std::memory_order_release);
    homing_active_.store(false, std::memory_order_release);

    HomingResult result;
    result.success = true;
    result.position_steps = rho_max_steps_;
    return result;
}

Result<void> HomingController::home_all() {
    homing_active_.store(true, std::memory_order_release);
    abort_requested_.store(false, std::memory_order_release);

    ESP_LOGI(TAG, "Starting full homing sequence...");

    // Enable motors
    theta_.set_enabled(true);
    rho_.set_enabled(true);

    // Home rho first (safer - moves to center)
    auto rho_result = seek_rho_max();
    if (!rho_result.success) {
        state_.store(HomingState::Error, std::memory_order_release);
        homing_active_.store(false, std::memory_order_release);
        return Result<void>::err(rho_result.error);
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    auto rho_min_result = seek_rho_min();
    if (!rho_min_result.success) {
        state_.store(HomingState::Error, std::memory_order_release);
        homing_active_.store(false, std::memory_order_release);
        return Result<void>::err(rho_min_result.error);
    }

    rho_max_steps_ = rho_min_result.position_steps;
    rho_.reset_position();

    vTaskDelay(pdMS_TO_TICKS(100));

    // Then home theta
    auto theta_result = calibrate_theta();
    if (!theta_result.success) {
        state_.store(HomingState::Error, std::memory_order_release);
        homing_active_.store(false, std::memory_order_release);
        return Result<void>::err(theta_result.error);
    }

    theta_steps_per_rotation_ = theta_result.position_steps;
    theta_.reset_position();
    rho_.reset_position();  // Reset again since theta moves affect rho

    state_.store(HomingState::Complete, std::memory_order_release);
    homing_active_.store(false, std::memory_order_release);

    ESP_LOGI(TAG, "Homing complete - Rho: %ld steps, Theta: %ld steps/rev",
             rho_max_steps_, theta_steps_per_rotation_);

    return Result<void>::ok();
}

} // namespace sand_table
