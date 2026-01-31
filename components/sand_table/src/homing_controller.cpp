#include "homing_controller.h"
#include "stepper_driver.h"
#include "esp_log.h"

namespace sand_table {

    static const char* TAG = "HomingController";

    // Global instance pointer for ISR access
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

        // Remove ISR handlers
        gpio_isr_handler_remove(PinConfig::THETA_HALL);
        gpio_isr_handler_remove(PinConfig::RHO_DIAG);

        g_homing_instance = nullptr;
    }

    Result<void> HomingController::init() {
        ESP_LOGD(TAG, "Initializing homing controller...");
        ESP_LOGD(TAG, "  Hall sensor pin: GPIO%d", PinConfig::THETA_HALL);
        ESP_LOGD(TAG, "  Rho DIAG pin: GPIO%d", PinConfig::RHO_DIAG);
        ESP_LOGD(TAG, "  StallGuard threshold: %d", MotionConfig::RHO_STALLGUARD_THRESHOLD);

        // Configure Hall sensor GPIO with interrupt
        gpio_config_t hall_config = {
            .pin_bit_mask = (1ULL << PinConfig::THETA_HALL),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE,  // Active low - trigger on falling edge
        };

        if (gpio_config(&hall_config) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure Hall sensor GPIO");
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Configure StallGuard DIAG pin with interrupt
        gpio_config_t diag_config = {
            .pin_bit_mask = (1ULL << PinConfig::RHO_DIAG),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_POSEDGE,  // DIAG goes high on stall
        };

        if (gpio_config(&diag_config) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure StallGuard DIAG GPIO");
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

        // Add StallGuard DIAG ISR
        if (gpio_isr_handler_add(PinConfig::RHO_DIAG, diag_isr_handler, this) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add StallGuard DIAG ISR");
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Configure StallGuard
        (void)rho_tmc_.set_stallguard_threshold(MotionConfig::RHO_STALLGUARD_THRESHOLD);
        (void)rho_tmc_.set_stallguard_min_speed(0xFFFFF);

        return Result<void>::ok();
    }

    void IRAM_ATTR HomingController::hall_isr_handler(void* arg) {
        auto* self = static_cast<HomingController*>(arg);
        self->hall_triggered_ = true;

        // Immediately stop RMT transmission (ISR-safe version)
        if (self->stepper_controller_ && self->homing_active_.load(std::memory_order_acquire)) {
            self->stepper_controller_->emergency_stop_from_isr();
        }
    }

    void IRAM_ATTR HomingController::diag_isr_handler(void* arg) {
        auto* self = static_cast<HomingController*>(arg);
        self->rho_stall_triggered_ = true;

        // Immediately stop RMT transmission (ISR-safe version)
        if (self->stepper_controller_ && self->homing_active_.load(std::memory_order_acquire)) {
            self->stepper_controller_->emergency_stop_from_isr();
        }
    }

    bool HomingController::is_hall_triggered() const {
        return gpio_get_level(PinConfig::THETA_HALL) == 0;
    }

    bool HomingController::is_rho_stalled() const {
        return gpio_get_level(PinConfig::RHO_DIAG) == 1;
    }

    void HomingController::abort() {
        abort_requested_.store(true, std::memory_order_release);
        if (stepper_controller_) {
            stepper_controller_->emergency_stop();
        }
        homing_active_.store(false, std::memory_order_release);
        state_.store(HomingState::Idle, std::memory_order_release);
    }

    int32_t HomingController::execute_homing_chunk(int32_t theta_steps, int32_t rho_steps) {
        if (!stepper_controller_) {
            ESP_LOGE(TAG, "No stepper controller set");
            return 0;
        }

        // Record position before move
        int32_t theta_before = theta_.position();
        int32_t rho_before = rho_.position();

        // Create a motion segment for the chunk
        MotionSegment segment;
        segment.delta_theta_steps = theta_steps;
        segment.delta_rho_steps = rho_steps;
        segment.delta_theta_rad = 0;  // Not used for step-based motion
        segment.delta_rho_norm = 0;
        segment.distance = static_cast<float>(std::max(std::abs(theta_steps), std::abs(rho_steps)));
        segment.nominal_velocity = kHomingFeedrate;
        segment.theta_entry_velocity = kHomingFeedrate;
        segment.theta_exit_velocity = kHomingFeedrate;
        segment.rho_entry_velocity = kHomingFeedrate;
        segment.rho_exit_velocity = kHomingFeedrate;
        segment.is_last_segment = true;

        // Execute the segment - this blocks until complete or stopped by ISR
        auto result = stepper_controller_->execute_segment(segment);

        // Calculate actual steps taken (may be less if ISR stopped early)
        int32_t theta_after = theta_.position();
        int32_t rho_after = rho_.position();

        int32_t theta_moved = std::abs(theta_after - theta_before);
        int32_t rho_moved = std::abs(rho_after - rho_before);

        // Return the dominant axis step count
        return std::max(theta_moved, rho_moved);
    }

    HomingController::HomingResult HomingController::seek_rho_max() {
        ESP_LOGI(TAG, "Seeking rho max...");
        state_.store(HomingState::RhoSeekingMax, std::memory_order_release);

        // Clear stall flag and set direction outward
        rho_stall_triggered_ = false;
        rho_.set_direction(true);

        int32_t total_steps = 0;
        while (total_steps < static_cast<int32_t>(kMaxHomingSteps)) {
            if (abort_requested_.load(std::memory_order_acquire)) {
                HomingResult result;
                result.error = MotionError::EmergencyStop;
                return result;
            }

            // Execute a chunk of rho-only steps
            int32_t chunk_steps = std::min(static_cast<int32_t>(kChunkSize),
                static_cast<int32_t>(kMaxHomingSteps) - total_steps);

            int32_t actual_steps = execute_homing_chunk(0, chunk_steps);
            total_steps += actual_steps;

            // Check if stall was detected (ISR set flag and stopped motion)
            if (rho_stall_triggered_) {
                ESP_LOGI(TAG, "Rho max found at step %ld", total_steps);
                HomingResult result;
                result.success = true;
                result.position_steps = total_steps;
                return result;
            }

            // If we got all requested steps, continue to next chunk
            // If we got fewer, something else stopped us
            if (actual_steps < chunk_steps && !rho_stall_triggered_) {
                // Unexpected stop
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

        ESP_LOGE(TAG, "Rho max not found within %lu steps", kMaxHomingSteps);
        HomingResult result;
        result.error = MotionError::HomingFailed;
        return result;
    }

    HomingController::HomingResult HomingController::seek_rho_min() {
        ESP_LOGI(TAG, "Seeking rho min...");
        state_.store(HomingState::RhoSeekingMin, std::memory_order_release);

        // Clear stall flag and set direction inward
        rho_stall_triggered_ = false;
        rho_.set_direction(false);

        // Brief pause to let StallGuard clear from previous stall
        vTaskDelay(pdMS_TO_TICKS(100));

        int32_t total_steps = 0;
        while (total_steps < static_cast<int32_t>(kMaxHomingSteps)) {
            if (abort_requested_.load(std::memory_order_acquire)) {
                HomingResult result;
                result.error = MotionError::EmergencyStop;
                return result;
            }

            // Execute a chunk of rho-only steps (negative for inward)
            int32_t chunk_steps = std::min(static_cast<int32_t>(kChunkSize),
                static_cast<int32_t>(kMaxHomingSteps) - total_steps);

            int32_t actual_steps = execute_homing_chunk(0, -chunk_steps);
            total_steps += actual_steps;

            if (rho_stall_triggered_) {
                ESP_LOGI(TAG, "Rho min found - total travel: %ld steps", total_steps);
                HomingResult result;
                result.success = true;
                result.position_steps = total_steps;
                return result;
            }

            if (actual_steps < chunk_steps && !rho_stall_triggered_) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

        ESP_LOGE(TAG, "Rho min not found within %lu steps", kMaxHomingSteps);
        HomingResult result;
        result.error = MotionError::HomingFailed;
        return result;
    }

    HomingController::HomingResult HomingController::calibrate_theta() {
        ESP_LOGI(TAG, "Calibrating theta...");
        state_.store(HomingState::ThetaCalibrating, std::memory_order_release);

        // If already on Hall sensor, back off first
        if (is_hall_triggered()) {
            ESP_LOGD(TAG, "Backing off Hall sensor...");
            hall_triggered_ = false;
            theta_.set_direction(false);  // Reverse
            rho_.set_direction(false);    // Rho follows theta due to coupling

            int32_t backoff_steps = 0;
            while (is_hall_triggered() && backoff_steps < static_cast<int32_t>(kMaxHomingSteps)) {
                if (abort_requested_.load(std::memory_order_acquire)) {
                    HomingResult result;
                    result.error = MotionError::EmergencyStop;
                    return result;
                }

                // Move theta with coupled rho compensation
                int32_t theta_chunk = std::min(static_cast<int32_t>(kChunkSize),
                    static_cast<int32_t>(kMaxHomingSteps) - backoff_steps);
                int32_t rho_chunk = theta_chunk / static_cast<int32_t>(kGearRatio);

                int32_t actual = execute_homing_chunk(-theta_chunk, -rho_chunk);
                backoff_steps += actual;
            }
        }

        // Seek forward to first Hall edge
        ESP_LOGD(TAG, "Seeking first Hall edge...");
        hall_triggered_ = false;
        theta_.set_direction(true);   // Forward
        rho_.set_direction(true);     // Rho follows theta

        int32_t total_steps = 0;
        bool first_edge_found = false;
        int32_t steps_since_first_edge = 0;

        while (total_steps < static_cast<int32_t>(kMaxHomingSteps)) {
            if (abort_requested_.load(std::memory_order_acquire)) {
                HomingResult result;
                result.error = MotionError::EmergencyStop;
                return result;
            }

            // Execute chunk with coupled rho compensation
            int32_t theta_chunk = std::min(static_cast<int32_t>(kChunkSize),
                static_cast<int32_t>(kMaxHomingSteps) - total_steps);
            int32_t rho_chunk = theta_chunk / static_cast<int32_t>(kGearRatio);

            int32_t actual = execute_homing_chunk(theta_chunk, rho_chunk);

            if (!first_edge_found) {
                total_steps += actual;
                if (hall_triggered_) {
                    ESP_LOGD(TAG, "First Hall edge found at step %ld", total_steps);
                    first_edge_found = true;
                    steps_since_first_edge = 0;
                    hall_triggered_ = false;  // Clear for second edge detection

                    // Re-enable Hall interrupt for second edge
                    // (it was cleared by finding first edge)
                }
            }
            else {
                steps_since_first_edge += actual;
                if (hall_triggered_) {
                    // Found second edge - one full rotation complete
                    ESP_LOGI(TAG, "Theta calibration complete: %ld steps/rotation", steps_since_first_edge);
                    HomingResult result;
                    result.success = true;
                    result.position_steps = steps_since_first_edge;
                    return result;
                }
            }
        }

        ESP_LOGE(TAG, "Theta calibration failed - Hall sensor not found");
        HomingResult result;
        result.error = MotionError::HomingFailed;
        return result;
    }

    HomingController::HomingResult HomingController::home_rho() {
        homing_active_.store(true, std::memory_order_release);
        abort_requested_.store(false, std::memory_order_release);

        // Enable motors
        theta_.set_enabled(true);
        rho_.set_enabled(true);

        // Seek max first
        auto max_result = seek_rho_max();
        if (!max_result.success) {
            homing_active_.store(false, std::memory_order_release);
            state_.store(HomingState::Error, std::memory_order_release);
            return max_result;
        }

        // Brief pause
        vTaskDelay(pdMS_TO_TICKS(100));

        // Seek min to measure travel
        auto min_result = seek_rho_min();
        if (!min_result.success) {
            homing_active_.store(false, std::memory_order_release);
            state_.store(HomingState::Error, std::memory_order_release);
            return min_result;
        }

        rho_max_steps_ = min_result.position_steps;
        rho_.reset_position();

        homing_active_.store(false, std::memory_order_release);
        state_.store(HomingState::Complete, std::memory_order_release);

        ESP_LOGI(TAG, "Rho homing complete: %ld steps travel", rho_max_steps_);
        return min_result;
    }

    HomingController::HomingResult HomingController::home_theta() {
        homing_active_.store(true, std::memory_order_release);
        abort_requested_.store(false, std::memory_order_release);

        // Enable motors
        theta_.set_enabled(true);
        rho_.set_enabled(true);

        auto result = calibrate_theta();

        if (result.success) {
            theta_steps_per_rotation_ = result.position_steps;
            theta_.reset_position();
            ESP_LOGI(TAG, "Theta homing complete: %ld steps/rotation", theta_steps_per_rotation_);
        }
        else {
            ESP_LOGE(TAG, "Theta homing failed");
            state_.store(HomingState::Error, std::memory_order_release);
        }

        homing_active_.store(false, std::memory_order_release);
        return result;
    }

    Result<void> HomingController::home_all() {
        homing_active_.store(true, std::memory_order_release);
        abort_requested_.store(false, std::memory_order_release);

        // Enable motors
        theta_.set_enabled(true);
        rho_.set_enabled(true);

        // Home rho first (for safety - moves to known position)
        auto rho_max = seek_rho_max();
        if (!rho_max.success) {
            homing_active_.store(false, std::memory_order_release);
            state_.store(HomingState::Error, std::memory_order_release);
            return Result<void>::err(rho_max.error);
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        auto rho_min = seek_rho_min();
        if (!rho_min.success) {
            homing_active_.store(false, std::memory_order_release);
            state_.store(HomingState::Error, std::memory_order_release);
            return Result<void>::err(rho_min.error);
        }

        rho_max_steps_ = rho_min.position_steps;

        vTaskDelay(pdMS_TO_TICKS(100));

        // Home theta
        auto theta_result = calibrate_theta();
        if (!theta_result.success) {
            homing_active_.store(false, std::memory_order_release);
            state_.store(HomingState::Error, std::memory_order_release);
            return Result<void>::err(theta_result.error);
        }

        theta_steps_per_rotation_ = theta_result.position_steps;

        // Reset positions
        theta_.reset_position();
        rho_.reset_position();

        ESP_LOGI(TAG, "Homing complete - Rho: %ld steps, Theta: %ld steps/rev",
            rho_max_steps_, theta_steps_per_rotation_);

        homing_active_.store(false, std::memory_order_release);
        state_.store(HomingState::Complete, std::memory_order_release);

        return Result<void>::ok();
    }

} // namespace sand_table
