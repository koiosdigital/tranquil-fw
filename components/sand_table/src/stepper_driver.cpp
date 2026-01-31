#include "stepper_driver.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cmath>
#include <algorithm>

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
        return Result<void>::ok();
    }

    Result<void> StepperDriver::init_gpio() {
        // Configure step pin with maximum drive strength for TMC2209
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
        gpio_set_drive_capability(pins_.step, GPIO_DRIVE_CAP_3);  // 40mA max drive
        gpio_set_level(pins_.step, 0);

        // Configure direction pin with maximum drive strength
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
        gpio_set_drive_capability(pins_.dir, GPIO_DRIVE_CAP_3);  // 40mA max drive
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
    // RmtStepSequencer Implementation
    // =============================================================================

    RmtStepSequencer::~RmtStepSequencer() {
        deinit();
    }

    Result<void> RmtStepSequencer::init(gpio_num_t theta_step, gpio_num_t rho_step) {
        if (initialized_) {
            return Result<void>::ok();
        }

        // Configure GPIO with maximum drive strength BEFORE RMT takes over
        // TMC2209 requires clean step edges - use strongest drive capability
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << theta_step) | (1ULL << rho_step),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_drive_capability(theta_step, GPIO_DRIVE_CAP_3);  // 40mA max drive
        gpio_set_drive_capability(rho_step, GPIO_DRIVE_CAP_3);    // 40mA max drive

        // Create completion semaphore
        completion_sem_ = xSemaphoreCreateBinary();
        if (!completion_sem_) {
            ESP_LOGE(TAG, "Failed to create completion semaphore");
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Configure RMT TX channel for theta
        rmt_tx_channel_config_t theta_config = {
            .gpio_num = theta_step,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = HardwareConfig::RMT_RESOLUTION_HZ,  // 10 MHz
            .mem_block_symbols = 64,  // Hardware memory block
            .trans_queue_depth = 8,   // Extra depth for dual-channel queuing
            .intr_priority = 0,
            .flags = {
                .invert_out = false,
                .with_dma = false,  // Use interrupt mode for low latency
                .io_loop_back = false,
                .io_od_mode = false,
            },
        };

        if (rmt_new_tx_channel(&theta_config, &theta_channel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create theta RMT channel");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Configure RMT TX channel for rho
        rmt_tx_channel_config_t rho_config = theta_config;
        rho_config.gpio_num = rho_step;

        if (rmt_new_tx_channel(&rho_config, &rho_channel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create rho RMT channel");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Create a simple copy encoder for raw symbols
        rmt_copy_encoder_config_t encoder_config = {};
        if (rmt_new_copy_encoder(&encoder_config, &copy_encoder_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create copy encoder");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Register TX done callback BEFORE enabling (only on theta - they're synchronized)
        rmt_tx_event_callbacks_t cbs = {
            .on_trans_done = tx_done_callback,
        };

        if (rmt_tx_register_event_callbacks(theta_channel_, &cbs, this) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register RMT callback");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Enable channels BEFORE creating sync manager
        if (rmt_enable(theta_channel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable theta RMT channel");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        if (rmt_enable(rho_channel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable rho RMT channel");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Allow RMT hardware to stabilize before first transmission
        // This helps prevent timing issues on the first segment (first move speed bug)
        vTaskDelay(pdMS_TO_TICKS(1));

        // Create sync manager AFTER channels are enabled
        rmt_channel_handle_t channels[] = { theta_channel_, rho_channel_ };
        rmt_sync_manager_config_t sync_config = {
            .tx_channel_array = channels,
            .array_size = 2,
        };

        if (rmt_new_sync_manager(&sync_config, &sync_manager_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create RMT sync manager");
            deinit();
            return Result<void>::err(MotionError::HardwareFault);
        }

        initialized_ = true;
        return Result<void>::ok();
    }

    void RmtStepSequencer::deinit() {
        if (sync_manager_) {
            rmt_del_sync_manager(sync_manager_);
            sync_manager_ = nullptr;
        }
        if (theta_channel_) {
            rmt_disable(theta_channel_);
            rmt_del_channel(theta_channel_);
            theta_channel_ = nullptr;
        }
        if (rho_channel_) {
            rmt_disable(rho_channel_);
            rmt_del_channel(rho_channel_);
            rho_channel_ = nullptr;
        }
        if (copy_encoder_) {
            rmt_del_encoder(copy_encoder_);
            copy_encoder_ = nullptr;
        }
        if (completion_sem_) {
            vSemaphoreDelete(completion_sem_);
            completion_sem_ = nullptr;
        }
        initialized_ = false;
    }

    rmt_symbol_word_t IRAM_ATTR RmtStepSequencer::make_step_symbol(uint16_t interval_us) {
        // At 10 MHz: 1 tick = 100ns, so 1us = 10 ticks
        constexpr uint32_t TICKS_PER_US = HardwareConfig::RMT_RESOLUTION_HZ / 1000000;
        constexpr uint32_t PULSE_TICKS = HardwareConfig::MIN_STEP_PULSE_US * TICKS_PER_US;
        constexpr uint32_t MAX_DURATION = 32767;  // 15-bit max for RMT symbol duration

        // Use uint32_t to avoid overflow (interval_us * 10 can exceed uint16_t max)
        uint32_t total_ticks = static_cast<uint32_t>(interval_us) * TICKS_PER_US;

        // Clamp total ticks to max representable value
        if (total_ticks > MAX_DURATION + PULSE_TICKS) {
            total_ticks = MAX_DURATION + PULSE_TICKS;
        }

        uint16_t delay_ticks = (total_ticks > PULSE_TICKS)
            ? static_cast<uint16_t>(total_ticks - PULSE_TICKS)
            : 1;

        rmt_symbol_word_t symbol = {
            .duration0 = static_cast<uint16_t>(PULSE_TICKS),  // HIGH for step pulse
            .level0 = 1,
            .duration1 = delay_ticks,  // LOW for remaining interval
            .level1 = 0,
        };
        return symbol;
    }

    rmt_symbol_word_t IRAM_ATTR RmtStepSequencer::make_idle_symbol(uint16_t interval_us) {
        // No step pulse - just maintain timing (must use same structure as step symbol)
        constexpr uint32_t TICKS_PER_US = HardwareConfig::RMT_RESOLUTION_HZ / 1000000;
        constexpr uint32_t PULSE_TICKS = HardwareConfig::MIN_STEP_PULSE_US * TICKS_PER_US;
        constexpr uint32_t MAX_DURATION = 32767;  // 15-bit max for RMT symbol duration

        // Use uint32_t to avoid overflow (interval_us * 10 can exceed uint16_t max)
        uint32_t total_ticks = static_cast<uint32_t>(interval_us) * TICKS_PER_US;

        // Clamp total ticks to max representable value
        if (total_ticks > MAX_DURATION + PULSE_TICKS) {
            total_ticks = MAX_DURATION + PULSE_TICKS;
        }

        uint16_t delay_ticks = (total_ticks > PULSE_TICKS)
            ? static_cast<uint16_t>(total_ticks - PULSE_TICKS)
            : 1;

        // Use same timing structure as step symbol, but keep output LOW
        // (duration0 for "pulse" period, duration1 for delay - both LOW)
        rmt_symbol_word_t symbol = {
            .duration0 = static_cast<uint16_t>(PULSE_TICKS),   // Same timing as step pulse
            .level0 = 0,                // But keep LOW (no pulse)
            .duration1 = delay_ticks,   // Delay period
            .level1 = 0,                // LOW
        };
        return symbol;
    }

    void IRAM_ATTR RmtStepSequencer::bresenham_step(bool& step_theta, bool& step_rho) {
        step_theta = false;
        step_rho = false;

        if (bresenham_->theta_remaining == 0 && bresenham_->rho_remaining == 0) {
            return;
        }

        // Standard Bresenham algorithm for coordinated motion
        // Use TOTALS (fixed values) for error updates, not remaining counts
        if (bresenham_->theta_total >= bresenham_->rho_total) {
            // Theta is major axis - always step theta when remaining > 0
            step_theta = (bresenham_->theta_remaining > 0);
            bresenham_->error -= bresenham_->rho_total;  // Use fixed total
            if (bresenham_->error < 0) {
                bresenham_->error += bresenham_->theta_total;  // Use fixed total
                step_rho = (bresenham_->rho_remaining > 0);
            }
        }
        else {
            // Rho is major axis - always step rho when remaining > 0
            step_rho = (bresenham_->rho_remaining > 0);
            bresenham_->error -= bresenham_->theta_total;  // Use fixed total
            if (bresenham_->error < 0) {
                bresenham_->error += bresenham_->rho_total;  // Use fixed total
                step_theta = (bresenham_->theta_remaining > 0);
            }
        }

        // Update remaining counts
        if (step_theta) {
            bresenham_->theta_remaining--;
        }
        if (step_rho) {
            bresenham_->rho_remaining--;
        }
    }

    void IRAM_ATTR RmtStepSequencer::encode_chunk(uint8_t buffer_idx) {
        auto& theta_buf = theta_symbols_[buffer_idx];
        auto& rho_buf = rho_symbols_[buffer_idx];
        size_t count = 0;

        int32_t theta_steps_encoded = 0;
        int32_t rho_steps_encoded = 0;

        for (size_t i = 0; i < kChunkSize && steps_encoded_ < total_steps_; i++) {
            uint16_t interval = intervals_[steps_encoded_];
            bool step_theta, step_rho;

            bresenham_step(step_theta, step_rho);

            theta_buf[i] = step_theta ? make_step_symbol(interval) : make_idle_symbol(interval);
            rho_buf[i] = step_rho ? make_step_symbol(interval) : make_idle_symbol(interval);

            if (step_theta) theta_steps_encoded++;
            if (step_rho) rho_steps_encoded++;

            steps_encoded_++;
            count++;
        }

        symbol_counts_[buffer_idx] = count;

        // Update position counters atomically using fetch_add
        if (theta_pos_ && theta_steps_encoded > 0) {
            theta_pos_->fetch_add(theta_steps_encoded * bresenham_->theta_dir,
                std::memory_order_release);
        }
        if (rho_pos_ && rho_steps_encoded > 0) {
            rho_pos_->fetch_add(rho_steps_encoded * bresenham_->rho_dir,
                std::memory_order_release);
        }
    }

    bool IRAM_ATTR RmtStepSequencer::tx_done_callback(
        rmt_channel_handle_t channel,
        const rmt_tx_done_event_data_t* edata,
        void* user_ctx)
    {
        auto* self = static_cast<RmtStepSequencer*>(user_ctx);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Check for stop request
        if (self->stop_requested_.load(std::memory_order_acquire)) {
            self->pending_tx_.store(0, std::memory_order_release);
            self->executing_.store(false, std::memory_order_release);
            xSemaphoreGiveFromISR(self->completion_sem_, &xHigherPriorityTaskWoken);
            return xHigherPriorityTaskWoken == pdTRUE;
        }

        // Decrement pending counter - this transmission just completed
        self->pending_tx_.fetch_sub(1, std::memory_order_acq_rel);

        // Switch buffer and try to encode next chunk
        uint8_t next_buffer = self->active_buffer_ ^ 1;
        self->active_buffer_ = next_buffer;
        self->encode_chunk(next_buffer);

        if (self->symbol_counts_[next_buffer] > 0) {
            // Queue next transmission - increment pending counter BEFORE queuing
            self->pending_tx_.fetch_add(1, std::memory_order_release);

            rmt_transmit_config_t tx_config = {
                .loop_count = 0,
                .flags = {
                    .eot_level = 0,  // Keep low when done
                    .queue_nonblocking = true,
                },
            };

            // Queue to both channels - sync manager ensures they start together
            rmt_transmit(self->theta_channel_, self->copy_encoder_,
                self->theta_symbols_[next_buffer],
                self->symbol_counts_[next_buffer] * sizeof(rmt_symbol_word_t),
                &tx_config);
            rmt_transmit(self->rho_channel_, self->copy_encoder_,
                self->rho_symbols_[next_buffer],
                self->symbol_counts_[next_buffer] * sizeof(rmt_symbol_word_t),
                &tx_config);
        }

        // FIXED: Check ACTUAL pending count (not stale value from before queuing)
        // Only signal completion when no transmissions pending AND all steps encoded
        if (self->pending_tx_.load(std::memory_order_acquire) == 0 &&
            self->steps_encoded_ >= self->total_steps_) {
            self->executing_.store(false, std::memory_order_release);
            xSemaphoreGiveFromISR(self->completion_sem_, &xHigherPriorityTaskWoken);
        }

        return xHigherPriorityTaskWoken == pdTRUE;
    }

    Result<void> RmtStepSequencer::execute(
        BresenhamState& bresenham,
        const uint16_t* intervals,
        uint32_t total_steps,
        std::atomic<int32_t>& theta_pos,
        std::atomic<int32_t>& rho_pos)
    {
        if (!initialized_) {
            return Result<void>::err(MotionError::InvalidState);
        }

        if (executing_.load(std::memory_order_acquire)) {
            return Result<void>::err(MotionError::InvalidState);
        }

        if (total_steps == 0) {
            return Result<void>::ok();
        }

        // Setup execution state
        bresenham_ = &bresenham;
        intervals_ = intervals;
        total_steps_ = total_steps;
        theta_pos_ = &theta_pos;
        rho_pos_ = &rho_pos;
        steps_encoded_ = 0;
        pending_tx_.store(0, std::memory_order_release);
        stop_requested_.store(false, std::memory_order_release);
        executing_.store(true, std::memory_order_release);

        // Clear semaphore
        xSemaphoreTake(completion_sem_, 0);

        // Encode first chunk into buffer 0
        active_buffer_ = 0;
        encode_chunk(0);

        if (symbol_counts_[0] == 0) {
            executing_.store(false, std::memory_order_release);
            return Result<void>::ok();
        }

        // Pre-encode second chunk for ping-pong buffering
        if (steps_encoded_ < total_steps_) {
            encode_chunk(1);
        }

        // Start transmission - track pending count
        pending_tx_.fetch_add(1, std::memory_order_release);

        rmt_transmit_config_t tx_config = {
            .loop_count = 0,
            .flags = {
                .eot_level = 0,
                .queue_nonblocking = false,  // Block on first transmit to ensure sync
            },
        };

        esp_err_t err = rmt_transmit(theta_channel_, copy_encoder_,
            theta_symbols_[0],
            symbol_counts_[0] * sizeof(rmt_symbol_word_t),
            &tx_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start theta transmission: %d", err);
            pending_tx_.store(0, std::memory_order_release);
            executing_.store(false, std::memory_order_release);
            return Result<void>::err(MotionError::HardwareFault);
        }

        err = rmt_transmit(rho_channel_, copy_encoder_,
            rho_symbols_[0],
            symbol_counts_[0] * sizeof(rmt_symbol_word_t),
            &tx_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start rho transmission: %d", err);
            pending_tx_.store(0, std::memory_order_release);
            rmt_tx_wait_all_done(theta_channel_, 100);
            executing_.store(false, std::memory_order_release);
            return Result<void>::err(MotionError::HardwareFault);
        }

        // Queue second chunk if available (set active_buffer_ FIRST to avoid race)
        if (symbol_counts_[1] > 0) {
            pending_tx_.fetch_add(1, std::memory_order_release);
            active_buffer_ = 1;  // Set before queuing to avoid race with callback
            tx_config.flags.queue_nonblocking = true;
            rmt_transmit(theta_channel_, copy_encoder_,
                theta_symbols_[1],
                symbol_counts_[1] * sizeof(rmt_symbol_word_t),
                &tx_config);
            rmt_transmit(rho_channel_, copy_encoder_,
                rho_symbols_[1],
                symbol_counts_[1] * sizeof(rmt_symbol_word_t),
                &tx_config);
        }

        ESP_LOGD(TAG, "RMT transmission started: %lu total steps", total_steps);

        // Wait for completion (with timeout)
        const TickType_t timeout_ticks = pdMS_TO_TICKS(30000);  // 30 second timeout
        if (xSemaphoreTake(completion_sem_, timeout_ticks) != pdTRUE) {
            ESP_LOGE(TAG, "RMT execution timeout");
            stop();
            return Result<void>::err(MotionError::Timeout);
        }

        // Check for stop request (emergency stop)
        if (stop_requested_.load(std::memory_order_acquire)) {
            return Result<void>::err(MotionError::EmergencyStop);
        }

        return Result<void>::ok();
    }

    void RmtStepSequencer::stop() {
        stop_requested_.store(true, std::memory_order_release);

        if (theta_channel_) {
            rmt_tx_wait_all_done(theta_channel_, 10);
        }
        if (rho_channel_) {
            rmt_tx_wait_all_done(rho_channel_, 10);
        }

        pending_tx_.store(0, std::memory_order_release);
        executing_.store(false, std::memory_order_release);

        // Signal completion in case execute() is waiting
        if (completion_sem_) {
            xSemaphoreGive(completion_sem_);
        }
    }

    void IRAM_ATTR RmtStepSequencer::stop_from_isr() {
        // ISR-safe version - only set flag and signal semaphore
        // No blocking calls allowed
        stop_requested_.store(true, std::memory_order_release);
        pending_tx_.store(0, std::memory_order_release);
        executing_.store(false, std::memory_order_release);

        if (completion_sem_) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(completion_sem_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
        rmt_sequencer_.deinit();
    }

    Result<void> CoordinatedStepperController::init() {
        auto result = rmt_sequencer_.init(theta_.step_pin(), rho_.step_pin());
        if (result.is_err()) {
            ESP_LOGE(TAG, "Failed to initialize RMT step sequencer");
            return result;
        }
        return Result<void>::ok();
    }

    void CoordinatedStepperController::enable() {
        theta_.set_enabled(true);
        rho_.set_enabled(true);

        // 250ms warmup delay after enabling motors
        // TMC drivers need time to stabilize after enable
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    void CoordinatedStepperController::disable() {
        theta_.set_enabled(false);
        rho_.set_enabled(false);
    }

    void CoordinatedStepperController::emergency_stop() {
        rmt_sequencer_.stop();
        bresenham_.clear();
    }

    void IRAM_ATTR CoordinatedStepperController::emergency_stop_from_isr() {
        rmt_sequencer_.stop_from_isr();
        // Don't clear bresenham from ISR - not critical
    }

    VelocityProfile CoordinatedStepperController::calculate_velocity_profile(
        const MotionSegment& segment,
        float steps_per_unit) const
    {
        VelocityProfile profile;

        // Calculate total steps - use max to match Bresenham's major axis count
        const uint32_t total_steps = static_cast<uint32_t>(
            std::max(std::abs(segment.delta_theta_steps), std::abs(segment.delta_rho_steps))
            );

        if (total_steps == 0) {
            return profile;
        }

        // Hardware limits
        const float max_step_rate = static_cast<float>(HardwareConfig::MAX_STEP_RATE_HZ);
        const float min_step_rate = 310.0f;  // Limited by RMT symbol duration

        // Use the DOMINANT motor's velocity profile (more steps = controls timing)
        // Each motor has independent velocity planning - rho reversal doesn't affect theta velocity
        const bool theta_is_dominant = std::abs(segment.delta_theta_steps) >= std::abs(segment.delta_rho_steps);

        // Select the dominant motor's velocities
        const float entry_rpm = theta_is_dominant ? segment.theta_entry_velocity : segment.rho_entry_velocity;
        const float exit_rpm = theta_is_dominant ? segment.theta_exit_velocity : segment.rho_exit_velocity;

        // Convert RPM to steps/s using the relationship:
        //   motion_time = distance / (rpm / 60)
        //   velocity_steps_s = total_steps / motion_time
        //                    = total_steps * (rpm / 60) / distance
        //                    = rpm * (total_steps / distance) / 60
        //                    = rpm * steps_per_unit / 60
        const float rpm_to_steps_s = steps_per_unit / 60.0f;

        float entry_steps_s = entry_rpm * rpm_to_steps_s;
        float exit_steps_s = exit_rpm * rpm_to_steps_s;
        float nominal_steps_s = segment.nominal_velocity * rpm_to_steps_s;

        // Clamp velocities to hardware limits
        entry_steps_s = std::max(min_step_rate, std::min(entry_steps_s, max_step_rate));
        exit_steps_s = std::max(min_step_rate, std::min(exit_steps_s, max_step_rate));
        nominal_steps_s = std::max(min_step_rate, std::min(nominal_steps_s, max_step_rate));

        // Ensure minimum motion time based on max step rate
        const float min_motion_time = static_cast<float>(total_steps) / max_step_rate;
        const float nominal_motion_time = static_cast<float>(total_steps) / nominal_steps_s;
        if (nominal_motion_time < min_motion_time) {
            nominal_steps_s = max_step_rate;
        }

        // Acceleration in steps/s^2
        // Use a reasonable default that gives smooth motion
        // For sand table: accelerate from 0 to max in ~0.5s = 50000/0.5 = 100000 steps/s^2
        // But that's aggressive. Use 10000 steps/s^2 for gentler acceleration.
        const float accel_steps_s2 = 10000.0f;  // steps/s^2

        // Calculate distances (in steps) for acceleration and deceleration
        // Using kinematic equation: v^2 = v0^2 + 2*a*d  =>  d = (v^2 - v0^2) / (2*a)
        float accel_dist = 0.0f;
        float decel_dist = 0.0f;

        if (nominal_steps_s > entry_steps_s) {
            accel_dist = (nominal_steps_s * nominal_steps_s - entry_steps_s * entry_steps_s) /
                (2.0f * accel_steps_s2);
        }
        if (nominal_steps_s > exit_steps_s) {
            decel_dist = (nominal_steps_s * nominal_steps_s - exit_steps_s * exit_steps_s) /
                (2.0f * accel_steps_s2);
        }

        // Check if we can reach cruise velocity (trapezoidal profile)
        // or if we need a triangle profile
        const float total_dist = static_cast<float>(total_steps);

        if (accel_dist + decel_dist <= total_dist) {
            // Trapezoidal profile: we reach cruise velocity
            profile.accel_steps = static_cast<uint32_t>(accel_dist);
            profile.decel_steps = static_cast<uint32_t>(decel_dist);
            profile.cruise_steps = total_steps - profile.accel_steps - profile.decel_steps;
            profile.cruise_velocity = nominal_steps_s;
        }
        else {
            // Triangle profile: can't reach cruise velocity
            // Find peak velocity: v_peak^2 = (v_entry^2 + v_exit^2 + 2*a*d) / 2
            float v_peak_sq = (entry_steps_s * entry_steps_s +
                exit_steps_s * exit_steps_s +
                2.0f * accel_steps_s2 * total_dist) / 2.0f;

            if (v_peak_sq < min_step_rate * min_step_rate) {
                v_peak_sq = min_step_rate * min_step_rate;
            }

            float v_peak = std::sqrt(v_peak_sq);
            v_peak = std::min(v_peak, max_step_rate);

            // Recalculate accel/decel distances with peak velocity
            accel_dist = (v_peak * v_peak - entry_steps_s * entry_steps_s) / (2.0f * accel_steps_s2);
            decel_dist = total_dist - accel_dist;

            profile.accel_steps = static_cast<uint32_t>(std::max(0.0f, accel_dist));
            profile.decel_steps = total_steps - profile.accel_steps;
            profile.cruise_steps = 0;
            profile.cruise_velocity = v_peak;
        }

        profile.entry_velocity = entry_steps_s;
        profile.exit_velocity = exit_steps_s;
        profile.acceleration = accel_steps_s2;

        return profile;
    }

    Result<void> CoordinatedStepperController::execute_segment(
        const MotionSegment& segment)
    {
        if (rmt_sequencer_.is_executing()) {
            return Result<void>::err(MotionError::InvalidState);
        }

        // Calculate total steps for Bresenham (major axis)
        const uint32_t total_steps = static_cast<uint32_t>(
            std::max(std::abs(segment.delta_theta_steps), std::abs(segment.delta_rho_steps)));

        // No steps to execute
        if (total_steps == 0) {
            return Result<void>::ok();
        }

        // Set directions
        theta_.set_direction(segment.delta_theta_steps >= 0);
        rho_.set_direction(segment.delta_rho_steps >= 0);

        // Setup Bresenham state
        bresenham_.theta_remaining = std::abs(segment.delta_theta_steps);
        bresenham_.rho_remaining = std::abs(segment.delta_rho_steps);
        bresenham_.theta_total = bresenham_.theta_remaining;  // Store original totals
        bresenham_.rho_total = bresenham_.rho_remaining;      // for fixed Bresenham error updates
        bresenham_.theta_dir = (segment.delta_theta_steps >= 0) ? 1 : -1;
        bresenham_.rho_dir = (segment.delta_rho_steps >= 0) ? 1 : -1;

        // Initialize Bresenham error term - consistent initialization based on major axis
        const int32_t major = std::max(bresenham_.theta_total, bresenham_.rho_total);
        bresenham_.error = major / 2;

        // Calculate velocity profile
        const float steps_per_unit = (segment.distance > 0.0f)
            ? static_cast<float>(total_steps) / segment.distance
            : static_cast<float>(total_steps);
        const VelocityProfile profile = calculate_velocity_profile(segment, steps_per_unit);

        // Pre-compute interval table (all FPU operations happen here, in task context)
        prepare_interval_table(profile, total_steps);

        // Debug: Validate first interval (for first move speed issue)
        constexpr uint32_t min_interval_us = 1000000 / HardwareConfig::MAX_STEP_RATE_HZ;  // 20us
        if (interval_table_.intervals[0] < min_interval_us) {
            ESP_LOGW(TAG, "First interval too fast: %u us (min: %lu us)",
                interval_table_.intervals[0], min_interval_us);
        }

        // Capture position before execution for drift debugging
        int32_t theta_before = theta_.position();
        int32_t rho_before = rho_.position();

        ESP_LOGD(TAG, "RMT segment: theta=%ld, rho=%ld, total=%lu, cruise_v=%.1f, interval[0]=%u us",
            segment.delta_theta_steps, segment.delta_rho_steps,
            total_steps, profile.cruise_velocity, interval_table_.intervals[0]);

        // Execute via RMT sequencer (blocking)
        auto result = rmt_sequencer_.execute(
            bresenham_,
            interval_table_.intervals,
            interval_table_.total_steps,
            theta_.position_,
            rho_.position_
        );

        return result;
    }

    void CoordinatedStepperController::prepare_interval_table(
        const VelocityProfile& profile,
        uint32_t total_steps)
    {
        interval_table_.clear();
        interval_table_.total_steps = std::min(total_steps, static_cast<uint32_t>(kMaxIntervalsPerSegment));

        // RMT symbol max duration: (32767 + 20) ticks @ 10MHz = 3278.7µs
        // Minimum velocity = 1,000,000 / 3278 ≈ 305 steps/s
        // Use 310 steps/s for margin
        const float min_velocity = 310.0f;  // steps/s (limited by RMT symbol max duration)
        const float max_velocity = static_cast<float>(HardwareConfig::MAX_STEP_RATE_HZ);

        for (uint32_t step = 0; step < interval_table_.total_steps; ++step) {
            float velocity;

            if (step < profile.accel_steps) {
                // Acceleration phase: v = sqrt(v_entry^2 + 2*a*s)
                velocity = std::sqrt(
                    profile.entry_velocity * profile.entry_velocity +
                    2.0f * profile.acceleration * static_cast<float>(step)
                );
            }
            else if (step < profile.accel_steps + profile.cruise_steps) {
                // Cruise phase
                velocity = profile.cruise_velocity;
            }
            else {
                // Deceleration phase
                const uint32_t decel_steps_taken = step - profile.accel_steps - profile.cruise_steps;
                const uint32_t decel_steps_remaining = (profile.decel_steps > decel_steps_taken)
                    ? profile.decel_steps - decel_steps_taken
                    : 0;
                velocity = std::sqrt(
                    profile.exit_velocity * profile.exit_velocity +
                    2.0f * profile.acceleration * static_cast<float>(decel_steps_remaining)
                );
            }

            // Clamp velocity to range that RMT can represent accurately
            velocity = std::max(min_velocity, std::min(velocity, max_velocity));

            // Convert velocity to interval in MICROSECONDS (for RMT)
            // interval_us = 1,000,000 / velocity
            uint32_t interval_us = static_cast<uint32_t>(1000000.0f / velocity);

            // Clamp interval to safe range
            // Minimum: 20µs (50kHz max step rate)
            // Maximum: 3225µs (~310Hz min step rate, fits in RMT symbol)
            constexpr uint32_t min_interval_us = 1000000 / HardwareConfig::MAX_STEP_RATE_HZ;
            constexpr uint32_t max_interval_us = 3225;  // ~310 steps/s, fits in single RMT symbol
            if (interval_us < min_interval_us) {
                interval_us = min_interval_us;
            }
            if (interval_us > max_interval_us) {
                interval_us = max_interval_us;
            }

            interval_table_.intervals[step] = static_cast<uint16_t>(interval_us);
        }
    }

} // namespace sand_table
